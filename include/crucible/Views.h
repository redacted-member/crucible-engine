// SPDX-License-Identifier: MIT
#pragma once

// GUI-agnostic read-back. A view is a read-only window onto the running engine, identified by {instance, id} ("master" /
// "meter", "params" / "filter_cutoff.band", "osc1" / "sample_grains") and carrying one of four typed payloads: a scalar
// (one float), a vector of floats (scopes, spectra, bands), points (a flat float array read in tuples of `stride`) or
// a mask (32-bit words holding bits, copied bit for bit into the float array). Every payload element is 32 bits wide.
//
// Live views are published by the audio thread, at most once per render call and no more often than the fastest
// subscriber asked, and only while somebody is subscribed: a view nobody subscribes to costs nothing, and with no
// subscription at all publish() is a single atomic load. Scalars are one atomic word holding value and sequence number.
// The other payloads go through four slots of a lock-free multi-reader buffer: the writer never waits (when every
// candidate slot is being read it skips that publication), readers on any thread copy a consistent snapshot and its
// sequence number and never see a torn payload. Pulled views are computed on the reading thread from control-side data
// (sample overviews, cloud points; read under RtShared::withLatest(), so a replaced sample is never freed under the
// reader) or from another view (spectra); their sequence number advances when the content changes. Nothing on the
// audio thread locks, allocates or waits.
//
// Threads: registration, find(), info(), subscribe() and unsubscribe() belong to one control thread; read() and
// sequence-bearing queries work from any thread; publish() is the audio thread's. The built-in views of an engine are
// listed in docs/ARCHITECTURE.md ("Views").

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace crucible
{

enum class ViewKind
{
    Scalar = 0, // one float
    Vector = 1, // up to maxElements floats
    Points = 2, // up to maxElements floats, tuples of `stride`
    Mask = 3    // up to maxElements 32-bit words
};

struct ViewInfo
{
    std::string instance;
    std::string id;
    std::string description;
    ViewKind kind = ViewKind::Vector;
    int maxElements = 1; // payload capacity in 32-bit elements
    int stride = 1;      // Points: floats per point
    bool live = true;    // false: pulled (computed by the reader)
};

// Where a producer writes. Elements are 32 bits; a producer sets the size it filled.
class ViewWriter
{
public:
    ViewWriter(float* storage, int capacityElements) : buffer(storage), cap(capacityElements) {}

    int capacity() const { return cap; }
    int size() const { return count; }
    float* data() { return buffer; }
    void setSize(int n) { count = n < 0 ? 0 : (n > cap ? cap : n); }

    void scalar(float v)
    {
        buffer[0] = v;
        count = 1;
    }
    void push(float v)
    {
        if (count < cap)
            buffer[count++] = v;
    }
    void pushWord(std::uint32_t w)
    {
        if (count < cap)
            std::memcpy(&buffer[count++], &w, sizeof w);
    }
    void copy(const float* src, int n)
    {
        n = n < 0 ? 0 : (n > cap ? cap : n);
        std::memcpy(buffer, src, static_cast<std::size_t>(n) * sizeof(float));
        count = n;
    }

private:
    float* buffer;
    int cap;
    int count = 0;
};

// What a producer is asked for: an operation code and two indices, all private to whoever registered the view.
struct ViewArgs
{
    int op = 0;
    int index = 0;
    int extra = 0;
};

// Fills `out` from `context`. Live producers run on the audio thread (no allocation, no locks); pulled ones on the
// reading thread.
using ViewProducer = void (*)(void* context, const ViewArgs& args, ViewWriter& out);

class ViewHub
{
public:
    static constexpr int kMaxViews = 8192;           // registered views, including those made by the resolver
    static constexpr int kMaxSubscribed = kMaxViews; // distinct live views that can ever be subscribed in one hub

    ViewHub();
    ~ViewHub();
    ViewHub(const ViewHub&) = delete;
    ViewHub& operator=(const ViewHub&) = delete;

    struct Spec
    {
        ViewInfo info;
        ViewProducer produce = nullptr;
        void* context = nullptr;
        ViewArgs args;
        int dependsOn = -1; // subscribing to this view subscribes to that one too (a spectrum needs its scope)
    };

    // ---- Control thread.
    // Registers a view and returns its index (-1 when the hub is full or the {instance, id} pair exists).
    int add(Spec spec);
    // Makes ids that are not in the list resolvable on demand (a parameter's view of voice 3); the resolver calls add().
    using Resolver = int (*)(void* context, ViewHub& hub, const std::string& instance, const std::string& id);
    void setResolver(Resolver resolver, void* context);
    // Sample rate for the rate limits.
    void prepare(double sampleRate);

    int count() const { return registered.load(std::memory_order_acquire); }
    const ViewInfo& info(int view) const;
    // Index of {instance, id}, or -1.
    int find(const std::string& instance, const std::string& id);

    // Starts publishing a live view (and what it depends on) at most maxHz times per second (<= 0: every render
    // call). Returns a subscription id (> 0), or 0 for an unknown view or when kMaxSubscribed views are in use.
    // Pulled views accept subscriptions too (they only keep their dependency live).
    int subscribe(int view, float maxHz = 0.0f);
    void unsubscribe(int subscription);
    // Number of live subscriptions of a view.
    int subscribers(int view) const;

    // ---- Any thread.
    // Copies the newest snapshot into out. Returns the element count (0 when nothing was published yet, or the view is
    // not subscribed), -1 for an unknown view and -2 when capacity is too small (nothing is copied). *sequence (may be
    // null) receives the publication counter: 0 before the first publication, then increasing by at least 1 per
    // publication; a payload with the same sequence number is the same snapshot. A scalar's counter is 32 bits wide.
    int read(int view, float* out, int capacity, std::uint64_t* sequence) const;
    // Publications skipped because every spare slot was being read, since the first subscription.
    std::uint64_t dropped(int view) const;

    // ---- Audio thread. Once per render call, with the number of samples rendered.
    void publish(int numSamples) noexcept;

private:
    static constexpr int kSlots = 4;
    struct Slot
    {
        std::unique_ptr<float[]> data;
        int count = 0;
        std::uint64_t sequence = 0;
        std::atomic<int> readers{0};
        std::atomic<bool> writing{false};
    };
    struct State
    {
        Spec spec;
        // Live views.
        std::atomic<bool> active{false};
        std::atomic<bool> fresh{false};
        std::atomic<double> interval{0.0};    // seconds, 0 = every render call
        double elapsed = 0.0;                 // samples since the last publication (audio thread)
        std::atomic<std::uint64_t> packed{0}; // scalars: sequence << 32 | value bits
        std::array<Slot, kSlots> slots;
        std::atomic<int> latest{-1};
        std::uint64_t sequence = 0; // audio thread
        int lastWritten = 0;        // audio thread
        std::atomic<std::uint64_t> skipped{0};
        bool listed = false; // control thread: in the live list
        // Pulled views.
        std::mutex pullMutex;
        std::uint64_t pullHash = 0;
        std::uint64_t pullSequence = 0;
    };
    struct Subscription
    {
        int id;
        int view;
        float hz;
        int dependency; // subscription id of the view this one depends on, or 0
    };

    State* stateOf(int view) const;
    State* ensureState(int view);
    int subscribeLocked(int view, float maxHz);
    void unsubscribeLocked(int subscription);
    void refreshActivity(int view);
    void publishOne(State& s) noexcept;
    int readLive(State& s, float* out, int capacity, std::uint64_t* sequence) const;
    int readPulled(State& s, float* out, int capacity, std::uint64_t* sequence) const;

    mutable std::mutex control;
    std::deque<Spec> specs; // control thread; stable references for info()
    std::unordered_map<std::string, int> byName;
    std::vector<std::unique_ptr<State>> owned; // control thread; states live as long as the hub
    std::unique_ptr<std::atomic<State*>[]> states;
    std::atomic<int> registered{0};
    std::vector<Subscription> subs;
    int nextSubscription = 1;
    Resolver resolver = nullptr;
    void* resolverContext = nullptr;
    std::atomic<double> sampleRate{48000.0};

    std::unique_ptr<std::atomic<State*>[]> live;
    std::atomic<int> liveCount{0};
    std::atomic<int> activeCount{0};
};

} // namespace crucible
