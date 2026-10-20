// SPDX-License-Identifier: MIT
#include <crucible/Views.h>

#include <algorithm>

namespace crucible
{
namespace
{
std::uint64_t fnv1a(const float* data, int count)
{
    std::uint64_t h = 1469598103934665603ULL;
    const auto* bytes = reinterpret_cast<const unsigned char*>(data);
    const std::size_t n = static_cast<std::size_t>(count) * sizeof(float);
    for (std::size_t i = 0; i < n; ++i)
    {
        h ^= bytes[i];
        h *= 1099511628211ULL;
    }
    return h ^ static_cast<std::uint64_t>(count);
}
} // namespace

ViewHub::ViewHub() : states(new std::atomic<State*>[kMaxViews]()), live(new std::atomic<State*>[kMaxSubscribed]()) {}

ViewHub::~ViewHub() = default;

int ViewHub::add(Spec spec)
{
    std::lock_guard<std::mutex> lock(control);
    if (static_cast<int>(specs.size()) >= kMaxViews || spec.produce == nullptr)
        return -1;
    const std::string key = spec.info.instance + "/" + spec.info.id;
    if (byName.count(key) != 0)
        return -1;
    spec.info.maxElements = std::max(spec.info.maxElements, 1);
    spec.info.stride = std::max(spec.info.stride, 1);
    if (spec.info.kind == ViewKind::Scalar)
        spec.info.maxElements = 1;
    const int index = static_cast<int>(specs.size());
    byName[key] = index;
    specs.push_back(std::move(spec));
    if (!specs.back().info.live)
    {
        auto s = std::make_unique<State>();
        s->spec = specs.back();
        states[static_cast<std::size_t>(index)].store(s.get(), std::memory_order_release);
        owned.push_back(std::move(s));
    }
    registered.store(index + 1, std::memory_order_release);
    return index;
}

void ViewHub::setResolver(Resolver fn, void* context)
{
    std::lock_guard<std::mutex> lock(control);
    resolver = fn;
    resolverContext = context;
}

void ViewHub::prepare(double rate)
{
    sampleRate.store(rate > 0.0 ? rate : 48000.0, std::memory_order_relaxed);
}

const ViewInfo& ViewHub::info(int view) const
{
    static const ViewInfo none;
    std::lock_guard<std::mutex> lock(control);
    return view >= 0 && view < static_cast<int>(specs.size()) ? specs[static_cast<std::size_t>(view)].info : none;
}

int ViewHub::find(const std::string& instance, const std::string& id)
{
    Resolver fn = nullptr;
    void* context = nullptr;
    {
        std::lock_guard<std::mutex> lock(control);
        const auto it = byName.find(instance + "/" + id);
        if (it != byName.end())
            return it->second;
        fn = resolver;
        context = resolverContext;
    }
    return fn != nullptr ? fn(context, *this, instance, id) : -1;
}

ViewHub::State* ViewHub::stateOf(int view) const
{
    if (view < 0 || view >= kMaxViews)
        return nullptr;
    return states[static_cast<std::size_t>(view)].load(std::memory_order_acquire);
}

// Control thread, holding `control`. Live views get their buffers and a place in the audio thread's list the first
// time they are subscribed; both stay for the life of the hub.
ViewHub::State* ViewHub::ensureState(int view)
{
    if (view < 0 || view >= static_cast<int>(specs.size()))
        return nullptr;
    if (State* s = stateOf(view))
        return s;
    const int listed = liveCount.load(std::memory_order_relaxed);
    if (listed >= kMaxSubscribed)
        return nullptr;
    auto s = std::make_unique<State>();
    s->spec = specs[static_cast<std::size_t>(view)];
    if (s->spec.info.kind != ViewKind::Scalar)
        for (auto& slot : s->slots)
            slot.data = std::make_unique<float[]>(static_cast<std::size_t>(s->spec.info.maxElements));
    s->listed = true;
    State* raw = s.get();
    owned.push_back(std::move(s));
    states[static_cast<std::size_t>(view)].store(raw, std::memory_order_release);
    live[static_cast<std::size_t>(listed)].store(raw, std::memory_order_relaxed);
    liveCount.store(listed + 1, std::memory_order_release);
    return raw;
}

int ViewHub::subscribeLocked(int view, float maxHz)
{
    State* s = ensureState(view);
    if (s == nullptr)
        return 0;
    Subscription sub{nextSubscription, view, maxHz, 0};
    const int dep = specs[static_cast<std::size_t>(view)].dependsOn;
    if (dep >= 0)
    {
        sub.dependency = subscribeLocked(dep, maxHz);
        if (sub.dependency == 0)
            return 0;
    }
    ++nextSubscription;
    subs.push_back(sub);
    refreshActivity(view);
    return sub.id;
}

int ViewHub::subscribe(int view, float maxHz)
{
    std::lock_guard<std::mutex> lock(control);
    return subscribeLocked(view, maxHz);
}

void ViewHub::unsubscribeLocked(int id)
{
    const auto it = std::find_if(subs.begin(), subs.end(), [id](const Subscription& s) { return s.id == id; });
    if (it == subs.end())
        return;
    const Subscription gone = *it;
    subs.erase(it);
    refreshActivity(gone.view);
    if (gone.dependency != 0)
        unsubscribeLocked(gone.dependency);
}

void ViewHub::unsubscribe(int subscription)
{
    std::lock_guard<std::mutex> lock(control);
    unsubscribeLocked(subscription);
}

int ViewHub::subscribers(int view) const
{
    std::lock_guard<std::mutex> lock(control);
    return static_cast<int>(
        std::count_if(subs.begin(), subs.end(), [view](const Subscription& s) { return s.view == view; }));
}

// Control thread, holding `control`: recomputes the rate limit and the active flag of a live view.
void ViewHub::refreshActivity(int view)
{
    State* s = stateOf(view);
    if (s == nullptr || !s->spec.info.live)
        return;
    bool any = false;
    bool unlimited = false;
    float fastest = 0.0f;
    for (const auto& sub : subs)
        if (sub.view == view)
        {
            any = true;
            if (sub.hz <= 0.0f)
                unlimited = true;
            else
                fastest = std::max(fastest, sub.hz);
        }
    const bool was = s->active.load(std::memory_order_relaxed);
    if (any)
    {
        s->interval.store(unlimited || fastest <= 0.0f ? 0.0 : 1.0 / static_cast<double>(fastest),
                          std::memory_order_relaxed);
        if (!was)
        {
            s->fresh.store(true, std::memory_order_relaxed);
            s->active.store(true, std::memory_order_release);
            activeCount.fetch_add(1, std::memory_order_relaxed);
        }
    }
    else if (was)
    {
        s->active.store(false, std::memory_order_release);
        activeCount.fetch_sub(1, std::memory_order_relaxed);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Audio thread.

void ViewHub::publish(int numSamples) noexcept
{
    if (activeCount.load(std::memory_order_relaxed) == 0)
        return;
    const double rate = sampleRate.load(std::memory_order_relaxed);
    const int n = liveCount.load(std::memory_order_acquire);
    for (int i = 0; i < n; ++i)
    {
        State* s = live[static_cast<std::size_t>(i)].load(std::memory_order_relaxed);
        if (!s->active.load(std::memory_order_acquire))
            continue;
        if (s->fresh.exchange(false, std::memory_order_acq_rel))
            s->elapsed = 0.0;
        else
        {
            const double period = s->interval.load(std::memory_order_relaxed) * rate;
            s->elapsed += numSamples;
            if (period > 0.0 && s->elapsed < period)
                continue;
            // Keep the remainder so that the cadence does not depend on the block size, but never bank a burst.
            s->elapsed = period > 0.0 && s->elapsed < 2.0 * period ? s->elapsed - period : 0.0;
        }
        publishOne(*s);
    }
}

void ViewHub::publishOne(State& s) noexcept
{
    const Spec& spec = s.spec;
    if (spec.info.kind == ViewKind::Scalar)
    {
        float value = 0.0f;
        ViewWriter writer(&value, 1);
        spec.produce(spec.context, spec.args, writer);
        std::uint32_t bits;
        std::memcpy(&bits, &value, sizeof bits);
        const std::uint64_t seq = ++s.sequence;
        s.packed.store((seq << 32) | bits, std::memory_order_release);
        return;
    }
    // Pick a slot that is not the newest and that nobody is reading. The seq_cst pair (writing / readers) is the
    // reader's mirror image: either the reader sees the slot as being written and retries, or the writer sees the
    // reader and picks another slot.
    const int newest = s.latest.load(std::memory_order_relaxed);
    for (int k = 1; k <= kSlots; ++k)
    {
        const int i = (s.lastWritten + k) % kSlots;
        if (i == newest)
            continue;
        Slot& slot = s.slots[static_cast<std::size_t>(i)];
        if (slot.readers.load(std::memory_order_seq_cst) != 0)
            continue;
        slot.writing.store(true, std::memory_order_seq_cst);
        if (slot.readers.load(std::memory_order_seq_cst) != 0)
        {
            slot.writing.store(false, std::memory_order_release);
            continue;
        }
        ViewWriter writer(slot.data.get(), spec.info.maxElements);
        spec.produce(spec.context, spec.args, writer);
        slot.count = writer.size();
        slot.sequence = ++s.sequence;
        slot.writing.store(false, std::memory_order_release);
        s.latest.store(i, std::memory_order_release);
        s.lastWritten = i;
        return;
    }
    s.skipped.fetch_add(1, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------------------------------------------
// Readers.

int ViewHub::readLive(State& s, float* out, int capacity, std::uint64_t* sequence) const
{
    if (s.spec.info.kind == ViewKind::Scalar)
    {
        const std::uint64_t word = s.packed.load(std::memory_order_acquire);
        if (sequence != nullptr)
            *sequence = word >> 32;
        if ((word >> 32) == 0)
            return 0;
        if (capacity < 1)
            return -2;
        const std::uint32_t bits = static_cast<std::uint32_t>(word & 0xffffffffU);
        std::memcpy(out, &bits, sizeof bits);
        return 1;
    }
    for (int attempt = 0; attempt < 100000; ++attempt)
    {
        const int i = s.latest.load(std::memory_order_acquire);
        if (i < 0)
        {
            if (sequence != nullptr)
                *sequence = 0;
            return 0;
        }
        Slot& slot = s.slots[static_cast<std::size_t>(i)];
        slot.readers.fetch_add(1, std::memory_order_seq_cst);
        if (slot.writing.load(std::memory_order_seq_cst))
        {
            slot.readers.fetch_sub(1, std::memory_order_release);
            continue;
        }
        const int n = slot.count;
        const std::uint64_t seq = slot.sequence;
        const bool fits = n <= capacity;
        if (fits && n > 0)
            std::memcpy(out, slot.data.get(), static_cast<std::size_t>(n) * sizeof(float));
        slot.readers.fetch_sub(1, std::memory_order_release);
        if (sequence != nullptr)
            *sequence = seq;
        return fits ? n : -2;
    }
    return -2;
}

int ViewHub::readPulled(State& s, float* out, int capacity, std::uint64_t* sequence) const
{
    std::lock_guard<std::mutex> lock(s.pullMutex);
    std::vector<float> scratch(static_cast<std::size_t>(s.spec.info.maxElements));
    ViewWriter writer(scratch.data(), s.spec.info.maxElements);
    s.spec.produce(s.spec.context, s.spec.args, writer);
    const int n = writer.size();
    const std::uint64_t hash = fnv1a(scratch.data(), n);
    if (s.pullSequence == 0 || hash != s.pullHash)
    {
        s.pullHash = hash;
        ++s.pullSequence;
    }
    if (sequence != nullptr)
        *sequence = s.pullSequence;
    if (n > capacity)
        return -2;
    if (n > 0)
        std::memcpy(out, scratch.data(), static_cast<std::size_t>(n) * sizeof(float));
    return n;
}

int ViewHub::read(int view, float* out, int capacity, std::uint64_t* sequence) const
{
    if (sequence != nullptr)
        *sequence = 0;
    if (view < 0 || view >= registered.load(std::memory_order_acquire))
        return -1;
    State* s = stateOf(view);
    if (s == nullptr)
        return 0; // live and never subscribed
    return s->spec.info.live ? readLive(*s, out, capacity, sequence) : readPulled(*s, out, capacity, sequence);
}

std::uint64_t ViewHub::dropped(int view) const
{
    const State* s = stateOf(view);
    return s != nullptr ? s->skipped.load(std::memory_order_relaxed) : 0;
}

} // namespace crucible
