// SPDX-License-Identifier: MIT
// fx.cabinet: guitar / bass speaker-cabinet simulation with procedurally generated impulse responses.
//
// No recorded response is used. The response of a cabinet is generated from the parametric model of
// include/crucible/fx/CabinetModel.h (box resonance, cone breakup, roll-off, back-wall comb, microphone position and
// distance) as a minimum-phase FIR of at most 2048 taps at 48 kHz (scaled with the rate), and run through the partitioned
// convolver of fx/PartitionedConvolver.h in its zero-latency form: the module has no latency (reported as 0).
//
// Threading follows fx.convolution. A builder thread owned by each instance generates the response and prepares the
// convolvers of a "kernel set" when the cabinet, the microphone position or the distance change (the audio thread posts the
// wanted settings with a try-lock and waits for nothing; changes wait until the controls have rested 50 ms). Finished sets
// come back through a one-slot mailbox (atomic exchange); the audio thread runs a new set on the live input until its
// convolver holds a whole kernel's worth of signal (output discarded), then crossfades from the set in use to it, and hands
// retired sets back to the builder through a lock-free ring, which frees them. The audio thread allocates nothing and takes
// no lock. At High quality (offline rendering) and while the host renders offline (Engine::setRenderOffline) the audio
// thread waits for the builder after posting, so a render does not depend on thread timing. In real time it never waits,
// not even for the first set: building one takes up to a few hundred milliseconds, which would be an audible dropout.
// Where there are no background threads (LinearPhaseWorker::threadless()) the builder runs inside
// LinearPhaseWorker::pump(), called by Engine::service() on the control side.
//
// Until the first set is ready the module passes its input through; a set fades in over 40 ms once primed.
//
// After the convolver: a 12 dB per octave low cut and high cut (Butterworth, coefficients ramped per chunk, the corner moves
// by at most a factor 1.12 per chunk so that a jump becomes a short sweep), then the output level and the mix, both through
// cascaded one-poles. Channels use the same response (the cabinet is mono); left and right are processed independently.
#include "FxModuleSupport.h"

#include <crucible/fx/AnalogMatchedFilters.h>
#include <crucible/fx/CabinetModel.h>
#include <crucible/fx/LinearPhase.h>
#include <crucible/fx/PartitionedConvolver.h>
#include <crucible/module/CabinetProbe.h>
#include <crucible/module/LatencyReporter.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>

namespace crucible::modules::fx
{
namespace
{
constexpr int kChunk = 128;
constexpr int kRetireSlots = 16;
constexpr double kFadeSeconds = 0.04;
constexpr double kSettleSeconds = 0.05;
constexpr float kMaxStep = 1.12f;

struct Request
{
    crucible::fx::CabinetSettings settings;
    double sampleRate = 48000.0;
    std::uint64_t serial = 0;

    bool sameKernel(const Request& o) const noexcept
    {
        return settings.cabinet == o.settings.cabinet && settings.micPosition == o.settings.micPosition &&
               settings.micDistanceCm == o.settings.micDistanceCm && sampleRate == o.sampleRate;
    }
};

std::atomic<int> gLiveSets{0};

// A built kernel, tagged with the (rate, serial) of the request it answers so that a set built for an earlier rate or an
// earlier prepare() is recognised as stale and dropped.
struct KernelSet
{
    KernelSet() { gLiveSets.fetch_add(1, std::memory_order_relaxed); }
    ~KernelSet() { gLiveSets.fetch_sub(1, std::memory_order_relaxed); }
    KernelSet(const KernelSet&) = delete;
    KernelSet& operator=(const KernelSet&) = delete;
    int kernelLength = 0;
    double rate = 0.0;
    std::uint64_t serial = 0;
    KernelSet* nextPending = nullptr; // link of the audio thread's list of sets that found the retire ring full
    std::array<NonUniformConvolver, 2> conv;
};

// Audio thread posts with try-lock (never waits); the builder takes the newest.
class RequestSlot
{
public:
    bool tryPost(const Request& r)
    {
        if (busy.test_and_set(std::memory_order_acquire))
            return false;
        data = r;
        version.fetch_add(1, std::memory_order_release);
        busy.clear(std::memory_order_release);
        return true;
    }
    bool fetch(Request& out, std::uint64_t& seen)
    {
        while (busy.test_and_set(std::memory_order_acquire))
            std::this_thread::yield();
        const bool fresh = version.load(std::memory_order_acquire) != seen;
        if (fresh)
        {
            out = data;
            seen = version.load(std::memory_order_acquire);
        }
        busy.clear(std::memory_order_release);
        return fresh;
    }

private:
    std::atomic_flag busy; // clear on construction (C++20)
    Request data;
    std::atomic<std::uint64_t> version{0};
};

class Instance final : public EffectModule,
                       public LatencyReporter,
                       public CabinetProbe,
                       public crucible::fx::LinearPhaseWorker::Client
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_cab_mix")), cabinet(localIndex(d, "fx_cab_cabinet")),
          micPosition(localIndex(d, "fx_cab_mic_position")), micDistance(localIndex(d, "fx_cab_mic_distance")),
          lowCut(localIndex(d, "fx_cab_lowcut")), highCut(localIndex(d, "fx_cab_highcut")), outDb(localIndex(d, "fx_cab_out"))
    {
    }

    ~Instance() override
    {
        stopBuilder();
        delete active;
        delete outgoing;
        delete priming;
        delete ready.exchange(nullptr);
        for (auto& s : retired)
            delete s.exchange(nullptr);
        while (KernelSet* s = pending)
        {
            pending = s->nextPending;
            delete s;
        }
    }

    void prepare(const ModuleContext& c) override
    {
        if (c.sampleRate != sr)
        {
            retireAll();
            if (KernelSet* late = ready.exchange(nullptr))
                retire(late);
            posted = false;
            seenValid = false;
            // Anything built for the earlier rate, finished or still in flight, is stale from here on.
            minSerial = postedRequest.serial + 1;
        }
        sr = c.sampleRate;
        highQuality = c.quality == Quality::High;
        renderOffline = c.renderOffline;
        fadeLength = std::max(16, static_cast<int>(std::lround(kFadeSeconds * sr)));
        fadeTable.assign(static_cast<size_t>(fadeLength) + 1, 0.0f);
        for (int i = 0; i <= fadeLength; ++i)
            fadeTable[static_cast<size_t>(i)] =
                static_cast<float>(0.5 - 0.5 * std::cos(3.14159265358979323846 * i / fadeLength));
        smooth = 1.0 - std::exp(-1.0 / (0.004 * sr));
        settleSamples = std::max(1, static_cast<int>(std::lround(kSettleSeconds * sr)));
        for (auto* v : {&inL, &inR, &wetL, &wetR, &oldL, &oldR, &primeL, &primeR})
            v->assign(kChunk, 0.0f);
        reset();
        startBuilder();
    }

    void reset() override
    {
        finishFade();
        if (priming != nullptr)
            retire(std::exchange(priming, nullptr));
        // The set in use starts over as a new instance's first set does: fed the input until it holds a whole kernel's
        // worth, then faded in. A reset instance then renders exactly what a new one renders.
        if (active != nullptr)
        {
            resetSet(*active);
            priming = std::exchange(active, nullptr);
            primed = 0;
        }
        lowPass.reset();
        highPass.reset();
        gridPos = 0;
        designed = false;
        mixStage1 = mixStage2 = 0.0;
        gainStage1 = gainStage2 = 0.0;
        stagesValid = false;
        dirty = false;
    }

    void deactivate() override
    {
        if (dirty)
        {
            reset();
            dirty = false;
        }
    }

    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        if (n <= 0)
            return;
        dirty = true;
        Request candidate;
        candidate.settings.cabinet = std::clamp(in.choice(cabinet), 0, crucible::fx::kCabinetCount - 1);
        candidate.settings.micPosition = std::clamp(in.value(micPosition) * 0.01f, 0.0f, 1.0f);
        candidate.settings.micDistanceCm = std::clamp(in.value(micDistance), 0.0f, 60.0f);
        candidate.sampleRate = sr;
        manageKernel(candidate, n);

        const float mixTarget = in.value(mix);
        const double gainTarget = in.value(outDb);
        const float hpHz = in.value(lowCut), lpHz = in.value(highCut);
        if (!stagesValid)
        {
            mixStage1 = mixStage2 = mixTarget;
            gainStage1 = gainStage2 = gainTarget;
            stagesValid = true;
        }
        // Work is cut at multiples of kChunk samples of the instance's own time, whatever the host block size.
        for (int pos = 0; pos < n;)
        {
            const int m = std::min(kChunk - gridPos, n - pos);
            if (gridPos == 0)
                designFilters(hpHz, lpHz);
            processChunk(l + pos, r + pos, m, mixTarget, gainTarget);
            gridPos = gridPos + m == kChunk ? 0 : gridPos + m;
            pos += m;
        }
    }

    // ---- CabinetProbe
    void holdBuilder(bool hold) override { held.store(hold, std::memory_order_release); }
    bool setPending() const override { return ready.load(std::memory_order_acquire) != nullptr; }
    double playingRate() const override { return playingRateSeen.load(std::memory_order_relaxed); }
    void retireSpares(int count) override
    {
        for (int i = 0; i < count; ++i)
            retire(new KernelSet);
    }

    int latencySamples() const override { return 0; }
    double tailSeconds() const override { return tail.load(std::memory_order_relaxed); }
    bool takeLatencyChanged() override { return false; }

private:
    // ---- builder thread
    void startBuilder()
    {
        if (builder.joinable())
            return;
        if (crucible::fx::LinearPhaseWorker::threadless())
        {
            if (!inlineBuilder)
            {
                crucible::fx::LinearPhaseWorker::instance().add(this);
                inlineBuilder = true;
            }
            return;
        }
        stopping.store(false);
        builder = std::thread([this] { builderLoop(); });
    }
    bool service(crucible::fx::LinearPhaseWorker::Tools&) override
    {
        freeRetired();
        Request request;
        if (!requests.fetch(request, built))
            return false;
        publish(build(request), request.serial);
        return true;
    }
    void stopBuilder()
    {
        if (inlineBuilder)
        {
            crucible::fx::LinearPhaseWorker::instance().remove(this);
            inlineBuilder = false;
        }
        if (!builder.joinable())
            return;
        stopping.store(true);
        {
            std::lock_guard<std::mutex> lock(wakeMutex);
        }
        wake.notify_all();
        builder.join();
    }
    void builderLoop()
    {
        std::unique_lock<std::mutex> lock(wakeMutex);
        while (!stopping.load())
        {
            lock.unlock();
            freeRetired();
            Request request;
            if (requests.fetch(request, built))
            {
                publish(build(request), request.serial);
            }
            lock.lock();
            wake.wait_for(lock, std::chrono::milliseconds(4), [this] { return stopping.load(); });
        }
    }
    void publish(KernelSet* set, std::uint64_t serial)
    {
        while (held.load(std::memory_order_acquire) && !stopping.load() && !inlineBuilder)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        delete ready.exchange(set, std::memory_order_acq_rel);
        completed.store(serial, std::memory_order_release);
    }
    void freeRetired()
    {
        if (held.load(std::memory_order_acquire))
            return;
        for (auto& s : retired)
            if (s.load(std::memory_order_relaxed) != nullptr)
                delete s.exchange(nullptr, std::memory_order_acq_rel);
    }

    KernelSet* build(const Request& rq)
    {
        auto* set = new KernelSet;
        set->serial = rq.serial;
        set->rate = rq.sampleRate;
        std::vector<float> ir;
        crucible::fx::generateCabinetIr(rq.settings, rq.sampleRate, ir);
        const ConvolverPlan plan = ConvolverPlan::zeroLatency();
        set->kernelLength = static_cast<int>(ir.size());
        for (auto& c : set->conv)
            c.prepare(plan, ir.data(), set->kernelLength);
        return set;
    }

    // ---- audio thread: kernel management
    void manageKernel(const Request& candidate, int n)
    {
        if (!seenValid || !candidate.sameKernel(seen))
        {
            seen = candidate;
            seenValid = true;
            resting = 0;
        }
        else
            resting += n;
        // The first set and a new rate are built at once; changes of the cabinet or microphone wait until they have rested.
        const bool first = !posted || candidate.sampleRate != postedRequest.sampleRate;
        const bool changed = posted && !candidate.sameKernel(postedRequest);
        if (first || (changed && resting >= settleSamples))
        {
            Request post = candidate;
            post.serial = postedRequest.serial + 1;
            // Offline the post must not be skipped when the builder holds the slot, or the block at which a change takes
            // effect would depend on thread timing. In real time nothing waits: the input passes until the set is ready.
            const bool offline =
                highQuality || (renderOffline != nullptr && renderOffline->load(std::memory_order_relaxed));
            const bool wait = offline && !crucible::fx::LinearPhaseWorker::threadless();
            bool accepted = requests.tryPost(post);
            while (!accepted && wait)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(50));
                accepted = requests.tryPost(post);
            }
            if (accepted)
            {
                postedRequest = post;
                posted = true;
                if (wait)
                    waitForBuilder();
            }
        }
        flushStash();
        if (fadePos < 0 && priming == nullptr)
            if (KernelSet* incoming = ready.exchange(nullptr, std::memory_order_acq_rel))
            {
                // A set answering a request from before the last rate change must not play: drop it.
                if (incoming->rate != sr || incoming->serial < minSerial)
                    retire(incoming);
                else
                {
                    priming = incoming;
                    primed = 0;
                }
            }
        double seenRate = 0.0;
        for (const KernelSet* k : {priming, active, outgoing})
            if (k != nullptr)
                seenRate = k->rate;
        playingRateSeen.store(seenRate, std::memory_order_relaxed);
    }

    void waitForBuilder()
    {
        while (completed.load(std::memory_order_acquire) < postedRequest.serial)
            std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    void startFade()
    {
        outgoing = active;
        active = std::exchange(priming, nullptr);
        fadePos = 0;
        tail.store(active->kernelLength / sr, std::memory_order_relaxed);
    }

    void finishFade()
    {
        if (outgoing != nullptr)
        {
            retire(outgoing);
            outgoing = nullptr;
        }
        fadePos = -1;
    }

    bool tryRetire(KernelSet* s)
    {
        for (auto& slot : retired)
        {
            KernelSet* expected = nullptr;
            if (slot.compare_exchange_strong(expected, s, std::memory_order_acq_rel))
                return true;
        }
        return false;
    }
    // The ring has 16 slots and the builder empties it every few milliseconds; a set waits in the pending list (linked
    // through the set itself: no allocation, no limit) when it is full, and flushStash() moves it into the ring later.
    void retire(KernelSet* s)
    {
        if (tryRetire(s))
            return;
        s->nextPending = pending;
        pending = s;
    }
    void flushStash()
    {
        KernelSet* list = std::exchange(pending, nullptr);
        while (list != nullptr)
        {
            KernelSet* s = list;
            list = s->nextPending;
            s->nextPending = nullptr;
            retire(s);
        }
    }
    void retireAll()
    {
        if (outgoing != nullptr)
            retire(std::exchange(outgoing, nullptr));
        if (priming != nullptr)
            retire(std::exchange(priming, nullptr));
        if (active != nullptr)
            retire(std::exchange(active, nullptr));
        fadePos = -1;
    }

    static void resetSet(KernelSet& s)
    {
        for (auto& c : s.conv)
            c.reset();
    }

    static void runSet(KernelSet& s, const float* l, const float* r, float* outL, float* outR, int m)
    {
        s.conv[0].process(l, outL, m);
        s.conv[1].process(r, outR, m);
    }

    // ---- low cut and high cut
    void designFilters(float hpHz, float lpHz)
    {
        if (!designed)
        {
            hpNow = hpHz;
            lpNow = lpHz;
        }
        else
        {
            hpNow *= std::clamp(hpHz / hpNow, 1.0f / kMaxStep, kMaxStep);
            lpNow *= std::clamp(lpHz / lpNow, 1.0f / kMaxStep, kMaxStep);
        }
        if (!designed || hpNow != hpDesigned)
        {
            design(highPass, crucible::fx::SectionKind::HighPass, std::clamp(static_cast<double>(hpNow), 10.0, 0.45 * sr),
                   designed ? kChunk : 0);
            hpDesigned = hpNow;
        }
        if (!designed || lpNow != lpDesigned)
        {
            design(lowPass, crucible::fx::SectionKind::LowPass, std::clamp(static_cast<double>(lpNow), 100.0, 0.45 * sr),
                   designed ? kChunk : 0);
            lpDesigned = lpNow;
        }
        designed = true;
    }
    void design(crucible::fx::BiquadChain& chain, crucible::fx::SectionKind kind, double hz, int ramp)
    {
        crucible::fx::PolePair poles[2];
        const int count = crucible::fx::butterworthPoles(2, poles);
        crucible::fx::Biquad sections[2];
        for (int i = 0; i < count; ++i)
            sections[i] = crucible::fx::bilinearSection(
                crucible::fx::analogPrototype(kind, 0.0, poles[i].q, poles[i].real ? 1 : 2), hz, sr);
        chain.set(sections, count, ramp);
    }

    void processChunk(float* l, float* r, int m, float mixTarget, double gainTarget)
    {
        std::copy(l, l + m, inL.begin());
        std::copy(r, r + m, inR.begin());
        if (priming != nullptr)
        {
            runSet(*priming, inL.data(), inR.data(), primeL.data(), primeR.data(), m);
            primed += m;
            if (fadePos < 0 && primed >= priming->kernelLength)
                startFade();
        }
        const bool fading = fadePos >= 0;
        if (active != nullptr)
            runSet(*active, inL.data(), inR.data(), wetL.data(), wetR.data(), m);
        if (fading && outgoing != nullptr)
            runSet(*outgoing, inL.data(), inR.data(), oldL.data(), oldR.data(), m);

        // Select the wet signal per sample: the cabinet, crossfaded from the previous cabinet or from the plain input.
        const bool haveActive = active != nullptr;
        for (int i = 0; i < m; ++i)
        {
            const size_t k = static_cast<size_t>(i);
            float wl = haveActive ? wetL[k] : inL[k], wr = haveActive ? wetR[k] : inR[k];
            if (fadePos >= 0)
            {
                const float w = fadeTable[static_cast<size_t>(fadePos)];
                const float fromL = outgoing != nullptr ? oldL[k] : inL[k], fromR = outgoing != nullptr ? oldR[k] : inR[k];
                wl = wl * w + fromL * (1.0f - w);
                wr = wr * w + fromR * (1.0f - w);
                if (++fadePos > fadeLength)
                    finishFade();
            }
            wetL[k] = wl;
            wetR[k] = wr;
        }
        for (int i = 0; i < m; ++i)
        {
            work[0][static_cast<size_t>(i)] = wetL[static_cast<size_t>(i)];
            work[1][static_cast<size_t>(i)] = wetR[static_cast<size_t>(i)];
        }
        highPass.process(work[0].data(), work[1].data(), m);
        lowPass.process(work[0].data(), work[1].data(), m);

        for (int i = 0; i < m; ++i)
        {
            const size_t k = static_cast<size_t>(i);
            mixStage1 += smooth * (static_cast<double>(mixTarget) - mixStage1);
            mixStage2 += smooth * (mixStage1 - mixStage2);
            gainStage1 += smooth * (gainTarget - gainStage1);
            gainStage2 += smooth * (gainStage1 - gainStage2);
            const float g = static_cast<float>(std::exp(gainStage2 * 0.11512925464970229));
            const float mx = static_cast<float>(mixStage2);
            l[i] = inL[k] * (1.0f - mx) + static_cast<float>(work[0][k]) * g * mx;
            r[i] = inR[k] * (1.0f - mx) + static_cast<float>(work[1][k]) * g * mx;
        }
    }

    // ---- state
    const int mix, cabinet, micPosition, micDistance, lowCut, highCut, outDb;
    double sr = 48000.0;
    bool highQuality = false;
    const std::atomic<bool>* renderOffline = nullptr; // ModuleContext::renderOffline
    bool dirty = false;

    KernelSet* active = nullptr;
    KernelSet* outgoing = nullptr;
    KernelSet* priming = nullptr; // a finished set being fed the live input before it fades in
    int primed = 0;
    int fadePos = -1;
    int fadeLength = 1920;
    std::vector<float> fadeTable;
    std::vector<float> inL, inR, wetL, wetR, oldL, oldR, primeL, primeR;
    std::array<std::array<double, kChunk>, 2> work{};
    crucible::fx::BiquadChain lowPass, highPass;
    int gridPos = 0;
    bool designed = false, stagesValid = false;
    float hpNow = 20.0f, lpNow = 20000.0f, hpDesigned = -1.0f, lpDesigned = -1.0f;
    double mixStage1 = 0.0, mixStage2 = 0.0, gainStage1 = 0.0, gainStage2 = 0.0, smooth = 0.005;
    Request seen, postedRequest;
    bool seenValid = false, posted = false;
    int resting = 0, settleSamples = 2400;
    KernelSet* pending = nullptr; // retired sets waiting for a free slot of the ring
    std::uint64_t minSerial = 0;  // sets answering an older request are stale
    std::atomic<bool> held{false};
    std::atomic<double> playingRateSeen{0.0};

    // Between the threads.
    RequestSlot requests;
    std::atomic<KernelSet*> ready{nullptr};
    std::array<std::atomic<KernelSet*>, kRetireSlots> retired{};
    std::atomic<std::uint64_t> completed{0};
    std::uint64_t built = 0;
    bool inlineBuilder = false;
    std::atomic<double> tail{0.0};

    // Builder thread.
    std::thread builder;
    std::mutex wakeMutex;
    std::condition_variable wake;
    std::atomic<bool> stopping{false};
};
} // namespace

ModuleDescriptor cabinetEffect()
{
    std::vector<std::string> names;
    for (int i = 0; i < crucible::fx::kCabinetCount; ++i)
        names.push_back(crucible::fx::cabinetSpec(i).name);
    auto d = effectDescriptor(
        "fx.cabinet", "Cabinet", "Distortion", CpuClass::Medium,
        "Guitar and bass speaker-cabinet simulation with generated impulse responses (no recordings): eight cabinets "
        "(1x12, 2x12, 1x10 and 2x10 open back or combo, closed 4x12, bass 1x15, 4x10 and 8x10) each modelled from its "
        "box resonance, cone breakup peaks and dips, high roll-off and, for open backs, the back-wall comb. Microphone "
        "position (centre to edge: high-frequency tilt and a moving notch) and distance (proximity bass, room "
        "reflection) reshape the response; low cut, high cut, output level and mix follow. The response is a minimum-"
        "phase FIR of at most 2048 taps at 48 kHz run through the zero-latency partitioned convolver, rebuilt off the "
        "audio thread and crossfaded in. Latency is 0 samples (reported).");
    auto& p = d.params;
    addOn(p, "cab", "Cabinet", "fx_cab", 1.0f);
    p.push_back(params::choice("fx_cab_cabinet", "Cabinet Model", "fx_cab", names, 2));
    p.push_back(params::floating("fx_cab_mic_position", "Cabinet Mic Position", "fx_cab", 0.0f, 100.0f, 0.0f, "%"));
    p.push_back(params::floating("fx_cab_mic_distance", "Cabinet Mic Distance", "fx_cab", 0.0f, 60.0f, 8.0f, "cm"));
    addHz(p, "cab_lowcut", "Cabinet Low Cut", "fx_cab_2", 20.0f, 400.0f, 20.0f, 0.4f);
    addHz(p, "cab_highcut", "Cabinet High Cut", "fx_cab_2", 2000.0f, 20000.0f, 20000.0f, 0.4f);
    addF(p, "cab_out", "Cabinet Output", "fx_cab_2", -24.0f, 24.0f, 0.0f, "dB");
    p.push_back(params::integer("fx_pos_cab", "Cabinet Slot", "fx_cab_2", 1, kNumModuleFx, kNumModuleFx));
    // Flagged appended: fx.crusher's note-tracking parameters already sit after every module's regular ones in the frozen
    // parameter order (tests/golden/param_order.txt), and these follow them without moving anything.
    for (auto& q : p)
        q.appended = 3;
    d.onParam = "fx_cab_on";
    d.positionParam = "fx_pos_cab";
    d.capabilities = {"latency"};
    d.pages.push_back({"fx_cab", "Cabinet", "fx", "Cabinet", "On, mix, cabinet model and microphone position and distance."});
    d.pages.push_back({"fx_cab_2", "Cabinet 2", "fx", "Cabinet Output", "Low cut, high cut, output level and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.cabinet")); };
    return d;
}

} // namespace crucible::modules::fx

namespace crucible
{
int CabinetProbe::liveSets()
{
    return modules::fx::gLiveSets.load(std::memory_order_relaxed);
}
} // namespace crucible
