// SPDX-License-Identifier: MIT
// fx.convolution: convolution with true impulse-response import (docs/plans/ENGINE-FX-PLAN.md, section 3.5, FX7).
//
// The response (mono, stereo or true-stereo with the kernels LL, LR, RL, RR) is handed over on the control thread
// (ConvolutionIr). A builder thread owned by each instance applies the edits (fx/IrEdit.h), resamples to the engine rate,
// and prepares the convolvers (fx/PartitionedConvolver.h) of a "kernel set". The audio thread posts the wanted settings
// with a try-lock (never waits), takes finished sets from a one-slot mailbox with an atomic exchange, crossfades from the
// set in use to the new one (after feeding it the recent input, so that its output is the one a kernel that had always been
// there would give), and hands retired sets back to the builder through a lock-free ring, which frees them. The
// audio thread therefore allocates nothing and takes no lock. At High quality (offline rendering), and while the host
// renders offline (Engine::setRenderOffline), the audio thread waits for the builder after posting, so a render does
// not depend on thread timing. Where there are no background threads (LinearPhaseWorker::threadless(), WebAssembly without pthreads) the builder runs only inside LinearPhaseWorker::pump(),
// which Engine::service() calls on the control side; the audio thread never pumps and never waits there, not even offline: a
// posted request is built by the next service() call and its kernel is taken at the start of the block after that.
//
// Two modes: Zero Latency (64 direct taps, partitions 32 / 128 / 1024 / 8192) and Low CPU (128 samples of latency,
// partitions 64 / 512 / 4096). The latency is reported (LatencyReporter) and the dry path is delayed to match.

#include "FxModuleSupport.h"

#include <crucible/Fft.h>
#include <crucible/fx/IrEdit.h>
#include <crucible/fx/LinearPhase.h>
#include <crucible/fx/PartitionedConvolver.h>
#include <crucible/module/ConvolutionIr.h>
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
constexpr int kChunk = 256;    // samples handled per pass
constexpr int kDryRing = 1024; // dry delay line (a power of two, above kChunk plus the longest latency)
constexpr int kResponsePoints = 256;
constexpr int kResponseOrder = 15;
constexpr int kRetireSlots = 16;
constexpr double kFadeSeconds = 0.05;    // kernel switch
constexpr double kSettleSeconds = 0.05;  // edit parameters must rest this long before a new kernel is built
constexpr double kHistorySeconds = 16.0; // input history a new kernel is primed with
constexpr int kPrimeRate = 4;            // history samples fed to a new kernel per sample of real time (real-time rendering)

struct Request
{
    IrEditSettings edit;
    int mode = 0;
    double sampleRate = 48000.0;
    std::uint64_t irVersion = 0;
    std::uint64_t serial = 0;

    bool sameKernel(const Request& o) const noexcept
    {
        return edit == o.edit && mode == o.mode && sampleRate == o.sampleRate && irVersion == o.irVersion;
    }
};

struct KernelSet
{
    bool empty = true;
    IrLayout layout = IrLayout::Mono;
    int latency = 0;
    int kernelLength = 0;
    std::uint64_t serial = 0;
    // conv[in * 2 + out]: LL, LR, RL, RR. Mono and stereo sets use only conv[0] (left) and conv[3] (right).
    std::array<NonUniformConvolver, 4> conv;
    std::array<float, kResponsePoints> responseDb{};
    bool trueStereo() const { return layout == IrLayout::TrueStereo; }
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

struct Source
{
    ImpulseResponse ir;
    std::string name;
    std::uint64_t version = 0;
};

void computeResponse(const ImpulseResponse& ir, double rate, std::array<float, kResponsePoints>& out)
{
    const int size = 1 << kResponseOrder;
    Fft fft(kResponseOrder);
    std::vector<float> buffer(static_cast<size_t>(2) * size);
    std::vector<double> power(static_cast<size_t>(size / 2 + 1), 0.0);
    for (const auto& channel : ir.channels)
    {
        // Folding the kernel modulo the transform size samples its spectrum exactly at the transform's bins.
        std::fill(buffer.begin(), buffer.end(), 0.0f);
        for (size_t i = 0; i < channel.size(); ++i)
            buffer[i & static_cast<size_t>(size - 1)] += channel[i];
        fft.forward(buffer.data());
        for (int k = 0; k <= size / 2; ++k)
        {
            const double re = buffer[static_cast<size_t>(2 * k)], im = buffer[static_cast<size_t>(2 * k + 1)];
            power[static_cast<size_t>(k)] += re * re + im * im;
        }
    }
    const double perInput = ir.layout == IrLayout::TrueStereo ? 2.0 : (ir.layout == IrLayout::Stereo ? 2.0 : 1.0);
    const double top = std::min(20000.0, 0.499 * rate);
    for (int i = 0; i < kResponsePoints; ++i)
    {
        const double hz = 20.0 * std::pow(top / 20.0, static_cast<double>(i) / (kResponsePoints - 1));
        const double pos = std::min(hz / rate * size, static_cast<double>(size / 2 - 1));
        const size_t k = static_cast<size_t>(pos);
        const double f = pos - static_cast<double>(k);
        const double p = (power[k] * (1.0 - f) + power[k + 1] * f) / perInput;
        out[static_cast<size_t>(i)] = static_cast<float>(10.0 * std::log10(std::max(p, 1.0e-20)));
    }
}

class Instance final : public EffectModule,
                       public LatencyReporter,
                       public ResponseView,
                       public ConvolutionIr,
                       public crucible::fx::LinearPhaseWorker::Client
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_cx_mix")), mode(localIndex(d, "fx_cx_mode")), trim(localIndex(d, "fx_cx_trim")),
          length(localIndex(d, "fx_cx_length")), reverse(localIndex(d, "fx_cx_reverse")),
          stretch(localIndex(d, "fx_cx_stretch")), fadeIn(localIndex(d, "fx_cx_fadein")),
          fadeOut(localIndex(d, "fx_cx_fadeout")), balance(localIndex(d, "fx_cx_balance")),
          lowCut(localIndex(d, "fx_cx_lowcut")), highCut(localIndex(d, "fx_cx_highcut")),
          predelay(localIndex(d, "fx_cx_predelay")), width(localIndex(d, "fx_cx_width")), outDb(localIndex(d, "fx_cx_out"))
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
        for (auto* s : stash)
            delete s;
    }

    // ---- EffectModule (control thread)
    void prepare(const ModuleContext& c) override
    {
        if (c.sampleRate != sr)
        {
            // The kernels belong to the old rate.
            retireAll();
            if (KernelSet* late = ready.exchange(nullptr))
                retire(late);
            posted = false;
            seenValid = false;
        }
        sr = c.sampleRate;
        highQuality = c.quality == Quality::High;
        renderOffline = c.renderOffline;
        fadeLength = std::max(16, static_cast<int>(std::lround(kFadeSeconds * sr)));
        fadeTable.assign(static_cast<size_t>(fadeLength) + 1, 0.0f);
        for (int i = 0; i <= fadeLength; ++i)
            fadeTable[static_cast<size_t>(i)] =
                static_cast<float>(0.5 - 0.5 * std::cos(3.14159265358979323846 * i / fadeLength));
        gainSmooth = 1.0 - std::exp(-1.0 / (0.004 * sr));
        settleSamples = std::max(1, static_cast<int>(std::lround(kSettleSeconds * sr)));
        for (auto* v : {&inL, &inR, &wetL, &wetR, &oldL, &oldR, &tmp, &primeInL, &primeInR, &primeOutL, &primeOutR})
            v->assign(kChunk, 0.0f);
        historySize = 1;
        while (historySize < static_cast<long>(kHistorySeconds * sr))
            historySize <<= 1;
        for (auto& h : history)
            h.assign(static_cast<size_t>(historySize), 0.0f);
        for (auto& ring : dry)
            ring.assign(kDryRing, 0.0f);
        reset();
        startBuilder();
    }

    void reset() override
    {
        if (active != nullptr)
            resetSet(*active);
        finishFade();
        if (priming != nullptr)
            retire(std::exchange(priming, nullptr));
        for (auto& ring : dry)
            std::fill(ring.begin(), ring.end(), 0.0f);
        for (auto& h : history)
            std::fill(h.begin(), h.end(), 0.0f);
        position = 0;
        ringPos = 0;
        mixNow = -1.0f;
        gainNow = -1.0;
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
        const int modeValue = std::clamp(in.choice(mode), 0, 1);
        const int wantedLatency = modeValue == 0 ? ConvolverPlan::kZeroLatencySamples : ConvolverPlan::kLowCpuLatencySamples;
        if (wantedLatency != reportedLatency.load(std::memory_order_relaxed))
        {
            reportedLatency.store(wantedLatency, std::memory_order_relaxed);
            latencyChanged.store(true, std::memory_order_relaxed);
        }
        Request candidate;
        candidate.mode = modeValue;
        candidate.sampleRate = sr;
        candidate.irVersion = irVersion.load(std::memory_order_acquire);
        candidate.edit.trimMs = in.value(trim);
        candidate.edit.lengthPercent = in.value(length);
        candidate.edit.reverse = in.on(reverse);
        candidate.edit.stretch = in.value(stretch);
        candidate.edit.fadeInMs = in.value(fadeIn);
        candidate.edit.fadeOutMs = in.value(fadeOut);
        candidate.edit.balanceDb = in.value(balance);
        candidate.edit.lowCutHz = in.value(lowCut);
        candidate.edit.highCutHz = in.value(highCut);
        candidate.edit.predelayMs = in.value(predelay);
        candidate.edit.width = in.value(width);
        manageKernel(candidate, n);

        const float mixTarget = in.value(mix);
        const double gainDbTarget = in.value(outDb);
        if (mixNow < 0.0f)
        {
            mixNow = mixTarget;
            mixStage1 = mixStage2 = mixTarget;
            gainStage1 = gainStage2 = gainStage3 = gainDbTarget;
            gainNow = std::exp(gainDbTarget * 0.11512925464970229);
        }

        for (int pos = 0; pos < n; pos += kChunk)
        {
            const int m = std::min(kChunk, n - pos);
            processChunk(l + pos, r + pos, m, mixTarget, gainDbTarget);
        }
        advancePriming(static_cast<long>(kPrimeRate) * n);
    }

    // ---- LatencyReporter
    int latencySamples() const override { return reportedLatency.load(std::memory_order_relaxed); }
    double tailSeconds() const override { return tail.load(std::memory_order_relaxed); }
    bool takeLatencyChanged() override { return latencyChanged.exchange(false, std::memory_order_relaxed); }

    // ---- ResponseView (audio thread)
    int responsePoints() const override { return kResponsePoints; }
    void fillResponse(float* out) override
    {
        const float g = 20.0f * std::log10(static_cast<float>(std::max(gainNow, 1.0e-6)));
        for (int i = 0; i < kResponsePoints; ++i)
            out[i] = (active != nullptr && !active->empty ? active->responseDb[static_cast<size_t>(i)] : 0.0f) +
                     (gainNow > 0.0 ? g : 0.0f);
    }

    // ---- ConvolutionIr (control thread)
    bool setImpulseResponse(const ImpulseResponse& ir, std::string name) override
    {
        if (ir.channels.empty() || ir.frames() <= 0)
            return false;
        auto s = std::make_shared<Source>();
        s->ir = ir;
        s->name = std::move(name);
        {
            std::lock_guard<std::mutex> lock(sourceMutex);
            s->version = ++versionCounter;
            source = std::move(s);
        }
        hasIr.store(true, std::memory_order_release);
        irVersion.store(versionCounter, std::memory_order_release);
        return true;
    }
    void clearImpulseResponse() override
    {
        {
            std::lock_guard<std::mutex> lock(sourceMutex);
            source.reset();
            ++versionCounter;
        }
        hasIr.store(false, std::memory_order_release);
        irVersion.store(versionCounter, std::memory_order_release);
    }
    bool hasImpulseResponse() const override { return hasIr.load(std::memory_order_acquire); }
    std::string impulseResponseName() const override
    {
        std::lock_guard<std::mutex> lock(sourceMutex);
        return source ? source->name : std::string();
    }
    bool impulseResponse(ImpulseResponse& out, std::string& name) const override
    {
        std::lock_guard<std::mutex> lock(sourceMutex);
        if (!source)
            return false;
        out = source->ir;
        name = source->name;
        return true;
    }

private:
    // ---- builder thread
    void startBuilder()
    {
        if (builder.joinable())
            return;
        if (crucible::fx::LinearPhaseWorker::threadless())
        {
            // No background threads: the shared worker's pump() (control thread) runs the builder.
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
        KernelSet* set = build(request);
        delete ready.exchange(set, std::memory_order_acq_rel);
        completed.store(request.serial, std::memory_order_release);
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
                KernelSet* set = build(request);
                delete ready.exchange(set, std::memory_order_acq_rel);
                completed.store(request.serial, std::memory_order_release);
            }
            lock.lock();
            wake.wait_for(lock, std::chrono::milliseconds(4), [this] { return stopping.load(); });
        }
    }
    void freeRetired()
    {
        for (auto& s : retired)
            if (s.load(std::memory_order_relaxed) != nullptr)
                delete s.exchange(nullptr, std::memory_order_acq_rel);
    }

    KernelSet* build(const Request& rq)
    {
        auto* set = new KernelSet;
        set->serial = rq.serial;
        std::shared_ptr<Source> src;
        {
            std::lock_guard<std::mutex> lock(sourceMutex);
            src = source;
        }
        if (!src || src->ir.channels.empty())
            return set;
        if (baseVersion != src->version || baseRate != rq.sampleRate)
        {
            base.sampleRate = rq.sampleRate;
            base.layout = src->ir.layout;
            base.channels.clear();
            for (const auto& ch : src->ir.channels)
                base.channels.push_back(resampleKaiser(ch, src->ir.sampleRate, rq.sampleRate));
            baseVersion = src->version;
            baseRate = rq.sampleRate;
        }
        const ImpulseResponse edited = applyIrEdit(base, rq.edit);
        const ConvolverPlan plan = rq.mode == 0 ? ConvolverPlan::zeroLatency() : ConvolverPlan::lowCpu();
        set->layout = edited.layout;
        set->latency = plan.latency;
        set->kernelLength = edited.frames();
        const auto kernel = [&](size_t i) -> const std::vector<float>&
        { return edited.channels[std::min(i, edited.channels.size() - 1)]; };
        if (edited.layout == IrLayout::TrueStereo && edited.channels.size() >= 4)
        {
            for (size_t i = 0; i < 4; ++i)
                set->conv[i].prepare(plan, kernel(i).data(), static_cast<int>(kernel(i).size()));
        }
        else
        {
            set->conv[0].prepare(plan, kernel(0).data(), static_cast<int>(kernel(0).size()));
            const size_t right = edited.layout == IrLayout::Stereo ? 1 : 0;
            set->conv[3].prepare(plan, kernel(right).data(), static_cast<int>(kernel(right).size()));
        }
        computeResponse(edited, rq.sampleRate, set->responseDb);
        set->empty = false;
        return set;
    }

    // ---- audio thread: kernel management
    void manageKernel(const Request& candidate, int n)
    {
        const bool have = hasIr.load(std::memory_order_acquire);
        if (have || posted)
        {
            if (!seenValid || !candidate.sameKernel(seen))
            {
                seen = candidate;
                seenValid = true;
                resting = 0;
            }
            else
                resting += n;
            // A new response, mode or rate is built at once; edits wait until the parameters have rested.
            const bool kernelChanged = !posted || candidate.irVersion != postedRequest.irVersion ||
                                       candidate.mode != postedRequest.mode ||
                                       candidate.sampleRate != postedRequest.sampleRate;
            const bool editChanged = posted && candidate.edit != postedRequest.edit;
            if (kernelChanged || (editChanged && resting >= settleSamples))
            {
                Request post = candidate;
                post.serial = postedRequest.serial + 1;
                // Offline the post must not be skipped when the builder happens to hold the slot (it polls it every few
                // milliseconds), or the block at which an edit takes effect would depend on thread timing.
                bool accepted = requests.tryPost(post);
                while (!accepted && offlineWait())
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(50));
                    accepted = requests.tryPost(post);
                }
                if (accepted)
                {
                    postedRequest = post;
                    posted = true;
                    if (offlineWait())
                        waitForBuilder();
                }
            }
        }
        flushStash();
        if (fadePos < 0)
        {
            if (KernelSet* incoming = ready.exchange(nullptr, std::memory_order_acq_rel))
            {
                if (priming != nullptr)
                    retire(std::exchange(priming, nullptr));
                if (incoming->empty)
                    startFade(incoming);
                else
                {
                    // Feed the new kernel the last stretch of input that it reaches back to, so that it starts the fade in
                    // the state of a kernel that had always been there. Offline this is done at once, otherwise it is spread
                    // over the following blocks.
                    priming = incoming;
                    const long reach = std::min<long>(
                        {static_cast<long>(incoming->kernelLength) + incoming->latency, historySize, position});
                    primed = position - reach;
                    if (offlineNow())
                        advancePriming(position);
                }
            }
            if (priming != nullptr && primed == position)
                startFade(std::exchange(priming, nullptr));
        }
    }

    // Feeds the set being primed with up to `budget` samples of the stored input; the output is discarded.
    void advancePriming(long budget)
    {
        while (priming != nullptr && primed < position && budget > 0)
        {
            const int m = static_cast<int>(std::min<long>({static_cast<long>(kChunk), position - primed, budget}));
            for (int i = 0; i < m; ++i)
            {
                const size_t at = static_cast<size_t>((primed + i) & (historySize - 1));
                primeInL[static_cast<size_t>(i)] = history[0][at];
                primeInR[static_cast<size_t>(i)] = history[1][at];
            }
            runSet(*priming, primeInL.data(), primeInR.data(), primeOutL.data(), primeOutR.data(), m);
            primed += m;
            budget -= m;
        }
    }

    // High quality, or the host renders offline (ModuleContext::renderOffline).
    bool offlineNow() const
    {
        return highQuality || (renderOffline != nullptr && renderOffline->load(std::memory_order_relaxed));
    }
    // Offline renders wait for the builder thread; without threads there is nothing to wait for on the audio thread.
    bool offlineWait() const { return offlineNow() && !crucible::fx::LinearPhaseWorker::threadless(); }

    void waitForBuilder()
    {
        while (completed.load(std::memory_order_acquire) < postedRequest.serial)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    void startFade(KernelSet* incoming)
    {
        outgoing = active;
        active = incoming;
        fadePos = 0;
        dryFrom = dryTo;
        dryTo = incoming->empty ? 0 : incoming->latency;
        tail.store(incoming->empty ? 0.0 : (incoming->kernelLength + incoming->latency) / sr, std::memory_order_relaxed);
    }

    void finishFade()
    {
        if (outgoing != nullptr)
        {
            retire(outgoing);
            outgoing = nullptr;
        }
        fadePos = -1;
        dryFrom = dryTo;
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
    // The ring has 16 slots and the builder empties it every few milliseconds; a set waits in the stash (four places, never
    // reached in practice) when the ring is full.
    void retire(KernelSet* s)
    {
        if (tryRetire(s))
            return;
        for (auto& slot : stash)
            if (slot == nullptr)
            {
                slot = s;
                return;
            }
    }
    void flushStash()
    {
        for (auto& slot : stash)
            if (slot != nullptr && tryRetire(slot))
                slot = nullptr;
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
        dryFrom = dryTo = 0;
    }

    static void resetSet(KernelSet& s)
    {
        for (auto& c : s.conv)
            c.reset();
    }

    // Runs one set on the block; writes zeros for an empty one.
    void runSet(KernelSet& s, const float* l, const float* r, float* outL, float* outR, int m)
    {
        if (s.empty)
        {
            std::fill(outL, outL + m, 0.0f);
            std::fill(outR, outR + m, 0.0f);
            return;
        }
        s.conv[0].process(l, outL, m);
        s.conv[3].process(r, outR, m);
        if (s.trueStereo())
        {
            // outL = LL * L + RL * R, outR = LR * L + RR * R.
            s.conv[2].process(r, tmp.data(), m);
            for (int i = 0; i < m; ++i)
                outL[i] += tmp[static_cast<size_t>(i)];
            s.conv[1].process(l, tmp.data(), m);
            for (int i = 0; i < m; ++i)
                outR[i] += tmp[static_cast<size_t>(i)];
        }
    }

    void processChunk(float* l, float* r, int m, float mixTarget, double gainDbTarget)
    {
        std::copy(l, l + m, inL.begin());
        std::copy(r, r + m, inR.begin());
        for (int i = 0; i < m; ++i)
        {
            const size_t at = static_cast<size_t>((position + i) & (historySize - 1));
            history[0][at] = inL[static_cast<size_t>(i)];
            history[1][at] = inR[static_cast<size_t>(i)];
        }
        position += m;
        const bool activeEmpty = active == nullptr || active->empty;
        const bool outgoingEmpty = outgoing == nullptr || outgoing->empty;
        const bool fading = fadePos >= 0;
        if (active != nullptr)
            runSet(*active, inL.data(), inR.data(), wetL.data(), wetR.data(), m);
        else
        {
            std::fill(wetL.begin(), wetL.begin() + m, 0.0f);
            std::fill(wetR.begin(), wetR.begin() + m, 0.0f);
        }
        if (fading && outgoing != nullptr)
            runSet(*outgoing, inL.data(), inR.data(), oldL.data(), oldR.data(), m);
        for (int i = 0; i < m; ++i)
        {
            const size_t at = static_cast<size_t>(ringPos & (kDryRing - 1));
            dry[0][at] = inL[static_cast<size_t>(i)];
            dry[1][at] = inR[static_cast<size_t>(i)];
            ++ringPos;
            float w = 1.0f;
            if (fadePos >= 0)
                w = fadeTable[static_cast<size_t>(fadePos)];
            const size_t from = static_cast<size_t>((ringPos - 1 - dryFrom) & (kDryRing - 1));
            const size_t to = static_cast<size_t>((ringPos - 1 - dryTo) & (kDryRing - 1));
            float wl = wetL[static_cast<size_t>(i)], wr = wetR[static_cast<size_t>(i)];
            float dl = dry[0][to], dr = dry[1][to];
            // Without a kernel the wet path carries the (delayed) input unchanged, so that loading, replacing or removing
            // a response crossfades between the wet signal and the passthrough instead of jumping at any mix.
            float pass = activeEmpty ? 1.0f : 0.0f;
            if (fadePos >= 0)
            {
                // Outgoing set (zero when there was none) fades out while the new one fades in.
                const float ow = outgoing != nullptr ? 1.0f - w : 0.0f;
                wl = wl * w + (outgoing != nullptr ? oldL[static_cast<size_t>(i)] * ow : 0.0f);
                wr = wr * w + (outgoing != nullptr ? oldR[static_cast<size_t>(i)] * ow : 0.0f);
                dl = dry[0][from] * (1.0f - w) + dl * w;
                dr = dry[1][from] * (1.0f - w) + dr * w;
                pass = (activeEmpty ? w : 0.0f) + (outgoingEmpty ? 1.0f - w : 0.0f);
                if (++fadePos > fadeLength)
                {
                    finishFade();
                }
            }
            // Mix goes through the same two one-poles as the level: a per-block linear ramp leaves a slope break at every block
            // boundary.
            mixStage1 += gainSmooth * (static_cast<double>(mixTarget) - mixStage1);
            mixStage2 += gainSmooth * (mixStage1 - mixStage2);
            mixNow = static_cast<float>(mixStage2);
            // The output level (in dB) goes through three one-poles in a row (about 4 ms each), so that a fast sweep of the
            // control leaves no kink at the block boundaries.
            gainStage1 += gainSmooth * (gainDbTarget - gainStage1);
            gainStage2 += gainSmooth * (gainStage1 - gainStage2);
            gainStage3 += gainSmooth * (gainStage2 - gainStage3);
            gainNow = std::exp(gainStage3 * 0.11512925464970229);
            const float k = mixNow;
            l[i] = dl * (1.0f - k) + (wl * static_cast<float>(gainNow) + dl * pass) * k;
            r[i] = dr * (1.0f - k) + (wr * static_cast<float>(gainNow) + dr * pass) * k;
        }
    }

    // ---- state
    const int mix, mode, trim, length, reverse, stretch, fadeIn, fadeOut, balance, lowCut, highCut, predelay, width, outDb;
    double sr = 48000.0;
    bool highQuality = false;
    const std::atomic<bool>* renderOffline = nullptr;
    bool dirty = false;

    // Shared with the control thread.
    mutable std::mutex sourceMutex;
    std::shared_ptr<Source> source;
    std::uint64_t versionCounter = 0;
    std::atomic<std::uint64_t> irVersion{0};
    std::atomic<bool> hasIr{false};

    // Audio thread.
    KernelSet* active = nullptr;
    KernelSet* outgoing = nullptr;
    KernelSet* priming = nullptr; // a finished set being fed the input history before it fades in
    long primed = 0;              // input position (samples since reset) the priming set has consumed
    long position = 0;            // input samples since reset
    long historySize = 1;
    std::array<std::vector<float>, 2> history;
    std::vector<float> primeInL, primeInR, primeOutL, primeOutR;
    int fadePos = -1; // -1 while no fade runs, otherwise 0..fadeLength
    int fadeLength = 2400;
    std::vector<float> fadeTable;
    int dryFrom = 0, dryTo = 0;
    std::array<std::vector<float>, 2> dry;
    long ringPos = 0;
    std::vector<float> inL, inR, wetL, wetR, oldL, oldR, tmp;
    float mixNow = -1.0f;
    double mixStage1 = 0.0, mixStage2 = 0.0;
    double gainNow = -1.0, gainStage1 = 0.0, gainStage2 = 0.0, gainStage3 = 0.0, gainSmooth = 0.005;
    Request seen, postedRequest;
    bool seenValid = false, posted = false;
    int resting = 0, settleSamples = 2400;
    std::array<KernelSet*, 4> stash{};

    // Between the threads.
    RequestSlot requests;
    std::atomic<KernelSet*> ready{nullptr};
    std::array<std::atomic<KernelSet*>, kRetireSlots> retired{};
    std::atomic<std::uint64_t> completed{0};
    std::uint64_t built = 0;
    bool inlineBuilder = false;
    std::atomic<int> reportedLatency{0};
    std::atomic<bool> latencyChanged{false};
    std::atomic<double> tail{0.0};

    // Builder thread.
    std::thread builder;
    std::mutex wakeMutex;
    std::condition_variable wake;
    std::atomic<bool> stopping{false};
    ImpulseResponse base;
    std::uint64_t baseVersion = 0;
    double baseRate = 0.0;
};
} // namespace

ModuleDescriptor convolutionEffect()
{
    auto d =
        effectDescriptor("fx.convolution", "Convolution", "Reverb", CpuClass::Heavy,
                         "Convolution with true impulse-response import: mono, stereo or true-stereo (four kernels, "
                         "left and right inputs to left and right outputs) responses of any sample rate and up to 30 s, "
                         "resampled to the engine rate. Non-uniform partitioned FFT convolution whose work is spread "
                         "evenly over the blocks, in two modes: zero latency, or low CPU with 128 samples of reported "
                         "latency. Response edits (trim start, length, reverse, stretch, fade in and out, early / late "
                         "balance, low and high cut, pre-delay, stereo width) are applied off the audio thread and the "
                         "new kernel is crossfaded in without cutting the tail.");
    auto& p = d.params;
    addOn(p, "cx", "Convolution", "fx_cx", 0.3f);
    p.push_back(params::choice("fx_cx_mode", "Convolution Mode", "fx_cx", {"Zero Latency", "Low CPU"}, 0));
    addF(p, "cx_out", "Convolution Output", "fx_cx", -24.0f, 24.0f, 0.0f, "dB");
    p.push_back(params::floating("fx_cx_trim", "Convolution Trim Start", "fx_cx_2", 0.0f, 2000.0f, 0.0f, "ms", 0.4f));
    p.push_back(params::floating("fx_cx_length", "Convolution Length", "fx_cx_2", 1.0f, 100.0f, 100.0f, "%"));
    p.push_back(params::toggle("fx_cx_reverse", "Convolution Reverse", "fx_cx_2", false));
    p.push_back(params::floating("fx_cx_stretch", "Convolution Stretch", "fx_cx_2", 0.5f, 2.0f, 1.0f, "x"));
    p.push_back(params::floating("fx_cx_fadein", "Convolution Fade In", "fx_cx_2", 0.0f, 1000.0f, 0.0f, "ms", 0.4f));
    p.push_back(params::floating("fx_cx_fadeout", "Convolution Fade Out", "fx_cx_2", 0.0f, 5000.0f, 0.0f, "ms", 0.4f));
    p.push_back(params::floating("fx_cx_balance", "Convolution Early / Late", "fx_cx_3", -12.0f, 12.0f, 0.0f, "dB"));
    p.push_back(params::frequency("fx_cx_lowcut", "Convolution Low Cut", "fx_cx_3", 20.0f, 1000.0f, 20.0f, 0.4f));
    p.push_back(params::frequency("fx_cx_highcut", "Convolution High Cut", "fx_cx_3", 1000.0f, 20000.0f, 20000.0f, 0.4f));
    p.push_back(params::floating("fx_cx_predelay", "Convolution Pre-delay", "fx_cx_3", 0.0f, 500.0f, 0.0f, "ms", 0.5f));
    p.push_back(params::floating("fx_cx_width", "Convolution Width", "fx_cx_3", 0.0f, 2.0f, 1.0f));
    p.push_back(params::integer("fx_pos_cx", "Convolution Slot", "fx_cx_3", 1, kNumModuleFx, kNumModuleFx));
    d.onParam = "fx_cx_on";
    d.positionParam = "fx_pos_cx";
    d.capabilities = {"latency", "response_view", "convolution_ir"};
    d.views = {"response"};
    d.pages.push_back({"fx_cx", "Convolution", "fx", "Convolution", "On, mix, mode and output."});
    d.pages.push_back({"fx_cx_2", "Convolution 2", "fx", "Convolution Shape",
                       "Trim start, length, reverse, stretch and fades of the response."});
    d.pages.push_back({"fx_cx_3", "Convolution 3", "fx", "Convolution Tone",
                       "Early / late balance, low and high cut, pre-delay, width and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.convolution")); };
    return d;
}

} // namespace crucible::modules::fx
