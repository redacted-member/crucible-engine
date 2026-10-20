// SPDX-License-Identifier: MIT
// fx.eq_pro: precision parametric / dynamic equaliser (docs/plans/ENGINE-FX-PLAN.md, section 3.3).
//
// Sixteen bands from include/crucible/fx/AnalogMatchedFilters.h (bell, shelves, tilt, band-pass, notch, all-pass, low and
// high cut up to 96 dB per octave), each on stereo, left, right, mid or side. Three phase modes:
//
//   Minimum  the matched biquads, fitted in magnitude to the analog prototype up to 20 kHz; no latency
//   Natural  the same fitted also in phase, so the phase follows the analog prototype as far as a biquad can; no latency
//   Linear   the magnitude of the same biquads as a symmetric FIR (PartitionedConvolver), designed on a worker thread and
//            swapped in without locks; the latency is reported (LatencyReporter) and the dry path is delayed to match
//
// Dynamic bands (bell, shelves, tilt) move their gain between the set gain and the set gain plus the range, following the
// gain computer of the dynamics core (fx/Dynamics.h) fed with the band-passed input (or the host's auxiliary input). In
// linear-phase mode static bands go to the FIR and dynamic bands stay minimum phase behind it.
//
// Order of processing: every band that works on left / right (stereo, left, right) in band order, then every band that
// works on mid / side, then the output gain. All bands of a domain share the same chain of double-precision direct-form
// sections whose coefficients move linearly across each control block.

#include "FxModuleSupport.h"

#include <crucible/Fft.h>
#include <crucible/fx/AnalogMatchedFilters.h>
#include <crucible/fx/ChannelMode.h>
#include <crucible/fx/Dynamics.h>
#include <crucible/fx/Lookahead.h>
#include <crucible/fx/LinearPhase.h>
#include <crucible/fx/Smoothing.h>
#include <crucible/module/LatencyReporter.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

namespace cfx = crucible::fx;

namespace crucible::modules::fx
{
namespace
{
constexpr int kBands = 16;
constexpr int kBlock = 64;   // samples processed per pass
constexpr int kSub = 16;     // dynamic bands update their filter this often
constexpr int kDynSteps = 8; // designs between the set gain and the set gain plus the range
constexpr int kResponsePoints = 256;
constexpr int kMovingRamp = 512;  // samples over which the filter follows a parameter that is moving
constexpr int kSettleRamp = 4096; // samples over which a resting band changes from the smooth to the fitted design
constexpr int kCalmBlocks = 8;    // blocks without a change before the fitted design replaces the smooth one
constexpr double kAutoGainLimitDb = 24.0; // auto gain never corrects by more than the output control's range
constexpr float kOutSmoothMs = 20.0f; // output gain and mix glide (each of two stages)
constexpr float kTableGlideMs = 10.0f; // dynamic bands: glide of the design table and range towards a new setting
constexpr float kFirFadeMs = 20.0f;   // linear phase: crossfade between two kernels (rounded to whole convolver blocks)
constexpr float kBandGlideMs = 15.0f; // static bands: glide of the filter towards a new design
constexpr int kSpectrumOrder = 11;
constexpr int kSpectrumBins = (1 << kSpectrumOrder) / 2 + 1;

const std::vector<std::string>& typeNames()
{
    static const std::vector<std::string> n = {"Bell",  "Low Shelf", "High Shelf", "Tilt",    "Band Pass",
                                               "Notch", "All Pass",  "Low Cut",    "High Cut"};
    return n;
}
const std::vector<std::string>& slopeNames()
{
    static const std::vector<std::string> n = {"6 dB/oct",  "12 dB/oct", "18 dB/oct", "24 dB/oct",
                                               "36 dB/oct", "48 dB/oct", "96 dB/oct"};
    return n;
}
const std::vector<std::string>& shapeNames()
{
    static const std::vector<std::string> n = {"Butterworth", "Bessel", "Linkwitz-Riley", "Resonant"};
    return n;
}
const std::vector<std::string>& channelNames()
{
    static const std::vector<std::string> n = {"Stereo", "Left", "Right", "Mid", "Side"};
    return n;
}
const std::vector<std::string>& dynNames()
{
    static const std::vector<std::string> n = {"Off", "Above", "Below"};
    return n;
}
const std::vector<std::string>& resolutionNames()
{
    static const std::vector<std::string> n = {"1k taps", "2k taps", "4k taps", "8k taps", "16k taps", "32k taps"};
    return n;
}

// Sixteen round frequencies, about a third of an octave apart in the lows and a half octave in the highs.
double defaultFrequency(int band)
{
    static const double hz[kBands] = {40, 60, 90, 130, 200, 290, 440, 650, 980, 1500, 2200, 3200, 4800, 7200, 11000, 16000};
    return hz[band];
}

bool gainType(cfx::BandType t)
{
    return t == cfx::BandType::Bell || t == cfx::BandType::LowShelf || t == cfx::BandType::HighShelf ||
           t == cfx::BandType::Tilt;
}

// Stacking sixteen boosts can ask for any gain; the output is identical below +24 dBFS and approaches +30 dBFS
// asymptotically above it, so that no setting produces an infinity or an unbounded signal (nothing in the engine gets
// near the bound).
inline float safetyLimit(float x)
{
    const float a = std::abs(x);
    if (!(a == a))
        return 0.0f;
    if (a <= 16.0f)
        return x;
    const float e = std::min(a - 16.0f, 1.0e6f);
    return std::copysign(16.0f + e / (1.0f + e / 14.0f), x);
}

// Two cascaded one-poles: a once-per-block change of the output gain or the mix becomes a ramp with a continuous slope.
// Snaps onto the target, so a constant setting stays bit-identical.
struct Smooth2
{
    // Double state: with a float state a step at a small coefficient (a host block of one sample) stalls far from the
    // target, and landing on it from there would be an audible step.
    double a = 0.0, b = 0.0;
    void set(float v) { a = b = v; }
    bool settled(float target) const { return a == target && b == target; }
    static double to(double v, double target, double coef)
    {
        const double diff = target - v;
        if (std::abs(diff) <= 1.0e-6 * std::max(1.0, std::abs(target)))
            return target;
        const double next = v + coef * diff;
        return next == v ? target : next;
    }
    float step(float target, float coef)
    {
        a = to(a, target, coef);
        b = to(b, a, coef);
        return static_cast<float>(b);
    }
};

cfx::Biquad lerp(const cfx::Biquad& a, const cfx::Biquad& b, double t)
{
    cfx::Biquad o;
    o.b0 = a.b0 + (b.b0 - a.b0) * t;
    o.b1 = a.b1 + (b.b1 - a.b1) * t;
    o.b2 = a.b2 + (b.b2 - a.b2) * t;
    o.a1 = a.a1 + (b.a1 - a.a1) * t;
    o.a2 = a.a2 + (b.a2 - a.a2) * t;
    return o;
}

// ---------------------------------------------------------------------------------------------------------------
// Request for a linear-phase kernel: the designed static bands of the current settings.
struct FirRequest
{
    struct Entry
    {
        cfx::BandFilter filter;
        int group = 0; // 0 stereo, 1 left, 2 right, 3 mid, 4 side
    };
    int lengthLog2 = 12;
    double sampleRate = 48000.0;
    int count = 0;
    std::array<Entry, kBands> entry;
};

struct BandIndex
{
    int on, type, freq, gain, q, slope, shape, ch, dyn, range, thr, ratio, att, rel, key;
};

struct Band
{
    // The values the design was made from; changed() compares them.
    std::array<float, 14> last{};
    bool haveLast = false;
    bool wasOn = false;
    bool settled = true; // the design in use is the accurate fit (not the smooth one used while parameters move)
    int calm = 0;        // control blocks since the parameters last changed
    cfx::BandSpec spec;
    bool enabled = false;
    bool dynamic = false;
    int channel = 0; // 0 stereo, 1 left, 2 right, 3 mid, 4 side
    cfx::BandFilter filter;
    // A static band follows `filter` (the design for the current settings) through two cascaded one-poles per coefficient:
    // glideA is the first stage, glideB the one in use. A change made once per host block then moves the filter without a
    // step or a slope break.
    cfx::BandFilter glideA, glideB;
    bool glideValid = false;
    bool gliding = false;
    cfx::BiquadChain chain;
    cfx::BiquadChain keyChain;
    std::array<cfx::Biquad, kDynSteps + 1> table;       // in use (second glide stage)
    std::array<cfx::Biquad, kDynSteps + 1> tableTarget; // designed for the current settings
    std::array<cfx::Biquad, kDynSteps + 1> tableStage;  // first glide stage
    bool tableSettled = true;
    bool tableHard = true; // next design is taken over at once (first design, reset, new sample rate)
    Smooth2 rangeSm;
    cfx::Biquad live; // dynamic: the section in use
    double sampleRate = 48000.0;
    cfx::DynamicsProcessor proc;
    float rangeDb = 0.0f, thresholdDb = -24.0f, ratio = 4.0f, attackMs = 10.0f, releaseMs = 120.0f;
    int dynMode = 0;
    bool external = false;
    bool domainMidSide() const { return channel >= 3; }
    int laneMask() const { return channel == 0 ? 3 : (channel == 1 || channel == 3 ? 1 : 2); }
};

class Instance final : public EffectModule,
                       public LatencyReporter,
                       public ResponseView,
                       public cfx::LinearPhaseWorker::Client
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : on(localIndex(d, "fx_ep_on")), mix(localIndex(d, "fx_ep_mix")), outDb(localIndex(d, "fx_ep_out")),
          phase(localIndex(d, "fx_ep_phase")), resolution(localIndex(d, "fx_ep_res")), autoGain(localIndex(d, "fx_ep_auto"))
    {
        for (int b = 0; b < kBands; ++b)
        {
            const std::string n = "fx_ep_b" + std::to_string(b + 1) + "_";
            const auto at = [&](const char* leaf) { return localIndex(d, n + leaf); };
            index[static_cast<size_t>(b)] = {at("on"),    at("type"),  at("freq"), at("gain"), at("q"),
                                             at("slope"), at("shape"), at("ch"),   at("dyn"),  at("range"),
                                             at("thr"),   at("ratio"), at("att"),  at("rel"),  at("key")};
        }
    }

    ~Instance() override
    {
        if (registered)
            cfx::LinearPhaseWorker::instance().remove(this);
    }

    // ---- EffectModule
    void prepare(const ModuleContext& c) override
    {
        sr = c.sampleRate;
        offline = c.quality == Quality::High;
        for (auto& b : bands)
        {
            b.sampleRate = sr;
            b.proc.prepare(sr);
            b.haveLast = false;
            b.tableHard = true;
        }
        // A design still in flight belongs to the old settings; let the worker finish it, then forget it together with the
        // installed kernel (made for the old sample rate), so that the next Linear session starts on a fresh design.
        if (registered)
            settleWorker();
        convolver.prepare(2);
        convolver.setFadeSamples(static_cast<int>(kFirFadeMs * 0.001f * static_cast<float>(sr)));
        convolver.dropKernel();
        firDirty = true;
        cooldown = 0;
        dryLook.prepare(2, 1 << 15, sr, 0.010);
        keyLook.prepare(2, 1 << 15, sr, 0.010);
        specFft = std::make_unique<Fft>(kSpectrumOrder);
        specWork.assign(static_cast<size_t>(2) << kSpectrumOrder, 0.0f);
        ring.assign(static_cast<size_t>(1) << kSpectrumOrder, 0.0f);
        ringPos = 0;
        reset();
        if (!registered)
        {
            cfx::LinearPhaseWorker::instance().add(this);
            registered = true;
        }
        prepared = true;
    }

    void reset() override
    {
        for (auto& b : bands)
        {
            b.chain.reset();
            b.keyChain.reset();
            b.proc.reset();
            b.haveLast = false;
            b.tableHard = true;
        }
        convolver.reset();
        dryLook.reset();
        keyLook.reset();
        duck = 1.0f;
        duckTarget = 1.0f;
        fadeInHold = 0;
        path = 0;
        firLog2 = resolutionNow;
        publishLatency(0);
        postedVersion = 0;
        std::fill(ring.begin(), ring.end(), 0.0f);
        mixValue = -1.0f;
        outDbNow = 0.0f;
        gainLinear = -1.0f;
        outSmooth = -1.0f;
        designed = false;
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
        dirty = true;
        readParameters(in, n);
        const bool anyExternal = in.hasAux();
        for (int pos = 0; pos < n; pos += kBlock)
        {
            const int m = std::min(kBlock, n - pos);
            processBlock(l + pos, r + pos, m, anyExternal ? in.auxLeft() + pos : nullptr,
                         anyExternal ? in.auxRight() + pos : nullptr);
        }
    }

    // ---- LatencyReporter
    int latencySamples() const override { return reportedLatency.load(std::memory_order_relaxed); }
    bool takeLatencyChanged() override { return latencyChanged.exchange(false, std::memory_order_relaxed); }

    // ---- ResponseView
    int responsePoints() const override { return kResponsePoints; }
    void fillResponse(float* out) override
    {
        const double top = std::min(20000.0, 0.499 * sr);
        for (int i = 0; i < kResponsePoints; ++i)
        {
            const double hz = 20.0 * std::pow(top / 20.0, static_cast<double>(i) / (kResponsePoints - 1));
            out[i] = static_cast<float>(responseDb(2.0 * 3.14159265358979323846 * hz / sr) + static_cast<double>(outDbNow));
        }
    }
    int spectrumBins() const override { return kSpectrumBins; }
    void fillSpectrum(float* out) override
    {
        const int size = 1 << kSpectrumOrder;
        for (int i = 0; i < size; ++i)
        {
            const float w = 0.5f - 0.5f * std::cos(6.2831853f * static_cast<float>(i) / static_cast<float>(size));
            const size_t at = (ringPos + static_cast<size_t>(i)) & (static_cast<size_t>(size) - 1);
            specWork[static_cast<size_t>(i)] = ring[at] * w;
        }
        std::fill(specWork.begin() + size, specWork.end(), 0.0f);
        specFft->forward(specWork.data());
        for (int k = 0; k < kSpectrumBins; ++k)
        {
            const float re = specWork[static_cast<size_t>(2 * k)], im = specWork[static_cast<size_t>(2 * k + 1)];
            const float scale = 4.0f / (static_cast<float>(size) * static_cast<float>(size) * 0.375f); // Hann window power
            out[k] = 10.0f * std::log10(std::max(re * re + im * im, 1.0e-20f) * scale);
        }
    }

    // ---- Worker
    bool service(cfx::LinearPhaseWorker::Tools& tools) override
    {
        FirRequest request;
        if (!requests.fetch(request, seenVersion))
        {
            convolver.mailbox().collect();
            return false;
        }
        tools.ensure();
        const int k = request.lengthLog2;
        const int points = cfx::LinearPhaseDesigner::gridPoints(k);
        for (int i = 0; i < 4; ++i)
            tools.scratch[i].assign(static_cast<size_t>(points), 1.0f);
        const bool anyMidSide = [&]
        {
            for (int i = 0; i < request.count; ++i)
                if (request.entry[static_cast<size_t>(i)].group >= 3)
                    return true;
            return false;
        }();
        // Magnitudes of the four lane groups on the grid (stereo bands act on both left and right).
        std::vector<float>& hl = tools.scratch[0];
        std::vector<float>& hr = tools.scratch[1];
        std::vector<float>& hm = tools.scratch[2];
        std::vector<float>& hs = tools.scratch[3];
        for (int i = 0; i < request.count; ++i)
        {
            const auto& e = request.entry[static_cast<size_t>(i)];
            for (int j = 0; j < points; ++j)
            {
                const double c = std::cos(cfx::LinearPhaseDesigner::gridOmega(k, j));
                double m2 = e.filter.gain * e.filter.gain;
                for (int s = 0; s < e.filter.count; ++s)
                    m2 *= cfx::biquadMagnitudeSquared(e.filter.section[static_cast<size_t>(s)], c);
                const float mag = static_cast<float>(std::sqrt(std::max(m2, 0.0)));
                switch (e.group)
                {
                case 0:
                    hl[static_cast<size_t>(j)] *= mag;
                    hr[static_cast<size_t>(j)] *= mag;
                    break;
                case 1:
                    hl[static_cast<size_t>(j)] *= mag;
                    break;
                case 2:
                    hr[static_cast<size_t>(j)] *= mag;
                    break;
                case 3:
                    hm[static_cast<size_t>(j)] *= mag;
                    break;
                default:
                    hs[static_cast<size_t>(j)] *= mag;
                    break;
                }
            }
        }
        // Matrix of the cascade: left / right bands first, then mid / side. T = Dec * diag(Hm, Hs) * Enc * diag(Hl, Hr).
        for (int i = 4; i < 8; ++i)
            tools.scratch[i].assign(static_cast<size_t>(points), 0.0f);
        std::vector<float>& t00 = tools.scratch[4];
        std::vector<float>& t01 = tools.scratch[5];
        std::vector<float>& t10 = tools.scratch[6];
        std::vector<float>& t11 = tools.scratch[7];
        for (int j = 0; j < points; ++j)
        {
            const size_t q = static_cast<size_t>(j);
            const float a = 0.5f * (hm[q] + hs[q]), b = 0.5f * (hm[q] - hs[q]);
            t00[q] = a * hl[q];
            t01[q] = b * hr[q];
            t10[q] = b * hl[q];
            t11[q] = a * hr[q];
        }
        // The taps reuse the arrays of the lane magnitudes, which are not needed any more.
        float* tapsStore[4] = {hl.data(), hr.data(), hm.data(), hs.data()};
        tools.designer.design(k, t00.data(), tapsStore[0]);
        tools.designer.design(k, t11.data(), tapsStore[3]);
        const float* pointers[4] = {tapsStore[0], nullptr, nullptr, tapsStore[3]};
        if (anyMidSide)
        {
            tools.designer.design(k, t01.data(), tapsStore[1]);
            tools.designer.design(k, t10.data(), tapsStore[2]);
            pointers[1] = tapsStore[1];
            pointers[2] = tapsStore[2];
        }
        convolver.mailbox().post(tools.builder.build(k, 2, pointers, seenVersion));
        convolver.mailbox().collect();
        completed.store(seenVersion, std::memory_order_release);
        return true;
    }

private:
    // -------------------------------------------------------------------------------------------------------------
    void readParameters(const EffectInputs& in, int n)
    {
        const int phaseMode = std::clamp(in.choice(phase), 0, 2);
        const int resolutionIndex = std::clamp(in.choice(resolution), 0, 5);
        const int log2 = cfx::kMinKernelLog2 + resolutionIndex;
        const cfx::DesignQuality quality = phaseMode == 1 ? cfx::DesignQuality::Natural : cfx::DesignQuality::Minimum;
        const bool forceAll = !designed || quality != qualityNow;
        bool designsChanged = forceAll;
        qualityNow = quality;
        wantFir = phaseMode == 2;
        if (wantFir && (log2 != resolutionNow || !wantFirPrev))
            firDirty = true;
        wantFirPrev = wantFir;
        resolutionNow = log2;
        float gain = in.value(outDb);
        outTarget = gain;
        wetMix = in.value(mix);
        autoOn = in.on(autoGain);

        for (int b = 0; b < kBands; ++b)
        {
            Band& band = bands[static_cast<size_t>(b)];
            const BandIndex& ix = index[static_cast<size_t>(b)];
            const bool bandOn = in.on(ix.on);
            if (!bandOn && band.haveLast && !band.wasOn && !forceAll)
                continue;
            std::array<float, 14> v;
            v[0] = bandOn ? 1.0f : 0.0f;
            v[1] = static_cast<float>(in.choice(ix.type));
            v[2] = in.value(ix.freq);
            v[3] = in.value(ix.gain);
            v[4] = in.value(ix.q);
            v[5] = static_cast<float>(in.choice(ix.slope));
            v[6] = static_cast<float>(in.choice(ix.shape));
            v[7] = static_cast<float>(in.choice(ix.ch));
            v[8] = static_cast<float>(in.choice(ix.dyn));
            v[9] = in.value(ix.range);
            v[10] = in.value(ix.thr);
            v[11] = in.value(ix.ratio);
            v[12] = in.value(ix.att);
            v[13] = in.value(ix.rel);
            const bool external = in.choice(ix.key) == 1;
            const bool first = !band.haveLast;
            const bool same = band.haveLast && external == band.external &&
                              std::memcmp(v.data(), band.last.data(), sizeof(float) * 14) == 0;
            band.wasOn = bandOn;
            if (same && !forceAll)
            {
                // Resting: the smooth design used while the band moved is replaced by the accurate fit, gently.
                if (!band.settled && ++band.calm >= kCalmBlocks)
                {
                    band.settled = true;
                    designBandNow(band, v, qualityNow, kSettleRamp);
                    designsChanged = true;
                    firDirty = true;
                }
                continue;
            }
            const bool moving = band.haveLast && !forceAll;
            band.last = v;
            band.haveLast = true;
            band.external = external;
            band.calm = 0;
            band.settled = !moving;
            designBandNow(band, v, moving ? cfx::DesignQuality::Smooth : qualityNow,
                          first ? 0 : (moving ? std::max(n, kMovingRamp) : n));
            designsChanged = true;
            firDirty = true;
        }
        if (designsChanged)
        {
            designed = true;
            updateAutoGain();
        }
    }

    static cfx::BandSpec specFrom(const std::array<float, 14>& v)
    {
        cfx::BandSpec spec;
        spec.type = static_cast<cfx::BandType>(std::clamp(static_cast<int>(v[1]), 0, 8));
        spec.freqHz = v[2];
        spec.gainDb = v[3];
        spec.q = v[4];
        spec.slope = static_cast<int>(v[5]);
        spec.shape = static_cast<cfx::CutShape>(std::clamp(static_cast<int>(v[6]), 0, 3));
        return spec;
    }

    void designBandNow(Band& band, const std::array<float, 14>& v, cfx::DesignQuality quality, int ramp)
    {
        const bool wasDynamic = band.dynamic;
        band.spec = specFrom(v);
        band.channel = std::clamp(static_cast<int>(v[7]), 0, 4);
        band.dynMode = std::clamp(static_cast<int>(v[8]), 0, 2);
        band.rangeDb = v[9];
        band.thresholdDb = v[10];
        band.ratio = v[11];
        band.attackMs = v[12];
        band.releaseMs = v[13];
        const bool isGain = gainType(band.spec.type);
        band.enabled = v[0] > 0.5f;
        // A band that is already dynamic stays on the dynamic path while the range passes through zero (a moving range would
        // otherwise switch between the two paths for a block).
        band.dynamic = band.enabled && isGain && band.dynMode != 0 && (band.rangeDb != 0.0f || wasDynamic);
        if (!band.enabled || (isGain && !band.dynamic && band.spec.gainDb == 0.0))
        {
            band.filter = {};
            if (band.enabled && band.glideValid && ramp > 0)
                band.gliding = true; // a moving gain passing through 0 dB glides on through the identity
            else
            {
                band.chain.set(band.filter, ramp);
                band.glideValid = false;
                band.gliding = false;
            }
            if (!band.enabled)
                band.dynamic = false;
            return;
        }
        if (band.dynamic)
        {
            band.glideValid = false;
            band.gliding = false;
            cfx::BandSpec s = band.spec;
            for (int k = 0; k <= kDynSteps; ++k)
            {
                s.gainDb = band.spec.gainDb + static_cast<double>(band.rangeDb) * k / kDynSteps;
                // A gain of exactly zero would be the identity (b0 = 1 and nothing else), which is not on the same
                // coefficient path as the neighbouring designs (the zeros cancel the poles instead): interpolating towards
                // it would bend the gain curve. A hair away from zero keeps the family continuous.
                if (std::abs(s.gainDb) < 1.0e-6)
                    s.gainDb = band.rangeDb < 0.0f ? -1.0e-6 : 1.0e-6;
                const auto fitted = quality == cfx::DesignQuality::Smooth ? qualityNow : quality;
                const cfx::BandFilter f = cfx::designBand(s, sr, fitted);
                band.tableTarget[static_cast<size_t>(k)] = f.count > 0 ? f.section[0] : cfx::Biquad{};
            }
            if (!wasDynamic || band.tableHard)
            {
                band.table = band.tableTarget;
                band.tableStage = band.tableTarget;
                band.rangeSm.set(std::abs(band.rangeDb));
                band.tableSettled = true;
                band.tableHard = false;
                band.live = band.table[0];
            }
            else
                band.tableSettled = false;
            band.filter = {};
            band.filter.count = 1;
            band.filter.section[0] = band.live;
            // The key: the band's own frequency and bandwidth.
            cfx::BandSpec key;
            key.type = cfx::BandType::BandPass;
            key.freqHz = band.spec.freqHz;
            key.q = std::max(band.spec.q, 0.5);
            band.keyChain.set(cfx::designBand(key, sr, cfx::DesignQuality::Matched), ramp);
            // A band that was dynamic already keeps its running coefficients: runBand re-sets them every kSub samples.
            if (!wasDynamic)
                band.chain.set(band.filter, 0);
            return;
        }
        band.filter = cfx::designBand(band.spec, sr, quality);
        if (band.glideValid && ramp > 0)
            band.gliding = true;
        else
        {
            band.glideA = band.glideB = band.filter;
            band.glideValid = true;
            band.gliding = false;
            band.chain.set(band.filter, ramp);
        }
    }

    // Output gain of the "auto gain" option: the mean of the static curve over the octaves, inverted.
    void updateAutoGain()
    {
        double sum = 0.0;
        const int points = 24;
        const double top = std::min(20000.0, 0.499 * sr);
        for (int i = 0; i < points; ++i)
        {
            const double hz = 40.0 * std::pow(top / 40.0, static_cast<double>(i) / (points - 1));
            sum += responseDb(2.0 * 3.14159265358979323846 * hz / sr);
        }
        // A stack of strong cuts or boosts has a mean far outside any useful range (a 96 dB per octave cut at 10 Hz is
        // hundreds of dB below zero over the octaves): the correction is limited, or it would lift the rounding noise of a
        // dead filter to full scale and the tail would never end.
        const double correction = std::isfinite(sum) ? -sum / points : 0.0;
        autoTarget = static_cast<float>(std::clamp(correction, -kAutoGainLimitDb, kAutoGainLimitDb));
    }

    double responseDb(double omega) const
    {
        double db = 0.0;
        for (const Band& band : bands)
        {
            if (!band.enabled)
                continue;
            if (band.dynamic)
            {
                cfx::BandFilter f;
                f.count = 1;
                f.section[0] = band.live;
                db += f.magnitudeDb(omega);
            }
            else
                db += band.filter.magnitudeDb(omega);
        }
        return db;
    }

    // -------------------------------------------------------------------------------------------------------------
    void postFirRequest()
    {
        FirRequest request;
        request.lengthLog2 = resolutionNow;
        request.sampleRate = sr;
        request.count = 0;
        for (const Band& band : bands)
        {
            if (!band.enabled || band.dynamic || band.filter.count == 0)
                continue;
            auto& e = request.entry[static_cast<size_t>(request.count++)];
            e.filter = band.filter;
            e.group = band.channel;
        }
        // Offline renders must not depend on thread timing, so a collision with the worker's poll is waited out there.
        if (offline)
            requests.post(request);
        else if (!requests.tryPost(request))
            return;
        postedVersion = requests.posted();
        firDirty = false;
        cooldown = 256;
    }

    // The latency follows the wet path: it is published when the path (or its length class) actually changes.
    void publishLatency(int samples)
    {
        if (samples != reportedLatency.load(std::memory_order_relaxed))
        {
            reportedLatency.store(samples, std::memory_order_relaxed);
            latencyChanged.store(true, std::memory_order_relaxed);
        }
    }

    // Control thread: waits (at most two seconds) until the worker has handled every request posted so far.
    void settleWorker()
    {
        const auto limit = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (completed.load(std::memory_order_acquire) < requests.posted() && std::chrono::steady_clock::now() < limit)
        {
            cfx::LinearPhaseWorker::instance().wake();
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    }

    // Offline (High quality): wait for the worker so that the result is the same whatever the thread timing. Without
    // background threads the design only runs in LinearPhaseWorker::pump() on the control side (Engine::service()); the
    // audio thread never pumps and cannot wait for it, so the design arrives at the next service() call, as in real time.
    void waitForDesign()
    {
        if (cfx::LinearPhaseWorker::threadless())
            return;
        while (completed.load(std::memory_order_acquire) < postedVersion)
        {
            cfx::LinearPhaseWorker::instance().wake();
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    // -------------------------------------------------------------------------------------------------------------
    void processBlock(float* l, float* r, int m, const float* auxL, const float* auxR)
    {
        // Path management: the linear-phase FIR and the IIR bands never run at the same time; a switch (between the two, or
        // between FIR length classes) ducks the wet path, and the dry delay and the reported latency follow the wet path.
        const bool classChange = wantFir && path == 1 && firLog2 != resolutionNow;
        if (wantFir != (path == 1) || classChange)
            duckTarget = 0.0f;
        else if (duckTarget == 0.0f)
        {
            duckTarget = 1.0f; // asked to switch back before the fade finished
            firDirty = true;   // a request for another length class may be on its way: replace it
        }
        if (wantFir && cooldown > 0)
            cooldown -= m;
        // While the FIR runs, a request for another length class waits until the fade is over (the convolver would restart
        // on it with a gap).
        const bool mayPost = !classChange || duck <= 0.0f;
        if (wantFir && firDirty && cooldown <= 0 && mayPost)
        {
            postFirRequest();
            if (offline)
                waitForDesign();
        }
        if (duckTarget == 0.0f && duck <= 0.0f)
        {
            const bool toFir = wantFir;
            if (!toFir || convolver.adopt(resolutionNow) || offline)
            {
                path = toFir ? 1 : 0;
                firLog2 = resolutionNow;
                for (auto& b : bands)
                {
                    b.chain.reset();
                    b.keyChain.reset();
                    b.proc.reset();
                }
                convolver.reset();
                duckTarget = 1.0f;
                fadeInHold = toFir ? cfx::linearPhaseLatency(firLog2) : 0;
                publishLatency(toFir ? cfx::linearPhaseLatency(firLog2) : 0);
                if (toFir)
                {
                    firDirty = true;
                    cooldown = 0;
                    postFirRequest();
                    if (offline)
                        waitForDesign();
                }
            }
        }
        const float duckStep = 1.0f / 240.0f;
        const int latency = path == 1 ? cfx::linearPhaseLatency(firLog2) : 0;
        dryLook.setDelay(latency);
        keyLook.setDelay(latency);

        float dryL[kBlock], dryR[kBlock];
        std::memcpy(dryL, l, sizeof(float) * static_cast<size_t>(m));
        std::memcpy(dryR, r, sizeof(float) * static_cast<size_t>(m));
        float* dry[2] = {dryL, dryR};
        dryLook.process(dry, m);

        float keyL[kBlock], keyR[kBlock];
        const float* keys[2] = {nullptr, nullptr};
        if (auxL != nullptr)
        {
            std::memcpy(keyL, auxL, sizeof(float) * static_cast<size_t>(m));
            std::memcpy(keyR, auxR, sizeof(float) * static_cast<size_t>(m));
            float* k[2] = {keyL, keyR};
            keyLook.process(k, m);
            keys[0] = keyL;
            keys[1] = keyR;
        }

        if (path == 1)
            convolver.process(l, r, m);

        double a[kBlock], b[kBlock];
        for (int i = 0; i < m; ++i)
        {
            a[i] = l[i];
            b[i] = r[i];
        }
        runBands(a, b, m, keys[0], keys[1]);

        // Output gain, auto gain, duck and mix, ramped across the block.
        const float outEnd = outTarget + (autoOn ? autoTarget : 0.0f);
        outDbNow = outEnd;
        const float dryTarget = std::clamp(wetMix, 0.0f, 1.0f);
        if (outSmooth < 0.0f)
        {
            outDb2.set(outEnd);
            mixDb2.set(dryTarget);
            outSmooth = 1.0f;
            gainLinear = dbToLinear(outEnd);
            mixValue = dryTarget;
        }
        const float gainEnd = dbToLinear(outEnd);
        const float smoothCoef = 1.0f - static_cast<float>(std::exp(-1.0 / (kOutSmoothMs * 0.001 * sr)));
        for (int i = 0; i < m; ++i)
        {
            gainLinear = outDb2.settled(outEnd) ? gainEnd : dbToLinear(outDb2.step(outEnd, smoothCoef));
            const float g = gainLinear;
            if (duck < duckTarget)
            {
                // The convolver starts from silence: its first real output arrives one kernel delay later, so the fade-in
                // waits for it (the step of a signal switched in mid-cycle would otherwise come through at full gain).
                if (fadeInHold > 0)
                    --fadeInHold;
                else
                    duck = std::min(duckTarget, duck + duckStep);
            }
            else if (duck > duckTarget)
                duck = std::max(duckTarget, duck - duckStep);
            mixValue = mixDb2.settled(dryTarget) ? dryTarget : mixDb2.step(dryTarget, smoothCoef);
            const float wl = static_cast<float>(a[i]) * g * duck;
            const float wr = static_cast<float>(b[i]) * g * duck;
            l[i] = safetyLimit(dryL[i] + mixValue * (wl - dryL[i]));
            r[i] = safetyLimit(dryR[i] + mixValue * (wr - dryR[i]));
            ring[ringPos & (ring.size() - 1)] = 0.5f * (l[i] + r[i]);
            ++ringPos;
        }
    }

    static float dbToLinear(float db) { return cfx::dyn::dbToGain(db); }

    // Left / right bands, then mid / side bands, on the double-precision lanes a and b.
    void runBands(double* a, double* b, int m, const float* keyL, const float* keyR)
    {
        const bool iir = path == 0;
        bool anyMidSide = false;
        for (const Band& band : bands)
            anyMidSide = anyMidSide || (band.enabled && band.domainMidSide() && (band.dynamic || iir));
        for (Band& band : bands)
            if (band.enabled && !band.domainMidSide())
                runBand(band, a, b, m, keyL, keyR, iir);
        if (!anyMidSide)
            return;
        for (int i = 0; i < m; ++i)
        {
            const double mid = 0.5 * (a[i] + b[i]), side = 0.5 * (a[i] - b[i]);
            a[i] = mid;
            b[i] = side;
        }
        // The key follows the domain of the band: mid / side keys are made once per block.
        float keyM[kBlock], keyS[kBlock];
        const float* km = nullptr;
        const float* ks = nullptr;
        if (keyL != nullptr)
        {
            for (int i = 0; i < m; ++i)
            {
                keyM[i] = 0.5f * (keyL[i] + keyR[i]);
                keyS[i] = 0.5f * (keyL[i] - keyR[i]);
            }
            km = keyM;
            ks = keyS;
        }
        for (Band& band : bands)
            if (band.enabled && band.domainMidSide())
                runBand(band, a, b, m, km, ks, iir);
        for (int i = 0; i < m; ++i)
        {
            const double left = a[i] + b[i], right = a[i] - b[i];
            a[i] = left;
            b[i] = right;
        }
    }

    // The design table and the range follow the new settings through two cascaded one-poles (per coefficient), so that a
    // change made once per host block moves the filter without a step or a slope break.
    static void glideTable(Band& band, int len, float rangeTarget)
    {
        const float coef = 1.0f - static_cast<float>(std::exp(-static_cast<double>(len) / (kTableGlideMs * 0.001 * band.sampleRate)));
        band.rangeSm.step(rangeTarget, coef);
        if (band.tableSettled)
            return;
        const double c = coef;
        const auto to = [c](double v, double target)
        {
            const double d = target - v;
            return std::abs(d) <= 1.0e-12 ? target : v + c * d;
        };
        bool settled = true;
        for (size_t k = 0; k < band.table.size(); ++k)
        {
            cfx::Biquad& a = band.tableStage[k];
            cfx::Biquad& b = band.table[k];
            const cfx::Biquad& t = band.tableTarget[k];
            a.b0 = to(a.b0, t.b0);
            a.b1 = to(a.b1, t.b1);
            a.b2 = to(a.b2, t.b2);
            a.a1 = to(a.a1, t.a1);
            a.a2 = to(a.a2, t.a2);
            b.b0 = to(b.b0, a.b0);
            b.b1 = to(b.b1, a.b1);
            b.b2 = to(b.b2, a.b2);
            b.a1 = to(b.a1, a.a1);
            b.a2 = to(b.a2, a.a2);
            settled = settled && b.b0 == t.b0 && b.b1 == t.b1 && b.b2 == t.b2 && b.a1 == t.a1 && b.a2 == t.a2;
        }
        band.tableSettled = settled;
    }

    static void glideFilter(Band& band, int len)
    {
        const double c = 1.0 - std::exp(-static_cast<double>(len) / (kBandGlideMs * 0.001 * band.sampleRate));
        const auto to = [c](double v, double target)
        {
            const double d = target - v;
            return std::abs(d) <= 1.0e-12 ? target : v + c * d;
        };
        const cfx::BandFilter& t = band.filter;
        const int count = std::max(t.count, band.glideB.count);
        band.glideA.gain = to(band.glideA.gain, t.gain);
        band.glideB.gain = to(band.glideB.gain, band.glideA.gain);
        bool settled = band.glideB.gain == t.gain;
        const cfx::Biquad identity;
        for (int k = 0; k < count; ++k)
        {
            const auto i = static_cast<size_t>(k);
            cfx::Biquad& x = band.glideA.section[i];
            cfx::Biquad& y = band.glideB.section[i];
            const cfx::Biquad& g = k < t.count ? t.section[i] : identity;
            x.b0 = to(x.b0, g.b0);
            x.b1 = to(x.b1, g.b1);
            x.b2 = to(x.b2, g.b2);
            x.a1 = to(x.a1, g.a1);
            x.a2 = to(x.a2, g.a2);
            y.b0 = to(y.b0, x.b0);
            y.b1 = to(y.b1, x.b1);
            y.b2 = to(y.b2, x.b2);
            y.a1 = to(y.a1, x.a1);
            y.a2 = to(y.a2, x.a2);
            settled = settled && y.b0 == g.b0 && y.b1 == g.b1 && y.b2 == g.b2 && y.a1 == g.a1 && y.a2 == g.a2;
        }
        band.glideA.count = band.glideB.count = count;
        if (settled)
            band.gliding = false;
    }

    void runBand(Band& band, double* a, double* b, int m, const float* keyA, const float* keyB, bool iir)
    {
        if (!band.dynamic)
        {
            if (band.gliding)
            {
                if (!iir)
                {
                    // The IIR chain is not running (linear phase): it takes the design over at once.
                    band.glideA = band.glideB = band.filter;
                    band.gliding = false;
                    band.chain.set(band.filter, 0);
                    return;
                }
                for (int pos = 0; pos < m; pos += kSub)
                {
                    const int len = std::min(kSub, m - pos);
                    glideFilter(band, len);
                    band.chain.set(band.glideB, len);
                    band.chain.process(a + pos, b + pos, len, band.laneMask());
                }
                return;
            }
            if (iir)
            {
                band.chain.process(a, b, m, band.laneMask());
            }
            return;
        }
        // ---- Dynamic band: key -> band-pass -> gain computer -> filter coefficients every kSub samples.
        float ka[kBlock], kb[kBlock];
        double ta[kBlock], tb[kBlock];
        const bool useExternal = band.external && keyA != nullptr;
        for (int i = 0; i < m; ++i)
        {
            ta[i] = useExternal ? static_cast<double>(keyA[i]) : a[i];
            tb[i] = useExternal ? static_cast<double>(keyB[i]) : b[i];
        }
        band.keyChain.process(ta, tb, m, 3);
        for (int i = 0; i < m; ++i)
        {
            ka[i] = static_cast<float>(ta[i]);
            kb[i] = static_cast<float>(tb[i]);
        }
        float onesA[kBlock], onesB[kBlock];
        for (int i = 0; i < m; ++i)
            onesA[i] = onesB[i] = 1.0f;
        cfx::DynamicsSettings s;
        s.kind = band.dynMode == 1 ? cfx::DynamicsKind::Compressor : cfx::DynamicsKind::Upward;
        s.thresholdDb = band.thresholdDb;
        s.ratio = std::max(band.ratio, 1.0f);
        s.kneeDb = 6.0f;
        s.rangeDb = std::abs(band.rangeDb);
        s.detector = cfx::DetectorMode::Rms;
        s.rmsWindowMs = 10.0f;
        s.attackMs = band.attackMs;
        s.releaseMs = band.releaseMs;
        s.peakHoldMs = 2.0f;
        s.channels = band.channel == 0 ? cfx::ChannelMode::StereoLinked
                                       : (band.laneMask() == 1 ? cfx::ChannelMode::Left : cfx::ChannelMode::Right);
        s.mix = 1.0f;
        s.lookaheadMs = 0.0f;
        band.proc.process(onesA, onesB, m, ka, kb, s, true);
        const float* gain = band.laneMask() == 2 ? onesB : onesA;
        const float rangeTarget = std::abs(band.rangeDb);
        for (int pos = 0; pos < m; pos += kSub)
        {
            const int len = std::min(kSub, m - pos);
            if (!band.tableSettled || !band.rangeSm.settled(rangeTarget))
                glideTable(band, len, rangeTarget);
            const float range = band.rangeSm.b;
            double e = 0.0;
            for (int i = 0; i < len; ++i)
                e += std::abs(6.0205999f * cfx::dyn::log2Fast(std::max(gain[pos + i], 1.0e-6f)));
            e /= len;
            const double u = range > 0.0f ? std::clamp(e / range, 0.0, 1.0) : 0.0;
            const double x = u * kDynSteps;
            const int k = std::min(static_cast<int>(x), kDynSteps - 1);
            band.live = lerp(band.table[static_cast<size_t>(k)], band.table[static_cast<size_t>(k + 1)], x - k);
            band.chain.set(&band.live, 1, len);
            band.chain.process(a + pos, b + pos, len, band.laneMask());
        }
    }

    // -------------------------------------------------------------------------------------------------------------
    const int on, mix, outDb, phase, resolution, autoGain;
    std::array<BandIndex, kBands> index{};
    std::array<Band, kBands> bands;
    double sr = 48000.0;
    bool offline = false;
    bool prepared = false;
    bool registered = false;
    bool dirty = false;
    bool designed = false;
    cfx::DesignQuality qualityNow = cfx::DesignQuality::Minimum;

    // Output stage.
    float outTarget = 0.0f, autoTarget = 0.0f, wetMix = 1.0f, mixValue = -1.0f;
    bool autoOn = false;
    float outDbNow = 0.0f, gainLinear = -1.0f, outSmooth = -1.0f;
    Smooth2 outDb2, mixDb2;
    float duck = 1.0f, duckTarget = 1.0f;

    // Linear phase.
    bool wantFir = false, wantFirPrev = false, firDirty = false;
    int path = 0; // 0 IIR, 1 FIR
    int resolutionNow = 12;
    int fadeInHold = 0; // samples the wet fade-in still waits for the convolver's first output
    int firLog2 = 12;   // length class the running FIR path (and the dry delay) uses
    int cooldown = 0;
    std::uint64_t postedVersion = 0, seenVersion = 0;
    std::atomic<std::uint64_t> completed{0};
    cfx::LatestRequestSlot<FirRequest> requests;
    cfx::PartitionedConvolver convolver;
    cfx::Lookahead dryLook, keyLook;
    std::atomic<int> reportedLatency{0};
    std::atomic<bool> latencyChanged{false};

    // Spectrum view.
    std::unique_ptr<Fft> specFft;
    std::vector<float> specWork, ring;
    size_t ringPos = 0;
};
} // namespace

ModuleDescriptor eqProEffect()
{
    auto d =
        effectDescriptor("fx.eq_pro", "Pro EQ", "EQ", CpuClass::Medium,
                         "Sixteen-band precision parametric equaliser: bell, shelves, tilt, band-pass, notch, all-pass and "
                         "low / high cuts up to 96 dB per octave (Butterworth, Bessel, Linkwitz-Riley or resonant), every "
                         "curve matched to its analog prototype up to 20 kHz so that nothing cramps near Nyquist. Each band "
                         "works on stereo, left, right, mid or side and can be dynamic (gain follows the key level above or "
                         "below a threshold; internal or external key). Minimum, natural or linear phase (the latency is "
                         "reported); analytic response curve and output spectrum as views; automatic output gain.");
    auto& p = d.params;
    addOn(p, "ep", "Pro EQ", "fx_ep", 1.0f);
    addF(p, "ep_out", "Pro EQ Output", "fx_ep", -24.0f, 24.0f, 0.0f, "dB");
    p.push_back(params::choice("fx_ep_phase", "Pro EQ Phase", "fx_ep", {"Minimum", "Natural", "Linear"}, 0));
    p.push_back(params::choice("fx_ep_res", "Pro EQ Linear Resolution", "fx_ep", resolutionNames(), 2));
    p.push_back(params::toggle("fx_ep_auto", "Pro EQ Auto Gain", "fx_ep", false));
    p.push_back(params::integer("fx_pos_ep", "Pro EQ Slot", "fx_ep_2", 1, kNumModuleFx, kNumFx + 5));
    d.pages.push_back(
        {"fx_ep", "Pro EQ", "fx", "Pro EQ", "On, mix, output, phase mode, linear-phase resolution and auto gain."});
    d.pages.push_back({"fx_ep_2", "Pro EQ 2", "fx", "Pro EQ Slot", "The chain slot."});
    for (int b = 0; b < kBands; ++b)
    {
        const std::string n = std::to_string(b + 1);
        const std::string id = "fx_ep_b" + n + "_";
        const std::string name = "Pro EQ Band " + n + " ";
        const std::string page1 = "fx_ep_b" + n, page2 = page1 + "_2", page3 = page1 + "_3";
        p.push_back(params::toggle(id + "on", name + "On", page1, false));
        p.push_back(params::choice(id + "type", name + "Type", page1, typeNames(), 0));
        p.push_back(params::modulatable(params::frequency(id + "freq", name + "Freq", page1, 10.0f, 24000.0f,
                                                          static_cast<float>(defaultFrequency(b)), 0.3f)));
        addF(p, "ep_b" + n + "_gain", name + "Gain", page1, -30.0f, 30.0f, 0.0f, "dB");
        addF(p, "ep_b" + n + "_q", name + "Q", page1, 0.1f, 40.0f, 1.0f, {}, 0.35f);
        p.push_back(params::choice(id + "slope", name + "Slope", page1, slopeNames(), 1));
        p.push_back(params::choice(id + "shape", name + "Cut Shape", page2, shapeNames(), 0));
        p.push_back(params::choice(id + "ch", name + "Channel", page2, channelNames(), 0));
        p.push_back(params::choice(id + "dyn", name + "Dynamic", page2, dynNames(), 0));
        addF(p, "ep_b" + n + "_range", name + "Dynamic Range", page2, -24.0f, 24.0f, -6.0f, "dB");
        addF(p, "ep_b" + n + "_thr", name + "Threshold", page2, -60.0f, 0.0f, -24.0f, "dB");
        addF(p, "ep_b" + n + "_ratio", name + "Ratio", page2, 1.0f, 100.0f, 4.0f, ":1", 0.4f);
        addF(p, "ep_b" + n + "_att", name + "Attack", page3, 0.1f, 500.0f, 10.0f, "ms", 0.35f);
        addF(p, "ep_b" + n + "_rel", name + "Release", page3, 5.0f, 5000.0f, 120.0f, "ms", 0.35f);
        p.push_back(params::choice(id + "key", name + "Key", page3, {"Internal", "External"}, 0));
        d.pages.push_back({page1, "EQ Band " + n, "fx", "Pro EQ Band " + n, "On, type, frequency, gain, Q and slope."});
        d.pages.push_back({page2, "EQ Band " + n + " 2", "fx", "Pro EQ Band " + n + " Dynamics",
                           "Cut shape, channel, dynamic mode, range, threshold and ratio."});
        d.pages.push_back(
            {page3, "EQ Band " + n + " 3", "fx", "Pro EQ Band " + n + " Times", "Attack, release and key source."});
    }
    d.onParam = "fx_ep_on";
    d.positionParam = "fx_pos_ep";
    d.capabilities = {"latency", "response_view"};
    d.views = {"response", "spectrum"};
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.eq_pro")); };
    return d;
}

} // namespace crucible::modules::fx
