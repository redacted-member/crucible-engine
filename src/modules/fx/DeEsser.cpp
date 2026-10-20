// SPDX-License-Identifier: MIT
// fx.deesser: a de-esser on the dynamics core's gain computer (include/crucible/fx/Dynamics.h).
//
// The detector reads a band of the signal: a band-pass of the set width in octaves around the centre frequency, or
// everything above it (high-pass). The band is cut by a short linear-phase FIR (a Kaiser-windowed ideal band-pass, about 1
// ms of delay on each side of its centre), so the band signal is in phase with the input delayed by the FIR's centre. Its
// level is measured as a short true RMS and compared with a threshold, either absolute (dB re full scale) or relative (the
// band level against the full-band level, so the threshold does not depend on how loud the voice is). The reduction follows
// the compressor curve with a soft knee, limited to the range, with its own attack and release in the dB domain.
//
// Split-band mode adds the missing part of the band to the delayed input: out = x + (g - 1) * band. With no reduction (g =
// 1) the output is the delayed input exactly, and because the band signal is in phase with x the band is lowered by g at
// every frequency of its flat top (the edges are the FIR's transitions, 6 dB down at the set band edges, about 1.4 kHz wide
// at 48 kHz). Away from the band the band signal is below -45 dB, so the output changes by that times (1 - g) at most.
// Wide-band mode multiplies the whole signal by the gain. The gain is derived at the FIR's centre and applied to the signal
// and the band delayed by a further lookahead, so it arrives before the sibilant. Listen outputs the band. Channel modes
// follow include/crucible/fx/ChannelMode.h. The reported latency is the FIR's half length plus the lookahead.
#include "FxModuleSupport.h"

#include <crucible/fx/ChannelMode.h>
#include <crucible/fx/Dynamics.h>
#include <crucible/fx/Lookahead.h>
#include <crucible/Simd.h>
#include <crucible/module/LatencyReporter.h>

#include <array>
#include <atomic>

namespace crucible::modules::fx
{
namespace
{
constexpr int kChunk = 64;
constexpr float kMaxLookaheadMs = 2.0f;
constexpr double kDetectorMs = 2.0;
constexpr double kKernelMs = 3.0; // time constant of each of the two poles that move the band kernel to a new design
constexpr float kKneeDb = 2.0f;
constexpr float kRelativeFloorDb = -70.0f; // below this full-band level the relative threshold does not act
constexpr double kKaiserBeta = 4.5;        // about 50 dB of stop-band attenuation
constexpr double kPi = 3.14159265358979323846;

inline float onePole(double timeMs, double sampleRate)
{
    return 1.0f - static_cast<float>(std::exp(-1.0 / (timeMs * 0.001 * sampleRate)));
}

double besselI0(double x)
{
    double sum = 1.0, term = 1.0;
    const double q = 0.25 * x * x;
    for (int k = 1; k < 40; ++k)
    {
        term *= q / (static_cast<double>(k) * k);
        sum += term;
        if (term < 1.0e-12 * sum)
            break;
    }
    return sum;
}

// Symmetric FIR on a doubled ring, evaluated with four-wide vectors. The newest sample is hist[pos]; hist[pos + k] is k
// samples old, so a symmetric kernel needs no reversal and the centre tap is hist[pos + half].
class BandFilter
{
public:
    void prepare(int halfLength, double sampleRate)
    {
        half = halfLength;
        alpha = 1.0f - static_cast<float>(std::exp(-1.0 / (kKernelMs * 0.001 * sampleRate)));
        taps = 2 * half + 1;
        padded = (taps + 3) & ~3;
        ring = 1;
        while (ring < padded + 8)
            ring <<= 1;
        history.assign(static_cast<size_t>(2 * ring), 0.0f);
        kernel.assign(static_cast<size_t>(padded), 0.0f);
        stage1.assign(static_cast<size_t>(padded), 0.0f);
        target.assign(static_cast<size_t>(padded), 0.0f);
        moving = false;
        pos = 0;
    }
    void reset()
    {
        std::fill(history.begin(), history.end(), 0.0f);
        pos = 0;
    }
    // A new kernel is reached by moving every tap toward it per sample through two cascaded poles (kKernelMs each), so the
    // response changes smoothly whatever the host block size. `immediate` jumps to it (after a reset, when there is no
    // signal to disturb).
    void setKernel(const float* h, bool immediate)
    {
        std::copy(h, h + taps, target.begin());
        if (immediate)
        {
            std::copy(h, h + taps, kernel.begin());
            std::copy(h, h + taps, stage1.begin());
            moving = false;
        }
        else
            moving = true;
    }
    int centre() const { return half; }
    // Takes one sample; returns the band output, whose centre is the sample `half` ago (see centreSample()).
    float push(float x)
    {
        if (moving)
            stepKernel();
        pos = pos == 0 ? ring - 1 : pos - 1;
        history[static_cast<size_t>(pos)] = history[static_cast<size_t>(pos + ring)] = x;
        const float* h = history.data() + pos;
        const float* k = kernel.data();
        crucible::simd::Float4 acc = crucible::simd::Float4::load(h) * crucible::simd::Float4::load(k);
        for (int i = 4; i < padded; i += 4)
            acc = acc + crucible::simd::Float4::load(h + i) * crucible::simd::Float4::load(k + i);
        alignas(16) float lanes[4];
        acc.store(lanes);
        return (lanes[0] + lanes[2]) + (lanes[1] + lanes[3]);
    }
    float centreSample() const { return history[static_cast<size_t>(pos + half)]; }

    // push() and centreSample() for n samples: band[i] and centreOut[i] are exactly what push(x[i]) and centreSample()
    // return one sample at a time. The samples of a batch are stored first and the outputs computed four at a time,
    // each with its own accumulator and the same multiplies and adds in the same order as push(). The single-sample
    // path loaded the sample it had just stored (a 16-byte load over a 4-byte store in flight is not forwarded and waits
    // for the store to retire) and had one dependency chain of 25 vector adds per sample. A batch of at most
    // ring - padded samples overwrites only ring slots that no output of the batch reads.
    void pushBlock(const float* x, int n, float* bandOut, float* centreOut)
    {
        using crucible::simd::Float4;
        // While the taps move toward a new design every sample has its own kernel: the single-sample path.
        while (moving && n > 0)
        {
            *bandOut++ = push(*x++);
            *centreOut++ = centreSample();
            --n;
        }
        const int batch = std::min(16, ring - padded);
        int at[16];
        for (int s = 0; s < n; s += batch)
        {
            const int c = std::min(batch, n - s);
            for (int j = 0; j < c; ++j)
            {
                pos = pos == 0 ? ring - 1 : pos - 1;
                history[static_cast<size_t>(pos)] = history[static_cast<size_t>(pos + ring)] = x[s + j];
                at[j] = pos;
            }
            const float* k = kernel.data();
            int j = 0;
            for (; j + 4 <= c; j += 4)
            {
                const float *h0 = history.data() + at[j], *h1 = history.data() + at[j + 1], *h2 = history.data() + at[j + 2],
                            *h3 = history.data() + at[j + 3];
                const Float4 k0 = Float4::load(k);
                Float4 a0 = Float4::load(h0) * k0, a1 = Float4::load(h1) * k0, a2 = Float4::load(h2) * k0,
                       a3 = Float4::load(h3) * k0;
                for (int i = 4; i < padded; i += 4)
                {
                    const Float4 ki = Float4::load(k + i);
                    a0 = a0 + Float4::load(h0 + i) * ki;
                    a1 = a1 + Float4::load(h1 + i) * ki;
                    a2 = a2 + Float4::load(h2 + i) * ki;
                    a3 = a3 + Float4::load(h3 + i) * ki;
                }
                alignas(16) float lanes[4][4];
                a0.store(lanes[0]);
                a1.store(lanes[1]);
                a2.store(lanes[2]);
                a3.store(lanes[3]);
                for (int q = 0; q < 4; ++q)
                    bandOut[s + j + q] = (lanes[q][0] + lanes[q][2]) + (lanes[q][1] + lanes[q][3]);
            }
            for (; j < c; ++j)
            {
                const float* h = history.data() + at[j];
                Float4 acc = Float4::load(h) * Float4::load(k);
                for (int i = 4; i < padded; i += 4)
                    acc = acc + Float4::load(h + i) * Float4::load(k + i);
                alignas(16) float lanes[4];
                acc.store(lanes);
                bandOut[s + j] = (lanes[0] + lanes[2]) + (lanes[1] + lanes[3]);
            }
            for (j = 0; j < c; ++j)
                centreOut[s + j] = history[static_cast<size_t>(at[j] + half)];
        }
    }

private:
    void stepKernel()
    {
        float gap = 0.0f;
        for (int i = 0; i < taps; ++i)
        {
            const auto k = static_cast<size_t>(i);
            stage1[k] += (target[k] - stage1[k]) * alpha;
            kernel[k] += (stage1[k] - kernel[k]) * alpha;
            gap = std::max(gap, std::max(std::abs(target[k] - stage1[k]), std::abs(target[k] - kernel[k])));
        }
        if (gap < 1.0e-8f)
        {
            std::copy(target.begin(), target.begin() + taps, kernel.begin());
            std::copy(target.begin(), target.begin() + taps, stage1.begin());
            moving = false;
        }
    }

    std::vector<float> history, kernel, stage1, target;
    float alpha = 0.05f;
    bool moving = false;
    int half = 1, taps = 3, padded = 4, ring = 8, pos = 0;
};

class Instance final : public EffectModule, public GainReductionMeter, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mode(localIndex(d, "fx_deesser_mode")), detector(localIndex(d, "fx_deesser_detector")),
          frequency(localIndex(d, "fx_deesser_frequency")), width(localIndex(d, "fx_deesser_width")),
          thresholdMode(localIndex(d, "fx_deesser_threshold_mode")), threshold(localIndex(d, "fx_deesser_threshold")),
          relative(localIndex(d, "fx_deesser_relative")), ratio(localIndex(d, "fx_deesser_ratio")),
          range(localIndex(d, "fx_deesser_range")), attack(localIndex(d, "fx_deesser_attack")),
          release(localIndex(d, "fx_deesser_release")), lookahead(localIndex(d, "fx_deesser_lookahead")),
          listen(localIndex(d, "fx_deesser_listen")), channels(localIndex(d, "fx_deesser_channels")),
          link(localIndex(d, "fx_deesser_link"))
    {
        lookaheadDefaultMs = d.params[static_cast<size_t>(lookahead)].defaultValue;
    }

    void prepare(const ModuleContext& c) override
    {
        rate = c.sampleRate;
        halfLength = std::max(8, static_cast<int>(std::lround(0.001 * rate)));
        const int maxDelay = static_cast<int>(std::ceil(kMaxLookaheadMs * 0.001 * rate)) + 2;
        lookSignal.prepare(2, maxDelay, rate);
        lookBand.prepare(2, maxDelay, rate);
        for (auto& f : filter)
            f.prepare(halfLength, rate);
        kernel.assign(static_cast<size_t>(2 * halfLength + 1), 0.0f);
        window.resize(static_cast<size_t>(halfLength + 1));
        const double i0 = besselI0(kKaiserBeta);
        for (int m = 0; m <= halfLength; ++m)
        {
            const double r = static_cast<double>(m) / halfLength;
            window[static_cast<size_t>(m)] = besselI0(kKaiserBeta * std::sqrt(std::max(0.0, 1.0 - r * r))) / i0;
        }
        detectorCoefficient = onePole(kDetectorMs, rate);
        // The lookahead is a parameter the module only sees in process(); until then it assumes the default, so the full
        // latency is reported from prepare() on and stays the same across reset() and enable / disable.
        lookaheadSamples =
            std::clamp(static_cast<int>(std::lround(lookaheadDefaultMs * 0.001 * rate)), 0, lookSignal.maxDelay());
        reset();
    }
    void reset() override
    {
        lookSignal.reset();
        lookBand.reset();
        lookSignal.setDelay(lookaheadSamples);
        lookBand.setDelay(lookaheadSamples);
        for (auto& f : filter)
            f.reset();
        for (auto& s : lane)
            s = Lane{};
        designed = false;
        meter.store(0.0f, std::memory_order_relaxed);
        latency.store(halfLength + lookaheadSamples, std::memory_order_relaxed);
        latencyChanged.store(false, std::memory_order_relaxed);
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
        using namespace crucible::fx;
        dirty = true;
        design(in.value(frequency), in.value(width), in.choice(detector) == 1);
        const int delay =
            std::clamp(static_cast<int>(std::lround(in.value(lookahead) * 0.001 * rate)), 0, lookSignal.maxDelay());
        if (delay != lookaheadSamples)
        {
            lookaheadSamples = delay;
            lookSignal.setDelay(delay);
            lookBand.setDelay(delay);
            latency.store(halfLength + delay, std::memory_order_relaxed);
            latencyChanged.store(true, std::memory_order_relaxed);
        }
        const bool wide = in.choice(mode) == 1, absolute = in.choice(thresholdMode) == 1, listening = in.on(listen);
        const float thresholdDb = absolute ? in.value(threshold) : in.value(relative);
        const float ratioValue = in.value(ratio), rangeDb = in.value(range);
        const float attackCoefficient = onePole(std::max(0.02, static_cast<double>(in.value(attack))), rate);
        const float releaseCoefficient = onePole(std::max(0.5, static_cast<double>(in.value(release))), rate);
        const ChannelMode channelMode =
            static_cast<ChannelMode>(std::clamp(in.choice(channels), 0, static_cast<int>(ChannelMode::Count) - 1));
        const float linkAmount = effectiveLink(channelMode, in.value(link) * 0.01f);
        const bool active0 = laneActive(channelMode, 0), active1 = laneActive(channelMode, 1);
        float maxReduction = 0.0f;

        float* bandBuffer[2] = {band[0].data(), band[1].data()};
        float* centreBuffer[2] = {centre[0].data(), centre[1].data()};
        for (int pos = 0; pos < n; pos += kChunk)
        {
            const int m = std::min(kChunk, n - pos);
            float* a = l + pos;
            float* b = r + pos;
            encode(channelMode, a, b, m);
            filter[0].pushBlock(a, m, band[0].data(), centre[0].data());
            filter[1].pushBlock(b, m, band[1].data(), centre[1].data());
            for (int i = 0; i < m; ++i)
            {
                for (int k = 0; k < 2; ++k)
                {
                    Lane& s = lane[k];
                    const float bandSample = band[k][static_cast<size_t>(i)], full = centre[k][static_cast<size_t>(i)];
                    s.bandPower += (bandSample * bandSample - s.bandPower) * detectorCoefficient;
                    s.fullPower += (full * full - s.fullPower) * detectorCoefficient;
                }
                // The gain computer runs every fourth sample (12 kHz at 48 kHz, far above the detector's 2 ms); the
                // ballistics below smooth its target at the full rate.
                if ((i & 3) == 0)
                {
                    float level[2];
                    for (int k = 0; k < 2; ++k)
                    {
                        const Lane& s = lane[k];
                        const float bandDb = dyn::powerToDb(s.bandPower);
                        if (absolute)
                            level[k] = bandDb;
                        else
                        {
                            const float fullDb = dyn::powerToDb(s.fullPower);
                            level[k] = fullDb > kRelativeFloorDb ? bandDb - fullDb : -200.0f;
                        }
                    }
                    linkLevels(level[0], level[1], linkAmount);
                    for (int k = 0; k < 2; ++k)
                    {
                        const bool processed = k == 0 ? active0 : active1;
                        const float target =
                            processed ? dyn::compressorReduction(level[k], thresholdDb, ratioValue, kKneeDb) : 0.0f;
                        lane[k].target = target > rangeDb ? rangeDb : target;
                    }
                }
                for (int k = 0; k < 2; ++k)
                {
                    Lane& s = lane[k];
                    s.reduction +=
                        (s.target - s.reduction) * (s.target > s.reduction ? attackCoefficient : releaseCoefficient);
                    maxReduction = std::max(maxReduction, s.reduction);
                    gain[k][static_cast<size_t>(i)] = dyn::dbToGain(-s.reduction);
                }
            }
            lookSignal.process(centreBuffer, m);
            lookBand.process(bandBuffer, m);
            if (listening)
                for (int i = 0; i < m; ++i)
                {
                    a[i] = bandBuffer[0][i];
                    b[i] = bandBuffer[1][i];
                }
            else if (wide)
                for (int i = 0; i < m; ++i)
                {
                    a[i] = centreBuffer[0][i] * gain[0][static_cast<size_t>(i)];
                    b[i] = centreBuffer[1][i] * gain[1][static_cast<size_t>(i)];
                }
            else
                for (int i = 0; i < m; ++i)
                {
                    a[i] = centreBuffer[0][i] + (gain[0][static_cast<size_t>(i)] - 1.0f) * bandBuffer[0][i];
                    b[i] = centreBuffer[1][i] + (gain[1][static_cast<size_t>(i)] - 1.0f) * bandBuffer[1][i];
                }
            decode(channelMode, a, b, m);
        }
        meter.store(maxReduction, std::memory_order_relaxed);
    }

    int meterBands() const override { return 1; }
    float gainReductionDb(int) const override { return meter.load(std::memory_order_relaxed); }
    int latencySamples() const override { return latency.load(std::memory_order_relaxed); }
    bool takeLatencyChanged() override { return latencyChanged.exchange(false, std::memory_order_relaxed); }

private:
    struct Lane
    {
        float bandPower = 0.0f, fullPower = 0.0f, reduction = 0.0f, target = 0.0f;
    };

    // Windowed ideal band-pass between low and high (high-pass: up to Nyquist), designed when the band changes.
    void design(float centreHz, float widthOctaves, bool highPass)
    {
        const double nyquist = 0.5 * rate;
        double low = highPass ? centreHz : centreHz * std::pow(2.0, -0.5 * widthOctaves);
        double high = highPass ? nyquist : std::min(centreHz * std::pow(2.0, 0.5 * widthOctaves), nyquist);
        low = std::clamp(low, 100.0, 0.9 * nyquist);
        high = std::max(high, std::min(low * 1.2, nyquist));
        if (designed && low == lastLow && high == lastHigh)
            return;
        const double wl = kPi * low / nyquist, wh = kPi * high / nyquist;
        for (int m = 0; m <= halfLength; ++m)
        {
            const double ideal = m == 0 ? (wh - wl) / kPi : (std::sin(wh * m) - std::sin(wl * m)) / (kPi * m);
            const float v = static_cast<float>(ideal * window[static_cast<size_t>(m)]);
            kernel[static_cast<size_t>(halfLength - m)] = v;
            kernel[static_cast<size_t>(halfLength + m)] = v;
        }
        for (auto& f : filter)
            f.setKernel(kernel.data(), !designed);
        lastLow = low;
        lastHigh = high;
        designed = true;
    }

    const int mode, detector, frequency, width, thresholdMode, threshold, relative, ratio, range, attack, release, lookahead,
        listen, channels, link;
    double rate = 48000.0;
    int halfLength = 48, lookaheadSamples = 0;
    float lookaheadDefaultMs = 0.0f;
    BandFilter filter[2];
    std::vector<float> kernel, window;
    crucible::fx::Lookahead lookSignal, lookBand;
    std::array<std::array<float, kChunk>, 2> band{}, centre{}, gain{};
    Lane lane[2];
    float detectorCoefficient = 0.01f;
    double lastLow = 0.0, lastHigh = 0.0;
    bool designed = false, dirty = false;
    std::atomic<float> meter{0.0f};
    std::atomic<int> latency{0};
    std::atomic<bool> latencyChanged{false};
};
} // namespace

ModuleDescriptor deEsserEffect()
{
    auto d = effectDescriptor(
        "fx.deesser", "De-esser", "Dynamics", CpuClass::Light,
        "De-esser. The detector reads a band (a band-pass of 0.5 to 3 octaves around 2 to 16 kHz, or "
        "everything above the frequency) as a short true RMS and compares it with an absolute threshold or "
        "with the band's share of the full-band level, so the setting does not depend on how loud the "
        "voice is. Split-band mode lowers only that band, cut by a linear-phase FIR so that it stays in phase with "
        "the signal (with no reduction the output is the delayed input exactly), wide-band mode lowers the whole signal. "
        "Ratio, "
        "maximum reduction, attack and release, lookahead of up to 2 ms (reported as latency), listen to the "
        "detector band, channel mode with link. Gain reduction is published for meters.");
    auto& p = d.params;
    p.push_back(params::toggle("fx_deesser_on", "De-esser On", "fx_deesser", false));
    p.push_back(params::choice("fx_deesser_mode", "De-esser Mode", "fx_deesser", {"Split-band", "Wide-band"}, 0));
    p.push_back(params::choice("fx_deesser_detector", "De-esser Detector", "fx_deesser", {"Band-pass", "High-pass"}, 0));
    addHz(p, "deesser_frequency", "De-esser Frequency", "fx_deesser", 2000.0f, 16000.0f, 6500.0f, 0.5f);
    addF(p, "deesser_width", "De-esser Width", "fx_deesser", 0.5f, 3.0f, 2.0f, "oct");
    p.push_back(
        params::choice("fx_deesser_threshold_mode", "De-esser Threshold Mode", "fx_deesser_2", {"Relative", "Absolute"}, 0));
    addF(p, "deesser_threshold", "De-esser Threshold", "fx_deesser_2", -60.0f, 0.0f, -30.0f, "dB");
    addF(p, "deesser_relative", "De-esser Relative Threshold", "fx_deesser_2", -60.0f, 0.0f, -10.0f, "dB");
    addF(p, "deesser_ratio", "De-esser Ratio", "fx_deesser_2", 1.0f, 100.0f, 8.0f, ":1", 0.4f);
    addF(p, "deesser_range", "De-esser Range", "fx_deesser_2", 0.0f, 18.0f, 12.0f, "dB");
    addF(p, "deesser_attack", "De-esser Attack", "fx_deesser_3", 0.05f, 20.0f, 1.0f, "ms", 0.4f);
    addF(p, "deesser_release", "De-esser Release", "fx_deesser_3", 5.0f, 500.0f, 60.0f, "ms", 0.4f);
    addF(p, "deesser_lookahead", "De-esser Lookahead", "fx_deesser_3", 0.0f, 2.0f, 1.0f, "ms");
    p.push_back(params::toggle("fx_deesser_listen", "De-esser Listen", "fx_deesser_3", false));
    p.push_back(params::integer("fx_pos_deesser", "De-esser Slot", "fx_deesser_3", 1, kNumModuleFx, kNumModuleFx));
    {
        std::vector<std::string> names;
        for (int i = 0; i < static_cast<int>(crucible::fx::ChannelMode::Count); ++i)
            names.emplace_back(crucible::fx::channelModeName(static_cast<crucible::fx::ChannelMode>(i)));
        p.push_back(params::choice("fx_deesser_channels", "De-esser Channels", "fx_deesser_4", names, 0));
    }
    addF(p, "deesser_link", "De-esser Link", "fx_deesser_4", 0.0f, 100.0f, 100.0f, "%");
    d.onParam = "fx_deesser_on";
    d.positionParam = "fx_pos_deesser";
    d.capabilities = {"gain_reduction", "latency"};
    d.pages.push_back({"fx_deesser", "De-esser", "fx", "De-esser", "On, mode, detector band, frequency and width."});
    d.pages.push_back({"fx_deesser_2", "De-esser 2", "fx", "De-esser Threshold",
                       "Threshold mode, absolute and relative threshold, ratio and range."});
    d.pages.push_back(
        {"fx_deesser_3", "De-esser 3", "fx", "De-esser Timing", "Attack, release, lookahead, listen and the chain slot."});
    d.pages.push_back({"fx_deesser_4", "De-esser 4", "fx", "De-esser Stereo", "Channel mode and link."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.deesser")); };
    return d;
}

} // namespace crucible::modules::fx
