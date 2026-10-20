// SPDX-License-Identifier: MIT
// fx.ensemble: a chorus built from bucket-brigade devices (include/crucible/fx/Bbd.h). Two families share one engine:
//
//   * String ensemble: one to three lines fed from the mono sum of the input. Their low-frequency oscillators are spread
//     evenly over one cycle (120 degrees apart with three lines, 180 with two) and the lines are panned across the stereo
//     field by Spread, with equal power, so the sum of the wet signal has the level of one line.
//   * Dimensional I-IV: two lines, one per channel, with fixed rate / depth pairs (four steps of a dimensional chorus), the
//     oscillators half a cycle apart, and the wet outputs crossfed with Crossfeed (negative values invert the other line).
//
// Depth is the peak pitch deviation of the delay modulation in cents: for an oscillator of rate f the delay swing is
// chosen so that the peak speed of the delay line equals 2^(cents / 1200) - 1 (sine and smooth random: swing x / (2 pi f),
// triangle: x / (4 f)). The swing is limited to what the delay and the device allow (the shortest delay of a device is
// stages / (2 x 0.95 x rate)); a deeper request is held at that limit. Rate, depth and delay are smoothed, so turning them
// never steps the delay.
//
// Cost: the devices run at the clock the delay needs (stages / (2 x delay)), and a device costs more the faster its clock,
// so a longer delay or fewer stages is cheaper and darker. The wet bandwidth is Bbd::bandwidthHz(clock), 0.3 x the clock:
// the defaults (256 stages, 8 ms) put the clock near 17 kHz and the bandwidth near 5 kHz, a dark string-ensemble voicing
// that keeps three lines inside the 0.5 % budget; 512 or 1024 stages are brighter and cost more.
//
// Changing the line count, the family or the stage count fades the wet signal out, changes the devices and fades it back in.
#include "FxModuleSupport.h"

#include <crucible/fx/Bbd.h>
#include <crucible/module/LatencyReporter.h>

#include <array>
#include <atomic>
#include <cstdint>

namespace crucible::modules::fx
{
namespace
{
using crucible::fx::Bbd;
using crucible::fx::BbdSettings;

constexpr int kChunk = 16;
constexpr double kTwoPi = 6.283185307179586;
constexpr double kPi = 3.141592653589793;
constexpr int kStageChoices[] = {256, 512, 1024};
constexpr int kMaxLines = 3;

// Rate (Hz) and depth (cents) of the four dimensional steps.
constexpr double kDimRate[4] = {0.25, 0.4, 0.65, 1.0};
constexpr double kDimDepth[4] = {5.0, 8.0, 11.0, 14.0};

struct Block
{
    int family = 0; // 0 string ensemble, 1 dimensional
    int lines = 3;
    int stages = 256;
    int shape = 0;
    double rate = 0.6;
    double depthCents = 10.0;
    double delay = 0.01;
    float mix = 0.5f;
    float spread = 0.7f;
    float crossfeed = -0.35f;
    float noiseDb = -100.0f;
    bool compander = true;
};

struct Config
{
    int family = 0, lines = 3, stages = 256, shape = 1;
    bool operator==(const Config& o) const
    {
        return family == o.family && lines == o.lines && stages == o.stages && shape == o.shape;
    }
    bool operator!=(const Config& o) const { return !(*this == o); }
};

// Smooth random oscillator: a new random level in [-1, 1] every half period, joined by raised-cosine segments.
struct RandomLfo
{
    std::uint64_t state = 1;
    double a = 0.0, b = 0.0, ph = 0.0;
    double next()
    {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return static_cast<double>(state >> 11) * (2.0 / 9007199254740992.0) - 1.0;
    }
    void seed(std::uint64_t s)
    {
        state = s * 0x9E3779B97F4A7C15ull + 0x2545F4914F6CDD1Dull;
        if (state == 0)
            state = 1;
        a = next();
        b = next();
        ph = 0.0;
    }
    // Advances by `segments` (rate x 2 x samples / sampleRate) and returns the level.
    double advance(double segments)
    {
        ph += segments;
        while (ph >= 1.0)
        {
            ph -= 1.0;
            a = b;
            b = next();
        }
        return a + (b - a) * (0.5 - 0.5 * std::cos(kPi * ph));
    }
};

class Instance final : public EffectModule, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_ensemble_mix")), mode(localIndex(d, "fx_ensemble_mode")),
          lines(localIndex(d, "fx_ensemble_lines")), shape(localIndex(d, "fx_ensemble_shape")),
          rate(localIndex(d, "fx_ensemble_rate")), depth(localIndex(d, "fx_ensemble_depth")),
          delay(localIndex(d, "fx_ensemble_delay")), stages(localIndex(d, "fx_ensemble_stages")),
          spread(localIndex(d, "fx_ensemble_spread")), crossfeed(localIndex(d, "fx_ensemble_crossfeed")),
          noise(localIndex(d, "fx_ensemble_noise")), compander(localIndex(d, "fx_ensemble_compander"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        sampleRate = c.sampleRate;
        for (int k = 0; k < kMaxLines; ++k)
        {
            bbd[k].prepare(sampleRate);
            seeds[k] = static_cast<std::uint64_t>(c.seed) * 16u + static_cast<std::uint64_t>(k) + 7u;
        }
        fadeStep = static_cast<float>(1.0 / (0.015 * sampleRate));
        smoothMix = static_cast<float>(1.0 - std::exp(-1.0 / (0.005 * sampleRate)));
        alpha40 = 1.0 - std::exp(-static_cast<double>(kChunk) / (0.04 * sampleRate));
        alpha100 = 1.0 - std::exp(-static_cast<double>(kChunk) / (0.1 * sampleRate));
        reset();
    }

    void reset() override
    {
        for (int k = 0; k < kMaxLines; ++k)
        {
            bbd[k].reset();
            bbd[k].setSeed(seeds[k] + 1000u);
            rnd[k].seed(seeds[k]);
        }
        needInit = true;
        centsCached = -1.0;
        spreadCached = -1.0f;
        crossCached = 2.0f;
        linesCached = 0;
        snapGains = true;
        switching = false;
        hold = 0;
        fade = 1.0f;
        snapMix = true;
        phase = 0.0;
        cleared = true;
    }

    void deactivate() override
    {
        if (!cleared)
            reset();
    }

    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        cleared = false;
        const Block b = readBlock(in);
        const Config want{b.family, b.lines, b.stages, b.shape};
        if (needInit)
        {
            active = want;
            applyConfig(b);
            rateS = b.rate;
            centsCached = b.depthCents;
            speedTarget = centsToSpeed(b.depthCents);
            xS = speedTarget;
            delayS = b.delay;
            needInit = false;
        }
        if (!switching && want != active)
            switching = true;
        if (noiseCached != b.noiseDb || companderCached != b.compander)
            applyDeviceSettings(b);
        tail.store(b.delay + 0.02, std::memory_order_relaxed);

        for (int pos = 0; pos < n; pos += kChunk)
        {
            const int m = std::min(kChunk, n - pos);
            const float fade0 = fade;
            bool applied = false;
            if (switching)
            {
                fade -= static_cast<float>(m) * fadeStep;
                if (fade <= 0.0f)
                {
                    fade = 0.0f;
                    applied = true;
                }
            }
            else if (hold > 0)
                hold -= m;
            else if (fade < 1.0f)
                fade = std::min(1.0f, fade + static_cast<float>(m) * fadeStep);
            chunk(l + pos, r + pos, m, b, fade0, fade);
            // The wet signal reached silence: the next chunk runs the new configuration on empty devices.
            if (applied)
            {
                active = want;
                applyConfig(b);
                switching = false;
                // New devices are empty: keep the wet signal off until the delay line has filled, so the start of the
                // sound does not arrive as a step.
                hold = static_cast<int>((b.delay + 0.004) * sampleRate);
            }
        }
    }

    int latencySamples() const override { return 0; }
    double tailSeconds() const override { return tail.load(std::memory_order_relaxed); }

private:
    static double centsToSpeed(double cents) { return std::pow(2.0, cents / 1200.0) - 1.0; }

    Block readBlock(const EffectInputs& in) const
    {
        Block b;
        const int md = std::clamp(in.choice(mode), 0, 4);
        b.family = md == 0 ? 0 : 1;
        b.lines = b.family == 1 ? 2 : std::clamp(in.choice(lines), 0, 2) + 1;
        b.stages = kStageChoices[std::clamp(in.choice(stages), 0, 2)];
        b.shape = std::clamp(in.choice(shape), 0, 2);
        b.rate = md == 0 ? static_cast<double>(in.value(rate)) : kDimRate[md - 1];
        b.depthCents = md == 0 ? static_cast<double>(in.value(depth)) : kDimDepth[md - 1];
        b.delay = 0.001 * static_cast<double>(in.value(delay));
        b.mix = in.value(mix);
        b.spread = in.value(spread);
        b.crossfeed = in.value(crossfeed);
        b.noiseDb = in.value(noise);
        b.compander = in.on(compander);
        return b;
    }

    void applyDeviceSettings(const Block& b)
    {
        BbdSettings s;
        s.stages = active.stages;
        s.driveDb = 0.0f;
        s.compander = b.compander && b.noiseDb > -99.0f; // without noise the compander is transparent
        s.noiseDb = b.noiseDb <= -99.0f ? -120.0f : b.noiseDb;
        s.breathing = 0.3f;
        s.glideMs = 0.5f;
        for (auto& d : bbd)
            d.setSettings(s);
        noiseCached = b.noiseDb;
        companderCached = b.compander;
    }

    // New devices: settings, clocks at the current delay, empty rings, oscillators spread over the cycle.
    void applyConfig(const Block& b)
    {
        applyDeviceSettings(b);
        spreadCached = -1.0f;
        crossCached = 2.0f;
        linesCached = 0;
        snapGains = true;
        for (int k = 0; k < kMaxLines; ++k)
        {
            bbd[k].setDelaySeconds(std::max(b.delay - extra(b.delay), bbd[k].minDelaySeconds()));
            bbd[k].reset();
            rnd[k].seed(seeds[k]);
        }
        delayS = b.delay;
    }

    double extra(double clockDelay) const
    {
        const double clock = Bbd::clockForDelay(active.stages, std::max(clockDelay, 1.0e-4));
        return 7.0 / sampleRate + 5.2 / (kTwoPi * Bbd::kFilterCutoffRatio * std::min(clock, 0.95 * sampleRate));
    }

    // Oscillator offset of line k (cycles).
    double offset(int k) const
    {
        return active.family == 1 ? 0.5 * k : static_cast<double>(k) / static_cast<double>(active.lines);
    }

    static double wave(int shapeIndex, double p)
    {
        p -= std::floor(p);
        if (shapeIndex == 0)
            return 1.0 - 4.0 * std::abs(p - 0.5); // triangle
        return std::sin(kTwoPi * p);
    }

    void chunk(float* l, float* r, int m, const Block& b, float fade0, float fade1)
    {
        // Smoothing of rate, depth and delay (per chunk).
        const double a40 = m == kChunk ? alpha40 : 1.0 - std::exp(-static_cast<double>(m) / (0.04 * sampleRate));
        const double a100 = m == kChunk ? alpha100 : 1.0 - std::exp(-static_cast<double>(m) / (0.1 * sampleRate));
        if (b.depthCents != centsCached)
        {
            centsCached = b.depthCents;
            speedTarget = centsToSpeed(b.depthCents);
        }
        rateS += (b.rate - rateS) * a100;
        xS += (speedTarget - xS) * a40;
        delayS += (b.delay - delayS) * a40;
        phase += rateS * static_cast<double>(m) / sampleRate;
        phase -= std::floor(phase);

        const double f = std::max(rateS, 1.0e-3);
        double swing = active.shape == 0 ? xS / (4.0 * f) : xS / (kTwoPi * f);
        const double clockSlow = Bbd::clockForDelay(active.stages, std::max(delayS, 1.0e-4));
        const double limit =
            delayS - (7.0 / sampleRate + 5.2 / (kTwoPi * Bbd::kFilterCutoffRatio * std::min(clockSlow, 0.95 * sampleRate))) -
            bbd[0].minDelaySeconds();
        swing = std::min(swing, std::max(limit, 0.0));

        float in[2][kChunk], out[kMaxLines][kChunk];
        if (active.family == 1)
            for (int i = 0; i < m; ++i)
            {
                in[0][i] = l[i];
                in[1][i] = r[i];
            }
        else
            for (int i = 0; i < m; ++i)
                in[0][i] = 0.5f * (l[i] + r[i]);
        for (int k = 0; k < active.lines; ++k)
        {
            double u;
            if (active.shape == 2)
                u = rnd[k].advance(2.0 * rateS * static_cast<double>(m) / sampleRate);
            else
                u = wave(active.shape, phase + offset(k));
            const double wanted = delayS + swing * u;
            bbd[k].setDelaySeconds(std::max(wanted - extra(wanted), bbd[k].minDelaySeconds()));
            const float* src = active.family == 1 ? in[k] : in[0];
            bbd[k].processBlock(src, out[k], m);
        }

        // Wet gains per line and output: targets are set when the controls change, the gains move toward them per sample.
        if (active.family == 0)
        {
            if (b.spread != spreadCached || active.lines != linesCached)
            {
                spreadCached = b.spread;
                linesCached = active.lines;
                float sl = 0.0f, sr = 0.0f;
                for (int k = 0; k < active.lines; ++k)
                {
                    const double p =
                        active.lines == 1 ? 0.0 : (2.0 * k / (active.lines - 1) - 1.0) * static_cast<double>(b.spread);
                    target[kLeft + k] = static_cast<float>(std::cos((p + 1.0) * kPi * 0.25));
                    target[kRight + k] = static_cast<float>(std::sin((p + 1.0) * kPi * 0.25));
                    sl += target[kLeft + k] * target[kLeft + k];
                    sr += target[kRight + k] * target[kRight + k];
                }
                target[kNorm] = 1.0f / std::sqrt(std::max(sl, sr));
                gainsMoving = true;
            }
        }
        else if (b.crossfeed != crossCached)
        {
            crossCached = b.crossfeed;
            target[kCross] = b.crossfeed;
            target[kNorm] = 1.0f / std::sqrt(1.0f + b.crossfeed * b.crossfeed);
            gainsMoving = true;
        }
        if (b.mix != target[kMix])
        {
            target[kMix] = b.mix;
            gainsMoving = true;
        }
        if (snapGains || snapMix)
        {
            for (int i = 0; i < kSlots; ++i)
                if (i == kMix ? snapMix : snapGains)
                    stage1[i] = stage2[i] = target[i];
            snapGains = snapMix = false;
            gainsMoving = !arrived();
        }
        const float fgStep = (fade1 - fade0) / static_cast<float>(m);
        float g = fade0;
        for (int i = 0; i < m; ++i)
        {
            if (gainsMoving)
                stepGains();
            const float* gl = stage2 + kLeft;
            const float* gr = stage2 + kRight;
            const float norm = stage2[kNorm], cross = stage2[kCross], mixNow = stage2[kMix];
            g += fgStep;
            float wl, wr;
            if (active.family == 1)
            {
                wl = (out[0][i] + cross * out[1][i]) * norm;
                wr = (out[1][i] + cross * out[0][i]) * norm;
            }
            else
            {
                wl = wr = 0.0f;
                for (int k = 0; k < active.lines; ++k)
                {
                    wl += gl[k] * out[k][i];
                    wr += gr[k] * out[k][i];
                }
                wl *= norm;
                wr *= norm;
            }
            const float dl = l[i], dr = r[i];
            const float gs = g * g * (3.0f - 2.0f * g); // smoothstep: the fade has no kink at either end
            l[i] = dl + (wl * gs - dl) * mixNow;
            r[i] = dr + (wr * gs - dr) * mixNow;
        }
    }

    // One sample of the smoothing of the wet gains and the mix toward their targets: two cascaded one-poles of about 5 ms,
    // so that a value that arrives once per host block moves smoothly in value and slope whatever the block size.
    void stepGains()
    {
        for (int i = 0; i < kSlots; ++i)
        {
            stage1[i] += (target[i] - stage1[i]) * smoothMix;
            stage2[i] += (stage1[i] - stage2[i]) * smoothMix;
        }
        if (arrived())
        {
            for (int i = 0; i < kSlots; ++i)
                stage1[i] = stage2[i] = target[i];
            gainsMoving = false;
        }
    }

    bool arrived() const
    {
        for (int i = 0; i < kSlots; ++i)
            if (std::abs(target[i] - stage1[i]) >= 1.0e-7f || std::abs(target[i] - stage2[i]) >= 1.0e-7f)
                return false;
        return true;
    }

    const int mix, mode, lines, shape, rate, depth, delay, stages, spread, crossfeed, noise, compander;

    double sampleRate = 48000.0;
    Bbd bbd[kMaxLines];
    RandomLfo rnd[kMaxLines];
    std::uint64_t seeds[kMaxLines] = {1, 2, 3};
    Config active;
    bool needInit = true, switching = false, cleared = true;
    int hold = 0;
    float fade = 1.0f, fadeStep = 0.001f, smoothMix = 0.01f;
    double phase = 0.0, rateS = 0.6, xS = 0.0, delayS = 0.01;
    double alpha40 = 0.01, alpha100 = 0.004, centsCached = -1.0, speedTarget = 0.0;
    // Smoothed values: the per-line left and right gains, the output norm, the crossfeed and the mix (stage1 feeds stage2,
    // which is used).
    static constexpr int kLeft = 0, kRight = kMaxLines, kNorm = 2 * kMaxLines, kCross = kNorm + 1, kMix = kNorm + 2,
                         kSlots = kNorm + 3;
    float stage1[kSlots] = {}, stage2[kSlots] = {}, target[kSlots] = {};
    float spreadCached = -1.0f, crossCached = 2.0f;
    int linesCached = 0;
    bool gainsMoving = false, snapGains = true, snapMix = true;
    float noiseCached = 0.0f;
    bool companderCached = true;
    std::atomic<double> tail{0.0};
};
} // namespace

ModuleDescriptor ensembleEffect()
{
    auto d = effectDescriptor(
        "fx.ensemble", "Ensemble", "Modulation", CpuClass::Medium,
        "Chorus made of bucket-brigade devices. String ensemble runs one to three lines whose oscillators are "
        "spread evenly over a cycle and panned across the stereo field; Dimensional I-IV runs two lines, one per "
        "channel, at four fixed rate and depth steps with a crossfeed between them. Depth is the peak pitch "
        "deviation in cents; the oscillator is a triangle, a sine or smooth random. The darkness of the wet "
        "signal follows the device clock (stages and delay), and the device noise and compander can be set. "
        "No latency.");
    auto& p = d.params;
    addOn(p, "ensemble", "Ensemble", "fx_ensemble", 0.5f);
    p.push_back(params::choice("fx_ensemble_mode", "Ensemble Mode", "fx_ensemble",
                               {"String ensemble", "Dimensional I", "Dimensional II", "Dimensional III", "Dimensional IV"},
                               0));
    p.push_back(params::choice("fx_ensemble_lines", "Ensemble Lines", "fx_ensemble", {"1", "2", "3"}, 2));
    p.push_back(
        params::choice("fx_ensemble_shape", "Ensemble Shape", "fx_ensemble", {"Triangle", "Sine", "Smooth random"}, 1));
    addF(p, "ensemble_rate", "Ensemble Rate", "fx_ensemble", 0.05f, 8.0f, 0.6f, "Hz", 0.5f);
    addF(p, "ensemble_depth", "Ensemble Depth", "fx_ensemble_2", 0.0f, 30.0f, 10.0f, "cents");
    addF(p, "ensemble_delay", "Ensemble Delay", "fx_ensemble_2", 3.0f, 40.0f, 8.0f, "ms", 0.5f);
    p.push_back(params::choice("fx_ensemble_stages", "Ensemble Stages", "fx_ensemble_2", {"256", "512", "1024"}, 0));
    addF(p, "ensemble_spread", "Ensemble Spread", "fx_ensemble_2", 0.0f, 1.0f, 0.7f);
    addF(p, "ensemble_crossfeed", "Ensemble Crossfeed", "fx_ensemble_2", -1.0f, 1.0f, -0.35f);
    addF(p, "ensemble_noise", "Ensemble Noise", "fx_ensemble_2", -100.0f, -40.0f, -100.0f, "dB");
    p.push_back(params::toggle("fx_ensemble_compander", "Ensemble Compander", "fx_ensemble_3", true));
    p.push_back(params::integer("fx_pos_ensemble", "Ensemble Slot", "fx_ensemble_3", 1, kNumModuleFx, kNumModuleFx));
    d.onParam = "fx_ensemble_on";
    d.positionParam = "fx_pos_ensemble";
    d.capabilities = {"latency"};
    d.pages.push_back(
        {"fx_ensemble", "Ensemble", "fx", "Ensemble", "On, mix, mode, line count, oscillator shape and rate."});
    d.pages.push_back({"fx_ensemble_2", "Ensemble 2", "fx", "Ensemble Devices",
                       "Depth, delay, stage count, stereo spread, crossfeed and device noise."});
    d.pages.push_back(
        {"fx_ensemble_3", "Ensemble 3", "fx", "Ensemble Compander", "Compander of the devices, and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.ensemble")); };
    return d;
}

} // namespace crucible::modules::fx
