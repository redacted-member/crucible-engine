// SPDX-License-Identifier: MIT
// fx.flanger2: a through-zero flanger.
//
//   * Two paths from the same input. The reference path is a fixed delay of R samples. The modulated path is a delay of
//     R + tau, read with the Kaiser-windowed sinc of include/crucible/fx/FractionalDelay.h (16 taps), where the offset
//     tau = Manual + Depth x u (milliseconds, limited to +-5 ms) moves with an oscillator or with the envelope of the input.
//     Both paths are delayed by R, which is the reported latency (the shortest delay a sinc read can take, plus the 5 ms of
//     travel on either side of zero), so tau can cross zero and go negative as it does between two tape machines: the
//     modulated path then runs ahead of the reference one and nothing in the signal path switches.
//   * The output is reference + Mix x (modulated - reference). At Mix 0.5 and no feedback the response is
//     0.5 (1 + e^(-j w tau)) up to the common delay: notches at f = (2k + 1) / (2 |tau|), and full level at tau = 0.
//   * Feedback returns the modulated path's output to its input through a soft saturation (Feedback Saturation), positive or
//     negative, up to +-0.95.
//   * Digital: the modulated path is the sinc-read delay. BBD: it is a bucket-brigade device (include/crucible/fx/Bbd.h)
//   whose
//     clock follows the delay, the dark, companded sound of an analog flanger. A device cannot be read shorter than its
//     minimum delay (about 3 ms for 256 stages, 5.6 ms for 512), so in this mode tau stops there instead of crossing zero.
//   * Source: oscillator (sine or triangle, the two channels Spread x half a cycle apart) or the envelope follower of the
//     input, whose level moves tau between Manual - Depth and Manual + Depth.
//
// Changing the mode, the device size, the source or the oscillator shape fades the wet path out, switches and fades it back
// in. Manual, Depth, Rate, Feedback and Spread are smoothed (Manual and Depth over 100 ms, because a change of the delay is
// a change of pitch) and tau is interpolated for every sample, so turning them never steps the delay.
#include "FxModuleSupport.h"

#include <crucible/fx/Bbd.h>
#include <crucible/fx/FractionalDelay.h>
#include <crucible/module/LatencyReporter.h>

#include <atomic>
#include <cstdint>

namespace crucible::modules::fx
{
namespace
{
using crucible::fx::Bbd;
using crucible::fx::BbdSettings;
using crucible::fx::FractionalDelay;

constexpr int kChunk = 16;
constexpr int kTaps = 16;
constexpr double kTwoPi = 6.283185307179586;
constexpr double kMaxTauSeconds = 0.005;
constexpr int kBbdStageChoices[] = {256, 512};

inline float tanhA(float x)
{
    x = x < -3.0f ? -3.0f : (x > 3.0f ? 3.0f : x);
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

struct Block
{
    int mode = 0, source = 0, shape = 0, bbdChoice = 0;
    double manual = 0.0, depth = 2.5, rate = 0.25;
    float mix = 0.5f, feedback = 0.4f, fbSat = 0.0f, spread = 0.5f, envSens = 0.0f, envAttack = 5.0f, envRelease = 120.0f;
};

struct Config
{
    int mode = 0, bbdChoice = 0, source = 0, shape = 0;
    bool operator==(const Config& o) const
    {
        return mode == o.mode && bbdChoice == o.bbdChoice && source == o.source && shape == o.shape;
    }
    bool operator!=(const Config& o) const { return !(*this == o); }
};

// The input of the device for one channel of a chunk: the dry sample plus the saturated previous output, scaled by the
// feedback ramp.
struct FeedbackInput
{
    const float* dry;
    float fb, dfb, previous, sat;

    float saturate(float x) const { return sat == 0.0f ? x : x + sat * (0.5f * tanhA(2.0f * x) - x); }

    static float next(void* context, int index, float previousOutput)
    {
        auto& f = *static_cast<FeedbackInput*>(context);
        if (index > 0)
            f.previous = f.saturate(previousOutput);
        f.fb += f.dfb;
        return f.dry[index] + f.fb * f.previous;
    }
};

class Instance final : public EffectModule, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_flanger2_mix")), mode(localIndex(d, "fx_flanger2_mode")),
          source(localIndex(d, "fx_flanger2_source")), manual(localIndex(d, "fx_flanger2_manual")),
          depth(localIndex(d, "fx_flanger2_depth")), rate(localIndex(d, "fx_flanger2_rate")),
          shape(localIndex(d, "fx_flanger2_shape")), feedback(localIndex(d, "fx_flanger2_feedback")),
          fbSat(localIndex(d, "fx_flanger2_fb_sat")), spread(localIndex(d, "fx_flanger2_spread")),
          bbdStages(localIndex(d, "fx_flanger2_bbd_stages")), envSens(localIndex(d, "fx_flanger2_env_sens")),
          envAttack(localIndex(d, "fx_flanger2_env_attack")), envRelease(localIndex(d, "fx_flanger2_env_release"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        sampleRate = c.sampleRate;
        maxTau = kMaxTauSeconds * sampleRate;
        refDelay = static_cast<int>(std::ceil(maxTau)) + kTaps / 2 + 1;
        std::size_t size = 1;
        while (size < static_cast<std::size_t>(refDelay) + 4)
            size <<= 1;
        for (int ch = 0; ch < 2; ++ch)
        {
            line[ch].prepare(static_cast<double>(refDelay) + maxTau + 2.0, kTaps);
            ring[ch].assign(size, 0.0f);
            bbd[ch].prepare(sampleRate);
            seeds[ch] = static_cast<std::uint64_t>(c.seed) * 16u + static_cast<std::uint64_t>(ch) + 31u;
        }
        ringMask = size - 1;
        fadeStep = static_cast<float>(1.0 / (0.015 * sampleRate));
        smoothMix = static_cast<float>(1.0 - std::exp(-1.0 / (0.005 * sampleRate)));
        alpha = 1.0 - std::exp(-static_cast<double>(kChunk) / (0.02 * sampleRate));
        alpha40 = 1.0 - std::exp(-static_cast<double>(kChunk) / (0.04 * sampleRate));
        alpha100 = 1.0 - std::exp(-static_cast<double>(kChunk) / (0.1 * sampleRate));
        reset();
    }

    void reset() override
    {
        for (int ch = 0; ch < 2; ++ch)
        {
            line[ch].reset();
            std::fill(ring[ch].begin(), ring[ch].end(), 0.0f);
            bbd[ch].reset();
            bbd[ch].setSeed(seeds[ch] + 1000u);
            fbPrev[ch] = 0.0f;
        }
        ringPos = 0;
        needInit = true;
        switching = false;
        hold = 0;
        fade = 1.0f;
        mixSmooth = mixPre = 0.0f;
        phase = 0.0;
        env = 0.0;
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
        const Config want{b.mode, b.bbdChoice, b.source, b.shape};
        if (needInit)
        {
            active = want;
            applyDevices();
            manualS = b.manual;
            depthS = b.depth;
            rateS = b.rate;
            fbS = b.feedback;
            fbNow = b.feedback;
            fbSatS = b.fbSat;
            spreadS = b.spread;
            mixSmooth = mixPre = b.mix;
            tauNow[0] = tauNow[1] = std::clamp(b.manual, -kMaxTauMs, kMaxTauMs) * 0.001 * sampleRate;
            needInit = false;
        }
        if (!switching && want != active)
            switching = true;
        const double lp = std::abs(static_cast<double>(b.feedback));
        tail.store(0.02 + (lp > 0.01 ? std::min(10.0, 0.012 * std::log(1.0e-3) / std::log(std::min(lp, 0.999))) : 0.0),
                   std::memory_order_relaxed);

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
            if (applied)
            {
                active = want;
                applyDevices();
                switching = false;
                // The new path is empty: keep the wet signal off until its delay has filled, so the start of the sound is
                // not a step.
                hold = refDelay + static_cast<int>(maxTau) + static_cast<int>(0.004 * sampleRate);
            }
        }
    }

    int latencySamples() const override { return refDelay; }
    double tailSeconds() const override { return tail.load(std::memory_order_relaxed); }

private:
    static constexpr double kMaxTauMs = 5.0;

    Block readBlock(const EffectInputs& in) const
    {
        Block b;
        b.mode = std::clamp(in.choice(mode), 0, 1);
        b.source = std::clamp(in.choice(source), 0, 1);
        b.shape = std::clamp(in.choice(shape), 0, 1);
        b.bbdChoice = std::clamp(in.choice(bbdStages), 0, 1);
        b.manual = static_cast<double>(in.value(manual));
        b.depth = static_cast<double>(in.value(depth));
        b.rate = static_cast<double>(in.value(rate));
        b.mix = in.value(mix);
        b.feedback = in.value(feedback);
        b.fbSat = in.value(fbSat);
        b.spread = in.value(spread);
        b.envSens = in.value(envSens);
        b.envAttack = in.value(envAttack);
        b.envRelease = in.value(envRelease);
        return b;
    }

    // Empty paths and a device of the chosen size.
    void applyDevices()
    {
        BbdSettings s;
        s.stages = kBbdStageChoices[active.bbdChoice];
        s.driveDb = 0.0f;
        s.compander = true;
        s.noiseDb = -120.0f;
        s.breathing = 0.3f;
        s.glideMs = 0.5f;
        for (int ch = 0; ch < 2; ++ch)
        {
            line[ch].reset();
            bbd[ch].setSettings(s);
            bbd[ch].setDelaySeconds(std::max(0.5 * (static_cast<double>(refDelay) / sampleRate), bbd[ch].minDelaySeconds()));
            bbd[ch].reset();
            fbPrev[ch] = 0.0f;
        }
    }

    static double wave(int shapeIndex, double p)
    {
        p -= std::floor(p);
        if (shapeIndex == 0)
            return std::sin(kTwoPi * p);
        return p < 0.25 ? 4.0 * p : (p < 0.75 ? 2.0 - 4.0 * p : 4.0 * p - 4.0);
    }

    // The device delay that puts the modulated path's output at R + tau, with the device's own extra delay taken off.
    double bbdDelay(int ch, double tauSamples) const
    {
        const double wanted = (static_cast<double>(refDelay) + tauSamples) / sampleRate;
        double device = wanted;
        for (int i = 0; i < 3; ++i) // the extra delay depends on the clock, which depends on the device delay
        {
            const double clock = Bbd::clockForDelay(kBbdStageChoices[active.bbdChoice], std::max(device, 1.0e-4));
            device =
                wanted - (7.0 / sampleRate + 8.5 / (kTwoPi * Bbd::kFilterCutoffRatio * std::min(clock, 0.95 * sampleRate)));
        }
        return std::max(device, bbd[ch].minDelaySeconds());
    }

    float saturate(float x) const { return fbSatF == 0.0f ? x : x + fbSatF * (0.5f * tanhA(2.0f * x) - x); }

    void chunk(float* l, float* r, int m, const Block& b, float fade0, float fade1)
    {
        const double a = m == kChunk ? alpha : 1.0 - std::exp(-static_cast<double>(m) / (0.02 * sampleRate));
        const double a40 = m == kChunk ? alpha40 : 1.0 - std::exp(-static_cast<double>(m) / (0.04 * sampleRate));
        const double a100 = m == kChunk ? alpha100 : 1.0 - std::exp(-static_cast<double>(m) / (0.1 * sampleRate));
        manualS += (b.manual - manualS) * a100;
        depthS += (b.depth - depthS) * a100;
        rateS += (b.rate - rateS) * a;
        fbS += (static_cast<double>(b.feedback) - fbS) * a40;
        fbSatS += (static_cast<double>(b.fbSat) - fbSatS) * a;
        spreadS += (static_cast<double>(b.spread) - spreadS) * a;
        fbSatF = static_cast<float>(fbSatS);
        float fbStart = fbNow, fbEnd = static_cast<float>(fbS);
        fbNow = fbEnd;

        // The envelope follower always runs, so that switching to it finds it settled.
        float pk = 0.0f;
        for (int i = 0; i < m; ++i)
            pk = std::max(pk, std::max(std::abs(l[i]), std::abs(r[i])));
        {
            const double tau = 0.001 * static_cast<double>(pk > env ? b.envAttack : b.envRelease);
            env += (static_cast<double>(pk) - env) *
                   (1.0 - std::exp(-static_cast<double>(m) / (std::max(tau, 1.0e-4) * sampleRate)));
        }
        double u[2];
        if (active.source == 0)
        {
            phase += rateS * static_cast<double>(m) / sampleRate;
            phase -= std::floor(phase);
            u[0] = wave(active.shape, phase);
            u[1] = wave(active.shape, phase + 0.5 * spreadS);
        }
        else
        {
            const double e = env * std::pow(10.0, static_cast<double>(b.envSens) / 20.0);
            u[0] = u[1] = 2.0 * e / (e + 0.05) - 1.0;
        }
        double tauEnd[2];
        for (int c = 0; c < 2; ++c)
            tauEnd[c] = std::clamp(manualS + depthS * u[c], -kMaxTauMs, kMaxTauMs) * 0.001 * sampleRate;
        const double inv = 1.0 / static_cast<double>(m);
        // While the wet path is off (fade-out finished, or waiting for a new path to fill) a jump of the offset is not
        // heard: take it at once, and keep the feedback out of the loop so that the jump leaves nothing in it.
        const bool wetOff = fade0 <= 0.0f && fade1 <= 0.0f;
        if (wetOff)
        {
            tauNow[0] = tauEnd[0];
            tauNow[1] = tauEnd[1];
            fbNow = 0.0f;
            fbPrev[0] = fbPrev[1] = 0.0f;
        }
        double tau0 = tauNow[0], tau1 = tauNow[1];
        const double dt0 = (tauEnd[0] - tau0) * inv, dt1 = (tauEnd[1] - tau1) * inv;
        tauNow[0] = tauEnd[0];
        tauNow[1] = tauEnd[1];

        if (wetOff)
            fbStart = fbEnd = 0.0f;
        const bool mixMoving = std::abs(b.mix - mixSmooth) > 1.0e-6f || std::abs(b.mix - mixPre) > 1.0e-6f;
        if (!mixMoving)
            mixSmooth = mixPre = b.mix;
        const float fgStep = (fade1 - fade0) / static_cast<float>(m);
        const float dfb = (fbEnd - fbStart) * static_cast<float>(inv);
        float fb = fbStart, gF = fade0;
        const double base = static_cast<double>(refDelay);
        const bool digital = active.mode == 0;
        float bl[kChunk], br[kChunk];
        if (!digital)
        {
            // The device delay is far longer than a chunk, so a chunk's output does not depend on its own input: each
            // channel runs as one call, with the feedback formed sample by sample inside it.
            bbd[0].setDelaySeconds(bbdDelay(0, tauEnd[0]));
            bbd[1].setDelaySeconds(bbdDelay(1, tauEnd[1]));
            const float* dry[2] = {l, r};
            float* wet[2] = {bl, br};
            for (int c = 0; c < 2; ++c)
            {
                FeedbackInput fi{dry[c], fbStart, dfb, fbPrev[c], fbSatF};
                bbd[c].processWithInput(&FeedbackInput::next, &fi, wet[c], m);
                fbPrev[c] = fi.saturate(wet[c][m - 1]);
            }
        }
        for (int i = 0; i < m; ++i)
        {
            fb += dfb;
            tau0 += dt0;
            tau1 += dt1;
            const float dl = l[i], dr = r[i];
            ring[0][static_cast<std::size_t>(ringPos)] = dl;
            ring[1][static_cast<std::size_t>(ringPos)] = dr;
            const float rl =
                ring[0][(static_cast<std::size_t>(ringPos) + ringMask + 1 - static_cast<std::size_t>(refDelay)) & ringMask];
            const float rr =
                ring[1][(static_cast<std::size_t>(ringPos) + ringMask + 1 - static_cast<std::size_t>(refDelay)) & ringMask];
            ringPos = static_cast<int>((static_cast<std::size_t>(ringPos) + 1) & ringMask);
            float yl, yr;
            if (digital)
            {
                const float xl = dl + fb * fbPrev[0], xr = dr + fb * fbPrev[1];
                line[0].write(xl);
                line[1].write(xr);
                yl = line[0].read(base + tau0);
                yr = line[1].read(base + tau1);
                fbPrev[0] = saturate(yl);
                fbPrev[1] = saturate(yr);
            }
            else
            {
                yl = bl[i];
                yr = br[i];
            }
            if (mixMoving)
            {
                mixPre += (b.mix - mixPre) * smoothMix; // two poles: the onset of a mix change has no kink
                mixSmooth += (mixPre - mixSmooth) * smoothMix;
            }
            gF += fgStep;
            const float k = mixSmooth * (gF * gF * (3.0f - 2.0f * gF)); // smoothstep: the fade has no kink at either end
            l[i] = rl + (yl - rl) * k;
            r[i] = rr + (yr - rr) * k;
        }
    }

    const int mix, mode, source, manual, depth, rate, shape, feedback, fbSat, spread, bbdStages, envSens, envAttack,
        envRelease;

    double sampleRate = 48000.0, maxTau = 240.0;
    int refDelay = 256, hold = 0;
    FractionalDelay line[2];
    Bbd bbd[2];
    std::vector<float> ring[2];
    std::size_t ringMask = 0;
    int ringPos = 0;
    std::uint64_t seeds[2] = {1, 2};
    Config active;
    bool needInit = true, switching = false, cleared = true;
    float fade = 1.0f, fadeStep = 0.001f, mixSmooth = 0.0f, mixPre = 0.0f, smoothMix = 0.01f;
    double alpha = 0.02, alpha40 = 0.01, alpha100 = 0.004, phase = 0.0, env = 0.0;
    double manualS = 0.0, depthS = 2.5, rateS = 0.25, fbS = 0.0, fbSatS = 0.0, spreadS = 0.5;
    double tauNow[2] = {0.0, 0.0};
    float fbSatF = 0.0f, fbNow = 0.0f;
    float fbPrev[2] = {};
    std::atomic<double> tail{0.02};
};
} // namespace

ModuleDescriptor flanger2Effect()
{
    auto d =
        effectDescriptor("fx.flanger2", "Flanger 2", "Modulation", CpuClass::Medium,
                         "Through-zero flanger: a fixed reference path and a modulated path read with a band-limited sinc, "
                         "whose offset from the reference crosses zero and goes negative as between two tape machines. The "
                         "offset follows an oscillator or the envelope of the input. Positive or negative feedback up to "
                         "0.95 with a saturation. The BBD mode runs the modulated path through a bucket-brigade device "
                         "instead. Reports the delay of the reference path as latency.");
    auto& p = d.params;
    addOn(p, "flanger2", "Flanger 2", "fx_flanger2", 0.5f);
    p.push_back(params::choice("fx_flanger2_mode", "Flanger 2 Mode", "fx_flanger2", {"Digital", "BBD"}, 0));
    p.push_back(params::choice("fx_flanger2_source", "Flanger 2 Source", "fx_flanger2", {"LFO", "Envelope"}, 0));
    addF(p, "flanger2_manual", "Flanger 2 Manual", "fx_flanger2", -5.0f, 5.0f, 0.0f, "ms");
    addF(p, "flanger2_depth", "Flanger 2 Depth", "fx_flanger2", 0.0f, 5.0f, 2.5f, "ms");
    addF(p, "flanger2_rate", "Flanger 2 Rate", "fx_flanger2_2", 0.02f, 10.0f, 0.25f, "Hz", 0.4f);
    p.push_back(params::choice("fx_flanger2_shape", "Flanger 2 Shape", "fx_flanger2_2", {"Sine", "Triangle"}, 0));
    addF(p, "flanger2_feedback", "Flanger 2 Feedback", "fx_flanger2_2", -0.95f, 0.95f, 0.4f);
    addF(p, "flanger2_fb_sat", "Flanger 2 Feedback Saturation", "fx_flanger2_2", 0.0f, 1.0f, 0.0f);
    addF(p, "flanger2_spread", "Flanger 2 Spread", "fx_flanger2_2", 0.0f, 1.0f, 0.5f);
    p.push_back(params::choice("fx_flanger2_bbd_stages", "Flanger 2 BBD Stages", "fx_flanger2_2", {"256", "512"}, 0));
    addF(p, "flanger2_env_sens", "Flanger 2 Envelope Sensitivity", "fx_flanger2_3", -40.0f, 40.0f, 0.0f, "dB");
    addF(p, "flanger2_env_attack", "Flanger 2 Envelope Attack", "fx_flanger2_3", 0.5f, 200.0f, 5.0f, "ms", 0.4f);
    addF(p, "flanger2_env_release", "Flanger 2 Envelope Release", "fx_flanger2_3", 5.0f, 2000.0f, 120.0f, "ms", 0.4f);
    p.push_back(params::integer("fx_pos_flanger2", "Flanger 2 Slot", "fx_flanger2_3", 1, kNumModuleFx, kNumModuleFx));
    d.onParam = "fx_flanger2_on";
    d.positionParam = "fx_pos_flanger2";
    d.capabilities = {"latency"};
    d.pages.push_back(
        {"fx_flanger2", "Flanger 2", "fx", "Flanger 2", "On, mix, mode, source, manual offset and sweep depth."});
    d.pages.push_back({"fx_flanger2_2", "Flanger 2 2", "fx", "Flanger 2 Sweep",
                       "Rate, shape, feedback, saturation, spread and BBD size."});
    d.pages.push_back({"fx_flanger2_3", "Flanger 2 3", "fx", "Flanger 2 Envelope", "Envelope follower and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.flanger2")); };
    return d;
}

} // namespace crucible::modules::fx
