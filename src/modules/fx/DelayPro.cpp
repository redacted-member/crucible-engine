// SPDX-License-Identifier: MIT
// fx.delay_pro: a stereo delay with two engines, Digital (Kaiser-sinc reads, include/crucible/fx/FractionalDelay.h) and
// Analog (bucket-brigade device, include/crucible/fx/Bbd.h). Common to both: tempo sync, feedback with saturation in the
// loop (a soft curve, never a hard clamp), low and high cut in the loop, stereo / ping-pong / cross feedback, a sine
// modulation, an all-pass diffusion chain, ducking from the input or the sidechain, freeze, and time changes by glide
// (the delay slews, so the pitch moves) or crossfade (digital only: two reads, no pitch).
//
// Signal flow per channel: read -> diffusion -> wet output; wet -> saturation -> low cut -> high cut -> x feedback ->
// routed (stereo, ping-pong or cross) and added to the input that is written into the line. The first echo is therefore
// free of the loop's saturation and cuts. Ducking and the mix act on the output only, so the loop is never ducked.
//
// The analog engine processes the device in chunks of kChunk samples, so the feedback reaches it kChunk samples late; the
// requested time is shortened by that lag and by the known extra delay of the device's reconstruction (see
// analogExtraSeconds), so the echo spacing is the set time.
#include "FxModuleSupport.h"

#include <crucible/fx/Bbd.h>
#include <crucible/fx/FractionalDelay.h>
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
using crucible::fx::FractionalDelay;

constexpr double kMaxDelaySeconds = 4.0;
constexpr int kChunk = 16;
constexpr double kGlideMaxRate = 0.5;
constexpr double kTwoPi = 6.283185307179586;
constexpr int kStageChoices[] = {256, 512, 1024, 2048, 4096};
// Freeze guard: the frozen loop has a gain of exactly 1, so any pass that gains a hair (an interpolated read of a moving
// delay) compounds for as long as the freeze lasts. The mean square of the loop signal is measured over windows of 8192
// samples (counted per channel); the level of the last two windows before the freeze is the reference. While frozen, a
// window more than 1 dB above the reference lowers the gain of the loop by 0.01 dB (down to -3 dB), a window within 0.5 dB
// of it gives the gain back. The guard never acts on a loop that holds or decays, so the frozen level is the one the loop
// had.
constexpr int kGuardCalls = 16384;
constexpr float kGuardAbove = 1.2589f; // +1 dB in power
constexpr float kGuardBelow = 1.1220f; // +0.5 dB in power
constexpr float kGuardCut = 0.99885f;  // -0.01 dB
constexpr float kGuardGive = 1.0005f;
constexpr float kGuardFloor = 0.7079f;

// Saturator with unity small-signal gain and a ceiling of 1 / g: the cubic t - 4 t^3 / 27 of t = g x, which reaches its
// maximum 1 with zero slope at t = 1.5 and is held there (no division, no hard corner).
inline float saturate(float x, float g, float invG)
{
    float t = g * x;
    t = t > 1.5f ? 1.5f : (t < -1.5f ? -1.5f : t);
    return (t - 0.14814815f * t * t * t) * invG;
}

struct AllPass
{
    std::vector<float> buf;
    int pos = 0;
    float process(float x, float g)
    {
        const float d = buf[static_cast<size_t>(pos)];
        const float w = x + g * d;
        buf[static_cast<size_t>(pos)] = w;
        if (++pos >= static_cast<int>(buf.size()))
            pos = 0;
        return d - g * w;
    }
};

// Per channel: diffusion all-passes and the loop filters.
struct Chain
{
    std::array<AllPass, 3> ap;
    float hpState = 0.0f, lpState = 0.0f;

    void prepare(double rate, double stretch)
    {
        constexpr double ms[3] = {3.7, 7.1, 11.9};
        for (size_t i = 0; i < 3; ++i)
            ap[i].buf.assign(static_cast<size_t>(std::max(8.0, ms[i] * 0.001 * rate * stretch)), 0.0f);
    }
    void reset()
    {
        for (auto& a : ap)
        {
            std::fill(a.buf.begin(), a.buf.end(), 0.0f);
            a.pos = 0;
        }
        hpState = lpState = 0.0f;
    }
    float diffuse(float x, float g)
    {
        for (auto& a : ap)
            x = a.process(x, g);
        return x;
    }
};

// Time glide: a rate-limited follower into a second one-pole, so the delay velocity (the pitch shift) is continuous and
// bounded by kGlideMaxRate samples per sample.
struct Glide
{
    double x1 = 0.0;
    double x2 = 0.0;
    void set(double v) { x1 = x2 = v; }
    bool settle(double target)
    {
        if (std::abs(target - x1) < 1.0e-6 && std::abs(x1 - x2) < 1.0e-6)
        {
            x1 = x2 = target;
            return true;
        }
        return false;
    }
    void step(double target, double a1, double a2)
    {
        double d = (target - x1) * a1;
        d = d > kGlideMaxRate ? kGlideMaxRate : (d < -kGlideMaxRate ? -kGlideMaxRate : d);
        x1 += d;
        x2 += (x1 - x2) * a2;
    }
};

struct Head
{
    double cur = 0.0;
    double next = 0.0;
    int pos = 0;
    bool fading = false;
};

struct Block
{
    bool analog = false;
    int stages = 1024;
    int stereo = 0;
    bool crossfade = false;
    float mix = 0.0f;
    float feedback = 0.0f;
    bool freeze = false;
    double target[2] = {0.0, 0.0}; // seconds
    float satG = 1.0f, satInvG = 1.0f;
    bool hpOn = false, lpOn = false;
    float hpG = 0.0f, lpG = 0.0f;
    float diffusion = 0.0f;
    double depthSeconds = 0.0;
    float duckDepth = 0.0f, duckThreshold = -30.0f, duckRelease = 250.0f;
    bool duckSidechain = false;
};

class Instance final : public EffectModule, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_delay_pro_mix")), mode(localIndex(d, "fx_delay_pro_mode")),
          time(localIndex(d, "fx_delay_pro_time")), sync(localIndex(d, "fx_delay_pro_sync")),
          feedback(localIndex(d, "fx_delay_pro_feedback")), stereo(localIndex(d, "fx_delay_pro_stereo")),
          ratio(localIndex(d, "fx_delay_pro_right_ratio")), lowCut(localIndex(d, "fx_delay_pro_low_cut")),
          highCut(localIndex(d, "fx_delay_pro_high_cut")), saturation(localIndex(d, "fx_delay_pro_saturation")),
          diffusion(localIndex(d, "fx_delay_pro_diffusion")), modDepth(localIndex(d, "fx_delay_pro_mod_depth")),
          modRate(localIndex(d, "fx_delay_pro_mod_rate")), timeChange(localIndex(d, "fx_delay_pro_time_change")),
          freeze(localIndex(d, "fx_delay_pro_freeze")), duckDepth(localIndex(d, "fx_delay_pro_duck_depth")),
          duckThreshold(localIndex(d, "fx_delay_pro_duck_threshold")),
          duckRelease(localIndex(d, "fx_delay_pro_duck_release")), duckSource(localIndex(d, "fx_delay_pro_duck_source")),
          stages(localIndex(d, "fx_delay_pro_analog_stages")), drive(localIndex(d, "fx_delay_pro_analog_drive")),
          noise(localIndex(d, "fx_delay_pro_analog_noise")), compander(localIndex(d, "fx_delay_pro_analog_compander")),
          breathing(localIndex(d, "fx_delay_pro_analog_breathing"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        rate = c.sampleRate;
        // Enough taps for a response flat to 18 kHz (0.008 dB at 44.1 kHz with 24 taps, 0.023 dB at 96 kHz with 16).
        const int taps = rate <= 50000.0 ? 24 : 16;
        for (int ch = 0; ch < 2; ++ch)
        {
            line[ch].prepare(kMaxDelaySeconds * rate + 64.0, taps);
            bbd[ch].prepare(rate);
            bbd[ch].setSeed(static_cast<std::uint64_t>(c.seed) * 2u + static_cast<std::uint64_t>(ch) + 1u);
            chain[ch].prepare(rate, ch == 0 ? 1.0 : 1.13);
        }
        glideA1 = glideA2 = 1.0 - std::exp(-1.0 / (0.04 * rate));
        fadeLength = std::max(16, static_cast<int>(std::lround(0.03 * rate)));
        fadeTable.assign(static_cast<size_t>(fadeLength) + 1, 0.0f);
        for (int i = 0; i <= fadeLength; ++i)
            fadeTable[static_cast<size_t>(i)] = static_cast<float>(0.5 - 0.5 * std::cos(kTwoPi * 0.5 * i / fadeLength));
        modeStep = static_cast<float>(1.0 / (0.02 * rate));
        smoothFast = static_cast<float>(1.0 - std::exp(-1.0 / (0.005 * rate)));
        duckAttack = static_cast<float>(1.0 - std::exp(-1.0 / (0.001 * rate)));
        duckSmooth = static_cast<float>(1.0 - std::exp(-1.0 / (0.002 * rate)));
        reset();
    }

    void reset() override
    {
        for (int ch = 0; ch < 2; ++ch)
        {
            line[ch].reset();
            bbd[ch].reset();
            chain[ch].reset();
            head[ch] = Head{};
            glide[ch].set(0.0);
            fbQueue[ch].fill(0.0f);
        }
        queuePos = 0;
        needInit = true;
        activeAnalog = false;
        activeStages = 1024;
        switching = false;
        injectRamp = 1.0f;
        snapAnalog = false;
        modeGain = 1.0f;
        mixSmooth = 0.0f;
        inputGain = 1.0f;
        feedbackSmooth = 0.0f;
        freezeBlend = 0.0f;
        guardAcc = guardNow = guardPrev = guardRef = 0.0f;
        guardCalls = 0;
        guardTrim = 1.0f;
        wasFrozen = false;
        duckEnv = 0.0f;
        duckGain = 1.0f;
        lfoSin = 0.0;
        lfoCos = 1.0;
        cleared = true;
    }

    // Switched off: forget the contents once, so switching on replays nothing.
    void deactivate() override
    {
        if (!cleared)
            reset();
    }

    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        cleared = false;
        Block b = readBlock(in);
        if (b.freeze && !wasFrozen)
            guardRef = std::max(guardNow, guardPrev);
        wasFrozen = b.freeze;
        if (needInit)
        {
            activeAnalog = b.analog;
            activeStages = b.stages;
            if (activeAnalog)
            {
                configureAnalog(in, b.stages);
                snapAnalog = true;
            }
            for (int ch = 0; ch < 2; ++ch)
            {
                const double t = b.target[ch] * rate;
                glide[ch].set(t);
                head[ch].cur = t;
                head[ch].fading = false;
            }
            mixSmooth = b.mix;
            feedbackSmooth = b.freeze ? 1.0f : b.feedback;
            inputGain = b.freeze ? 0.0f : 1.0f;
            freezeBlend = b.freeze ? 1.0f : 0.0f;
            needInit = false;
        }
        if (activeAnalog)
            updateAnalog(in);
        wantAnalog = b.analog;
        wantStages = b.stages;
        publishTail(b);

        const float* auxL = in.hasAux() ? in.auxLeft() : nullptr;
        const float* auxR = in.hasAux() ? in.auxRight() : nullptr;
        // Sidechain ducking follows the auxiliary input; with none connected there is nothing to duck by.
        const bool useAux = b.duckSidechain && auxL != nullptr;
        if (b.duckSidechain && auxL == nullptr)
            b.duckDepth = 0.0f;
        const float releaseCoef =
            static_cast<float>(1.0 - std::exp(-1.0 / (0.001 * static_cast<double>(b.duckRelease) * rate)));
        const double lfoW = kTwoPi * static_cast<double>(in.value(modRate)) / rate;
        const double lfoC = std::cos(lfoW), lfoS = std::sin(lfoW);

        for (int pos = 0; pos < n; pos += kChunk)
        {
            const int m = std::min(kChunk, n - pos);
            float g0 = modeGain;
            if (switching)
            {
                modeGain -= static_cast<float>(m) * modeStep;
                if (modeGain <= 0.0f)
                    modeGain = 0.0f;
            }
            else if (modeGain < 1.0f)
                modeGain = std::min(1.0f, modeGain + static_cast<float>(m) * modeStep);
            const float g1 = modeGain;

            float wetL[kChunk], wetR[kChunk];
            float* x[2] = {l + pos, r + pos};
            if (activeAnalog)
                analogChunk(x, m, b, wetL, wetR, lfoC, lfoS);
            else
                digitalChunk(x, m, b, wetL, wetR, lfoC, lfoS);
            finish(x, m, b, wetL, wetR, g0, g1, useAux ? auxL + pos : nullptr, useAux ? auxR + pos : nullptr, releaseCoef);

            if (switching && modeGain <= 0.0f)
                changeEngine(in);
        }
        // Keep the quadrature oscillator at unit radius.
        const double radius = std::sqrt(lfoSin * lfoSin + lfoCos * lfoCos);
        if (radius > 0.0)
        {
            lfoSin /= radius;
            lfoCos /= radius;
        }
    }

    int latencySamples() const override { return 0; }
    double tailSeconds() const override { return tail.load(std::memory_order_relaxed); }
    bool tailInfinite() const override { return infinite.load(std::memory_order_relaxed); }

private:
    Block readBlock(const EffectInputs& in)
    {
        Block b;
        b.analog = in.choice(mode) == 1;
        b.stages = kStageChoices[std::clamp(in.choice(stages), 0, 4)];
        b.stereo = std::clamp(in.choice(stereo), 0, 2);
        b.crossfade = in.choice(timeChange) == 1;
        b.mix = in.value(mix);
        b.feedback = in.value(feedback);
        b.freeze = in.on(freeze);
        const int s = in.choice(sync);
        double ms = in.value(time);
        if (s > 0)
            ms = 60000.0 / std::max(in.bpm(), 1.0) * static_cast<double>(lfoSyncBeats(s));
        ms = std::min(ms, kMaxDelaySeconds * 1000.0 * 0.98);
        b.depthSeconds = static_cast<double>(in.value(modDepth)) * 0.001;
        b.target[0] = ms * 0.001;
        b.target[1] = std::min(ms * static_cast<double>(in.value(ratio)), kMaxDelaySeconds * 1000.0 * 0.98) * 0.001;
        const float sat = in.value(saturation);
        b.satG = 0.25f * std::pow(16.0f, sat); // ceiling 1 / g: 4 (none) down to 0.25 (hard)
        b.satInvG = 1.0f / b.satG;
        const float lo = in.value(lowCut), hi = in.value(highCut);
        b.hpOn = lo > 20.5f;
        b.lpOn = hi < 19000.0f;
        b.hpG = static_cast<float>(std::tan(3.14159265358979 * std::min(static_cast<double>(lo), 0.45 * rate) / rate));
        b.hpG /= (1.0f + b.hpG);
        b.lpG = static_cast<float>(std::tan(3.14159265358979 * std::min(static_cast<double>(hi), 0.45 * rate) / rate));
        b.lpG /= (1.0f + b.lpG);
        b.diffusion = in.value(diffusion);
        b.duckDepth = in.value(duckDepth);
        b.duckThreshold = in.value(duckThreshold);
        b.duckRelease = in.value(duckRelease);
        b.duckSidechain = in.choice(duckSource) == 1;
        return b;
    }

    void publishTail(const Block& b)
    {
        const double longest = std::max(b.target[0], b.target[1]);
        const bool inf = b.freeze || b.feedback >= 0.999f;
        double t = longest;
        if (!inf && b.feedback > 0.01f)
            t = longest * (1.0 + std::min(400.0, std::ceil(std::log(1.0e-3) / std::log(static_cast<double>(b.feedback)))));
        tail.store(std::min(t, 120.0), std::memory_order_relaxed);
        infinite.store(inf, std::memory_order_relaxed);
    }

    void configureAnalog(const EffectInputs& in, int st)
    {
        BbdSettings s;
        s.stages = st;
        s.driveDb = in.value(drive);
        s.noiseDb = in.value(noise);
        s.compander = in.on(compander);
        s.breathing = in.value(breathing);
        s.glideMs = 20.0f;
        for (auto& dev : bbd)
            dev.setSettings(s);
        cached = s;
    }

    void updateAnalog(const EffectInputs& in)
    {
        BbdSettings s = cached;
        s.stages = activeStages;
        s.driveDb = in.value(drive);
        s.noiseDb = in.value(noise);
        s.compander = in.on(compander);
        s.breathing = in.value(breathing);
        if (s.driveDb != cached.driveDb || s.noiseDb != cached.noiseDb || s.compander != cached.compander ||
            s.breathing != cached.breathing)
        {
            for (auto& dev : bbd)
                dev.setSettings(s);
            cached = s;
        }
    }

    // The fade-out reached silence: change the engine (or the stage count) and clear everything.
    void changeEngine(const EffectInputs& in)
    {
        for (int ch = 0; ch < 2; ++ch)
        {
            line[ch].reset();
            chain[ch].reset();
            fbQueue[ch].fill(0.0f);
            head[ch].fading = false;
        }
        queuePos = 0;
        activeAnalog = wantAnalog;
        activeStages = wantStages;
        if (activeAnalog)
        {
            configureAnalog(in, activeStages);
            snapAnalog = true;
        }
        switching = false;
        // The new engine starts empty and silent. The input enters it through a smooth ramp, so neither the first
        // arrival nor the repeats of it begin with a step.
        modeGain = 1.0f;
        injectRamp = 0.0f;
        for (int ch = 0; ch < 2; ++ch)
        {
            glide[ch].set(lastTarget[ch] * rate);
            head[ch].cur = lastTarget[ch] * rate;
        }
    }

    // Saturation and the low and high cut of one channel's wet signal (the feedback gain is applied when routing).
    float conditionFeedback(int ch, float y, const Block& b)
    {
        guardAcc += y * y;
        if (++guardCalls >= kGuardCalls)
            endGuardWindow(b);
        Chain& c = chain[ch];
        float p = y;
        if (freezeBlend < 1.0f)
        {
            float q = saturate(y, b.satG, b.satInvG);
            if (b.hpOn)
            {
                const float v = (q - c.hpState) * b.hpG;
                const float lp = v + c.hpState;
                c.hpState = lp + v;
                q -= lp;
            }
            if (b.lpOn)
            {
                const float v = (q - c.lpState) * b.lpG;
                const float lp = v + c.lpState;
                c.lpState = lp + v;
                q = lp;
            }
            p = q + (y - q) * freezeBlend;
        }
        return p * guardTrim;
    }

    void endGuardWindow(const Block& b)
    {
        guardPrev = guardNow;
        guardNow = guardAcc / static_cast<float>(guardCalls);
        guardAcc = 0.0f;
        guardCalls = 0;
        if (b.freeze)
        {
            const float limit = std::max(guardRef * kGuardAbove, 1.0e-8f),
                        settled = std::max(guardRef * kGuardBelow, 1.0e-8f);
            if (guardNow > limit)
                guardTrim = std::max(kGuardFloor, guardTrim * kGuardCut);
            else if (guardNow < settled)
                guardTrim = std::min(1.0f, guardTrim * kGuardGive);
        }
        else
            guardTrim += (1.0f - guardTrim) * 0.2f;
    }

    // Moves the feedback, input and freeze gains to their targets; true once they have all arrived (and are exact).
    bool controlsSettled(const Block& b)
    {
        const float fbT = b.freeze ? 1.0f : b.feedback;
        const float inT = b.freeze ? 0.0f : 1.0f;
        const float frT = b.freeze ? 1.0f : 0.0f;
        if (std::abs(feedbackSmooth - fbT) < 1.0e-6f && std::abs(inputGain - inT) < 1.0e-6f &&
            std::abs(freezeBlend - frT) < 1.0e-4f)
        {
            feedbackSmooth = fbT;
            inputGain = inT;
            freezeBlend = frT;
            return true;
        }
        return false;
    }

    void advanceControls(const Block& b)
    {
        feedbackSmooth += ((b.freeze ? 1.0f : b.feedback) - feedbackSmooth) * smoothFast;
        inputGain += ((b.freeze ? 0.0f : 1.0f) - inputGain) * smoothFast;
        freezeBlend += ((b.freeze ? 1.0f : 0.0f) - freezeBlend) * smoothFast;
    }

    void digitalChunk(float* const x[2], int m, const Block& b, float* wetL, float* wetR, double lfoC, double lfoS)
    {
        const double minD = 17.0;
        const double maxD = line[0].maxDelay() - b.depthSeconds * rate - 3.0;
        double tgt[2];
        for (int ch = 0; ch < 2; ++ch)
        {
            tgt[ch] = std::clamp(b.target[ch] * rate, minD, maxD);
            lastTarget[ch] = tgt[ch] / rate;
        }
        const double depth = b.depthSeconds * rate;
        const bool modOn = depth > 0.0;
        const bool diffOn = b.diffusion > 0.001f;
        const float apG = 0.72f * b.diffusion;
        const bool smoothing = !controlsSettled(b);
        const bool glideOn = !b.crossfade && !(glide[0].settle(tgt[0]) & glide[1].settle(tgt[1]));
        const double hiLimit = line[0].maxDelay() - 2.0;
        float* wet[2] = {wetL, wetR};
        // 1. Read positions (the modulation, the glide or the crossfade heads) for the whole chunk: they do not depend
        // on the signal.
        double at[2][kChunk], atNext[2][kChunk];
        float fadeW[2][kChunk];
        bool fadeOn[2][kChunk];
        bool early = true;
        for (int i = 0; i < m; ++i)
        {
            double mod[2] = {0.0, 0.0};
            if (modOn)
            {
                const double s = lfoSin * lfoC + lfoCos * lfoS;
                lfoCos = lfoCos * lfoC - lfoSin * lfoS;
                lfoSin = s;
                mod[0] = depth * lfoSin;
                mod[1] = depth * lfoCos;
            }
            for (int ch = 0; ch < 2; ++ch)
            {
                fadeOn[ch][i] = false;
                if (b.crossfade)
                {
                    Head& h = head[ch];
                    if (!h.fading && std::abs(tgt[ch] - h.cur) > 0.25)
                    {
                        h.fading = true;
                        h.next = tgt[ch];
                        h.pos = 0;
                    }
                    at[ch][i] = h.cur + mod[ch] - 1.0;
                    if (h.fading)
                    {
                        fadeOn[ch][i] = true;
                        atNext[ch][i] = h.next + mod[ch] - 1.0;
                        fadeW[ch][i] = fadeTable[static_cast<size_t>(h.pos)];
                        early = early && line[ch].canReadEarly(atNext[ch][i], i);
                        if (++h.pos >= fadeLength)
                        {
                            h.cur = h.next;
                            h.fading = false;
                        }
                    }
                    glide[ch].set(h.cur);
                }
                else
                {
                    if (glideOn)
                        glide[ch].step(tgt[ch], glideA1, glideA2);
                    head[ch].cur = glide[ch].x2;
                    at[ch][i] = std::clamp(glide[ch].x2 + mod[ch], minD, hiLimit) - 1.0;
                }
                early = early && line[ch].canReadEarly(at[ch][i], i);
            }
        }
        // 2. When no read of the chunk reaches a sample written in it (any delay longer than the chunk plus the kernel),
        // all reads are done first: they are independent, while in the loop below each one sat on a long dependency
        // chain (the kernel's serial sum, then diffusion, saturation, filters, routing) that kept them from
        // overlapping. Same values: readEarly() reads exactly what read() reads at that sample.
        float tap[2][kChunk], tapNext[2][kChunk];
        if (early)
            for (int ch = 0; ch < 2; ++ch)
                for (int i = 0; i < m; ++i)
                {
                    tap[ch][i] = line[ch].readEarly(at[ch][i], i);
                    if (fadeOn[ch][i])
                        tapNext[ch][i] = line[ch].readEarly(atNext[ch][i], i);
                }
        // 3. The loop: crossfade, diffusion, feedback conditioning, routing, writing.
        for (int i = 0; i < m; ++i)
        {
            if (smoothing)
                advanceControls(b);
            float p[2];
            for (int ch = 0; ch < 2; ++ch)
            {
                float v = early ? tap[ch][i] : line[ch].read(at[ch][i]);
                if (fadeOn[ch][i])
                {
                    const float v2 = early ? tapNext[ch][i] : line[ch].read(atNext[ch][i]);
                    v += (v2 - v) * fadeW[ch][i];
                }
                if (diffOn)
                    v = chain[ch].diffuse(v, apG);
                p[ch] = conditionFeedback(ch, v, b);
                wet[ch][i] = v;
            }
            route(x[0][i], x[1][i], p[0], p[1], b.stereo);
            line[0].write(routed[0]);
            line[1].write(routed[1]);
        }
    }

    // Smoothstep of the ramp that follows an engine change; advances one sample per call.
    float injectGain()
    {
        if (injectRamp >= 1.0f)
            return 1.0f;
        injectRamp = std::min(1.0f, injectRamp + modeStep);
        return injectRamp * injectRamp * (3.0f - 2.0f * injectRamp);
    }

    // Writes the two line inputs of this sample to routed[]. (Denormals are flushed by the engine around every block.)
    void route(float xl, float xr, float p0, float p1, int stereoMode)
    {
        const float fb = feedbackSmooth;
        const float g = inputGain * injectGain();
        switch (stereoMode)
        {
        case 1: // ping-pong: the input enters the left line, each line feeds the other
            routed[0] = (xl + xr) * 0.5f * g + fb * p1;
            routed[1] = fb * p0;
            break;
        case 2: // cross: each line feeds the other, the inputs stay on their sides
            routed[0] = xl * g + fb * p1;
            routed[1] = xr * g + fb * p0;
            break;
        default:
            routed[0] = xl * g + fb * p0;
            routed[1] = xr * g + fb * p1;
            break;
        }
    }

    // Seconds of delay the device adds beyond its nominal stages / (2 x clock): the chunked feedback, the read of the
    // staircase and the group delay of the two filters.
    double analogExtraSeconds(double clockHz) const
    {
        return (static_cast<double>(kChunk) + 7.0) / rate + 5.2 / (kTwoPi * Bbd::kFilterCutoffRatio * clockHz);
    }

    void analogChunk(float* const x[2], int m, const Block& b, float* wetL, float* wetR, double lfoC, double lfoS)
    {
        double sinV = 0.0, cosV = 1.0;
        if (b.depthSeconds > 0.0)
        {
            for (int i = 0; i < m; ++i)
            {
                const double s = lfoSin * lfoC + lfoCos * lfoS;
                lfoCos = lfoCos * lfoC - lfoSin * lfoS;
                lfoSin = s;
            }
            sinV = lfoSin;
            cosV = lfoCos;
        }
        const double mods[2] = {b.depthSeconds * sinV, b.depthSeconds * cosV};
        float in[2][kChunk], out[2][kChunk];
        for (int ch = 0; ch < 2; ++ch)
        {
            const double tgt = b.target[ch] * rate;
            lastTarget[ch] = b.target[ch];
            if (!glide[ch].settle(tgt))
                for (int i = 0; i < m; ++i)
                    glide[ch].step(tgt, glideA1, glideA2);
            const double wanted = glide[ch].x2 / rate + mods[ch];
            const double nominal = wanted - analogExtraSeconds(bbd[ch].clockHz());
            bbd[ch].setDelaySeconds(std::max(nominal, bbd[ch].minDelaySeconds()));
        }
        if (snapAnalog)
        {
            // A fresh device starts with its clock at the requested rate.
            for (auto& dev : bbd)
                dev.reset();
            snapAnalog = false;
        }
        const bool diffOn = b.diffusion > 0.001f;
        const float apG = 0.72f * b.diffusion;
        // The feedback queue holds the conditioned wet signal of the last kChunk samples; it enters the device kChunk
        // samples later.
        float fbS[kChunk], gS[kChunk];
        if (controlsSettled(b))
            for (int i = 0; i < m; ++i)
            {
                fbS[i] = feedbackSmooth;
                gS[i] = inputGain * injectGain();
            }
        else
            for (int i = 0; i < m; ++i)
            {
                advanceControls(b);
                fbS[i] = feedbackSmooth;
                gS[i] = inputGain * injectGain();
            }
        for (int i = 0; i < m; ++i)
        {
            const size_t q = static_cast<size_t>((queuePos + i) % kChunk);
            const float q0 = fbQueue[0][q], q1 = fbQueue[1][q];
            const float xl = x[0][i], xr = x[1][i];
            const float g = gS[i], fb = fbS[i];
            switch (b.stereo)
            {
            case 1:
                in[0][i] = (xl + xr) * 0.5f * g + fb * q1;
                in[1][i] = fb * q0;
                break;
            case 2:
                in[0][i] = xl * g + fb * q1;
                in[1][i] = xr * g + fb * q0;
                break;
            default:
                in[0][i] = xl * g + fb * q0;
                in[1][i] = xr * g + fb * q1;
                break;
            }
        }
        for (int ch = 0; ch < 2; ++ch)
            bbd[ch].processBlock(in[ch], out[ch], m);
        float* wet[2] = {wetL, wetR};
        for (int i = 0; i < m; ++i)
        {
            const size_t q = static_cast<size_t>((queuePos + i) % kChunk);
            for (int ch = 0; ch < 2; ++ch)
            {
                float v = out[ch][i];
                if (diffOn)
                    v = chain[ch].diffuse(v, apG);
                wet[ch][i] = v;
                fbQueue[ch][q] = conditionFeedback(ch, v, b);
            }
        }
        queuePos = (queuePos + m) % kChunk;
    }

    // Ducking, the engine-change fade, the mix.
    void finish(float* const x[2], int m, const Block& b, const float* wetL, const float* wetR, float g0, float g1,
                const float* auxL, const float* auxR, float releaseCoef)
    {
        const bool duck = b.duckDepth > 0.001f;
        const float step = (g1 - g0) / static_cast<float>(m);
        float g = g0;
        if (duck)
        {
            // Envelope per sample; the target gain is taken from the envelope at the end of the chunk.
            for (int i = 0; i < m; ++i)
            {
                const float a = auxL != nullptr ? std::max(std::abs(auxL[i]), std::abs(auxR[i]))
                                                : std::max(std::abs(x[0][i]), std::abs(x[1][i]));
                duckEnv += (a - duckEnv) * (a > duckEnv ? duckAttack : releaseCoef);
            }
            const float db = 20.0f * std::log10(std::max(duckEnv, 1.0e-6f));
            const float amount = std::clamp((db - b.duckThreshold) * (1.0f / 6.0f), 0.0f, 1.0f);
            duckTarget = std::pow(10.0f, -b.duckDepth * amount * 0.05f);
        }
        else
            duckTarget = 1.0f;
        const bool duckMoving = std::abs(duckTarget - duckGain) > 1.0e-6f;
        if (!duckMoving)
            duckGain = duckTarget;
        const bool mixMoving = std::abs(b.mix - mixSmooth) > 1.0e-6f;
        if (!mixMoving)
            mixSmooth = b.mix;
        for (int i = 0; i < m; ++i)
        {
            if (duckMoving)
                duckGain += (duckTarget - duckGain) * duckSmooth;
            if (mixMoving)
                mixSmooth += (b.mix - mixSmooth) * smoothFast;
            g += step;
            const float gain = duckGain * g;
            const float dl = x[0][i], dr = x[1][i];
            x[0][i] = dl + (wetL[i] * gain - dl) * mixSmooth;
            x[1][i] = dr + (wetR[i] * gain - dr) * mixSmooth;
        }
        // A request to change the engine or stage count starts the fade-out at the end of the chunk.
        if (!switching && (wantAnalog != activeAnalog || (activeAnalog && wantStages != activeStages)))
            switching = true;
    }

    const int mix, mode, time, sync, feedback, stereo, ratio, lowCut, highCut, saturation, diffusion, modDepth, modRate,
        timeChange, freeze, duckDepth, duckThreshold, duckRelease, duckSource, stages, drive, noise, compander, breathing;

    double rate = 48000.0;
    FractionalDelay line[2];
    Bbd bbd[2];
    Chain chain[2];
    Head head[2];
    Glide glide[2];
    double glideA1 = 0.0005, glideA2 = 0.0005;
    float injectRamp = 1.0f;
    bool snapAnalog = false;
    double lastTarget[2] = {0.0, 0.0};
    std::array<float, kChunk> fbQueue[2];
    int queuePos = 0;
    float routed[2] = {0.0f, 0.0f};
    std::vector<float> fadeTable;
    int fadeLength = 1440;
    float modeStep = 0.001f, smoothFast = 0.004f, duckAttack = 0.02f, duckSmooth = 0.01f;
    bool needInit = true, activeAnalog = false, switching = false, cleared = true;
    bool wantAnalog = false;
    int activeStages = 1024, wantStages = 1024;
    BbdSettings cached;
    float modeGain = 1.0f, mixSmooth = 0.0f, inputGain = 1.0f, feedbackSmooth = 0.0f, freezeBlend = 0.0f;
    float duckEnv = 0.0f, duckGain = 1.0f, duckTarget = 1.0f;
    float guardAcc = 0.0f, guardNow = 0.0f, guardPrev = 0.0f, guardRef = 0.0f, guardTrim = 1.0f;
    int guardCalls = 0;
    bool wasFrozen = false;
    double lfoSin = 0.0, lfoCos = 1.0;
    std::atomic<double> tail{0.0};
    std::atomic<bool> infinite{false};
};
} // namespace

ModuleDescriptor delayProEffect()
{
    auto d = effectDescriptor(
        "fx.delay_pro", "Delay Pro", "Delay", CpuClass::Medium,
        "Stereo delay with two engines. Digital reads the line with Kaiser-windowed sinc interpolation, so "
        "the delay time is exact and the response stays flat to 18 kHz at any fractional time, also while "
        "modulated. Analog is a bucket-brigade device: the delay is set by the clock, so the bandwidth falls "
        "as the time grows, with companding noise and breathing and a soft clip. Tempo sync, feedback with "
        "a soft saturation in the loop (the repeats stay bounded at any setting), low and high cut in the "
        "loop, stereo, ping-pong or cross feedback, sine modulation, diffusion, ducking from the input or "
        "the sidechain, freeze, and time changes by glide (the pitch moves) or crossfade (Digital only, no "
        "pitch). No latency; the tail length is reported.");
    auto& p = d.params;
    addOn(p, "delay_pro", "Delay Pro", "fx_delay_pro", 0.3f);
    p.push_back(params::choice("fx_delay_pro_mode", "Delay Pro Mode", "fx_delay_pro", {"Digital", "Analog"}, 0));
    addF(p, "delay_pro_time", "Delay Pro Time", "fx_delay_pro", 1.0f, 4000.0f, 375.0f, "ms", 0.4f);
    p.push_back(params::choice("fx_delay_pro_sync", "Delay Pro Sync", "fx_delay_pro", params::tempoSyncNames(), 0));
    addF(p, "delay_pro_feedback", "Delay Pro Feedback", "fx_delay_pro", 0.0f, 1.0f, 0.4f);
    p.push_back(
        params::choice("fx_delay_pro_stereo", "Delay Pro Stereo", "fx_delay_pro_2", {"Stereo", "Ping-pong", "Cross"}, 0));
    addF(p, "delay_pro_right_ratio", "Delay Pro Right Time Ratio", "fx_delay_pro_2", 0.25f, 2.0f, 1.0f, "x", 0.5f);
    addHz(p, "delay_pro_low_cut", "Delay Pro Low Cut", "fx_delay_pro_2", 20.0f, 2000.0f, 20.0f, 0.3f);
    addHz(p, "delay_pro_high_cut", "Delay Pro High Cut", "fx_delay_pro_2", 500.0f, 20000.0f, 8000.0f, 0.3f);
    addF(p, "delay_pro_saturation", "Delay Pro Saturation", "fx_delay_pro_2", 0.0f, 1.0f, 0.25f);
    addF(p, "delay_pro_diffusion", "Delay Pro Diffusion", "fx_delay_pro_3", 0.0f, 1.0f, 0.0f);
    addF(p, "delay_pro_mod_depth", "Delay Pro Modulation Depth", "fx_delay_pro_3", 0.0f, 10.0f, 0.0f, "ms", 0.5f);
    addF(p, "delay_pro_mod_rate", "Delay Pro Modulation Rate", "fx_delay_pro_3", 0.05f, 10.0f, 0.6f, "Hz", 0.5f);
    p.push_back(
        params::choice("fx_delay_pro_time_change", "Delay Pro Time Change", "fx_delay_pro_2", {"Glide", "Crossfade"}, 0));
    p.push_back(params::toggle("fx_delay_pro_freeze", "Delay Pro Freeze", "fx_delay_pro_3", false));
    addF(p, "delay_pro_duck_depth", "Delay Pro Duck Depth", "fx_delay_pro_4", 0.0f, 40.0f, 0.0f, "dB");
    addF(p, "delay_pro_duck_threshold", "Delay Pro Duck Threshold", "fx_delay_pro_4", -60.0f, 0.0f, -30.0f, "dB");
    addF(p, "delay_pro_duck_release", "Delay Pro Duck Release", "fx_delay_pro_4", 20.0f, 2000.0f, 250.0f, "ms", 0.5f);
    p.push_back(
        params::choice("fx_delay_pro_duck_source", "Delay Pro Duck Source", "fx_delay_pro_4", {"Input", "Sidechain"}, 0));
    p.push_back(params::choice("fx_delay_pro_analog_stages", "Delay Pro Analog Stages", "fx_delay_pro_5",
                               {"256", "512", "1024", "2048", "4096"}, 2));
    addF(p, "delay_pro_analog_drive", "Delay Pro Analog Drive", "fx_delay_pro_5", -12.0f, 24.0f, 0.0f, "dB");
    addF(p, "delay_pro_analog_noise", "Delay Pro Analog Noise", "fx_delay_pro_5", -90.0f, -40.0f, -70.0f, "dB");
    p.push_back(params::toggle("fx_delay_pro_analog_compander", "Delay Pro Analog Compander", "fx_delay_pro_5", true));
    addF(p, "delay_pro_analog_breathing", "Delay Pro Analog Breathing", "fx_delay_pro_5", 0.0f, 1.0f, 0.5f);
    p.push_back(params::integer("fx_pos_delay_pro", "Delay Pro Slot", "fx_delay_pro_5", 1, kNumModuleFx, kNumModuleFx));
    d.onParam = "fx_delay_pro_on";
    d.positionParam = "fx_pos_delay_pro";
    d.capabilities = {"latency"};
    d.pages.push_back({"fx_delay_pro", "Delay Pro", "fx", "Delay Pro", "On, mix, engine, time, tempo sync and feedback."});
    d.pages.push_back(
        {"fx_delay_pro_2", "Delay Pro 2", "fx", "Delay Pro Loop",
         "Stereo mode, right-channel time ratio, saturation, low and high cut in the loop and the time-change mode."});
    d.pages.push_back(
        {"fx_delay_pro_3", "Delay Pro 3", "fx", "Delay Pro Movement", "Diffusion, modulation depth and rate, and freeze."});
    d.pages.push_back({"fx_delay_pro_4", "Delay Pro 4", "fx", "Delay Pro Ducking",
                       "Ducking of the repeats by the input or the sidechain."});
    d.pages.push_back({"fx_delay_pro_5", "Delay Pro 5", "fx", "Delay Pro Analog",
                       "Stage count, drive, noise, compander and breathing of the Analog engine, and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.delay_pro")); };
    return d;
}

} // namespace crucible::modules::fx
