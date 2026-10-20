// SPDX-License-Identifier: MIT
// fx.transient: a level-independent transient shaper. Two envelope followers read the louder of the two channels, a fast one
// and a slow one; their ratio in dB (fast over slow) is positive on an onset, while the fast follower is ahead, and negative
// in the decay that follows, while the slow one still holds the level. The attack gain is applied in proportion to the
// positive ratio and the sustain gain to the negative one, both through the same smooth curve (zero below 1 dB, full from
// 6 dB), after a 10 ms peak hold of the level that keeps a low tone from rippling the followers. So a steady tone, whose two
// envelopes agree, passes unchanged and a note shapes the same at any level.
//
// The gain is derived from the input and applied to the input delayed by the lookahead, so it can arrive before an onset;
// the lookahead is reported as latency. The optional clip guard bends only the part of the output above -1 dBFS toward full
// scale.
#include "FxModuleSupport.h"

#include <crucible/fx/Dynamics.h>
#include <crucible/fx/Lookahead.h>
#include <crucible/module/LatencyReporter.h>

#include <atomic>

namespace crucible::modules::fx
{
namespace
{
constexpr int kChunk = 64;
constexpr float kMaxLookaheadMs = 5.0f;
constexpr float kFloor = 1.0e-9f;
constexpr float kRatioStartDb = 1.0f;
constexpr float kRatioFullDb = 6.0f;
constexpr float kGuardStart = 0.891f;
constexpr double kHoldMs =
    10.0; // the level is held for one period of 100 Hz before the followers, so a low tone does not ripple them

inline float onePole(double timeMs, double sampleRate)
{
    return 1.0f - static_cast<float>(std::exp(-1.0 / (timeMs * 0.001 * sampleRate)));
}

class Instance final : public EffectModule, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_transient_mix")), attack(localIndex(d, "fx_transient_attack")),
          sustain(localIndex(d, "fx_transient_sustain")), speed(localIndex(d, "fx_transient_speed")),
          lookahead(localIndex(d, "fx_transient_lookahead")), guard(localIndex(d, "fx_transient_guard")),
          output(localIndex(d, "fx_transient_output"))
    {
        lookaheadDefaultMs = d.params[static_cast<size_t>(lookahead)].defaultValue;
    }

    void prepare(const ModuleContext& c) override
    {
        rate = c.sampleRate;
        look.prepare(2, static_cast<int>(std::ceil(kMaxLookaheadMs * 0.001 * rate)) + 2, rate);
        smoothGain = onePole(0.6, rate);
        const int holdSamples = std::max(1, static_cast<int>(std::lround(kHoldMs * 0.001 * rate)));
        hold.prepare(holdSamples);
        hold.setWindow(holdSamples);
        // The lookahead is a parameter the module only sees in process(); until then it assumes the default, so the latency
        // is reported from prepare() on and stays the same across reset() and enable / disable.
        lookaheadSamples = std::clamp(static_cast<int>(std::lround(lookaheadDefaultMs * 0.001 * rate)), 0, look.maxDelay());
        reset();
    }
    void reset() override
    {
        look.reset();
        look.setDelay(lookaheadSamples);
        hold.reset();
        fast = slow = kFloor;
        target = 0.0f;
        gainDb = 0.0f;
        mixSmooth = -1.0f;
        outStage1 = outStage2 = -1.0f;
        reported = 0;
        latency.store(lookaheadSamples, std::memory_order_relaxed);
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
        dirty = true;
        const float attackDb = in.value(attack), sustainDb = in.value(sustain);
        const double speedValue = std::clamp(static_cast<double>(in.value(speed)), 0.25, 4.0);
        // Fast follower: 0.5 ms up, 12 ms down. Slow follower: 25 ms up, 160 ms down. Speed divides all four times.
        const float fastUp = onePole(0.5 / speedValue, rate), fastDown = onePole(12.0 / speedValue, rate);
        const float slowUp = onePole(25.0 / speedValue, rate), slowDown = onePole(160.0 / speedValue, rate);
        const int delay = std::clamp(static_cast<int>(std::lround(in.value(lookahead) * 0.001 * rate)), 0, look.maxDelay());
        if (delay != look.delay())
        {
            look.setDelay(delay);
            lookaheadSamples = delay;
            latency.store(delay, std::memory_order_relaxed);
            latencyChanged.store(true, std::memory_order_relaxed);
        }
        const float mixTarget = in.value(mix);
        if (mixSmooth < 0.0f)
            mixSmooth = mixTarget;
        const float outTarget = crucible::fx::dyn::dbToGain(in.value(output));
        if (outStage1 < 0.0f)
            outStage1 = outStage2 = outTarget;
        const bool clipGuard = in.on(guard);
        const float mixStep = 1.0f - std::exp(-1.0f / (0.005f * static_cast<float>(rate)));

        float gain[kChunk];
        for (int pos = 0; pos < n; pos += kChunk)
        {
            const int m = std::min(kChunk, n - pos);
            float* a = l + pos;
            float* b = r + pos;
            for (int i = 0; i < m; ++i)
            {
                const float level = std::max(hold.push(std::max(std::abs(a[i]), std::abs(b[i]))), kFloor);
                fast += (level - fast) * (level > fast ? fastUp : fastDown);
                slow += (level - slow) * (level > slow ? slowUp : slowDown);
                // The curve is evaluated every second sample (24 kHz at 48 kHz, far above the followers); the smoothing
                // below runs at the full rate.
                if ((i & 1) == 0)
                {
                    // 20 log10 of the ratio: 6.0206 dB per octave.
                    const float ratioDb = 6.0205999f * crucible::fx::dyn::log2Fast(fast / slow);
                    const float magnitude = std::abs(ratioDb);
                    float t = (magnitude - kRatioStartDb) * (1.0f / (kRatioFullDb - kRatioStartDb));
                    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
                    const float shape = t * t * (3.0f - 2.0f * t);
                    target = (ratioDb > 0.0f ? attackDb : sustainDb) * shape;
                }
                gainDb += (target - gainDb) * smoothGain;
                // The output gain moves per sample through two poles of about 5 ms, so a value that arrives once per host
                // block is smooth.
                outStage1 += (outTarget - outStage1) * mixStep;
                outStage2 += (outStage1 - outStage2) * mixStep;
                gain[i] = crucible::fx::dyn::dbToGain(gainDb) * outStage2;
            }
            float* io[2] = {a, b};
            look.process(io, m);
            for (int i = 0; i < m; ++i)
            {
                mixSmooth += (mixTarget - mixSmooth) * mixStep;
                float yl = a[i] * gain[i], yr = b[i] * gain[i];
                if (clipGuard)
                {
                    yl = guardSample(yl);
                    yr = guardSample(yr);
                }
                a[i] += (yl - a[i]) * mixSmooth;
                b[i] += (yr - b[i]) * mixSmooth;
            }
        }
    }

    int latencySamples() const override { return latency.load(std::memory_order_relaxed); }
    bool takeLatencyChanged() override { return latencyChanged.exchange(false, std::memory_order_relaxed); }

private:
    // Identity up to kGuardStart, then a tanh knee that reaches 1.0 asymptotically (value and slope are continuous).
    static float guardSample(float x)
    {
        const float magnitude = std::abs(x);
        if (magnitude <= kGuardStart)
            return x;
        const float room = 1.0f - kGuardStart;
        const float y = kGuardStart + room * std::tanh((magnitude - kGuardStart) / room);
        return x < 0.0f ? -y : y;
    }

    const int mix, attack, sustain, speed, lookahead, guard, output;
    double rate = 48000.0;
    crucible::fx::Lookahead look;
    int lookaheadSamples = 0;
    float lookaheadDefaultMs = 0.0f;
    crucible::fx::HoldMax hold;
    float fast = kFloor, slow = kFloor, target = 0.0f, gainDb = 0.0f, smoothGain = 0.01f, mixSmooth = -1.0f,
          outStage1 = -1.0f, outStage2 = -1.0f;
    int reported = 0;
    bool dirty = false;
    std::atomic<int> latency{0};
    std::atomic<bool> latencyChanged{false};
};
} // namespace

ModuleDescriptor transientEffect()
{
    auto d =
        effectDescriptor("fx.transient", "Transient Shaper", "Dynamics", CpuClass::Light,
                         "Level-independent transient shaper. A fast and a slow envelope follower read the louder channel; "
                         "their ratio in dB is positive on an onset and negative in the decay. Attack and sustain gains of "
                         "plus or minus 15 dB are applied through a smooth curve that is zero below 1 dB of ratio and full "
                         "from 6 dB, so a steady tone is untouched and a note is shaped the same at any input level. "
                         "Speed scales the follower times, the lookahead (up to 5 ms, reported as latency) lets the gain "
                         "arrive before an onset, and the clip guard bends the output above -1 dBFS toward full scale.");
    auto& p = d.params;
    addOn(p, "transient", "Transient", "fx_transient", 1.0f);
    addF(p, "transient_attack", "Transient Attack", "fx_transient", -15.0f, 15.0f, 0.0f, "dB");
    addF(p, "transient_sustain", "Transient Sustain", "fx_transient", -15.0f, 15.0f, 0.0f, "dB");
    addF(p, "transient_speed", "Transient Speed", "fx_transient_2", 0.25f, 4.0f, 1.0f, "x", 0.5f);
    addF(p, "transient_lookahead", "Transient Lookahead", "fx_transient_2", 0.0f, 5.0f, 0.0f, "ms");
    p.push_back(params::toggle("fx_transient_guard", "Transient Clip Guard", "fx_transient_2", true));
    addF(p, "transient_output", "Transient Output", "fx_transient_2", -12.0f, 12.0f, 0.0f, "dB");
    p.push_back(params::integer("fx_pos_transient", "Transient Slot", "fx_transient_2", 1, kNumModuleFx, kNumModuleFx));
    d.onParam = "fx_transient_on";
    d.positionParam = "fx_pos_transient";
    d.capabilities = {"latency"};
    d.pages.push_back({"fx_transient", "Transient", "fx", "Transient Shaper", "On, mix, attack and sustain gain."});
    d.pages.push_back({"fx_transient_2", "Transient 2", "fx", "Transient Shaper Detector",
                       "Follower speed, lookahead, clip guard, output gain and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.transient")); };
    return d;
}

} // namespace crucible::modules::fx
