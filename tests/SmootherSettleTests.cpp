// SPDX-License-Identifier: MIT
// Smoothed controls must end exactly on their target. A float one-pole stalls when coef * (target - v) rounds to nothing,
// which can happen before a relative-epsilon snap fires; the control then never reaches its target, the moving per-sample
// path runs forever and a constant setting never returns to the bit-identical static path. One test per module: change
// the controls, render long enough, and the smoothed values equal the targets (and the output keeps its guarantees).

#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/fx/Dynamics.h>
#include <crucible/fx/Fdn.h>
#include <crucible/fx/Maximizer.h>
#include <crucible/fx/Smoothing.h>
#include <crucible/module/ModuleRack.h>

#include <cmath>
#include <vector>

using namespace crucible;
using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;
using crucible::testing::kRate;
namespace cfx = crucible::fx;

namespace
{
template <class Fn> void blocks(std::vector<float>& l, std::vector<float>& r, int block, Fn&& fn)
{
    for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
    {
        const int n = static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - pos));
        fn(l.data() + pos, r.data() + pos, n);
    }
}

std::vector<float> floats(const Signal& x)
{
    return std::vector<float>(x.begin(), x.end());
}

std::vector<float> noiseFloats(size_t n, float level, unsigned seed)
{
    std::vector<float> x(n);
    unsigned s = seed;
    for (float& v : x)
    {
        s = s * 1664525u + 1013904223u;
        v = level * (static_cast<float>(s >> 8) / 8388608.0f - 1.0f);
    }
    return x;
}
} // namespace

class SmootherSettleTests : public UnitTest
{
public:
    SmootherSettleTests() : UnitTest("Smoother settle", "Effects") {}

    void runTest() override
    {
        beginTest("the shared step lands on the target when the step rounds to nothing");
        {
            // The case measured in the review: a threshold of -20 dB approached with a 6 ms one-pole at 44.1 kHz.
            const float coef = 1.0f - std::exp(-1000.0f / (6.0f * 44100.0f));
            float v = -20.00027f;
            for (int i = 0; i < 100000 && v != -20.0f; ++i)
                v = cfx::onePoleStepSnap(v, -20.0f, coef);
            expect(v == -20.0f, "reaches -20 exactly");
            float w = 0.0f;
            for (int i = 0; i < 100000; ++i)
                w = cfx::onePoleStepSnap(w, 1.0f, coef);
            expect(w == 1.0f, "reaches 1 exactly");
        }

        beginTest("dynamics: after a change and a settle every smoothed control sits on its target");
        {
            for (const double rate : {44100.0, 48000.0, 96000.0})
            {
                cfx::DynamicsProcessor p;
                p.prepare(rate);
                cfx::DynamicsSettings s;
                std::vector<float> a = noiseFloats(static_cast<size_t>(0.3 * rate), 0.3f, 1u), b = a;
                blocks(a, b, 64, [&](float* l, float* r, int n) { p.process(l, r, n, nullptr, nullptr, s); });
                expect(p.controlsSettled(), "settled on a constant setting");
                s.thresholdDb = -20.0f;
                s.ratio = 3.1f;
                s.kneeDb = 5.0f;
                s.mix = 0.999991f;
                s.makeupDb = 1.3f;
                s.rangeDb = 48.0f;
                s.scBellHz = 2300.0f;
                a = noiseFloats(static_cast<size_t>(3.0 * rate), 0.3f, 2u);
                b = a;
                blocks(a, b, 64, [&](float* l, float* r, int n) { p.process(l, r, n, nullptr, nullptr, s); });
                logMessage(String("dynamics at ") + String(rate, 0) + " Hz: settled " +
                           String(p.controlsSettled() ? 1 : 0));
                expect(p.controlsSettled(), "all controls on their targets at " + std::to_string(static_cast<int>(rate)));
            }
        }

        beginTest("maximizer: ceiling, gain and clip share settle exactly and the output never exceeds the set ceiling");
        {
            for (const double rate : {44100.0, 48000.0})
                for (const float ceilingDb : {-6.0f, -3.0f, -0.3f})
                {
                    cfx::TruePeakLimiter lim;
                    lim.prepare(rate, cfx::TruePeakLimiter::Detect::TruePeak);
                    cfx::LimiterSettings s;
                    std::vector<float> a = noiseFloats(static_cast<size_t>(0.3 * rate), 1.0f, 3u), b = a;
                    blocks(a, b, 64, [&](float* l, float* r, int n) { lim.process(l, r, n, s); });
                    s.ceilingDb = ceilingDb;
                    s.gainDb = 4.0f;
                    s.clipShare = 0.5f;
                    a = noiseFloats(static_cast<size_t>(3.0 * rate), 1.0f, 4u);
                    b = a;
                    blocks(a, b, 64, [&](float* l, float* r, int n) { lim.process(l, r, n, s); });
                    // After the settle: one more second of loud noise; nothing may exceed the ceiling.
                    a = noiseFloats(static_cast<size_t>(1.0 * rate), 1.0f, 5u);
                    b = a;
                    blocks(a, b, 64, [&](float* l, float* r, int n) { lim.process(l, r, n, s); });
                    float peak = 0.0f;
                    for (size_t i = 0; i < a.size(); ++i)
                        peak = std::max({peak, std::abs(a[i]), std::abs(b[i])});
                    const float ceiling = cfx::dyn::dbToGain(ceilingDb);
                    logMessage(String("maximizer ") + String(rate, 0) + " Hz, ceiling " + String(ceilingDb, 1) +
                               " dB: settled " + String(lim.controlsSettled() ? 1 : 0) + ", peak above ceiling " +
                               String(static_cast<double>(peak - ceiling), 9));
                    expect(lim.controlsSettled(), "controls on their targets");
                    expect(peak <= ceiling, "output peak does not exceed the set ceiling");
                }
        }

        beginTest("reverb: mix, width, size, density and modulation depth settle exactly");
        {
            for (const double rate : {44100.0, 48000.0})
            {
                cfx::AlgorithmicReverb rv;
                rv.prepare(rate, 16, 7u);
                cfx::ReverbSettings s;
                std::vector<float> a = noiseFloats(static_cast<size_t>(0.3 * rate), 0.2f, 6u), b = a;
                blocks(a, b, 64, [&](float* l, float* r, int n) { rv.process(l, r, n, s); });
                s.mix = 0.37f;
                s.width = 0.83f;
                s.size = 0.61f;
                s.density = 0.43f;
                s.modDepth = 0.21f;
                s.early = 0.4f;
                a = noiseFloats(static_cast<size_t>(8.0 * rate), 0.2f, 7u);
                b = a;
                blocks(a, b, 64, [&](float* l, float* r, int n) { rv.process(l, r, n, s); });
                logMessage(String("reverb at ") + String(rate, 0) + " Hz: settled " + String(rv.controlsSettled() ? 1 : 0));
                expect(rv.controlsSettled(), "all controls on their targets at " + std::to_string(static_cast<int>(rate)));
            }
        }

        beginTest("eq_pro: after output and mix change and a settle the impulse response equals a fresh instance's");
        {
            const auto render = [](double rate, bool changeFirst)
            {
                ModuleRack rack(ModuleRegistry::instance());
                ModuleContext ctx;
                ctx.sampleRate = rate;
                rack.prepare(ctx);
                std::vector<float> values = ParameterMap::defaultValues();
                std::vector<float> legacy(static_cast<size_t>(kNumModDests), 0.0f);
                const auto set = [&](const std::string& id, float v)
                { values[static_cast<size_t>(ParameterMap::indexOf(id))] = v; };
                for (int i = 0; i < rack.size(); ++i)
                    set(rack.descriptor(i).onParam, 0.0f);
                set("fx_ep_on", 1.0f);
                set("fx_ep_b1_on", 1.0f);
                set("fx_ep_b1_freq", 1200.0f);
                set("fx_ep_b1_gain", 4.0f);
                set("fx_ep_b1_q", 1.5f);
                const auto run = [&](size_t n, size_t impulseAt)
                {
                    std::vector<float> l(n, 0.0f), r;
                    if (impulseAt < n)
                        l[impulseAt] = 1.0f;
                    r = l;
                    blocks(l, r, 32,
                           [&](float* pl, float* pr, int m)
                           { rack.process(pl, pr, m, values.data(), legacy.data(), nullptr, 120.0, nullptr, nullptr); });
                    return l;
                };
                if (changeFirst)
                    run(static_cast<size_t>(0.2 * rate), ~size_t{0});
                set("fx_ep_out", -6.3f);
                set("fx_ep_mix", 0.37f);
                run(static_cast<size_t>(3.0 * rate), ~size_t{0});
                return run(4096, 100);
            };
            for (const double rate : {44100.0, 48000.0})
            {
                const std::vector<float> changed = render(rate, true), fresh = render(rate, false);
                size_t differing = 0;
                for (size_t i = 0; i < changed.size(); ++i)
                    differing += changed[i] != fresh[i] ? 1 : 0;
                logMessage(String("eq_pro at ") + String(rate, 0) + " Hz: " + String(static_cast<double>(differing), 0) +
                           " of 4096 impulse response samples differ from a fresh instance");
                expectEquals(static_cast<int>(differing), 0, "impulse response bit-identical to a fresh instance");
            }
        }
    }
};

static SmootherSettleTests smootherSettleTests;
