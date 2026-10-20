// SPDX-License-Identifier: MIT
// Unison detune, spread, blend and scatter as per-voice modulation targets.

#include "GraphSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
// Side energy over mid energy: how wide the stack sounds.
double width(const Rendered& r, size_t from)
{
    double mid = 0.0, side = 0.0;
    for (size_t i = from; i < r.left.size(); ++i)
    {
        const double m = 0.5 * (r.left[i] + r.right[i]), s = 0.5 * (r.left[i] - r.right[i]);
        mid += m * m;
        side += s * s;
    }
    return side / std::max(mid, 1.0e-20);
}

Rendered renderWith(const std::vector<float>& values, const ModRoute* route, int block, int samples = 16384)
{
    SynthEngine engine;
    prepare(engine, values);
    if (route != nullptr)
        engine.modRoutes().add(*route);
    engine.noteOn(52, 1.0f);
    return renderBlocks(engine, samples, block, true);
}

double maxDifference(const Rendered& a, const Rendered& b)
{
    double d = 0.0;
    for (size_t i = 0; i < a.left.size(); ++i)
        d = std::max({d, static_cast<double>(std::abs(a.left[i] - b.left[i])),
                      static_cast<double>(std::abs(a.right[i] - b.right[i]))});
    return d;
}

std::vector<float> stackPatch()
{
    auto v = graphPatch();
    setValue(v, "osc1_uni_voices", 7.0f);
    setValue(v, "osc1_uni_detune", 30.0f);
    setValue(v, "osc1_uni_spread", 1.0f);
    setValue(v, "osc1_uni_mode", 1.0f);
    setValue(v, "osc1_uni_rand", 0.0f);
    return v;
}
} // namespace

class UnisonModulationTests : public UnitTest
{
public:
    UnisonModulationTests() : UnitTest("Unison modulation", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("detune, spread, blend and scatter of every oscillator slot are per-voice targets");
        {
            const int slots = ModuleRegistry::instance().layout().voice.oscillators;
            for (int o = 1; o <= slots; ++o)
            {
                const auto p = "osc" + std::to_string(o);
                for (const char* suffix :
                     {"_uni_spread", "_uni_blend", "_uni_scatter", "_smp_uni_detune", "_smp_uni_spread"})
                {
                    const int i = ParameterMap::indexOf(p + suffix);
                    expect(i >= 0 && ParameterMap::voiceTargetOfParam(i) >= 0, p + suffix);
                    expect(ModRouteStore::canTarget(p + suffix), p + suffix);
                }
                // Slot 1's stack detune keeps its fixed destination; no second entry.
                const int d = ParameterMap::indexOf(p + "_uni_detune");
                expect(ModRouteStore::canTarget(p + "_uni_detune"));
                expect((ParameterMap::voiceTargetOfParam(d) >= 0) == (o > 1));
            }
            // Counts, mode and random phase are not continuous per-voice values.
            for (const char* id : {"osc1_uni_voices", "osc1_uni_mode", "osc1_uni_rand", "osc1_smp_uni_voices"})
                expect(!ModRouteStore::canTarget(id), id);
        }

        beginTest("a route to the spread narrows the stack, equal to setting the parameter");
        {
            const auto v = stackPatch();
            ModRoute route;
            route.source = static_cast<int>(ModSource::Velocity);
            route.target = "osc1_uni_spread";
            route.amount = -1.0f;
            const auto plain = renderWith(v, nullptr, 512);
            const auto routed = renderWith(v, &route, 512);
            expect(width(plain, 4000) > 0.05, "the plain stack is wide");
            expect(width(routed, 4000) < 1.0e-6, "spread pulled to zero");
            auto narrow = v;
            setValue(narrow, "osc1_uni_spread", 0.0f);
            expect(maxDifference(routed, renderWith(narrow, nullptr, 512)) < 1.0e-5, "same as the parameter");
        }

        beginTest("routes to the detune of slot 2, to blend and to scatter change the sound as their parameters do");
        {
            auto v = stackPatch();
            setValue(v, "osc2_level", 0.8f);
            setValue(v, "osc2_uni_voices", 5.0f);
            setValue(v, "osc2_uni_rand", 0.0f);
            const auto check = [&](const char* target, float amount, float paramValue)
            {
                ModRoute route;
                route.source = static_cast<int>(ModSource::Velocity);
                route.target = target;
                route.amount = amount;
                const auto base = renderWith(v, nullptr, 512);
                const auto routed = renderWith(v, &route, 512);
                auto set = v;
                setValue(set, target, paramValue);
                const auto param = renderWith(set, nullptr, 512);
                expect(maxDifference(base, routed) > 1.0e-3, std::string(target) + " is audible");
                expect(maxDifference(routed, param) < 1.0e-4, std::string(target) + " equals the parameter");
            };
            check("osc2_uni_detune", 0.25f, 20.0f + 25.0f);
            check("osc2_uni_blend", -0.5f, 0.5f);
            setValue(v, "osc2_uni_mode", 7.0f);
            check("osc2_uni_scatter", 0.5f, 0.5f);
        }

        beginTest("deterministic, and identical across block sizes on the 32-sample grid");
        {
            auto v = stackPatch();
            setValue(v, "osc1_uni_rand", 1.0f);
            ModRoute route;
            route.source = static_cast<int>(ModSource::Velocity);
            route.target = "osc1_uni_scatter";
            route.amount = 0.7f;
            const auto a = renderWith(v, &route, 512);
            const auto b = renderWith(v, &route, 512);
            const auto c = renderWith(v, &route, 32);
            const auto d = renderWith(v, &route, 128);
            expect(maxDifference(a, b) == 0.0, "repeatable");
            expect(maxDifference(a, c) == 0.0, "block 32");
            expect(maxDifference(a, d) == 0.0, "block 128");
        }

        beginTest("with no route the stack is unchanged (all four values stay at their parameters)");
        {
            const auto v = stackPatch();
            expect(maxDifference(renderWith(v, nullptr, 512), renderWith(v, nullptr, 64)) == 0.0);
        }
    }
};

static UnisonModulationTests unisonModulationTests;
