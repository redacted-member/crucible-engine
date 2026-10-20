// SPDX-License-Identifier: MIT
#include <crucible/module/ConvolutionIr.h>
#include <crucible/Engine.h>
#include <crucible/module/Module.h>
#include <crucible/module/ModuleRack.h>
#include "TestSupport.h"

#include <set>

using namespace crucible;
using namespace crucible::testing;

class ModuleTests : public UnitTest
{
public:
    ModuleTests() : UnitTest("Module registry", "Engine") {}

    void runTest() override
    {
        const auto& registry = ModuleRegistry::instance();

        beginTest("descriptors are complete and ids unique; every parameter belongs to a module and a page");
        {
            std::set<std::string> ids;
            for (const auto& m : registry.modules())
            {
                expect(ids.insert(m.id).second, "duplicate module " + m.id);
                expect(!m.name.empty() && !m.category.empty() && m.version >= 1 &&
                           (!m.params.empty() || !m.lateVoiceMod.empty() || m.id == "osc.wavetable"),
                       m.id);
                if (m.kind == ModuleKind::Effect)
                {
                    expect(static_cast<bool>(m.createEffect), m.id + " has no factory");
                    expect(ParameterMap::indexOf(m.onParam) >= 0 && ParameterMap::indexOf(m.positionParam) >= 0, m.id);
                }
            }
            for (const auto& p : ParameterMap::params())
            {
                expect(registry.find(p.module) != nullptr, p.id + " has no module");
                expect(ParameterMap::pages()[static_cast<size_t>(p.page)].id == p.pageId, p.id);
            }
            expectEquals(static_cast<int>(registry.ofKind(ModuleKind::Effect).size()), kNumModuleFx);
            expect(registry.layout().id == "crucible.default" && registry.layout().sections.size() == 5);
        }

        beginTest("generic modulation targets are continuous parameters of registered modules");
        {
            const auto& targets = ParameterMap::modTargets();
            expect(!targets.empty());
            for (const int t : targets)
                expect(ParameterMap::params()[static_cast<size_t>(t)].type == ParamType::Float);
        }

        beginTest("the module rack runs every effect at its defaults and at random settings without NaN");
        {
            ModuleRack rack(ModuleRegistry::instance());
            ModuleContext ctx;
            ctx.sampleRate = kRate;
            rack.prepare(ctx);
            expectEquals(rack.size(), kNumModuleFx);
            expect(rack.findCapability<ConvolutionIr>() != nullptr);
            expect(rack.findCapability<GainReductionMeter>("fx.multiband") != nullptr);
            expect(rack.find("fx.nothing") == nullptr);
            auto values = ParameterMap::defaultValues();
            const auto& defs = ParameterMap::params();
            std::vector<float> legacy(static_cast<size_t>(kNumModDests), 0.0f);
            Rng rng;
            Rng moduleOnlyRng; // modules beyond the original twenty draw from their own stream
            for (int round = 0; round < 6; ++round)
            {
                for (int i = 0; i < rack.size(); ++i)
                    for (const auto& p : rack.descriptor(i).params)
                    {
                        const auto g = static_cast<size_t>(ParameterMap::indexOf(p.id));
                        const auto& d = defs[g];
                        float v = d.min + (d.max - d.min) * 0.5f * ((i < kNumFx ? rng : moduleOnlyRng).bipolar() + 1.0f);
                        if (d.type != ParamType::Float)
                            v = std::round(v);
                        values[g] = round == 0 ? d.defaultValue : v;
                    }
                for (int i = 0; i < rack.size(); ++i)
                    values[static_cast<size_t>(ParameterMap::indexOf(rack.descriptor(i).onParam))] = 1.0f;
                values[static_cast<size_t>(ParameterMap::indexOf("fx_cx_on"))] = round < 2 ? 1.0f : 0.0f;
                std::vector<float> l(4800), r(4800);
                for (size_t i = 0; i < l.size(); ++i)
                {
                    l[i] = 0.3f * rng.bipolar();
                    r[i] = 0.3f * rng.bipolar();
                }
                for (int pos = 0; pos < 4800; pos += 32)
                    rack.process(l.data() + pos, r.data() + pos, 32, values.data(), legacy.data(), nullptr, 120.0);
                // Every effect on at random settings: stages with make-up gain stack (prefix peaks of 100 to 700 occur
                // before the newest effects), so the bound only catches a runaway; per-effect bounds are in the effects' own
                // tests.
                bool ok = true;
                for (size_t i = 0; i < l.size(); ++i)
                    // Finite and bounded: stacked make-up gains at random settings legitimately reach a few hundred
                    // (fx.comp_clean alone peaks at 171 on this input), so 1000 is the bound (maintainer decision); each
                    // module's own extremes are checked by the module check's "all at max" case.
                    ok = ok && std::isfinite(l[i]) && std::isfinite(r[i]) && std::abs(l[i]) < 1000.0f;
                expect(ok, "round " + String(round));
            }
        }
    }
};

class RouteTests : public UnitTest
{
public:
    RouteTests() : UnitTest("Modulation routes", "Engine") {}

    void runTest() override
    {
        beginTest("route store: validation, replace, per-parameter listing, serialisation");
        {
            ModRouteStore store;
            expect(store.add({static_cast<int>(ModSource::Macro1), "fx_ensemble_rate", 0.5f}));
            expect(store.add({static_cast<int>(ModSource::Lfo1), "filter_cutoff", 0.3f, 2, 0.2f}));
            expect(store.add({kSourceCC + 74, "fx_delay_pro_mix", -0.25f, 0, 0.0f, 50.0f}));
            expect(!store.add({static_cast<int>(ModSource::Macro1), "osc1_table", 0.5f}), "choices are not targets");
            expect(!store.add({static_cast<int>(ModSource::Macro1), "nope", 0.5f}));
            expect(!store.add({0, "fx_ensemble_rate", 0.5f}), "no source");
            {
                const float nan = std::numeric_limits<float>::quiet_NaN();
                ModRouteStore sane;
                expect(sane.add({static_cast<int>(ModSource::Macro1), "fx_ensemble_rate", nan, 0, nan, nan}));
                const auto r = sane.routes();
                expect(r.size() == 1 && r[0].amount == 0.0f && r[0].curve == 0.0f && r[0].smoothingMs == 0.0f,
                       "non-finite route values are stored as 0");
            }
            expect(store.add({static_cast<int>(ModSource::Macro1), "fx_ensemble_rate", 0.75f})); // replaces
            expectEquals(static_cast<int>(store.routes().size()), 3);
            expect(store.routesFor("fx_ensemble_rate").size() == 1 && store.routesFor("fx_ensemble_rate")[0].amount == 0.75f);
            ModRouteStore back;
            back.deserialise(store.serialise());
            const auto r = back.routes();
            expect(r.size() == 3 && r[2].source == kSourceCC + 74 && r[2].smoothingMs == 50.0f && r[1].polarity == 2);
            expect(back.remove(kSourceCC + 74, "fx_delay_pro_mix") && !back.remove(kSourceCC + 74, "fx_delay_pro_mix"));
            expectEquals(routeSourceName(kSourceCC + 7), std::string("CC 7"));
            expectEquals(routeSourceName(static_cast<int>(ModSource::Lfo2)), std::string("LFO 2"));
        }

        beginTest("global targets: routes and generic matrix slots move effect parameters; read-back reports it");
        {
            Engine e;
            e.prepare(kRate, 256);
            e.setParam("macro1", 1.0f);
            e.modRoutes().add({static_cast<int>(ModSource::Macro1), "fx_ensemble_rate", 0.5f});
            std::vector<float> l(256), r(256);
            for (int i = 0; i < 20; ++i)
                e.process(l.data(), r.data(), 256);
            const auto& d = e.model().params()[static_cast<size_t>(e.model().indexOf("fx_ensemble_rate"))];
            expectWithinAbsoluteError(e.modulatedValue("fx_ensemble_rate"), d.defaultValue + 0.5f * (d.max - d.min), 1.0e-4f);
            expectWithinAbsoluteError(
                e.modulatedValue("fx_delay_pro_time"),
                e.model().params()[static_cast<size_t>(e.model().indexOf("fx_delay_pro_time"))].defaultValue, 1.0e-6f);

            // A matrix slot pointed at a generic destination does the same.
            const auto& names = ParameterMap::modDestNames();
            int dst = -1;
            for (size_t i = 0; i < names.size(); ++i)
                if (names[i] == "Delay Pro Feedback")
                    dst = static_cast<int>(i);
            expect(dst >= kNumModDests);
            e.setParam("mod1_src", static_cast<float>(ModSource::Macro1));
            e.setParam("mod1_dst", static_cast<float>(dst));
            e.setParam("mod1_amt", -1.0f);
            for (int i = 0; i < 20; ++i)
                e.process(l.data(), r.data(), 256);
            expectWithinAbsoluteError(e.modulatedValue("fx_delay_pro_feedback"), 0.0f, 1.0e-6f); // clamped at the minimum
        }

        beginTest("CC-driven route to a voice destination changes the sound; routes leave sound alone at zero");
        {
            const auto render = [](bool route, int ccValue)
            {
                Engine e;
                e.prepare(kRate, 256);
                e.setParam("filter_cutoff", 300.0f);
                e.setParam("osc1_pos", 1.0f); // saw: harmonics for the filter to act on
                if (route)
                    e.modRoutes().add({kSourceCC + 74, "filter_cutoff", 1.0f});
                MidiInput midi[2] = {{0, 0xb0, 74, static_cast<std::uint8_t>(ccValue)}, {0, 0x90, 48, 110}};
                std::vector<float> l(4800), r(4800);
                e.process(l.data(), r.data(), 4800, midi, 2);
                e.process(l.data(), r.data(), 4800);
                double energyHigh = 0.0;
                for (size_t i = 1; i < l.size(); ++i)
                    energyHigh += (l[i] - l[i - 1]) * (l[i] - l[i - 1]);
                return std::make_pair(l, energyHigh);
            };
            const auto plain = render(false, 127);
            const auto zero = render(true, 0);
            const auto open = render(true, 127);
            expect(plain.first == zero.first, "a route at zero output is bit-transparent");
            expect(open.second > plain.second * 4.0, "CC 74 opens the filter");
        }
    }
};

static ModuleTests moduleTests;
static RouteTests routeTests;
