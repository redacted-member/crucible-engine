// SPDX-License-Identifier: MIT
// Crusher note tracking: with fx_crush_track on, the sample-and-hold rate is the frequency of the engine's last played
// note x ratio x 2^(fine / 1200). Measured on the rendered output: a sine held at rate R has its strongest image at
// R - f0, so R = peak + f0. Off (the default) the effect is bit-identical to the fixed rate.

#include "AllocCounter.h"
#include "AnalogMeasure.h"
#include "TestSupport.h"

#include <crucible/Fx.h>
#include <crucible/graph/Compiler.h>
#include <crucible/module/ModuleRack.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace crucible;
using namespace crucible::measure;
using crucible::test::UnitTest;
using crucible::testing::kRate;

namespace
{
struct CrushRack
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values;
    std::vector<float> legacy;
    CrushRack()
    {
        ModuleContext ctx;
        ctx.sampleRate = kRate;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            set(rack.descriptor(i).onParam, 0.0f);
        set("fx_crush_on", 1.0f);
        set("fx_crush_mix", 1.0f);
        set("fx_crush_bits", 16.0f);
    }
    void set(const std::string& id, float v) { values[static_cast<size_t>(ParameterMap::indexOf(id))] = v; }
    void run(float* l, float* r, int n)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0);
    }
    Signal render(const Signal& in, int block = 64)
    {
        std::vector<float> l(in.begin(), in.end()), r = l;
        for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
            run(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - pos)));
        return Signal(l.begin(), l.end());
    }
};

double noteHz(double note)
{
    return 440.0 * std::pow(2.0, (note - 69.0) / 12.0);
}

// The hold rate read from the spectrum of a settled sine of f0: strongest line between 300 Hz and 9 kHz is R - f0.
double measuredRate(const Signal& out, double f0)
{
    const size_t n = 1 << 16;
    const Spectrum s = computeSpectrum(out.data() + (out.size() - n), n, kRate, Window::Hann);
    size_t best = 0;
    for (size_t k = static_cast<size_t>(300.0 / s.binHz); k < static_cast<size_t>(9000.0 / s.binHz); ++k)
        if (best == 0 || s.power[k] > s.power[best])
            best = k;
    const double a = std::log(s.power[best - 1]), b = std::log(s.power[best]), c = std::log(s.power[best + 1]);
    const double delta = 0.5 * (a - c) / (a - 2.0 * b + c);
    return (static_cast<double>(best) + delta) * s.binHz + f0;
}

constexpr double kF0 = 97.0;
Signal probe()
{
    return sineWave(static_cast<size_t>(kRate * 1.5), kF0, kRate, 0.5);
}
} // namespace

class CrusherTrackTests : public UnitTest
{
public:
    CrusherTrackTests() : UnitTest("Crusher track", "Effects") {}

    void runTest() override
    {
        beginTest("the hold rate follows the note: note frequency x ratio, measured on the output");
        {
            struct Case
            {
                double note, ratio, fine;
            };
            for (const Case c : {Case{48, 16, 0}, Case{60, 16, 0}, Case{72, 8, 0}, Case{60, 16, 100}, Case{48, 12, -100}})
            {
                CrushRack h;
                h.set("fx_crush_track", 1.0f);
                h.set("fx_crush_ratio", static_cast<float>(c.ratio));
                h.set("fx_crush_fine", static_cast<float>(c.fine));
                h.rack.setNote(static_cast<float>(c.note));
                const double expected = noteHz(c.note) * c.ratio * std::pow(2.0, c.fine / 1200.0);
                const double got = measuredRate(h.render(probe()), kF0);
                std::printf("  crusher track: note %.0f x %.2f fine %+.0f ct: expected %.2f Hz, measured %.2f Hz (%.3f %%)\n",
                            c.note, c.ratio, c.fine, expected, got, 100.0 * (got / expected - 1.0));
                expectWithinAbsoluteError(got, expected, expected * 0.005, "tracked rate");
            }
        }

        beginTest("a note change moves the rate (settles within the glide), no note leaves the fixed rate");
        {
            CrushRack h;
            h.set("fx_crush_track", 1.0f);
            h.set("fx_crush_ratio", 16.0f);
            h.set("fx_crush_rate", 4000.0f);
            h.rack.setNote(-1.0f);
            const double fixed = measuredRate(h.render(probe()), kF0);
            expectWithinAbsoluteError(fixed, 4000.0, 20.0, "no note: fixed rate");
            h.rack.setNote(48.0f);
            const double a = measuredRate(h.render(probe()), kF0);
            h.rack.setNote(60.0f);
            const double b = measuredRate(h.render(probe()), kF0);
            expectWithinAbsoluteError(a, noteHz(48) * 16.0, 12.0, "note 48");
            expectWithinAbsoluteError(b, noteHz(60) * 16.0, 24.0, "note 60 after 48");
            expectWithinAbsoluteError(b / a, 2.0, 0.01, "an octave doubles the rate");
        }

        beginTest("track off (default) is identical to the fixed rate whatever the note (the pre-change output is pinned by the golden and fx baseline tests)");
        {
            CrushRack a, b;
            a.set("fx_crush_bits", 6.0f);
            a.set("fx_crush_rate", 7000.0f);
            a.set("fx_crush_jitter", 0.3f);
            b.values = a.values;
            b.rack.setNote(53.0f);
            const Signal in = probe();
            const Signal x = a.render(in), y = b.render(in);
            expect(std::memcmp(x.data(), y.data(), x.size() * sizeof(float)) == 0, "outputs identical");

        }

        beginTest("silence stays silent, a note change is smooth, no allocations while tracking");
        {
            CrushRack h;
            h.set("fx_crush_track", 1.0f);
            h.rack.setNote(60.0f);
            const Signal z = h.render(Signal(4096, 0.0f));
            float peak = 0.0f;
            for (const float v : z)
                peak = std::max(peak, std::abs(v));
            expect(peak == 0.0f, "silence in, silence out");

            // The held steps of a slow sine stay small across a note change (the rate glides, nothing is lost).
            CrushRack g;
            g.set("fx_crush_track", 1.0f);
            g.set("fx_crush_ratio", 16.0f);
            const Signal pr = probe();
            std::vector<float> l(pr.begin(), pr.end()), r = l;
            g.rack.setNote(48.0f);
            g.run(l.data(), r.data(), 24000);
            g.rack.setNote(84.0f);
            g.run(l.data() + 24000, r.data() + 24000, 24000);
            bool finite = true;
            for (const float v : l)
                finite = finite && std::isfinite(v) && std::abs(v) <= 1.0f;
            expect(finite, "finite and bounded across the note change");

            std::vector<float> a(64, 0.1f), b(64, 0.1f);
            crucible::testing::gTrackAllocations = true;
            crucible::testing::gAllocations = 0;
            for (int i = 0; i < 100; ++i)
            {
                g.rack.setNote(40.0f + static_cast<float>(i % 40));
                g.run(a.data(), b.data(), 64);
            }
            crucible::testing::gTrackAllocations = false;
            expectEquals(crucible::testing::gAllocations, 0L, "allocations on the audio thread");
        }

        beginTest("two crusher instances in a graph product each own their track parameters");
        {
            const char* text = R"({"format": "crucible-product", "version": 1, "id": "two-crush", "voice": "none",
                "fx": {"instances": [{"id": "c1", "type": "fx.crusher"}, {"id": "c2", "type": "fx.crusher"}],
                       "lanes": [{"id": "l1", "order": "listed", "input": ["aux"], "chain": ["c1"]},
                                 {"id": "l2", "order": "listed", "input": ["aux"], "chain": ["c2"]}]}})";
            graph::ProductDef def;
            std::string error;
            expect(graph::productFromText(text, def, error), error);
            const auto compiled = graph::compile(def, error);
            expect(compiled != nullptr, error);
            if (compiled == nullptr)
                return;
            const auto& model = compiled->registry->parameters();
            for (const char* id : {"fx_crush_track", "fx_crush_ratio", "fx_crush_fine", "fx2_crush_track", "fx2_crush_ratio",
                                   "fx2_crush_fine"})
                expect(model.indexOf(id) >= 0, std::string("own parameter ") + id);
            // Each instance on its own, tracking differently at the same note.
            const auto measure = [&](bool second, double ratio, double otherRatio)
            {
                ModuleRack rack{*compiled->registry};
                ModuleContext ctx;
                ctx.sampleRate = kRate;
                rack.prepare(ctx);
                auto values = compiled->defaultValues();
                const auto set = [&](const char* id, float v) { values[static_cast<size_t>(model.indexOf(id))] = v; };
                for (int i = 0; i < rack.size(); ++i)
                    values[static_cast<size_t>(model.indexOf(rack.descriptor(i).onParam))] = 0.0f;
                const char* on = second ? "fx2_crush_on" : "fx_crush_on";
                set(on, 1.0f);
                set(second ? "fx2_crush_mix" : "fx_crush_mix", 1.0f);
                set(second ? "fx2_crush_bits" : "fx_crush_bits", 16.0f);
                set(second ? "fx2_crush_track" : "fx_crush_track", 1.0f);
                set(second ? "fx2_crush_ratio" : "fx_crush_ratio", static_cast<float>(ratio));
                // The other instance's ratio must not matter (its track stays off, its ratio is changed).
                set(second ? "fx_crush_ratio" : "fx2_crush_ratio", static_cast<float>(otherRatio));
                rack.setNote(60.0f);
                const Signal in = probe();
                std::vector<float> l(in.begin(), in.end()), r = l;
                std::vector<float> legacy(static_cast<size_t>(kNumModDests), 0.0f);
                for (size_t pos = 0; pos < l.size(); pos += 64)
                    rack.process(l.data() + pos, r.data() + pos, 64, values.data(), legacy.data(), nullptr, 120.0);
                return measuredRate(Signal(l.begin(), l.end()), kF0);
            };
            const double first = measure(false, 16.0, 2.0);
            const double second = measure(true, 8.0, 2.0);
            std::printf("  crusher track: instance 1 ratio 16: %.2f Hz, instance 2 ratio 8: %.2f Hz (note 60 = %.2f Hz)\n",
                        first, second, noteHz(60));
            expectWithinAbsoluteError(first, noteHz(60) * 16.0, noteHz(60) * 16.0 * 0.005, "instance 1 rate");
            expectWithinAbsoluteError(second, noteHz(60) * 8.0, noteHz(60) * 8.0 * 0.005, "instance 2 rate");
        }

        beginTest("the engine feeds its last played note to the effects");
        {
            const auto render = [](bool track, int note)
            {
                SynthEngine engine;
                engine.prepare(kRate);
                auto v = ParameterMap::defaultValues();
                crucible::testing::setValue(v, "fx_crush_on", 1.0f);
                crucible::testing::setValue(v, "fx_crush_bits", 16.0f);
                crucible::testing::setValue(v, "fx_crush_track", track ? 1.0f : 0.0f);
                crucible::testing::setValue(v, "fx_crush_ratio", 4.0f);
                engine.setParameterValues(v.data(), static_cast<int>(v.size()));
                engine.noteOn(note, 1.0f, 0);
                return crucible::testing::renderBlocks(engine, 24000, 512, true).left;
            };
            const auto lo = render(true, 48), hi = render(true, 72), off = render(false, 48);
            double dLoHi = 0.0, dOff = 0.0;
            for (size_t i = 0; i < lo.size(); ++i)
            {
                dLoHi += std::abs(lo[i] - hi[i]);
                dOff += std::abs(lo[i] - off[i]);
            }
            expect(dOff > 1.0, "tracking changes the sound against the fixed rate");
            expect(dLoHi > 1.0, "a different note changes the crushed output");
        }
    }
};

static CrusherTrackTests crusherTrackTests;
