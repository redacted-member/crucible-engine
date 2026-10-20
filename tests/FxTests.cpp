// SPDX-License-Identifier: MIT
#include <crucible/Dsp.h>
#include <crucible/Engine.h>
#include <crucible/Fx.h>
#include <crucible/Modulation.h>
#include "TestSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
struct Stereo
{
    std::vector<float> l, r;
};

Stereo makeSine(double freq, float amp, int n)
{
    Stereo s;
    s.l.resize(static_cast<size_t>(n));
    s.r.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
        s.l[static_cast<size_t>(i)] = s.r[static_cast<size_t>(i)] =
            amp * std::sin(static_cast<float>(2.0 * 3.14159265358979 * freq * i / kRate));
    return s;
}

Stereo makeNoise(float amp, int n, std::uint32_t seed = 7)
{
    Rng rng;
    rng.state = seed;
    Stereo s;
    s.l.resize(static_cast<size_t>(n));
    s.r.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
    {
        s.l[static_cast<size_t>(i)] = amp * rng.bipolar();
        s.r[static_cast<size_t>(i)] = amp * rng.bipolar();
    }
    return s;
}

Stereo makeImpulse(int n, float amp = 1.0f)
{
    Stereo s;
    s.l.assign(static_cast<size_t>(n), 0.0f);
    s.r.assign(static_cast<size_t>(n), 0.0f);
    s.l[0] = s.r[0] = amp;
    return s;
}

// Runs one effect in blocks (default 32, the engine's control chunk).
template <class FxType, class Params> void run(FxType& fx, Stereo& s, const Params& p, int block = 32)
{
    const int n = static_cast<int>(s.l.size());
    for (int pos = 0; pos < n; pos += block)
        fx.process(s.l.data() + pos, s.r.data() + pos, std::min(block, n - pos), p);
}

float peakOf(const std::vector<float>& v, size_t from = 0, size_t to = 0)
{
    float m = 0.0f;
    for (size_t i = from; i < (to == 0 ? v.size() : to); ++i)
        m = std::max(m, std::abs(v[i]));
    return m;
}

float rmsOf(const std::vector<float>& v, size_t from = 0, size_t to = 0)
{
    double sum = 0.0;
    size_t count = 0;
    for (size_t i = from; i < (to == 0 ? v.size() : to); ++i, ++count)
        sum += static_cast<double>(v[i]) * v[i];
    return static_cast<float>(std::sqrt(sum / std::max<size_t>(count, 1)));
}

bool allFinite(const Stereo& s)
{
    for (size_t i = 0; i < s.l.size(); ++i)
        if (!std::isfinite(s.l[i]) || !std::isfinite(s.r[i]))
            return false;
    return true;
}

// Noise followed by a sine, through one effect at the given block size.
template <class FxType, class Params> Stereo noiseAndSine(FxType& fx, Params p, int block)
{
    p.on = true;
    fx.prepare(kRate);
    auto sig = makeNoise(0.5f, 48000);
    const auto sine = makeSine(440.0, 0.8f, 48000);
    for (size_t i = 24000; i < sig.l.size(); ++i)
        sig.l[i] = sig.r[i] = sine.l[i];
    run(fx, sig, p, block);
    return sig;
}
} // namespace

// The rack-style effects that remain (limiter, crusher, width, filter, multiband, trance gate, tremolo, ring / shift)
// and the product engine with the current effect modules. The first-generation effects were removed before v1; the
// matching behaviour of their successors is tested in the module tests.
class FxTests : public UnitTest
{
public:
    FxTests() : UnitTest("FX", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("every rack effect is finite and bounded on noise and sines, at block sizes 1, 32 and 512");
        for (const int block : {1, 32, 512})
        {
            const auto check = [&](const char* name, const Stereo& sig)
            {
                expect(allFinite(sig), String(name) + " non-finite");
                expect(peakOf(sig.l) < 8.0f && peakOf(sig.r) < 8.0f, String(name) + " peak " + String(peakOf(sig.l)));
            };
            {
                FxLimiter fx;
                check("limiter", noiseAndSine(fx, FxLimiterParams{}, block));
            }
            {
                FxCrusher fx;
                check("crusher", noiseAndSine(fx, FxCrusherParams{}, block));
            }
            {
                FxWidth fx;
                check("width", noiseAndSine(fx, FxWidthParams{}, block));
            }
            {
                CRUCIBLE_HEAP(FxFilter, fx);
                check("filter", noiseAndSine(fx, FxFilterParams{}, block));
            }
            {
                CRUCIBLE_HEAP(FxMultiband, fx);
                check("multiband", noiseAndSine(fx, FxMultibandParams{}, block));
            }
            {
                FxTranceGate fx;
                check("trance gate", noiseAndSine(fx, FxTranceGateParams{}, block));
            }
            {
                FxTremolo fx;
                check("tremolo", noiseAndSine(fx, FxTremoloParams{}, block));
            }
            {
                CRUCIBLE_HEAP(FxRingShift, fx);
                check("ring / shift", noiseAndSine(fx, FxRingShiftParams{}, block));
            }
        }

        beginTest("every effect parameter at its minimum and maximum stays finite and bounded");
        {
            const auto& defs = ParameterMap::params();
            const std::string fxPrefix = "fx_";
            for (const auto& def : defs)
            {
                if (def.id.rfind(fxPrefix, 0) != 0 || def.id.rfind("fx_pos_", 0) == 0 || def.type == ParamType::Bool)
                    continue;
                for (const float value : {def.min, def.max})
                {
                    auto values = ParameterMap::defaultValues();
                    // Enable every effect, this parameter at an extreme, heavy feedback elsewhere.
                    for (const auto& other : defs)
                        if (other.id.rfind("fx_", 0) == 0 && other.id.size() > 3 &&
                            other.id.compare(other.id.size() - 3, 3, "_on") == 0 && other.id.rfind("fx_ep_b", 0) != 0)
                            setValue(values, other.id, 1.0f);
                    // The convolution module is costly; it joins only for its own parameters.
                    if (def.id.rfind("fx_cx_", 0) != 0)
                        setValue(values, "fx_cx_on", 0.0f);
                    setValue(values, def.id, value);
                    CRUCIBLE_HEAP(SynthEngine, e);
                    prepare(e, values);
                    e.noteOn(48, 1.0f);
                    e.noteOn(60, 1.0f);
                    const auto r = renderBlocks(e, 24000, 64);
                    expect(r.finite, String(def.id) + " at " + String(value) + " non-finite");
                    expect(r.peak < 40.0f, String(def.id) + " at " + String(value) + " peak " + String(r.peak));
                }
            }
        }

        beginTest("zero mix leaves the signal untouched");
        {
            const auto zeroMix = [&](const char* name, auto& fx, auto p)
            {
                p.on = true;
                p.mix = 0.0f;
                fx.prepare(kRate);
                const auto reference = makeNoise(0.3f, 4800);
                auto out = reference;
                run(fx, out, p);
                const float diff = rmsOf(out.l) - rmsOf(reference.l);
                expect(std::abs(diff) < 1.0e-3f, String(name) + " zero-mix diff " + String(diff));
            };
            FxCrusher crusher;
            zeroMix("crusher", crusher, FxCrusherParams{});
            FxWidth width;
            zeroMix("width", width, FxWidthParams{});
            CRUCIBLE_HEAP(FxFilter, filter);
            zeroMix("filter", filter, FxFilterParams{});
            CRUCIBLE_HEAP(FxMultiband, multiband);
            zeroMix("multiband", multiband, FxMultibandParams{});
            FxTranceGate trance;
            zeroMix("trance gate", trance, FxTranceGateParams{});
            CRUCIBLE_HEAP(FxRingShift, ring);
            zeroMix("ring / shift", ring, FxRingShiftParams{});
        }

        beginTest("width and crusher");
        {
            // Width 0 collapses to mono.
            FxWidth width;
            width.prepare(kRate);
            FxWidthParams wp;
            wp.on = true;
            wp.width = 0.0f;
            auto st = makeNoise(0.3f, 4800);
            run(width, st, wp);
            double lr = 0.0;
            for (size_t i = 2000; i < st.l.size(); ++i)
                lr += std::abs(st.l[i] - st.r[i]);
            expect(lr < 1.0e-2, "mono collapse " + String(lr));

            // Crusher: 3 bits quantises to at most 2^3 + 1 levels.
            FxCrusher cr;
            cr.prepare(kRate);
            FxCrusherParams cp;
            cp.on = true;
            cp.bits = 3.0f;
            cp.rateHz = 48000.0f;
            auto cs = makeSine(300.0, 0.9f, 9600);
            run(cr, cs, cp);
            std::set<int> levels;
            for (const float v : cs.l)
                levels.insert(static_cast<int>(std::round(v * 4.0f)));
            expect(levels.size() <= 9, "levels " + String(static_cast<int>(levels.size())));
            cp.rateHz = 4800.0f;
            FxCrusher cr2;
            cr2.prepare(kRate);
            auto cs2 = makeSine(300.0, 0.9f, 9600);
            run(cr2, cs2, cp);
            int changes = 0;
            for (size_t i = 1; i < cs2.l.size(); ++i)
                changes += cs2.l[i] != cs2.l[i - 1] ? 1 : 0;
            expect(changes < 1200, "sample-and-hold transitions " + String(changes));
        }

        beginTest("limiter keeps every sample under the ceiling and recovers");
        {
            FxLimiter lim;
            lim.prepare(kRate);
            FxLimiterParams p;
            p.on = true;
            p.ceilingDb = -6.0f;
            p.gainDb = 12.0f;
            p.lookaheadMs = 2.0f;
            p.releaseMs = 50.0f;
            auto s = makeNoise(1.0f, 48000);
            for (size_t i = 24000; i < s.l.size(); ++i)
                s.l[i] = s.r[i] = 0.01f * std::sin(0.05f * static_cast<float>(i));
            run(lim, s, p);
            const float ceiling = std::pow(10.0f, -6.0f / 20.0f);
            expect(peakOf(s.l, 0, 24000) <= ceiling + 1.0e-4f, "peak " + String(peakOf(s.l, 0, 24000)));
            // After the loud part, a quiet tone is restored to its input gain (12 dB up) within the release.
            const float quiet = peakOf(s.l, 46000);
            expect(quiet > 0.03f, "recovered level " + String(quiet));
        }

        beginTest("effect order matters: saturator then delay differs from delay then saturator");
        {
            const auto render = [&](int satSlot, int delaySlot)
            {
                auto v = ParameterMap::defaultValues();
                setValue(v, "fx_saturator_on", 1.0f);
                setValue(v, "fx_saturator_drive", 30.0f);
                setValue(v, "fx_delay_pro_on", 1.0f);
                setValue(v, "fx_delay_pro_mix", 0.5f);
                setValue(v, "fx_delay_pro_time", 50.0f);
                setValue(v, "fx_pos_saturator", static_cast<float>(satSlot));
                setValue(v, "fx_pos_delay_pro", static_cast<float>(delaySlot));
                setValue(v, "amp_release", 0.01f);
                CRUCIBLE_HEAP(SynthEngine, e);
                prepare(e, v);
                e.noteOn(52, 0.3f);
                renderBlocks(e, 6000);
                e.noteOff(52);
                return renderBlocks(e, 18000, 512, true);
            };
            const auto a = render(1, 8);
            const auto b = render(9, 8);
            double diff = 0.0;
            for (size_t i = 0; i < a.left.size(); ++i)
                diff += std::abs(a.left[i] - b.left[i]);
            expect(diff > 1.0, "order has no effect: " + String(diff));
        }

        beginTest("engine: tails ring out after note off, a macro route drives an effect mix");
        {
            auto v = ParameterMap::defaultValues();
            setValue(v, "fx_delay_pro_on", 1.0f);
            setValue(v, "fx_delay_pro_mix", 0.6f);
            setValue(v, "fx_delay_pro_time", 200.0f);
            setValue(v, "fx_delay_pro_feedback", 0.5f);
            setValue(v, "amp_release", 0.01f);
            CRUCIBLE_HEAP(SynthEngine, e);
            prepare(e, v);
            e.noteOn(60, 1.0f);
            renderBlocks(e, 4800);
            e.noteOff(60);
            renderBlocks(e, 9600);                              // let the voice finish
            const auto tail = renderBlocks(e, 4800, 512, true); // echo of the first 0.1 s arrives
            expect(tail.peak > 0.001f, "no delay tail: " + String(tail.peak));
            const auto later = renderBlocks(e, 96000);
            expect(later.finite);

            // Macro 1 -> reverb mix (a generic route onto a global parameter).
            const auto tailEnergy = [&](float macro)
            {
                Engine eng;
                eng.prepare(kRate, 512);
                eng.setParam("fx_ra_on", 1.0f);
                eng.setParam("fx_ra_mix", 0.0f);
                eng.setParam("fx_ra_decay", 3.0f);
                eng.setParam("amp_release", 0.01f);
                eng.setParam("macro1", macro);
                eng.modRoutes().add({static_cast<int>(ModSource::Macro1), "fx_ra_mix", 1.0f});
                const int total = 4800 + 4800 + 24000;
                std::vector<float> l(static_cast<size_t>(total)), r(static_cast<size_t>(total));
                for (int pos = 0; pos < total; pos += 512)
                {
                    const int n = std::min(512, total - pos);
                    std::vector<MidiInput> ev;
                    if (pos == 0)
                        ev.push_back({0, 0x90, 60, 127});
                    if (pos <= 4800 && pos + 512 > 4800)
                        ev.push_back({4800 - pos, 0x80, 60, 0});
                    eng.process(l.data() + pos, r.data() + pos, n, ev.empty() ? nullptr : ev.data(),
                                static_cast<int>(ev.size()));
                }
                return rmsOf(l, 9600);
            };
            expect(tailEnergy(1.0f) > 10.0f * tailEnergy(0.0f) + 1.0e-6f);
        }
    }
};

static FxTests fxTests;
