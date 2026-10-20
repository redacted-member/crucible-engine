// SPDX-License-Identifier: MIT
// The vector kernels against the scalar code: Float4 lane arithmetic, the block half-band decimator and complete
// renders of every warp that has a kernel, with the kernels switched on and off in one binary (simd::setKernelsEnabled).
// In a CRUCIBLE_FORCE_SCALAR build both sides are scalar and the tests check the switch itself.

#include <crucible/HalfBand.h>
#include <crucible/Simd.h>
#include <crucible/Spectral.h>
#include "TestSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
struct KernelGuard
{
    explicit KernelGuard(bool on) { simd::setKernelsEnabled(on); }
    ~KernelGuard() { simd::setKernelsEnabled(true); }
};

double errorDb(const std::vector<float>& a, const std::vector<float>& b)
{
    double err = 0.0, sig = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
    {
        const double d = static_cast<double>(a[i]) - b[i];
        err += d * d;
        sig += static_cast<double>(b[i]) * b[i];
    }
    return 10.0 * std::log10(std::max(err, 1.0e-30) / std::max(sig, 1.0e-30));
}

std::vector<float> render(const std::vector<std::pair<std::string, float>>& params, bool kernels)
{
    const KernelGuard guard(kernels);
    auto values = ParameterMap::defaultValues();
    for (const auto& [id, v] : params)
        setValue(values, id, v);
    setValue(values, "amp_sustain", 1.0f);
    SynthEngine engine;
    prepare(engine, values);
    engine.noteOn(40, 1.0f);
    engine.noteOn(57, 0.8f);
    engine.noteOn(72, 0.9f);
    const auto r = renderBlocks(engine, 24000, 512, true);
    std::vector<float> out = r.left;
    out.insert(out.end(), r.right.begin(), r.right.end());
    return out;
}
} // namespace

class SimdTests : public UnitTest
{
public:
    SimdTests() : UnitTest("SIMD", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("Float4 lanes match scalar arithmetic");
        {
            using simd::Float4;
            using simd::Int4;
            const float a[4] = {-2.75f, 0.5f, 3.25f, -0.125f}, b[4] = {1.5f, -0.5f, 3.25f, 7.0f};
            float out[4];
            const auto check = [&](const char* name, Float4 v, float (*f)(float, float))
            {
                v.store(out);
                for (int i = 0; i < 4; ++i)
                    expect(out[i] == f(a[i], b[i]), std::string(name) + " lane " + std::to_string(i));
            };
            const Float4 va = Float4::load(a), vb = Float4::load(b);
            check("add", va + vb, [](float x, float y) { return x + y; });
            check("sub", va - vb, [](float x, float y) { return x - y; });
            check("mul", va * vb, [](float x, float y) { return x * y; });
            check("min", Float4::min(va, vb), [](float x, float y) { return std::min(x, y); });
            check("max", Float4::max(va, vb), [](float x, float y) { return std::max(x, y); });
            check("floor", Float4::floor(va), [](float x, float) { return std::floor(x); });
            check("geq select", Float4::select(Float4::geq(va, vb), va, vb),
                  [](float x, float y) { return x >= y ? x : y; });
            check("gt select", Float4::select(Float4::gt(va, vb), vb, va), [](float x, float y) { return x > y ? y : x; });
            check("mulAdd", Float4::mulAdd(va, vb, vb), [](float x, float y) { return x + y * y; });
            expect(std::abs((va * vb).sum() - ((a[0] * b[0] + a[1] * b[1]) + (a[2] * b[2] + a[3] * b[3]))) < 1.0e-6f, "sum");
            expect(Float4::anyTrue(Float4::geq(va, vb)) && !Float4::anyTrue(Float4::gt(Float4::splat(-9.0f), va)),
                   "anyTrue");
            Float4::fromInt(Float4::load(a).truncate()).store(out);
            for (int i = 0; i < 4; ++i)
                expect(out[i] == std::trunc(a[i]), "truncate lane " + std::to_string(i));
            const float tableA[8] = {0, 1, 2, 3, 4, 5, 6, 7}, tableB[8] = {10, 11, 12, 13, 14, 15, 16, 17};
            const float* tables[4] = {tableA, tableB, tableB, tableA};
            const std::int32_t idx[4] = {1, 2, 3, 4};
            Float4::gather(tables, Int4::load(idx), 1).store(out);
            expect(out[0] == 2.0f && out[1] == 13.0f && out[2] == 14.0f && out[3] == 5.0f, "gather");
            expect(Int4::min(Int4::load(idx), Int4::splat(2)).lane<3>() == 2, "int min");
        }

        beginTest("block half-band decimation equals the per-sample filter");
        {
            Rng rng;
            for (const int count : {32, 64, 7, 128})
            {
                HalfBandDecimator block, single;
                std::vector<float> in(static_cast<size_t>(2 * count)), a(static_cast<size_t>(count)),
                    b(static_cast<size_t>(count));
                double worst = 0.0;
                for (int round = 0; round < 6; ++round)
                {
                    for (auto& x : in)
                        x = rng.bipolar();
                    block.processBlock(in.data(), a.data(), count);
                    for (int k = 0; k < count; ++k)
                    {
                        b[static_cast<size_t>(k)] =
                            single.process(in[static_cast<size_t>(2 * k)], in[static_cast<size_t>(2 * k + 1)]);
                        worst = std::max(
                            worst, static_cast<double>(std::abs(a[static_cast<size_t>(k)] - b[static_cast<size_t>(k)])));
                    }
                }
                expect(worst < 1.0e-6, "count " + std::to_string(count) + " worst difference " + std::to_string(worst));
            }
        }

        struct Case
        {
            const char* name;
            std::vector<std::pair<std::string, float>> params;
        };
        const auto unison = [](int voices)
        {
            return std::vector<std::pair<std::string, float>>{{"osc1_uni_voices", static_cast<float>(voices)},
                                                              {"osc1_uni_detune", 30.0f}};
        };
        const auto warp = [&](float mode, int voices, float a, float b)
        {
            auto p = unison(voices);
            p.insert(p.end(), {{"osc1_warp", mode}, {"osc1_warp_a", a}, {"osc1_warp_b", b}});
            return p;
        };
        const std::vector<Case> cases = {
            {"plain, 1 voice", unison(1)},
            {"plain, 3 voices", unison(3)},
            {"plain, 7 voices", unison(7)},
            {"plain, 8 voices, morphing table",
             [&]
             {
                 auto p = unison(8);
                 p.push_back({"osc1_pos", 0.37f});
                 return p;
             }()},
            {"FM, 4 voices", warp(1.0f, 4, 0.4f, 0.3f)},
            {"FM, 5 voices", warp(1.0f, 5, 0.7f, 0.9f)},
            {"hard sync, 4 voices", warp(2.0f, 4, 0.4f, 0.0f)},
            {"hard sync, 2 voices", warp(2.0f, 2, 0.8f, 0.2f)},
            {"hard sync, 8 voices", warp(2.0f, 8, 0.2f, 0.5f)},
            {"fold, 4 voices", warp(14.0f, 4, 0.4f, 0.1f)},
            {"fold, 6 voices", warp(14.0f, 6, 0.9f, 0.6f)},
            {"ring, 4 voices", warp(13.0f, 4, 0.7f, 0.2f)},
            {"bend (scalar warp), 4 voices", warp(3.0f, 4, 0.5f, 0.2f)},
        };
        for (const auto& c : cases)
        {
            beginTest(std::string("scalar and vector renders agree: ") + c.name);
            const auto fast = render(c.params, true);
            const auto slow = render(c.params, false);
            const double e = errorDb(fast, slow);
            expect(e < -110.0, "error " + std::to_string(e) + " dB below the signal");
        }

        // Spectral Sweep synthesises one cycle per distinct mip; stacked octaves need several mips in one chunk, and the
        // vector path resolves all cycles of a group before it renders any of them.
        for (const float mode : {3.0f, 4.0f, 5.0f, 6.0f})
        {
            beginTest("scalar and vector renders agree: spectral sweep with stacked unison, mode " +
                      std::to_string(static_cast<int>(mode)));
            spectral::FactoryTables::instance().ensure(2);
            auto p = unison(8);
            p.insert(p.end(),
                     {{"osc1_uni_mode", mode}, {"osc1_wt_interp", 1.0f}, {"osc1_pos", 0.43f}, {"osc1_table", 2.0f}});
            const auto fast = render(p, true);
            const auto slow = render(p, false);
            const double e = errorDb(fast, slow);
            expect(e < -110.0, "error " + std::to_string(e) + " dB below the signal");
        }

        beginTest("vector kernels are deterministic");
        {
            const auto p = warp(1.0f, 5, 0.5f, 0.5f);
            expect(render(p, true) == render(p, true));
        }
    }
};

static SimdTests simdTests;
