// SPDX-License-Identifier: MIT
// fx.shaper (drawn-curve waveshaper, docs/plans/ENGINE-FX-PLAN.md common acceptance): through the module rack. The identity
// curve is transparent, a drawn hard clip equals a reference clipper, a curve with curvature matches its analytic shape at
// 64 points, and the acceptance rows (latency, reset, bypass, silence, no zipper, alias, CPU) are printed as numbers.

#include "AllocCounter.h"
#include "CpuTiming.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Fx.h>
#include <crucible/Patch.h>
#include <crucible/graph/Compiler.h>
#include <crucible/graph/Product.h>
#include <crucible/ModMatrix.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/ModuleRack.h>
#include <crucible/module/ShaperCurve.h>

#include <atomic>
#include <cmath>
#include <limits>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(undefined_behavior_sanitizer)
#define CRUCIBLE_TEST_UNTIMED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__) || !defined(NDEBUG) ||                                    \
    (defined(__x86_64__) && defined(__APPLE__))
#define CRUCIBLE_TEST_UNTIMED 1
#endif
#include <crucible/Simd.h>
#if !CRUCIBLE_SIMD_NATIVE
#define CRUCIBLE_TEST_UNTIMED 1
#endif

using namespace crucible;
using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;
using crucible::testing::kRate;

namespace
{
using Samples = std::vector<float>;
constexpr int kOs1x = 1, kOs2x = 2, kOs4x = 3;

// fx.shaper alone in the rack (every other effect off).
struct Shp
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values, legacy;
    LatencyReporter* latency = nullptr;
    ShaperCurve* curve = nullptr;
    int idOn, idMix, idDrive, idOut, idDc, idOs;

    explicit Shp(Quality quality = Quality::Standard, double rate = kRate)
    {
        ModuleContext ctx;
        ctx.sampleRate = rate;
        ctx.quality = quality;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            values[static_cast<size_t>(ParameterMap::indexOf(rack.descriptor(i).onParam))] = 0.0f;
        const auto ix = [](const char* id) { return ParameterMap::indexOf(id); };
        idOn = ix("fx_shaper_on");
        idMix = ix("fx_shaper_mix");
        idDrive = ix("fx_shaper_drive");
        idOut = ix("fx_shaper_output");
        idDc = ix("fx_shaper_dc");
        idOs = ix("fx_shaper_os");
        set(idOn, 1.0f);
        latency = rack.findCapability<LatencyReporter>("fx.shaper");
        curve = rack.findCapability<ShaperCurve>("fx.shaper");
    }
    void set(int id, float v) { values[static_cast<size_t>(id)] = v; }
    bool setCurve(const std::vector<ShaperPoint>& p, bool symmetric = false)
    {
        std::string error;
        return curve->setCurve(p.data(), static_cast<int>(p.size()), symmetric, error);
    }
    void run(float* l, float* r, int n)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, nullptr, nullptr);
    }
    Samples renderFloat(const Samples& in, int block = 256)
    {
        Samples l = in, r = in;
        for (size_t p = 0; p < l.size(); p += static_cast<size_t>(block))
            run(l.data() + p, r.data() + p, static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - p)));
        return l;
    }
    Signal render(const Signal& in, int block = 256)
    {
        Samples f(in.size());
        for (size_t i = 0; i < in.size(); ++i)
            f[i] = static_cast<float>(in[i]);
        return toDouble(renderFloat(f, block));
    }
};

// Deterministic noise in -1..1 scaled by a.
Samples noise(size_t n, float a, std::uint32_t seed = 7u)
{
    Samples x(n);
    std::uint32_t st = seed;
    for (auto& v : x)
    {
        st = st * 1664525u + 1013904223u;
        v = a * (static_cast<float>(st >> 8) / 8388608.0f - 1.0f);
    }
    return x;
}

// The curve of the header's formula, written out again independently of the engine's table.
double referenceCurve(const std::vector<ShaperPoint>& p, bool symmetric, double x)
{
    const double a = symmetric ? std::abs(x) : x;
    std::vector<ShaperPoint> q = p;
    if (symmetric && q.front().x > 0.0f)
        q.insert(q.begin(), ShaperPoint{0.0f, 0.0f, 0.0f});
    double y;
    if (a <= q.front().x)
        y = q.front().y;
    else if (a >= q.back().x)
        y = q.back().y;
    else
    {
        size_t i = 0;
        while (a >= q[i + 1].x)
            ++i;
        const double t = (a - q[i].x) / (static_cast<double>(q[i + 1].x) - q[i].x);
        y = q[i].y + (static_cast<double>(q[i + 1].y) - q[i].y) * std::pow(t, std::pow(4.0, q[i].curve));
    }
    return symmetric ? std::copysign(y, x) : y;
}
} // namespace

class ShaperTests : public UnitTest
{
public:
    ShaperTests() : UnitTest("FxShaper", "Effects") {}

    void runTest() override
    {
        descriptor();
        identity();
        hardClip();
        curvature();
        validation();
        views();
        patches();
        instances();
        latencyResetBypassSilence();
        latencyBeforeFirstBlock();
        rates();
        blockSizes();
        zipperAndClick();
        dcBlock();
        aliasing();
        extremes();
        realtime();
        cpu();
    }

private:
    void identity()
    {
        beginTest("identity curve at 0 dB drive and mix 1: the delayed input bit for bit, whatever the block size");
        for (const int block : {1, 7, 64, 256, 512})
        {
            Shp s;
            const Samples in = noise(6000, 0.9f);
            const Samples out = s.renderFloat(in, block);
            const size_t lat = static_cast<size_t>(s.latency->latencySamples());
            size_t bad = 0;
            double worst = 0.0;
            for (size_t i = 0; i < in.size(); ++i)
            {
                const float want = i >= lat ? in[i - lat] : 0.0f;
                if (out[i] != want)
                    ++bad;
                worst = std::max(worst, static_cast<double>(std::abs(out[i] - want)));
            }
            if (block == 256)
                std::printf("  identity, latency %zu samples: %zu of %zu samples differ, worst %.3g\n", lat, bad, in.size(),
                            worst);
            expect(bad == 0, "block " + String(block) + ": " + String(static_cast<int>(bad)) + " samples differ");
        }
        {
            // Above full scale the identity curve is the straight line, not a clip.
            Shp s;
            const Samples in = noise(4000, 3.0f);
            const Samples out = s.renderFloat(in);
            const size_t lat = static_cast<size_t>(s.latency->latencySamples());
            bool same = true;
            for (size_t i = lat; i < in.size(); ++i)
                same = same && out[i] == in[i - lat];
            expect(same, "+10 dB peaks pass unclipped");
        }
    }

    void hardClip()
    {
        beginTest("a drawn hard clip at +-0.5 equals a reference clipper (1x, no latency)");
        Shp s;
        s.set(s.idOs, static_cast<float>(kOs1x));
        expect(s.setCurve({{-1.0f, -0.5f, 0.0f}, {-0.5f, -0.5f, 0.0f}, {0.5f, 0.5f, 0.0f}, {1.0f, 0.5f, 0.0f}}));
        const Samples in = noise(8000, 1.6f);
        const Samples out = s.renderFloat(in);
        expectEquals(s.latency->latencySamples(), 0, "latency at 1x");
        double worst = 0.0;
        for (size_t i = 3000; i < in.size(); ++i) // after the 10 ms cross-fade to the new curve
            worst = std::max(worst, static_cast<double>(std::abs(out[i] - std::clamp(in[i], -0.5f, 0.5f))));
        std::printf("  hard clip curve against std::clamp: worst deviation %.3g\n", worst);
        expect(worst < 1.0e-6, "worst " + String(worst, 9));

        // The same with drive: the reference clips drive * x.
        s.set(s.idDrive, 6.0f);
        const Samples in2 = noise(16000, 0.7f, 11u);
        Samples out2 = s.renderFloat(in2);
        const float g = std::pow(10.0f, 6.0f / 20.0f);
        double worst2 = 0.0;
        for (size_t i = 10000; i < in2.size(); ++i) // after the drive has glided to 6 dB
            worst2 = std::max(worst2, static_cast<double>(std::abs(out2[i] - std::clamp(in2[i] * g, -0.5f, 0.5f))));
        expect(worst2 < 1.0e-5, "with 6 dB drive, worst " + String(worst2, 9));
    }

    void curvature()
    {
        beginTest("64 points with curvature, asymmetric and symmetric: the output matches the analytic curve at 64 x values");
        for (const bool symmetric : {false, true})
        {
            std::vector<ShaperPoint> pts;
            for (int i = 0; i < 64; ++i)
            {
                const float f = static_cast<float>(i) / 63.0f;
                ShaperPoint p;
                p.x = symmetric ? f : -1.0f + 2.0f * f;
                p.y = 0.8f * std::sin(6.0f * f + 0.3f) * (symmetric ? 1.0f : 0.9f);
                p.curve = std::cos(11.0f * f);
                pts.push_back(p);
            }
            if (symmetric)
                pts[0].y = 0.0f; // x = 0
            Shp s;
            s.set(s.idOs, static_cast<float>(kOs1x));
            expect(s.setCurve(pts, symmetric));
            // 64 x values spread over the whole range, in a deterministic shuffle, one per sample after a settle stretch.
            Samples in(2400 + 64, 0.0f);
            std::vector<double> xs;
            for (int k = 0; k < 64; ++k)
                xs.push_back(-0.995 + 1.99 * static_cast<double>((k * 37) % 64) / 63.0);
            for (int k = 0; k < 64; ++k)
                in[2400 + static_cast<size_t>(k)] = static_cast<float>(xs[static_cast<size_t>(k)]);
            const Samples out = s.renderFloat(in);
            double worst = 0.0;
            for (int k = 0; k < 64; ++k)
                worst = std::max(worst, std::abs(static_cast<double>(out[2400 + static_cast<size_t>(k)]) -
                                                 referenceCurve(pts, symmetric, static_cast<float>(xs[static_cast<size_t>(k)]))));
            std::printf("  %s 64-point curve with curvature: worst deviation from the analytic shape %.3g\n",
                        symmetric ? "symmetric" : "asymmetric", worst);
            expect(worst < 2.0e-6, String(symmetric ? "symmetric " : "asymmetric ") + String(worst, 9));
            if (symmetric)
            {
                // Odd symmetry exactly.
                bool odd = true;
                for (int k = 0; k < 64; ++k)
                    odd = odd && std::abs(referenceCurve(pts, true, -xs[static_cast<size_t>(k)]) +
                                          referenceCurve(pts, true, xs[static_cast<size_t>(k)])) < 1e-12;
                expect(odd);
            }
        }
    }

    static std::vector<ShaperPoint> softS()
    {
        return {{-1.0f, -1.0f, 0.0f}, {-0.5f, -0.4f, 0.3f}, {0.5f, 0.4f, -0.3f}, {1.0f, 1.0f, 0.0f}};
    }
    // A straight line of slope 0.5: not the identity, so the wet path runs, yet linear for |x| <= 1.
    static std::vector<ShaperPoint> halfLine() { return {{-1.0f, -0.5f, 0.0f}, {1.0f, 0.5f, 0.0f}}; }

    void descriptor()
    {
        beginTest("descriptor: current, spelled-out ids, short names of six characters or fewer, curve view and capability");
        const ModuleDescriptor* d = ModuleRegistry::instance().find("fx.shaper");
        expect(d != nullptr);
        if (d == nullptr)
            return;
        expect(d->status == ModuleStatus::Current);
        bool idsOk = true, shortOk = true;
        for (const auto& p : d->params)
        {
            idsOk = idsOk && (p.id.rfind("fx_shaper_", 0) == 0 || p.id == "fx_pos_shaper");
            shortOk = shortOk && !p.shortName.empty() && p.shortName.size() <= 6;
        }
        expect(idsOk, "every id begins fx_shaper_");
        expect(shortOk, "short names");
        expect(d->views.size() == 1 && d->views[0] == "curve", "views");
        expect(std::find(d->capabilities.begin(), d->capabilities.end(), "shaper_curve") != d->capabilities.end(),
               "capability shaper_curve");
        expectEquals(ModuleRegistry::instance().ofKind(ModuleKind::Effect).at(kNumModuleFx - 3)->id, std::string("fx.shaper"),
                     "registered after fx.saturator, before fx.multiband_drive and fx.cabinet");
    }

    void validation()
    {
        beginTest("invalid curves are refused with a reason and leave the curve as it was");
        Shp s;
        std::string error;
        const auto refuse = [&](const std::vector<ShaperPoint>& p, bool symmetric, const char* what)
        {
            error.clear();
            const bool ok = s.curve->setCurve(p.data(), static_cast<int>(p.size()), symmetric, error);
            expect(!ok && !error.empty(), String(what) + ": " + error);
        };
        expect(s.setCurve(softS()), "a valid curve");
        refuse({{0.0f, 0.0f, 0.0f}}, false, "one point");
        std::vector<ShaperPoint> many;
        for (int i = 0; i < 65; ++i)
            many.push_back({-1.0f + 2.0f * static_cast<float>(i) / 64.0f, 0.0f, 0.0f});
        refuse(many, false, "65 points");
        refuse({{0.5f, 0.0f, 0.0f}, {0.5f, 0.1f, 0.0f}}, false, "equal x");
        refuse({{0.5f, 0.0f, 0.0f}, {0.2f, 0.1f, 0.0f}}, false, "descending x");
        refuse({{-1.5f, 0.0f, 0.0f}, {1.0f, 0.1f, 0.0f}}, false, "x below -1");
        refuse({{-1.0f, 0.0f, 0.0f}, {1.5f, 0.1f, 0.0f}}, false, "x above 1");
        refuse({{-1.0f, -1.2f, 0.0f}, {1.0f, 1.0f, 0.0f}}, false, "y below -1");
        refuse({{-1.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 1.5f}}, false, "curve above 1");
        refuse({{-1.0f, -1.0f, -1.5f}, {1.0f, 1.0f, 0.0f}}, false, "curve below -1");
        refuse({{-1.0f, std::nanf(""), 0.0f}, {1.0f, 1.0f, 0.0f}}, false, "NaN y");
        refuse({{-1.0f, -1.0f, 0.0f}, {std::numeric_limits<float>::infinity(), 1.0f, 0.0f}}, false, "infinite x");
        refuse({{-0.5f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}}, true, "symmetric with a negative x");
        refuse({{0.0f, 0.2f, 0.0f}, {1.0f, 1.0f, 0.0f}}, true, "symmetric with y != 0 at x = 0");
        error.clear();
        expect(!s.curve->setCurve(nullptr, 2, false, error) && !error.empty(), "null points");
        std::vector<ShaperPoint> back;
        bool sym = true;
        expect(s.curve->curve(back, sym) && !sym && back.size() == 4 && back[1].x == -0.5f, "the first curve is still set");
        expect(s.setCurve({{0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}}, true), "symmetric identity half is valid");
        expect(s.setCurve({{-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}}), "a flat curve is valid");
        // 64 points are the limit.
        many.pop_back();
        expect(s.setCurve(many), "64 points");

        beginTest("engine: setShaperCurve rejects an unknown instance and a module without a curve; NULL restores the identity");
        Engine e;
        e.prepare(kRate, 256);
        const auto pts = softS();
        expect(!e.setShaperCurve("fx.nothing", pts.data(), 4, false, error) && error.find("unknown") != std::string::npos,
               error);
        expect(!e.setShaperCurve("fx.saturator", pts.data(), 4, false, error) && error.find("no shaper curve") != std::string::npos,
               error);
        expect(e.setShaperCurve("fx.shaper", pts.data(), 4, false, error), error);
        std::vector<ShaperPoint> got;
        bool symmetric = true;
        expect(e.shaperCurve("fx.shaper", got, symmetric) && got.size() == 4 && !symmetric);
        expect(e.setShaperCurve("fx.shaper", nullptr, 0, false, error), error);
        expect(!e.shaperCurve("fx.shaper", got, symmetric), "identity again");
        expect(!e.shaperCurve("fx.saturator", got, symmetric) && !e.shaperCurve("fx.nothing", got, symmetric));
    }

    void views()
    {
        beginTest("the pulled \"curve\" view returns the drawn curve at 257 points and follows every change");
        Engine e;
        e.prepare(kRate, 256);
        const int view = e.views().find("fx.shaper", "curve");
        expect(view >= 0, "view exists");
        if (view < 0)
            return;
        expectEquals(e.views().info(view).maxElements, ShaperCurve::kViewPoints);
        expect(!e.views().info(view).live, "pulled");
        const auto read = [&]
        {
            std::vector<float> v(static_cast<size_t>(ShaperCurve::kViewPoints));
            const int n = e.views().read(view, v.data(), static_cast<int>(v.size()), nullptr);
            v.resize(static_cast<size_t>(std::max(n, 0)));
            return v;
        };
        auto v = read();
        double worst = 0.0;
        expectEquals(static_cast<int>(v.size()), ShaperCurve::kViewPoints);
        for (size_t i = 0; i < v.size(); ++i)
            worst = std::max(worst, std::abs(static_cast<double>(v[i]) - (-1.0 + 2.0 * static_cast<double>(i) / 256.0)));
        expect(worst < 1.0e-6, "default is y = x, worst " + String(worst, 9));
        std::string error;
        const auto pts = softS();
        expect(e.setShaperCurve("fx.shaper", pts.data(), 4, false, error), error);
        v = read();
        worst = 0.0;
        for (size_t i = 0; i < v.size(); ++i)
            worst = std::max(worst, std::abs(static_cast<double>(v[i]) -
                                             referenceCurve(pts, false, -1.0 + 2.0 * static_cast<double>(i) / 256.0)));
        std::printf("  curve view against the analytic curve: worst deviation %.3g\n", worst);
        expect(worst < 1.0e-6, "drawn curve, worst " + String(worst, 9));
        const std::vector<ShaperPoint> half = {{0.0f, 0.0f, 0.0f}, {0.5f, 0.9f, 0.5f}, {1.0f, 0.6f, 0.0f}};
        expect(e.setShaperCurve("fx.shaper", half.data(), 3, true, error), error);
        v = read();
        worst = 0.0;
        for (size_t i = 0; i < v.size(); ++i)
            worst = std::max(worst, std::abs(static_cast<double>(v[i]) -
                                             referenceCurve(half, true, -1.0 + 2.0 * static_cast<double>(i) / 256.0)));
        expect(worst < 1.0e-6 && v[0] == -v[256] + 0.0f, "symmetric curve is odd, worst " + String(worst, 9));
    }

    static std::string patchText(Engine& e) { return patchToJson(capturePatch(e)); }

    void patches()
    {
        beginTest("patches save and load the curve; save -> load -> save is bit-stable; the rendered sound is the same");
        {
            Engine a;
            a.prepare(kRate, 256);
            std::vector<ShaperPoint> pts;
            for (int i = 0; i < 20; ++i)
            {
                const float f = static_cast<float>(i) / 19.0f;
                pts.push_back({-0.97f + 1.9f * f + 0.001f * static_cast<float>(i * i % 7), 0.7f * std::sin(5.0f * f) + 0.0123f,
                               0.7f * std::cos(9.0f * f)});
            }
            std::string error;
            expect(a.setShaperCurve("fx.shaper", pts.data(), static_cast<int>(pts.size()), false, error), error);
            const std::string text = patchText(a);
            expect(text.find("shaperCurves") != std::string::npos, "written");
            Patch p;
            expect(patchFromJson(text, p, error), error);
            expect(p.shaperCurves.size() == 1 && p.shaperCurves[0].instance == "fx.shaper" &&
                       p.shaperCurves[0].points.size() == pts.size(),
                   "parsed");
            Engine b;
            b.prepare(kRate, 256);
            applyPatch(b, p);
            std::vector<ShaperPoint> back;
            bool sym = true;
            expect(b.shaperCurve("fx.shaper", back, sym) && !sym && back.size() == pts.size());
            bool same = back.size() == pts.size();
            for (size_t i = 0; same && i < pts.size(); ++i)
                same = std::memcmp(&back[i], &pts[i], sizeof(ShaperPoint)) == 0;
            expect(same, "points identical bit for bit");
            expect(patchText(b) == text, "save -> load -> save is identical");

            // The same sound: render both engines with the shaper on.
            const auto render = [&](Engine& e)
            {
                e.setParam("fx_shaper_on", 1.0f);
                e.setParam("fx_shaper_drive", 9.0f);
                std::vector<float> l = noise(6144, 0.6f), r = l;
                for (size_t at = 0; at < l.size(); at += 256)
                    e.process(l.data() + at, r.data() + at, 256);
                return l;
            };
            Engine c;
            c.prepare(kRate, 256);
            c.setShaperCurve("fx.shaper", pts.data(), static_cast<int>(pts.size()), false, error);
            expect(render(b) == render(c), "applied patch renders like the direct call");
        }

        beginTest("symmetric curves and the flag survive; old patches (no key) load as the identity and clear a curve");
        {
            Engine a;
            a.prepare(kRate, 256);
            const std::vector<ShaperPoint> half = {{0.25f, 0.5f, 0.2f}, {1.0f, 0.9f, 0.0f}};
            std::string error;
            expect(a.setShaperCurve("fx.shaper", half.data(), 2, true, error), error);
            Patch p;
            expect(patchFromJson(patchText(a), p, error), error);
            Engine b;
            b.prepare(kRate, 256);
            applyPatch(b, p);
            std::vector<ShaperPoint> back;
            bool sym = false;
            expect(b.shaperCurve("fx.shaper", back, sym) && sym && back.size() == 2, "symmetric flag");

            Engine fresh;
            fresh.prepare(kRate, 256);
            const std::string plain = patchText(fresh);
            expect(plain.find("shaperCurves") == std::string::npos, "no key while the curve is the default");
            Patch q;
            expect(patchFromJson(plain, q, error), error);
            applyPatch(b, q);
            expect(!b.shaperCurve("fx.shaper", back, sym), "a patch without a curve restores the identity");
        }

        beginTest("malformed curve entries are skipped (the instance stays on the identity curve)");
        {
            Engine a;
            a.prepare(kRate, 256);
            const auto pts = softS();
            std::string error;
            expect(a.setShaperCurve("fx.shaper", pts.data(), 4, false, error), error);
            std::string text = patchText(a);
            const auto at = text.find("\"points\": \"");
            expect(at != std::string::npos);
            text.insert(at + 11, "!!!");
            Patch p;
            expect(patchFromJson(text, p, error), error);
            expect(p.shaperCurves.empty(), "entry dropped");
            Patch q;
            std::string valid = patchText(a);
            // An out-of-range value: the curve is refused when applied.
            expect(patchFromJson(valid, q, error), error);
            q.shaperCurves[0].points[1].y = 7.0f;
            Engine b;
            b.prepare(kRate, 256);
            applyPatch(b, q);
            std::vector<ShaperPoint> back;
            bool sym;
            expect(!b.shaperCurve("fx.shaper", back, sym), "an invalid curve in a patch leaves the identity");
        }
    }

    void instances()
    {
        beginTest("a second instance (fx.shaper.2) in a graph product has its own curve, saved and restored by instance id");
        const char* text = R"({"format": "crucible-product", "version": 1, "id": "two-shapers", "voice": "none",
            "fx": {"instances": [{"id": "s1", "type": "fx.shaper"}, {"id": "s2", "type": "fx.shaper"}],
                   "lanes": [{"id": "l1", "order": "listed", "input": ["aux"], "chain": ["s1"]},
                             {"id": "l2", "order": "listed", "input": ["aux"], "chain": ["s2"]}]}})";
        std::string error;
        graph::ProductDef def;
        expect(graph::productFromText(text, def, error), error);
        EngineConfig cfg;
        cfg.compiled = graph::compile(def, error);
        expect(cfg.compiled != nullptr, error);
        if (cfg.compiled == nullptr)
            return;
        Engine e(cfg);
        e.prepare(kRate, 128);
        const auto a = softS();
        const std::vector<ShaperPoint> b = {{-1.0f, -0.2f, 0.0f}, {1.0f, 0.2f, 0.0f}};
        expect(e.setShaperCurve("fx.shaper", a.data(), 4, false, error), error);
        expect(e.setShaperCurve("fx.shaper.2", b.data(), 2, false, error), error);
        std::vector<ShaperPoint> back;
        bool sym = false;
        expect(e.shaperCurve("fx.shaper", back, sym) && back.size() == 4, "first keeps its curve");
        expect(e.shaperCurve("fx.shaper.2", back, sym) && back.size() == 2, "second has its own");
        const int v1 = e.views().find("fx.shaper", "curve"), v2 = e.views().find("fx.shaper.2", "curve");
        expect(v1 >= 0 && v2 >= 0 && v1 != v2, "a curve view per instance");
        std::vector<float> o1(257), o2(257);
        e.views().read(v2, o2.data(), 257, nullptr);
        expect(std::abs(o2[256] - 0.2f) < 1e-6f, "view of the second instance");
        const std::string saved = patchText(e);
        Patch p;
        expect(patchFromJson(saved, p, error), error);
        expect(p.shaperCurves.size() == 2 && p.shaperCurves[1].instance == "fx.shaper.2", "two entries");
        Engine f(cfg);
        f.prepare(kRate, 128);
        applyPatch(f, p);
        expect(patchText(f) == saved, "bit-stable");
    }

    void latencyBeforeFirstBlock()
    {
        beginTest("os 8x set before prepare: the reported latency is the measured delay before any process call");
        for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
        {
            Shp s(q);
            s.set(s.idOs, 4.0f);
            ModuleContext ctx;
            ctx.sampleRate = kRate;
            ctx.quality = q;
            s.rack.prepare(ctx);
            s.rack.adoptLatencyParams(s.values.data());
            const int reported = s.latency->latencySamples();
            expect(s.rack.reportedLatency() >= 0);
            const Samples n = noise(1 << 14, 0.5f, 21u);
            const Signal in = toDouble(n);
            const double measured = measureLatency(in, s.render(in));
            expect(reported > 0 && std::abs(measured - reported) <= 1.0,
                   "quality " + String(static_cast<int>(q)) + ": reported " + String(reported) + ", measured " +
                       String(measured, 3));
        }
        {
            // Through the engine: parameter set, then prepare; the host reads the latency right after.
            const auto idx = [](const char* id) { return ParameterMap::indexOf(id); };
            EngineConfig cfg;
            cfg.quality = Quality::Draft;
            Engine e(cfg);
            e.setParam(idx("fx_shaper_on"), 1.0f);
            e.prepare(kRate, 128);
            const int autoLatency = e.latencySamples();
            e.setParam(idx("fx_shaper_os"), 4.0f);
            e.prepare(kRate, 128);
            const int eightX = e.latencySamples();
            expect(eightX > autoLatency, "8x " + String(eightX) + " vs auto " + String(autoLatency));
            Shp ref(Quality::Draft);
            ref.set(ref.idOs, 4.0f);
            const Samples n = noise(1 << 14, 0.5f, 21u);
            const Signal in = toDouble(n);
            const double measured = measureLatency(in, ref.render(in));
            expect(std::abs(measured - eightX) <= 1.0, "engine reports " + String(eightX) + ", measured " + String(measured, 3));
        }
    }

    void latencyResetBypassSilence()
    {
        beginTest("latency: reported equals measured to one sample at 1x, 2x, 4x and 8x in every quality");
        for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
            for (const int os : {1, 2, 3, 4})
            {
                Shp s(q);
                s.set(s.idOs, static_cast<float>(os));
                expect(s.setCurve(halfLine()));
                const Samples n = noise(1 << 14, 0.5f, 21u);
                const Signal in = toDouble(n);
                const Signal out = s.render(in);
                const double measured = measureLatency(in, out);
                const int reported = s.latency->latencySamples();
                std::printf("  quality %d, os choice %d: reported %d, measured %.3f samples\n", static_cast<int>(q), os, reported,
                            measured);
                expect(std::abs(measured - reported) <= 1.0,
                       "quality " + String(static_cast<int>(q)) + " os " + String(os) + ": " + String(measured, 3));
            }
        {
            beginTest("changing the oversampling factor reports the new latency (takeLatencyChanged)");
            Shp s;
            const Samples n = noise(512, 0.3f);
            s.renderFloat(n);
            const int before = s.latency->latencySamples();
            s.latency->takeLatencyChanged();
            s.set(s.idOs, 1.0f);
            s.renderFloat(n);
            expect(s.latency->latencySamples() == 0 && before > 0 && s.latency->takeLatencyChanged() &&
                       !s.latency->takeLatencyChanged(),
                   "from " + String(before) + " to " + String(s.latency->latencySamples()));
        }

        beginTest("reset: a reset instance renders what a fresh one does; bypass is the input; silence in is exact zero out");
        {
            Shp a, b;
            expect(a.setCurve(softS()) && b.setCurve(softS()));
            a.set(a.idDrive, 12.0f);
            b.set(b.idDrive, 12.0f);
            const Samples in = noise(5000, 0.8f, 3u);
            a.renderFloat(noise(3000, 0.9f, 9u)); // dirty state
            a.rack.reset();
            const Samples x = a.renderFloat(in), y = b.renderFloat(in);
            size_t differ = 0;
            for (size_t i = 0; i < x.size(); ++i)
                differ += x[i] != y[i] ? 1u : 0u;
            expectEquals(static_cast<int>(differ), 0, "samples that differ after reset");

            Shp off;
            expect(off.setCurve(softS()));
            off.set(off.idOn, 0.0f);
            const Samples z = off.renderFloat(in);
            expect(z == in, "bypassed: the input bit for bit");

            Shp mix0;
            expect(mix0.setCurve(softS()));
            mix0.set(mix0.idMix, 0.0f);
            const Samples w = mix0.renderFloat(in);
            const size_t lat = static_cast<size_t>(mix0.latency->latencySamples());
            size_t bad = 0;
            for (size_t i = 0; i < in.size(); ++i)
                bad += w[i] != (i >= lat ? in[i - lat] : 0.0f) ? 1u : 0u;
            expectEquals(static_cast<int>(bad), 0, "mix 0 is the latency-delayed input");

            for (const bool custom : {false, true})
                for (const bool dc : {false, true})
                {
                    Shp q;
                    if (custom)
                        expect(q.setCurve({{-1.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 0.5f}, {1.0f, 0.7f, -0.5f}}));
                    q.set(q.idDrive, 12.0f);
                    q.set(q.idDc, dc ? 1.0f : 0.0f);
                    const Samples silence(20000, 0.0f);
                    const Samples o = q.renderFloat(silence);
                    float peak = 0.0f;
                    for (const float v : o)
                        peak = std::max(peak, std::abs(v));
                    expect(peak == 0.0f, String(custom ? "curve" : "identity") + (dc ? " + DC block" : "") +
                                             ": silence in, peak out " + String(peak, 9));
                }
        }
    }

    void rates()
    {
        beginTest("44.1, 48, 96 and 192 kHz: unity at identity, hard clip against the reference, latency reported");
        for (const double rate : {44100.0, 48000.0, 96000.0, 192000.0})
        {
            Shp u(Quality::Standard, rate);
            const Samples in = noise(4000, 0.9f, 5u);
            const Samples out = u.renderFloat(in);
            const size_t lat = static_cast<size_t>(u.latency->latencySamples());
            bool exact = true;
            for (size_t i = lat; i < in.size(); ++i)
                exact = exact && out[i] == in[i - lat];
            Shp c(Quality::Standard, rate);
            c.set(c.idOs, 1.0f);
            expect(c.setCurve({{-1.0f, -0.5f, 0.0f}, {-0.5f, -0.5f, 0.0f}, {0.5f, 0.5f, 0.0f}, {1.0f, 0.5f, 0.0f}}));
            const Samples n2 = noise(6000, 1.5f, 6u);
            const Samples o2 = c.renderFloat(n2);
            double worst = 0.0;
            for (size_t i = 3000; i < n2.size(); ++i)
                worst = std::max(worst, static_cast<double>(std::abs(o2[i] - std::clamp(n2[i], -0.5f, 0.5f))));
            std::printf("  %.0f Hz: identity exact %d, hard clip deviation %.3g, latency %zu\n", rate, exact ? 1 : 0, worst, lat);
            expect(exact && worst < 1.0e-6, String(rate, 0) + " Hz");
        }
    }

    void blockSizes()
    {
        beginTest("a custom curve renders the same whatever the host block size (static settings)");
        Samples first;
        for (const int block : {1, 7, 64, 256, 512})
        {
            Shp s;
            expect(s.setCurve(softS()));
            s.set(s.idDrive, 9.0f);
            s.set(s.idOut, -3.0f);
            s.set(s.idMix, 0.8f);
            const Samples out = s.renderFloat(noise(8000, 0.5f, 13u), block);
            if (first.empty())
                first = out;
            double worst = 0.0;
            for (size_t i = 0; i < out.size(); ++i)
                worst = std::max(worst, static_cast<double>(std::abs(out[i] - first[i])));
            expect(worst < 1.0e-6, "block " + String(block) + ": " + String(worst, 9));
        }
    }

    void zipperAndClick()
    {
        beginTest("no zipper on drive, output and mix (ramp and 2 Hz square), at host blocks of 1, 7 and 512 samples");
        struct Control
        {
            const char* name;
            const char* id;
            float lo, hi;
        };
        // The curve is a straight line of slope 0.5, so the carrier stays a clean sine whatever the gain does; a 6 dB drive
        // on the -6 dBFS carrier just reaches the end of the line.
        const Control controls[] = {{"drive", "fx_shaper_drive", 0.0f, 6.0f},
                                    {"output", "fx_shaper_output", -12.0f, 0.0f},
                                    {"mix", "fx_shaper_mix", 0.0f, 1.0f}};
        for (const Control& c : controls)
            for (const int block : {1, 7, 512})
            {
                const auto render = [&](const Signal& in, const Signal& automation)
                {
                    Shp s;
                    expect(s.setCurve(halfLine()));
                    Samples l(in.size()), r;
                    for (size_t i = 0; i < in.size(); ++i)
                        l[i] = static_cast<float>(in[i]);
                    r = l;
                    const int id = ParameterMap::indexOf(c.id);
                    for (size_t p = 0; p < l.size(); p += static_cast<size_t>(block))
                    {
                        s.set(id, c.lo + (c.hi - c.lo) * static_cast<float>(automation[p]));
                        s.run(l.data() + p, r.data() + p, static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - p)));
                    }
                    return toDouble(l);
                };
                const ZipperResult z = zipperTest(render, kRate, 1000.0, -6.0);
                std::printf("  %s, host block %d: ramp step %.1f dB, 2 Hz square %.1f dB\n", c.name, block, z.rampDb, z.squareDb);
                expect(z.rampDb <= -90.0, String(c.name) + " ramp zipper at block " + String(block) + ": " + String(z.rampDb, 1));
                expect(z.squareDb <= -90.0,
                       String(c.name) + " square zipper at block " + String(block) + ": " + String(z.squareDb, 1));
            }

        beginTest("a curve change on a sine is cross-faded: no step above -60 dB");
        {
            double worst = -300.0;
            for (const int block : {16, 256})
            {
                Shp s;
                expect(s.setCurve(halfLine()));
                const size_t total = 24000;
                Signal in = sineWave(total, 1000.0, kRate, 0.5);
                Samples l(total), r;
                for (size_t i = 0; i < total; ++i)
                    l[i] = static_cast<float>(in[i]);
                r = l;
                for (size_t p = 0; p < total; p += static_cast<size_t>(block))
                {
                    if (p == 9600) // a different slope, in the middle of the stream
                        expect(s.setCurve({{-1.0f, -0.2f, 0.0f}, {1.0f, 0.2f, 0.0f}}));
                    s.run(l.data() + p, r.data() + p, static_cast<int>(std::min<size_t>(static_cast<size_t>(block), total - p)));
                }
                const ClickResult k = clickTest(toDouble(l), 1000.0, kRate, 4800);
                worst = std::max(worst, k.stepDb);
            }
            std::printf("  curve change: largest step %.1f dB\n", worst);
            expect(worst <= -60.0, String(worst, 1));
        }
    }

    void dcBlock()
    {
        beginTest("DC block: an asymmetric curve puts DC on a sine; the block removes it and leaves the tone");
        const auto level = [this](bool dc, double* tone)
        {
            Shp s;
            s.set(s.idDc, dc ? 1.0f : 0.0f);
            expect(s.setCurve({{-1.0f, -0.3f, 0.0f}, {0.0f, 0.2f, 0.0f}, {1.0f, 0.9f, 0.0f}}));
            const Signal out = s.render(sineWave(48000, 440.0, kRate, 0.5));
            if (tone != nullptr)
                *tone = toneAmplitude(out.data() + 24000, 19200, 440.0, kRate);
            return dcOffset(out.data() + 24000, 19200);
        };
        double toneOff = 0.0, toneOn = 0.0;
        const double off = level(false, &toneOff), on = level(true, &toneOn);
        std::printf("  mean with the DC block off %.4f, on %.6f; 440 Hz amplitude %.4f / %.4f\n", off, on, toneOff, toneOn);
        expect(std::abs(off) > 0.05 && std::abs(on) < 2.0e-4 && std::abs(toneOn - toneOff) < 0.002,
               "off " + String(off, 4) + " on " + String(on, 6));
    }

    void aliasing()
    {
        beginTest("aliasing of a drawn hard clip (5 kHz sine, -6 dBFS, +12 dB drive): oversampling lowers it");
        const std::vector<ShaperPoint> clip = {{-1.0f, -0.5f, 0.0f}, {-0.25f, -0.5f, 0.0f}, {0.25f, 0.5f, 0.0f}, {1.0f, 0.5f, 0.0f}};
        double alias[4] = {};
        for (int os = 1; os <= 4; ++os)
        {
            const auto render = [&](const Signal& in)
            {
                Shp s;
                s.set(s.idOs, static_cast<float>(os));
                s.set(s.idDrive, 12.0f);
                expect(s.setCurve(clip));
                return s.render(in);
            };
            alias[os - 1] = aliasAtDrive(render, kRate, 5000.0, {-6.0}, 0.3)[0].aliasDb;
            std::printf("  %dx: alias power %.1f dB\n", 1 << (os - 1), alias[os - 1]);
        }
        // Smooth (curved) shapes alias far less than the hard corners.
        const auto renderSmooth = [&](const Signal& in)
        {
            Shp s;
            s.set(s.idDrive, 12.0f);
            expect(s.setCurve({{-1.0f, -0.8f, 0.0f}, {-0.3f, -0.5f, 0.6f}, {0.3f, 0.5f, -0.6f}, {1.0f, 0.8f, 0.0f}}));
            return s.render(in);
        };
        const double smooth = aliasAtDrive(renderSmooth, kRate, 5000.0, {-6.0}, 0.3)[0].aliasDb;
        std::printf("  smooth curve at the Standard default (4x): alias power %.1f dB\n", smooth);
        // Hard corners are the worst case for a shaper without antiderivative anti-aliasing: the oversampler alone takes 4x
        // (the Standard default) 25 dB below 1x and 8x (High) another 20 dB; a smooth drawing needs no more than 4x.
        expect(alias[1] <= alias[0] - 12.0 && alias[2] <= alias[1] - 7.0 && alias[3] <= alias[2] - 15.0,
               "each doubling of the oversampling lowers the alias power");
        expect(alias[2] <= -35.0, "4x (Standard default) at most -35 dB for hard corners: " + String(alias[2], 1));
        expect(alias[3] <= -55.0, "8x (High) at most -55 dB for hard corners: " + String(alias[3], 1));
        expect(smooth <= -40.0, "a smooth curve at 4x at most -40 dB: " + String(smooth, 1));
    }

    void extremes()
    {
        beginTest("60 s at random parameter extremes and random curves stays finite and bounded");
        Shp s;
        std::uint32_t st = 12345u;
        const auto rnd = [&]
        {
            st = st * 1664525u + 1013904223u;
            return static_cast<float>(st >> 8) / 16777216.0f; // 0..1
        };
        const int blocks = static_cast<int>(60.0 * kRate / 512.0);
        Samples l(512), r(512);
        float peak = 0.0f;
        bool finite = true;
        for (int b = 0; b < blocks; ++b)
        {
            if (b % 40 == 0)
            {
                const int n = 2 + static_cast<int>(rnd() * 62.0f);
                const bool sym = rnd() < 0.3f;
                std::vector<ShaperPoint> pts;
                for (int i = 0; i < n; ++i)
                {
                    const float f = (static_cast<float>(i) + 0.5f * rnd()) / static_cast<float>(n);
                    pts.push_back({sym ? f : -1.0f + 2.0f * f, 2.0f * rnd() - 1.0f, 2.0f * rnd() - 1.0f});
                }
                if (sym && pts[0].x == 0.0f)
                    pts[0].y = 0.0f;
                s.setCurve(pts, sym);
            }
            if (b % 7 == 0)
            {
                const auto extreme = [&](float lo, float hi) { return rnd() < 0.5f ? lo : (rnd() < 0.5f ? hi : lo + (hi - lo) * rnd()); };
                s.set(s.idDrive, extreme(-24.0f, 24.0f));
                s.set(s.idOut, extreme(-24.0f, 12.0f));
                s.set(s.idMix, extreme(0.0f, 1.0f));
                s.set(s.idDc, rnd() < 0.5f ? 1.0f : 0.0f);
                s.set(s.idOs, static_cast<float>(static_cast<int>(rnd() * 5.0f) % 5));
            }
            for (size_t i = 0; i < 512; ++i)
                l[i] = r[i] = (rnd() < 0.5f ? 1.0f : 0.25f) * (2.0f * rnd() - 1.0f);
            s.run(l.data(), r.data(), 512);
            for (size_t i = 0; i < 512; ++i)
            {
                finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]);
                peak = std::max({peak, std::abs(l[i]), std::abs(r[i])});
            }
        }
        std::printf("  60 s of random extremes: peak %.2f, finite %d\n", peak, finite ? 1 : 0);
        expect(finite && peak <= 100.0, "peak " + String(peak, 2));
    }

    void realtime()
    {
        beginTest("no allocation on the audio thread while drive, output, mix, oversampling and the DC block change");
        {
            Shp s(Quality::High);
            expect(s.setCurve(softS()));
            Samples l(256, 0.3f), r(256, 0.3f);
            for (int i = 0; i < 40; ++i)
                s.run(l.data(), r.data(), 256);
            // Curves are published from the control thread (outside the tracked region); the audio thread takes them.
            std::vector<std::vector<ShaperPoint>> curves = {softS(), halfLine(), {{-1.0f, 0.0f, 0.5f}, {0.0f, 0.5f, -0.5f}, {1.0f, -0.5f, 0.0f}}};
            long total = 0;
            for (int i = 0; i < 400; ++i)
            {
                if (i % 20 == 0)
                    expect(s.setCurve(curves[static_cast<size_t>(i / 20) % curves.size()], false));
                s.set(s.idDrive, static_cast<float>((i % 49) - 24));
                s.set(s.idOut, static_cast<float>((i % 37) - 24));
                s.set(s.idMix, static_cast<float>(i % 11) / 10.0f);
                s.set(s.idDc, static_cast<float>(i % 3 == 0));
                s.set(s.idOs, static_cast<float>(i / 10 % 5));
                for (size_t k = 0; k < 256; ++k)
                    l[k] = r[k] = 0.4f * std::sin(0.05f * static_cast<float>(k + 256u * static_cast<size_t>(i)));
                crucible::testing::gAllocations = 0;
                crucible::testing::gTrackAllocations = true;
                s.run(l.data(), r.data(), 256);
                crucible::testing::gTrackAllocations = false;
                total += crucible::testing::gAllocations;
            }
            std::printf("  allocations over 400 blocks: %ld\n", total);
            expect(total == 0, String(static_cast<int>(total)));
        }
    }

    void cpu()
    {
        beginTest("CPU per quality (stereo, 48 kHz, 512-sample blocks, 64-point curve with curvature, +12 dB drive)");
#ifndef CRUCIBLE_TEST_UNTIMED
        crucible::testing::ScopedUserInteractiveQos qos;
        std::vector<ShaperPoint> pts;
        for (int i = 0; i < 64; ++i)
        {
            const float f = static_cast<float>(i) / 63.0f;
            pts.push_back({-1.0f + 2.0f * f, 0.8f * std::sin(6.0f * f + 0.3f), std::cos(11.0f * f)});
        }
        for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
        {
            Shp s(q);
            expect(s.setCurve(pts));
            s.set(s.idDrive, 12.0f);
            Samples l(512), r(512);
            double best = 1.0e9;
            for (int attempt = 0; attempt < 3; ++attempt)
            {
                const int blocks = static_cast<int>(2.0 * kRate / 512.0);
                long phase = 0;
                double used = 0.0;
                for (int b = 0; b < blocks; ++b)
                {
                    for (size_t k = 0; k < 512; ++k)
                        l[k] = r[k] = static_cast<float>(0.5 * std::sin(2.0 * 3.14159265358979 * 220.0 * static_cast<double>(phase++) / kRate));
                    const double c0 = crucible::testing::threadCpuSeconds();
                    s.run(l.data(), r.data(), 512);
                    used += crucible::testing::threadCpuSeconds() - c0;
                }
                best = std::min(best, 100.0 * used / (static_cast<double>(blocks) * 512.0 / kRate));
            }
            std::printf("  quality %d (%dx): %.3f %% of a core\n", static_cast<int>(q), 1 << (q == Quality::Draft ? 1 : (q == Quality::Standard ? 2 : 3)), best);
            // Measured on an idle core: 0.25 %, 0.66 % and 2.2 %; the ceilings leave room for a loaded machine.
            const double limit = q == Quality::Draft ? 0.6 : (q == Quality::Standard ? 1.2 : 3.5);
            expect(best <= limit, "quality " + String(static_cast<int>(q)) + " " + String(best, 3));
        }
#else
        logMessage("timed checks skipped in this build");
#endif
    }
};

static ShaperTests shaperTests;

// The curve is published from one thread while another renders (run under ThreadSanitizer: no data race, no torn table).
class ShaperRaceTests : public UnitTest
{
public:
    ShaperRaceTests() : UnitTest("FxShaperRace", "Effects") {}

    void runTest() override
    {
        beginTest("setCurve / resetCurve from a control thread while the audio thread renders");
        Shp s;
        s.set(s.idDrive, 6.0f);
        std::atomic<bool> stop{false};
        std::atomic<long> published{0};
        std::thread control(
            [&]
            {
                std::uint32_t st = 99u;
                while (!stop.load())
                {
                    st = st * 1664525u + 1013904223u;
                    std::vector<ShaperPoint> pts;
                    const int n = 2 + static_cast<int>((st >> 20) % 30);
                    for (int i = 0; i < n; ++i)
                        pts.push_back({-1.0f + 2.0f * (static_cast<float>(i) + 0.5f) / static_cast<float>(n),
                                       std::sin(static_cast<float>(i + published.load())), 0.5f});
                    std::string error;
                    if ((st >> 8) % 5 == 0)
                        s.curve->resetCurve();
                    else
                        s.curve->setCurve(pts.data(), n, false, error);
                    std::vector<float> view(ShaperCurve::kViewPoints);
                    s.curve->fillCurveView(view.data());
                    published.fetch_add(1);
                }
            });
        Samples l(256), r(256);
        bool finite = true;
        for (int b = 0; b < 3000; ++b)
        {
            for (size_t k = 0; k < 256; ++k)
                l[k] = r[k] = 0.6f * std::sin(0.07f * static_cast<float>(k + 256u * static_cast<size_t>(b)));
            s.run(l.data(), r.data(), 256);
            for (const float v : l)
                finite = finite && std::isfinite(v) && std::abs(v) < 100.0f;
        }
        stop.store(true);
        control.join();
        std::printf("  %ld curves published during 3000 blocks\n", published.load());
        expect(finite && published.load() > 0);
    }
};

static ShaperRaceTests shaperRaceTests;
