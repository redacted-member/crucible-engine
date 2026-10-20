// SPDX-License-Identifier: MIT
// The six shaper curves appended to the saturation library (Rectify half, Rectify full, Sine fold, Linear fold,
// Zero-square, Asymmetric): static transfer, THD against drive, aliasing at 5 kHz, DC after blocking, ADAA orders
// agreeing, and the choice list of fx.saturator (the earlier entries unchanged, the new ones appended).
// Every claim is measured on rendered output.

#include "AnalogMeasure.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Fx.h>
#include <crucible/ModMatrix.h>
#include <crucible/Saturation.h>
#include <crucible/module/ModuleRack.h>

#include <chrono>
#include <cmath>
#include <cstdio>

#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(undefined_behavior_sanitizer)
#define CRUCIBLE_TEST_SANITIZED 1
#endif
#endif
#ifndef CRUCIBLE_TEST_SANITIZED
#define CRUCIBLE_TEST_SANITIZED 0
#endif

using namespace crucible;
using namespace crucible::measure;
using namespace crucible::saturation;
using crucible::test::String;
using crucible::test::UnitTest;

namespace
{
constexpr double kFs = 48000.0;
constexpr int kFirstNew = 10; // Model::RectHalf
constexpr int kNumNew = 6;
const char* const kLibraryNames[kNumNew] = {"RectHalf", "RectFull", "SineFold", "LinFold", "ZeroSq", "Asym"};
const char* const kModuleNames[kNumNew] = {"Rectify Half", "Rectify Full", "Sine Fold",
                                           "Linear Fold",  "Zero-Square",  "Asymmetric"};
const char* const kLegacyModuleNames[10] = {"Soft Clip",   "Hard Clip",      "Op-Amp Rail",  "Diode", "Wavefolder",
                                            "Tube Triode", "Tube Push-Pull", "Tube Pentode", "Tape",  "Transformer"};

Model modelAt(int i)
{
    return static_cast<Model>(kFirstNew + i);
}

// The curve itself: no anti-aliasing, no DC blocker, unity drive.
double staticCurve(Model m, double x)
{
    Saturator s;
    s.prepare(kFs);
    Settings c;
    c.model = m;
    c.adaa = false;
    c.dcCornerHz = 0.0f;
    s.set(c);
    return static_cast<double>(s.process(static_cast<float>(x)));
}

Signal runTone(const Settings& s, double hz, double amp, size_t tail, double seconds = 0.0, int factor = 4)
{
    OversampledSaturator o;
    o.prepare(kFs, factor, OversamplingFilter::HalfBand, Quality::Standard, 512);
    o.set(s);
    const size_t settle = 12000;
    const size_t n = std::max(settle + tail, static_cast<size_t>(seconds * kFs));
    std::vector<float> x(n);
    for (size_t i = 0; i < n; ++i)
        x[i] = static_cast<float>(amp * std::sin(2.0 * kPi * hz * static_cast<double>(i) / kFs));
    for (size_t i = 0; i < n; i += 512)
        o.process(x.data() + i, static_cast<int>(std::min<size_t>(512, n - i)));
    return Signal(x.begin() + static_cast<std::ptrdiff_t>(n - tail), x.end());
}

Settings settingsFor(Model m, double driveDb)
{
    Settings s;
    s.model = m;
    s.driveDb = static_cast<float>(driveDb);
    return s;
}

// fx.saturator alone in the rack.
auto saturatorRenderer(int model, float drive, Quality q)
{
    return [=](const Signal& in)
    {
        ModuleRack rack(ModuleRegistry::instance());
        ModuleContext ctx;
        ctx.sampleRate = kFs;
        ctx.quality = q;
        rack.prepare(ctx);
        auto v = ParameterMap::defaultValues();
        std::vector<float> legacy(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            v[static_cast<size_t>(ParameterMap::indexOf(rack.descriptor(i).onParam))] = 0.0f;
        const auto set = [&](const char* id, float value) { v[static_cast<size_t>(ParameterMap::indexOf(id))] = value; };
        set("fx_saturator_on", 1.0f);
        set("fx_saturator_model", static_cast<float>(model));
        set("fx_saturator_drive", drive);
        set("fx_saturator_mix", 1.0f);
        set("fx_saturator_autogain", 0.0f);
        std::vector<float> l(in.size()), r(in.size());
        for (size_t i = 0; i < in.size(); ++i)
            l[i] = r[i] = static_cast<float>(in[i]);
        for (size_t p = 0; p < l.size(); p += 256)
        {
            const int len = static_cast<int>(std::min<size_t>(256, l.size() - p));
            rack.process(l.data() + p, r.data() + p, len, v.data(), legacy.data(), nullptr, 120.0, nullptr, nullptr);
        }
        return toDouble(l);
    };
}

// The full rectifier has no component at the input frequency: its output repeats at twice the tone, so THD and aliasing
// are measured against that component (the tone is halved to keep the measured fundamental where the others have it).
double outputFundamental(int index, double toneHz)
{
    return index == 1 ? 2.0 * toneHz : toneHz;
}

// Worst aliasing over the 5 and 7 kHz rows at one drive, measured on the module output.
double aliasWorst(int index, float drive, Quality q)
{
    double worst = -300.0;
    for (const double hz : {5000.0, 7000.0})
    {
        const double in = index == 1 ? 0.5 * hz : hz;
        const size_t settle = static_cast<size_t>(0.3 * kFs), window = 1 << 15;
        const Signal out = saturatorRenderer(kFirstNew + index, drive, q)(sineWave(settle + window, in, kFs, dbToAmplitude(-6.0)));
        worst = std::max(worst, measureAliasing(out.data() + settle, window, kFs, hz).aliasDb);
    }
    return worst;
}

} // namespace

class SaturationShapesTests : public UnitTest
{
public:
    SaturationShapesTests() : UnitTest("Saturation shapes", "Engine") {}

    void runTest() override
    {
        beginTest("model names: the earlier ten unchanged, the six new ones appended");
        {
            expect(kNumModels == 16, "model count " + String(kNumModels));
            const auto& names = modelNames();
            expect(static_cast<int>(names.size()) == kNumModels);
            expect(names[4] == "Foldback" && names[9] == "Transformer");
            for (int i = 0; i < kNumNew && kFirstNew + i < static_cast<int>(names.size()); ++i)
                expect(names[static_cast<size_t>(kFirstNew + i)] == kLibraryNames[i], names[static_cast<size_t>(kFirstNew + i)]);
        }

        beginTest("static transfer of every new curve");
        {
            const auto near = [&](double got, double want, const std::string& what)
            { expect(std::abs(got - want) < 1.0e-5, what + ": " + String(got, 6) + " want " + String(want, 6)); };
            near(staticCurve(Model::RectHalf, 0.5), std::tanh(0.5), "rectify half +0.5");
            near(staticCurve(Model::RectHalf, -0.5), 0.0, "rectify half -0.5");
            near(staticCurve(Model::RectFull, 0.5), std::tanh(0.5), "rectify full +0.5");
            near(staticCurve(Model::RectFull, -0.5), std::tanh(0.5), "rectify full -0.5");
            near(staticCurve(Model::SineFold, 0.5), std::sin(0.5), "sine fold 0.5");
            near(staticCurve(Model::SineFold, kPi / 2.0), 1.0, "sine fold pi/2");
            near(staticCurve(Model::SineFold, kPi), 0.0, "sine fold pi");
            near(staticCurve(Model::SineFold, -0.7), -std::sin(0.7), "sine fold -0.7");
            near(staticCurve(Model::LinFold, 0.25), 0.5, "linear fold 0.25");
            near(staticCurve(Model::LinFold, 0.5), 1.0, "linear fold 0.5");
            near(staticCurve(Model::LinFold, 0.75), 0.5, "linear fold 0.75");
            near(staticCurve(Model::LinFold, 1.0), 0.0, "linear fold 1");
            near(staticCurve(Model::LinFold, -0.25), -0.5, "linear fold -0.25");
            near(staticCurve(Model::ZeroSquare, 0.1), std::tanh(0.8), "zero-square +0.1");
            near(staticCurve(Model::ZeroSquare, -0.1), -std::tanh(0.8), "zero-square -0.1");
            near(staticCurve(Model::Asym, 0.5), std::tanh(0.5), "asymmetric +0.5");
            near(staticCurve(Model::Asym, -0.5), 0.4 * std::tanh(-0.5 / 0.4), "asymmetric -0.5");
            for (int i = 0; i < kNumNew; ++i)
                near(staticCurve(modelAt(i), 0.0), 0.0, std::string(kLibraryNames[i]) + " at 0");
        }

        beginTest("THD against drive: rising for full rectifier, zero-square and asymmetric; the half rectifier settles; folds exempt");
        {
            // The rectifiers and the zero-square approach a fixed shape (a half or full wave of a square), so their THD
            // settles instead of growing without limit: the test asks for one direction of travel, not a rise.
            for (const int i : {0, 1, 4, 5})
            {
                double thd[5];
                int k = 0;
                for (const double drive : {0.0, 6.0, 12.0, 18.0, 24.0})
                {
                    const double f0 = outputFundamental(i, 1000.0);
                    thd[k++] = harmonicProfile(runTone(settingsFor(modelAt(i), drive), 1000.0, 0.3, 32768), kFs, f0, 20).thdDb;
                }
                std::printf("  %-9s THD at drive 0..24 dB: %.1f %.1f %.1f %.1f %.1f\n", kLibraryNames[i], thd[0], thd[1], thd[2],
                            thd[3], thd[4]);
                bool up = true;
                double lo = thd[0], hi = thd[0];
                for (int j = 1; j < 5; ++j)
                {
                    up = up && thd[j] >= thd[j - 1] - 0.1;
                    lo = std::min(lo, thd[j]);
                    hi = std::max(hi, thd[j]);
                }
                if (i == 0)
                    expect(hi - lo < 1.5, "half rectifier THD stays on its limit shape (spread " + String(hi - lo, 2) + " dB)");
                else
                    expect(up, std::string(kLibraryNames[i]) + " THD is not rising with drive");
            }
        }

        beginTest("harmonic signatures: the half rectifier and the asymmetric curve make H2, the full rectifier only even harmonics");
        {
            const auto half = harmonicProfile(runTone(settingsFor(Model::RectHalf, 6.0), 1000.0, 0.3, 32768), kFs, 1000.0, 6);
            expect(half.harmonicDb(2) > -30.0, "half H2 " + String(half.harmonicDb(2), 1));
            const auto full = harmonicProfile(runTone(settingsFor(Model::RectFull, 6.0), 1000.0, 0.3, 32768), kFs, 1000.0, 6);
            expect(full.harmonicDb(1) < full.harmonicDb(2) - 60.0 && full.harmonicDb(3) < full.harmonicDb(2) - 60.0 &&
                       full.harmonicDb(5) < full.harmonicDb(2) - 60.0,
                   "full H1 " + String(full.harmonicDb(1), 1) + " H2 " + String(full.harmonicDb(2), 1) + " H3 " +
                       String(full.harmonicDb(3), 1));
            const auto asym = harmonicProfile(runTone(settingsFor(Model::Asym, 12.0), 1000.0, 0.3, 32768), kFs, 1000.0, 6);
            expect(asym.harmonicDb(2) > -40.0, "asymmetric H2 " + String(asym.harmonicDb(2), 1));
            for (const Model m : {Model::SineFold, Model::LinFold, Model::ZeroSquare})
            {
                const auto p = harmonicProfile(runTone(settingsFor(m, 12.0), 1000.0, 0.3, 32768), kFs, 1000.0, 6);
                expect(p.harmonicDb(2) < -90.0 && p.harmonicDb(4) < -90.0, modelName(m) + " H2 " + String(p.harmonicDb(2), 1));
            }
        }

        beginTest("DC after blocking: every new curve leaves less than -60 dBFS, corner 8 Hz");
        {
            for (int i = 0; i < kNumNew; ++i)
                for (const double drive : {0.0, 12.0})
                {
                    const Signal y = runTone(settingsFor(modelAt(i), drive), 1000.0, 0.5, 32768, 1.5);
                    expect(dcDbfs(y) < -60.0, std::string(kLibraryNames[i]) + " drive " + String(drive, 0) + " DC " +
                                                  String(dcDbfs(y), 1) + " dB");
                }
            // The blocker is what removes it: without it the rectifiers carry real DC.
            Settings s = settingsFor(Model::RectFull, 6.0);
            s.dcCornerHz = 0.0f;
            const Signal raw = runTone(s, 1000.0, 0.5, 32768, 1.5);
            expect(dcDbfs(raw) > -20.0, "full rectifier without blocker " + String(dcDbfs(raw), 1) + " dB");
        }

        beginTest("second-order ADAA (table) agrees with the first order (closed form) at 100 Hz");
        {
            for (int i = 0; i < kNumNew; ++i)
            {
                double level[2][6];
                for (int order = 1; order <= 2; ++order)
                {
                    Settings s = settingsFor(modelAt(i), 12.0);
                    s.adaaOrder = order;
                    Saturator sat;
                    sat.prepare(kFs * 4);
                    sat.set(s);
                    std::vector<float> x(1 << 17);
                    for (size_t k = 0; k < x.size(); ++k)
                        x[k] = static_cast<float>(0.3 * std::sin(2.0 * kPi * 100.0 * static_cast<double>(k) / (kFs * 4)));
                    sat.process(x.data(), x.data(), static_cast<int>(x.size()));
                    const auto p = harmonicProfile(toDouble(x, 1 << 15), kFs * 4, i == 1 ? 200.0 : 100.0, 6);
                    for (int h = 1; h <= 6; ++h)
                        level[order - 1][h - 1] = p.harmonicDb(h);
                }
                for (int h = 0; h < 6; ++h)
                    if (level[0][h] > -100.0)
                        expect(std::abs(level[0][h] - level[1][h]) < 0.5, std::string(kLibraryNames[i]) + " H" + String(h + 1) +
                                                                              " order 1 " + String(level[0][h], 2) + " order 2 " +
                                                                              String(level[1][h], 2));
            }
        }

        beginTest("library: aliasing of the curves behind the oversampler (second order ADAA, drive 9 dB, 5 kHz, -6 dBFS)");
        {
            for (int i = 0; i < kNumNew; ++i)
            {
                double a[3];
                int k = 0;
                for (const int factor : {2, 4, 8})
                {
                    Settings s = settingsFor(modelAt(i), 9.0);
                    s.adaaOrder = 2;
                    const double in = i == 1 ? 2500.0 : 5000.0;
                    a[k++] = measureAliasing(runTone(s, in, 0.5, 32768, 1.0, factor), 0, kFs, 5000.0).aliasDb;
                }
                std::printf("  library %-9s alias at 2x / 4x / 8x: %.1f %.1f %.1f\n", kLibraryNames[i], a[0], a[1], a[2]);
            }
        }

        beginTest("fx.saturator: model choices keep the ten earlier entries and append the six new ones");
        {
            const ModuleDescriptor* d = ModuleRegistry::instance().find("fx.saturator");
            expect(d != nullptr);
            if (d != nullptr)
                for (const auto& p : d->params)
                    if (p.id == "fx_saturator_model")
                    {
                        expect(static_cast<int>(p.choices.size()) == 16, "choices " + String(static_cast<int>(p.choices.size())));
                        for (int i = 0; i < 16 && i < static_cast<int>(p.choices.size()); ++i)
                            expect(p.choices[static_cast<size_t>(i)] == (i < 10 ? kLegacyModuleNames[i] : kModuleNames[i - 10]),
                                   p.choices[static_cast<size_t>(i)]);
                    }
        }

        beginTest("fx.saturator: aliasing over the drive range, Standard at most -90 dB, High at most -100 dB");
        {
            std::vector<float> drives;
            for (int k = 3; k <= 36; ++k)
                drives.push_back(static_cast<float>(k));
            for (int i = 0; i < kNumNew; ++i)
            {
                double worstStd = -300.0, worstHigh = -300.0;
                std::printf("  %-12s alias Standard/High at drive 3..36 dB step 1:", kModuleNames[i]);
                for (const float d : drives)
                {
                    const double s = aliasWorst(i, d, Quality::Standard), h = aliasWorst(i, d, Quality::High);
                    std::printf(" %.0f/%.0f", s, h);
                    worstStd = std::max(worstStd, s);
                    worstHigh = std::max(worstHigh, h);
                }
                std::printf("\n");
                expect(worstStd <= -90.0, std::string(kModuleNames[i]) + " Standard " + String(worstStd, 1));
                expect(worstHigh <= -100.0, std::string(kModuleNames[i]) + " High " + String(worstHigh, 1));
            }
        }

        beginTest("fx.saturator: finite output and DC below -60 dB at every setting of the new models");
        {
            for (int i = 0; i < kNumNew; ++i)
                for (const float drive : {0.0f, 12.0f, 36.0f})
                {
                    const Signal y = saturatorRenderer(kFirstNew + i, drive, Quality::Standard)(sineWave(60000, 1000.0, kFs, 0.5));
                    bool finite = true;
                    for (const double v : y)
                        finite = finite && std::isfinite(v) && std::abs(v) < 8.0;
                    expect(finite, std::string(kModuleNames[i]) + " drive " + String(drive, 0));
                    const Signal tail(y.end() - 24000, y.end());
                    expect(dcDbfs(tail) < -60.0, std::string(kModuleNames[i]) + " drive " + String(drive, 0) + " DC " +
                                                     String(dcDbfs(tail), 1));
                }
        }

        beginTest("CPU: one library instance of each new curve (Standard oversampling, drive 12 dB) at most 1 % of a core");
        {
            // Timing is meaningless in debug and sanitizer builds.
#if defined(NDEBUG) && !defined(__SANITIZE_ADDRESS__) && !defined(__SANITIZE_THREAD__) && !CRUCIBLE_TEST_SANITIZED
            const Model reference[] = {Model::Soft, Model::Fold, Model::Diode};
            const auto timeModel = [&](Model m)
            {
                double best = 1.0e9;
                for (int attempt = 0; attempt < 3; ++attempt)
                {
                    OversampledSaturator o;
                    o.prepare(kFs, recommendedOversampling(m, Quality::Standard), OversamplingFilter::HalfBand, Quality::Standard, 512);
                    o.set(settingsFor(m, 12.0));
                    std::vector<float> x(512);
                    const int blocks = static_cast<int>(4.0 * kFs / 512.0);
                    long phase = 0;
                    const auto start = std::chrono::steady_clock::now();
                    for (int b = 0; b < blocks; ++b)
                    {
                        for (auto& v : x)
                            v = static_cast<float>(0.5 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(phase++) / kFs));
                        o.process(x.data(), 512);
                    }
                    best = std::min(best, 100.0 * std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() / 4.0);
                }
                return best;
            };
            for (const Model m : reference)
                std::printf("  CPU %-9s (reference) %.2f %% of a core\n", modelName(m).c_str(), timeModel(m));
            for (int i = 0; i < kNumNew; ++i)
            {
                const double best = timeModel(modelAt(i));
                std::printf("  CPU %-9s %.2f %% of a core\n", kLibraryNames[i], best);
                expectCpu(best <= 1.0, std::string(kLibraryNames[i]) + " " + String(best, 2) + " % of a core");
            }
#endif
        }
    }
};


static SaturationShapesTests saturationShapesTests;
