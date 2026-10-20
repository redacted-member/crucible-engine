// SPDX-License-Identifier: MIT
// The analog filter interface and the ladder family (transistor ladder, OTA cascade, coupled diode ladder) measured
// with the P1 harness: self-oscillation tracking and threshold, bass loss, slopes, level dependence, aliasing at full
// drive, stability under 60 s of random modulation, the four-lane path and CPU. Every claim is a number
// (docs/plans/ENGINE-COMMUNITY-PLAN.md, sections 2.6, 2.7 and 3.1).

#include "AnalogMeasure.h"
#include "TestSupport.h"

#include <crucible/Filters.h>
#include <crucible/Simd.h>
#include <crucible/filters/FilterModel.h>

#include <chrono>
#include <memory>

#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(undefined_behavior_sanitizer)
#define CRUCIBLE_TEST_SANITIZED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define CRUCIBLE_TEST_SANITIZED 1
#endif
#ifndef CRUCIBLE_TEST_SANITIZED
#define CRUCIBLE_TEST_SANITIZED 0
#endif

using namespace crucible;
using namespace crucible::testing;
namespace m = crucible::measure;

namespace
{
constexpr double kFs = 48000.0;

// Number as text for failure messages.
std::string num(double v, int decimals = 3)
{
    return String(v, decimals).str();
}

struct ModelCase
{
    const char* name;
    FilterModel model;
    std::unique_ptr<AnalogFilter> (*make)();
    double slopeDb;        // expected slope 2 to 4 octaves above the cutoff
    double slopeTolerance; // and its tolerance
    double onsetBand;      // the self-oscillation threshold lies in [gain, gain + onsetBand]
};

const std::array<ModelCase, 3>& models()
{
    static const std::array<ModelCase, 3> cases = {{
        {"transistor ladder", FilterModel::TransistorLadder, makeTransistorLadder, -24.0, 1.5, 0.3},
        {"OTA cascade", FilterModel::OtaCascade, makeOtaCascade, -24.0, 1.5, 0.3},
        {"coupled diode ladder", FilterModel::CoupledDiodeLadder, makeDiodeLadder, -18.0, 3.0, 0.5},
    }};
    return cases;
}

std::unique_ptr<AnalogFilter> build(const ModelCase& c, Quality q, int lanes = 1, int maxBlock = 128)
{
    auto f = c.make();
    f->prepare(kFs, q, lanes, maxBlock);
    return f;
}

// Block-wise in-place processing, as an engine would run it.
void run(AnalogFilter& f, std::vector<float>& x, int block = 128, const std::vector<float>* mod = nullptr)
{
    for (size_t off = 0; off < x.size(); off += static_cast<size_t>(block))
    {
        const int n = static_cast<int>(std::min<size_t>(static_cast<size_t>(block), x.size() - off));
        f.process(x.data() + off, n, mod != nullptr ? mod->data() + off : nullptr);
    }
}

// Sample-wise adapter for the measurement primitives (a fresh filter per measurement, from the factory).
struct Sample
{
    std::shared_ptr<AnalogFilter> f;
    double operator()(double v)
    {
        float x = static_cast<float>(v);
        f->process(&x, 1);
        return x;
    }
};

struct Oscillation
{
    double freqHz = 0.0;
    double levelDbfs = -300.0;
    double variationDb = 0.0;
};

// Full resonance on silence with the start-up noise on; the last 40 % is analysed.
Oscillation oscillate(const ModelCase& c, Quality q, float fc, double seconds, float tolerance = 0.0f,
                      std::uint32_t seed = 1, float res = 1.0f)
{
    auto f = build(c, q);
    f->setSeed(seed);
    f->setControls(fc, res, 0.0f, 0.0f, tolerance);
    f->setStartupNoise(true);
    std::vector<float> x(static_cast<size_t>(seconds * kFs), 0.0f);
    x[0] = 1.0e-3f; // the noise alone needs several seconds to reach the limit cycle at 40 Hz
    run(*f, x);
    const size_t from = static_cast<size_t>(0.6 * static_cast<double>(x.size()));
    const auto tail = m::toDouble(x, from);
    Oscillation o;
    o.levelDbfs = m::rmsDbfs(tail);
    double lo = 1.0e9, hi = -1.0e9;
    const size_t seg = tail.size() / 8;
    for (size_t s = 0; s < 8; ++s)
    {
        const double db = m::rmsDbfs(tail.data() + s * seg, seg);
        lo = std::min(lo, db);
        hi = std::max(hi, db);
    }
    o.variationDb = hi - lo;
    if (o.levelDbfs > -100.0)
        o.freqHz = m::estimateFrequency(tail, kFs, fc);
    return o;
}

// Level change of a ring-down / build-up between two windows after a small kick, in dB.
double growthDb(const ModelCase& c, double k)
{
    auto f = build(c, Quality::Standard);
    const float res = static_cast<float>(k) / f->feedbackForResonance(1.0f);
    f->setControls(1000.0f, res, 0.0f, 0.0f, 0.0f);
    std::vector<float> x(static_cast<size_t>(0.6 * kFs), 0.0f);
    x[0] = 1.0e-4f;
    run(*f, x);
    const auto rms = [&](double t0, double t1)
    {
        const auto a = static_cast<size_t>(t0 * kFs), b = static_cast<size_t>(t1 * kFs);
        return m::rmsDbfs(m::toDouble(x, a, b - a));
    };
    return rms(0.40, 0.50) - rms(0.10, 0.20);
}

double dcGainMeasured(const ModelCase& c, float res, float character)
{
    const auto make = [&]
    {
        auto f = std::shared_ptr<AnalogFilter>(build(c, Quality::Standard));
        f->setControls(4000.0f, res, 0.0f, character, 0.0f);
        return Sample{f};
    };
    return m::steppedGainDb(make, 25.0, kFs, 0.01, 0.5, 0.3);
}

double peakGainDb(const ModelCase& c, double levelDbfs, float res)
{
    double best = -300.0;
    for (double hz = 640.0; hz <= 1250.0; hz *= 1.07)
    {
        const auto make = [&]
        {
            auto f = std::shared_ptr<AnalogFilter>(build(c, Quality::Standard));
            f->setControls(1000.0f, res, 0.0f, 0.0f, 0.0f);
            return Sample{f};
        };
        best = std::max(best, m::steppedGainDb(make, hz, kFs, std::pow(10.0, levelDbfs / 20.0), 0.3, 0.1));
    }
    return best;
}

// Aliasing of a 5 kHz sine through the filter (non-harmonic energy below 20 kHz against the fundamental), dB.
double aliasing(const ModelCase& c, Quality q, float amplitude, float drive, float fc = 8000.0f, float res = 0.5f)
{
    auto f = build(c, q);
    f->setControls(fc, res, drive, 0.0f, 0.0f);
    const size_t n = 1u << 16, settle = 1u << 14;
    std::vector<float> x(n + settle);
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = amplitude * static_cast<float>(std::sin(2.0 * m::kPi * 5000.0 * static_cast<double>(i) / kFs));
    run(*f, x);
    const auto sig = m::toDouble(x, settle, n);
    return m::measureAliasing(sig.data(), sig.size(), kFs, 5000.0).aliasDb;
}

// Random control and audio activity for `seconds`; returns the largest output magnitude, NaN when non-finite.
float stress(const ModelCase& c, Quality q, double seconds, bool quad, std::uint64_t seed)
{
    m::Random rng(seed);
    const int lanes = quad ? 4 : 1;
    auto f = build(c, q, lanes, 128);
    std::vector<std::vector<float>> io(4, std::vector<float>(128)), mod(4, std::vector<float>(128));
    float peak = 0.0f;
    const int blocks = static_cast<int>(seconds * kFs / 128.0);
    for (int b = 0; b < blocks; ++b)
    {
        for (int l = 0; l < lanes; ++l)
        {
            const float cutoff = 20.0f * std::pow(1000.0f, static_cast<float>(rng.uniform()));
            const float res = static_cast<float>(rng.uniform());
            const float drive = rng.uniform() < 0.3 ? static_cast<float>(rng.uniform()) : 0.0f;
            const float chr = static_cast<float>(rng.uniform());
            const float tol = rng.uniform() < 0.5 ? static_cast<float>(rng.uniform()) : 0.0f;
            f->setLaneControls(l, cutoff, res, drive, chr, tol);
            const float level = static_cast<float>(std::pow(10.0, -4.0 * rng.uniform() + 0.5));
            float walk = static_cast<float>(rng.gaussian());
            for (size_t i = 0; i < 128; ++i)
            {
                io[static_cast<size_t>(l)][i] =
                    (rng.uniform() < 0.01 ? 4.0f : 1.0f) * level * static_cast<float>(rng.gaussian() * 0.3);
                walk += static_cast<float>(rng.gaussian()) * 0.05f;
                mod[static_cast<size_t>(l)][i] = std::clamp(walk, -3.0f, 3.0f);
            }
        }
        if (quad)
        {
            float* p[4] = {io[0].data(), io[1].data(), io[2].data(), io[3].data()};
            const float* mp[4] = {mod[0].data(), mod[1].data(), mod[2].data(), mod[3].data()};
            f->processQuad(p, 128, mp);
        }
        else
            f->process(io[0].data(), 128, mod[0].data());
        for (int l = 0; l < lanes; ++l)
            for (const float v : io[static_cast<size_t>(l)])
            {
                if (!std::isfinite(v))
                    return std::numeric_limits<float>::quiet_NaN();
                peak = std::max(peak, std::abs(v));
            }
    }
    return peak;
}

// Percent of one core per voice for 128-sample blocks of noise at 48 kHz, the best of three runs.
double cpuPercent(const ModelCase& c, Quality q, bool quad)
{
    const int blocks = static_cast<int>(3.0 * kFs / 128.0);
    double best = 1.0e9;
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        auto f = build(c, q, quad ? 4 : 1, 128);
        for (int l = 0; l < (quad ? 4 : 1); ++l)
            f->setLaneControls(l, 2000.0f, 0.7f, 0.2f, 0.0f, 0.0f);
        std::vector<std::vector<float>> io(4, std::vector<float>(128));
        m::Random rng(3);
        for (auto& v : io)
            for (auto& s : v)
                s = static_cast<float>(rng.gaussian() * 0.2);
        const auto start = std::chrono::steady_clock::now();
        for (int b = 0; b < blocks; ++b)
        {
            if (quad)
            {
                float* p[4] = {io[0].data(), io[1].data(), io[2].data(), io[3].data()};
                f->processQuad(p, 128, nullptr);
            }
            else
                f->process(io[0].data(), 128);
            for (auto& v : io)
                for (auto& s : v)
                    s = std::clamp(s, -1.0f, 1.0f);
        }
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        best = std::min(best, 100.0 * secs / 3.0 / (quad ? 4.0 : 1.0));
    }
    return best;
}
} // namespace

class AnalogFilterLadderTests : public UnitTest
{
public:
    AnalogFilterLadderTests() : UnitTest("AnalogFilterLadder", "Filters") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("the legacy filter indices and names are unchanged and the new models are appended");
        {
            expectEquals(static_cast<int>(FilterModel::DirtyLowpass), 16);
            expectEquals(static_cast<int>(FilterModel::TransistorLadder), 17);
            expectEquals(static_cast<int>(FilterModel::OtaCascade), 18);
            expectEquals(static_cast<int>(FilterModel::CoupledDiodeLadder), 19);
            expectEquals(static_cast<int>(FilterModel::Count), 25); // the Sallen-Key family follows (AnalogFilterSkTests)
            const auto& names = filterModelNames();
            expectEquals(static_cast<int>(names.size()), 25);
            const char* legacy[17] = {"Lowpass 12",  "Highpass 12", "Bandpass 12", "Notch",     "Peak",      "Lowpass 24",
                                      "Highpass 24", "Bandpass 24", "Ladder LP",   "Ladder HP", "Ladder BP", "Diode LP",
                                      "Comb +",      "Comb -",      "Formant",     "Phaser",    "Dirty LP"};
            for (size_t i = 0; i < 17; ++i)
                expectEquals(names[i], std::string(legacy[i]));
        }

        beginTest("self-oscillation tracks the cutoff: 5 cents at Standard and High, 15 cents at Draft, steady amplitude");
        {
            for (const auto& c : models())
            {
                for (const float fc : {40.0f, 100.0f, 250.0f, 1000.0f, 4000.0f, 8000.0f})
                {
                    const double seconds = fc < 60.0f ? 8.0 : (fc < 300.0f ? 4.0 : 2.5);
                    for (const Quality q : {Quality::Standard, Quality::High})
                    {
                        if (q == Quality::High && fc < 100.0f)
                            continue; // the same loop at a quarter of the cost, 40 Hz alone takes 14 s
                        const auto o = oscillate(c, q, fc, seconds);
                        const double cents = m::centsBetween(o.freqHz, fc);
                        expect(o.levelDbfs > -40.0 && std::abs(cents) <= 5.0 && o.variationDb < 3.0,
                               std::string(c.name) + (q == Quality::High ? " High " : " Standard ") + num(fc) +
                                   " Hz: " + num(cents, 2) + " cents, " + num(o.levelDbfs, 1) + " dBFS, steadiness " +
                                   num(o.variationDb, 2) + " dB");
                    }
                    if (fc >= 100.0f)
                    {
                        const auto d = oscillate(c, Quality::Draft, fc, seconds);
                        const double cents = m::centsBetween(d.freqHz, fc);
                        expect(d.levelDbfs > -40.0 && std::abs(cents) <= 15.0 && d.variationDb < 3.0,
                               std::string(c.name) + " Draft " + num(fc) + " Hz: " + num(cents, 2) + " cents");
                    }
                }
            }
        }

        beginTest("self-oscillation threshold: the ladders start at k = 4.0 to 4.3, the diode ladder just above its limit");
        {
            for (const auto& c : models())
            {
                const double kc = build(c, Quality::Standard)->selfOscillationGain();
                const double step = kc > 8.0 ? 0.1 : 0.05;
                double onset = 0.0;
                bool grewBelow = false;
                for (double k = kc - 6.0 * step; k <= kc + 12.0 * step; k += step)
                {
                    const double g = growthDb(c, k);
                    if (k <= kc - 0.5 * step && g > 0.3)
                        grewBelow = true;
                    if (onset == 0.0 && g > 0.3)
                        onset = k;
                }
                expect(!grewBelow, std::string(c.name) + " builds up below its linear threshold");
                expect(onset >= kc && onset <= kc + c.onsetBand,
                       std::string(c.name) + " onset k = " + num(onset, 2) + ", linear threshold " + num(kc, 2));
            }
        }

        beginTest("bass loss follows -20 log10(1 + k) and character gives it back (-40 dBFS)");
        {
            for (const auto& c : models())
                for (const float res : {0.0f, 0.25f, 0.5f, 0.75f, 0.95f})
                {
                    auto probe = build(c, Quality::Standard);
                    const double k = probe->feedbackForResonance(res);
                    const double natural = dcGainMeasured(c, res, 0.0f);
                    const double expected = 20.0 * std::log10(probe->dcGain(res, 0.0f));
                    expectWithinAbsoluteError(natural, expected, 0.5,
                                              std::string(c.name) + " res " + num(res) + " k " + num(k, 2));
                    if (c.model != FilterModel::OtaCascade)
                        expectWithinAbsoluteError(expected, -20.0 * std::log10(1.0 + k), 0.01, std::string(c.name) + " law");
                    expectWithinAbsoluteError(dcGainMeasured(c, res, 1.0f), 0.0, 0.5,
                                              std::string(c.name) + " compensated, res " + num(res));
                }
            // The OTA cascade keeps more of its bass than the transistor ladder at the same resonance.
            expect(dcGainMeasured(models()[1], 0.9f, 0.0f) > dcGainMeasured(models()[0], 0.9f, 0.0f) + 4.0);
        }

        beginTest("slopes 2 to 4 octaves above the cutoff: 24 +-1.5 dB/oct (transistor, OTA), 18 +-3 (diode)");
        {
            const auto grid = m::logSpacedFrequencies(1000.0, 4000.0, 12);
            for (const auto& c : models())
                for (const Quality q : {Quality::Standard, Quality::High})
                {
                    const auto make = [&]
                    {
                        auto f = std::shared_ptr<AnalogFilter>(build(c, q));
                        f->setControls(250.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                        return Sample{f};
                    };
                    const auto curve = m::steppedResponse(make, grid, kFs, 0.01, 0.4, 0.2);
                    const double slope = m::slopeDbPerOctave(curve, 1000.0, 4000.0);
                    expectWithinAbsoluteError(slope, c.slopeDb, c.slopeTolerance,
                                              std::string(c.name) + (q == Quality::High ? " High" : " Standard"));
                }
        }

        beginTest("the OTA cascade has a four-step input high-pass (off, 60, 180, 540 Hz); the other models ignore it");
        {
            const auto gain = [&](const ModelCase& c, int step, double hz)
            {
                const auto make = [&]
                {
                    auto f = std::shared_ptr<AnalogFilter>(build(c, Quality::Standard));
                    f->setControls(8000.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    f->setHighpassStep(step);
                    return Sample{f};
                };
                return m::steppedGainDb(make, hz, kFs, 0.01, 0.4, 0.2);
            };
            expectWithinAbsoluteError(gain(models()[1], 0, 100.0), 0.0, 0.3, "off");
            const double corner[4] = {0.0, 60.0, 180.0, 540.0};
            for (int step = 1; step <= 3; ++step)
            {
                expectWithinAbsoluteError(gain(models()[1], step, corner[step]), -3.01, 0.5,
                                          "corner of step " + num(step, 0));
                expectWithinAbsoluteError(gain(models()[1], step, corner[step] / 8.0), -18.1, 1.0,
                                          "one pole below the corner");
            }
            expectWithinAbsoluteError(gain(models()[0], 3, 100.0), gain(models()[0], 0, 100.0), 0.01, "transistor ladder");
        }

        beginTest("level dependence: the resonant peak compresses monotonically from -40 to +6 dBFS, the legacy filter not");
        {
            for (const auto& c : models())
            {
                const double a = peakGainDb(c, -40.0, 0.9f), b = peakGainDb(c, -12.0, 0.9f), d = peakGainDb(c, 0.0, 0.9f),
                             e = peakGainDb(c, 6.0, 0.9f);
                expect(a > b + 0.3 && b > d + 0.3 && d > e + 0.3, std::string(c.name) + ": " + num(a, 2) + ", " + num(b, 2) +
                                                                      ", " + num(d, 2) + ", " + num(e, 2) + " dB");
                expect(a - e > 6.0, std::string(c.name) + " total compression " + num(a - e, 2) + " dB");
            }
            // Legacy models are unchanged and level independent.
            const auto legacyPeak = [&](double level)
            {
                double best = -300.0;
                for (double hz = 640.0; hz <= 1250.0; hz *= 1.07)
                {
                    const auto make = [&]
                    {
                        FilterChannel ch;
                        ch.configure({static_cast<int>(FilterModel::Lowpass12), 1000.0f, 0.9f}, kFs);
                        return [ch](double x) mutable { return static_cast<double>(ch.process(static_cast<float>(x))); };
                    };
                    best = std::max(best, m::steppedGainDb(make, hz, kFs, std::pow(10.0, level / 20.0), 0.3, 0.1));
                }
                return best;
            };
            expectWithinAbsoluteError(legacyPeak(-40.0), legacyPeak(6.0), 0.05, "legacy 12 dB low-pass");
        }

        beginTest("nonlinear aliasing at 5 kHz and full drive: -90 dB at Standard (-6 dBFS), -100 dB at High (0 dBFS)");
        {
            for (const auto& c : models())
            {
                const double std6 = aliasing(c, Quality::Standard, 0.5f, 1.0f);
                const double std0 = aliasing(c, Quality::Standard, 1.0f, 1.0f);
                const double high0 = aliasing(c, Quality::High, 1.0f, 1.0f);
                const double plain = aliasing(c, Quality::Standard, 1.0f, 0.0f);
                logMessage(std::string(c.name) + ": Standard " + num(std6, 1) + " dB at -6 dBFS, " + num(std0, 1) +
                           " dB at 0 dBFS; High " + num(high0, 1) + " dB at 0 dBFS; no drive " + num(plain, 1) + " dB");
                expect(std6 <= -90.0, std::string(c.name) + " Standard at -6 dBFS: " + num(std6, 1) + " dB");
                expect(high0 <= -100.0, std::string(c.name) + " High at 0 dBFS: " + num(high0, 1) + " dB");
                expect(plain <= -90.0, std::string(c.name) + " Standard, no drive, 0 dBFS: " + num(plain, 1) + " dB");
            }
        }

        beginTest("60 s of random cutoff, resonance, drive, tolerance and audio activity stays finite and bounded");
        {
            const double seconds = CRUCIBLE_TEST_SANITIZED ? 6.0 : 60.0;
            for (const auto& c : models())
            {
                for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
                {
                    const float peak = stress(c, q, q == Quality::High ? seconds * 0.5 : seconds, false, 11);
                    expect(std::isfinite(peak) && peak < 100.0f,
                           std::string(c.name) + " tier " + num(static_cast<int>(q), 0) + " peak " + num(peak));
                }
                const float peak = stress(c, Quality::Standard, seconds, true, 12);
                expect(std::isfinite(peak) && peak < 100.0f, std::string(c.name) + " four lanes, peak " + num(peak));
            }
        }

        beginTest("processQuad equals four mono filters with per-lane controls");
        {
            for (const auto& c : models())
            {
                auto quad = build(c, Quality::Standard, 4);
                std::array<std::unique_ptr<AnalogFilter>, 4> mono;
                m::Random rng(5);
                for (int l = 0; l < 4; ++l)
                {
                    mono[static_cast<size_t>(l)] = build(c, Quality::Standard);
                    const float cutoff = 200.0f * std::pow(2.0f, static_cast<float>(l) * 1.7f);
                    const float res = 0.2f + 0.2f * static_cast<float>(l);
                    quad->setLaneControls(l, cutoff, res, 0.1f * static_cast<float>(l), 0.25f * static_cast<float>(l), 0.0f);
                    mono[static_cast<size_t>(l)]->setControls(cutoff, res, 0.1f * static_cast<float>(l),
                                                              0.25f * static_cast<float>(l), 0.0f);
                }
                double worst = 0.0;
                for (int b = 0; b < 400; ++b)
                {
                    std::vector<std::vector<float>> a(4, std::vector<float>(128));
                    for (auto& v : a)
                        for (auto& s : v)
                            s = static_cast<float>(rng.gaussian() * 0.3);
                    auto ref = a;
                    float* p[4] = {a[0].data(), a[1].data(), a[2].data(), a[3].data()};
                    quad->processQuad(p, 128);
                    for (size_t l = 0; l < 4; ++l)
                    {
                        mono[l]->process(ref[l].data(), 128);
                        for (size_t i = 0; i < 128; ++i)
                            worst = std::max(worst, static_cast<double>(std::abs(a[l][i] - ref[l][i])));
                    }
                }
                expect(worst < 1.0e-4, std::string(c.name) + " largest lane difference " + num(worst));
            }
        }

        beginTest("processQuad is at least 2.5 times faster than four scalar calls");
        {
            // Needs real vector instructions and an optimised build.
#if defined(NDEBUG) && !CRUCIBLE_TEST_SANITIZED && CRUCIBLE_SIMD_NATIVE
            for (const auto& c : models())
            {
                double speedup = 0.0;
                for (int attempt = 0; attempt < 3 && speedup < 2.5; ++attempt)
                {
                    const double mono = cpuPercent(c, Quality::Standard, false);
                    const double quad = cpuPercent(c, Quality::Standard, true);
                    speedup = mono / quad;
                }
                expect(speedup >= 2.5, std::string(c.name) + " speed-up " + num(speedup, 2));
            }
#endif
        }

        beginTest("CPU per voice at Standard (48 kHz, 128-sample blocks): processQuad within 20 % of the plan's target");
        {
            // The absolute numbers are those of an Apple M-class core (the plan's reference machine); other hosts
            // only record them.
#if defined(NDEBUG) && !CRUCIBLE_TEST_SANITIZED && CRUCIBLE_SIMD_NEON
            // Plan targets (section 2.6): ladders at most 0.12 % per voice, diode ladder 0.18 %, and the bench fails
            // at target + 20 %. The target is met through processQuad (four voices per call). One voice on its own
            // is latency bound (one dependent chain of divisions per sample) and runs about three times slower per
            // voice: it is recorded here, with a regression bound, and listed as unmet in docs/MEASUREMENTS.md.
            for (const auto& c : models())
            {
                double mono = 1.0e9, quad = 1.0e9;
                const double target = c.model == FilterModel::CoupledDiodeLadder ? 0.18 : 0.12;
                for (int attempt = 0; attempt < 2 && quad > target * 1.2; ++attempt)
                {
                    mono = std::min(mono, cpuPercent(c, Quality::Standard, false));
                    quad = std::min(quad, cpuPercent(c, Quality::Standard, true));
                }
                logMessage(std::string(c.name) + ": one voice " + num(mono, 3) + " %, through processQuad " + num(quad, 3) +
                           " % per voice, plan target " + num(target, 2) + " %");
                expect(quad <= target * 1.2, std::string(c.name) + " processQuad " + num(quad, 3) + " % per voice");
                expect(mono <= 0.6, std::string(c.name) + " one voice " + num(mono, 3) + " %");
            }
#endif
        }

        beginTest("tolerance: seeded, deterministic, bounded, zero-mean; zero tolerance ignores the seed");
        {
            const auto& c = models()[0];
            std::vector<double> cents;
            for (std::uint32_t seed = 1; seed <= 48; ++seed)
            {
                const auto o = oscillate(c, Quality::Standard, 1000.0f, 1.6, 1.0f, seed);
                cents.push_back(m::centsBetween(o.freqHz, 1000.0));
            }
            double sum = 0.0, worst = 0.0;
            for (const double v : cents)
            {
                sum += v;
                worst = std::max(worst, std::abs(v));
            }
            expect(worst <= 100.0 && worst > 5.0, "largest deviation " + num(worst, 1) + " cents");
            const double mean = sum / static_cast<double>(cents.size());
            expect(std::abs(mean) < 15.0, "mean " + num(mean, 1) + " cents");

            const auto render = [&](std::uint32_t seed, float tolerance)
            {
                auto f = build(c, Quality::Standard);
                f->setSeed(seed);
                f->setControls(900.0f, 0.8f, 0.2f, 0.0f, tolerance);
                std::vector<float> x(4096);
                for (size_t i = 0; i < x.size(); ++i)
                    x[i] = 0.3f * static_cast<float>(std::sin(0.05 * static_cast<double>(i)));
                run(*f, x);
                return x;
            };
            expect(render(7, 1.0f) == render(7, 1.0f), "same seed, same output");
            expect(render(7, 1.0f) != render(8, 1.0f), "different seeds differ");
            expect(render(7, 0.0f) == render(8, 0.0f), "zero tolerance is independent of the seed");
        }

        beginTest("start-up noise: off, silence stays silent; on, the loop builds up");
        {
            for (const auto& c : models())
            {
                auto f = build(c, Quality::Standard);
                f->setControls(1000.0f, 1.0f, 0.0f, 0.0f, 0.0f);
                std::vector<float> x(48000, 0.0f);
                run(*f, x);
                float peak = 0.0f;
                for (const float v : x)
                    peak = std::max(peak, std::abs(v));
                expectEquals(peak, 0.0f, std::string(c.name) + " silence");
                expect(oscillate(c, Quality::Standard, 1000.0f, 2.5).levelDbfs > -40.0,
                       std::string(c.name) + " builds up with the noise on");
            }
        }

        beginTest("cutoff modulation in octaves equals the same cutoff set directly; latency is reported");
        {
            for (const auto& c : models())
            {
                auto a = build(c, Quality::Standard), b = build(c, Quality::Standard);
                a->setControls(500.0f, 0.5f, 0.0f, 0.0f, 0.0f);
                b->setControls(1000.0f, 0.5f, 0.0f, 0.0f, 0.0f);
                std::vector<float> x(8192), y;
                m::Random rng(9);
                for (auto& s : x)
                    s = static_cast<float>(rng.gaussian() * 0.2);
                y = x;
                const std::vector<float> mod(x.size(), 1.0f);
                run(*a, x, 128, &mod);
                run(*b, y);
                double worst = 0.0;
                for (size_t i = 4096; i < x.size(); ++i)
                    worst = std::max(worst, static_cast<double>(std::abs(x[i] - y[i])));
                expect(worst < 1.0e-3, std::string(c.name) + " modulation vs direct: " + num(worst));
                expect(build(c, Quality::Draft)->latency() > 0.0f, "Draft keeps the resampling chain for the drive shaper");
                expect(build(c, Quality::Standard)->latency() > 0.0f && build(c, Quality::Standard)->latency() < 12.0f);
                expect(build(c, Quality::High)->latency() < 24.0f);
            }
        }

        beginTest("the legacy FilterChannel runs the new models like the interface does");
        {
            for (const auto& c : models())
            {
                FilterChannel ch;
                ch.prepare(kFs, Quality::Standard);
                FilterSettings s;
                s.model = static_cast<int>(c.model);
                s.cutoffHz = 700.0f;
                s.resonance = 0.8f;
                s.drive = 0.4f;
                s.character = 0.3f;
                ch.configure(s, kFs);
                auto f = build(c, Quality::Standard, 1, 32);
                f->setControls(700.0f, 0.8f, 0.4f, 0.3f, 0.0f);
                std::vector<float> x(4096);
                for (size_t i = 0; i < x.size(); ++i)
                {
                    const auto t = static_cast<double>(i);
                    x[i] = 0.4f * static_cast<float>(std::sin(0.03 * t) + 0.3 * std::sin(0.31 * t));
                }
                auto ref = x;
                run(*f, ref, 1);
                double worst = 0.0;
                for (size_t i = 0; i < x.size(); ++i)
                    worst = std::max(worst, static_cast<double>(std::abs(ch.process(x[i]) - ref[i])));
                expect(worst < 1.0e-6, std::string(c.name) + " FilterChannel vs interface: " + num(worst));

                // Through the stereo unit: drive is the model's own, so the legacy input shaper is bypassed.
                FilterUnit unit;
                unit.prepare(kFs, Quality::Standard);
                unit.configure(s, 0.4f, kFs);
                float l = 0.3f, r = -0.3f;
                for (int i = 0; i < 64; ++i)
                {
                    l = 0.3f;
                    r = -0.3f;
                    unit.process(l, r);
                }
                expect(std::isfinite(l) && std::isfinite(r) && std::abs(l + r) < 1.0e-3f,
                       std::string(c.name) + " stereo unit");
            }
        }
    }
};

static AnalogFilterLadderTests analogFilterLadderTests;
