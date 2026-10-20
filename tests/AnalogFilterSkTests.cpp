// SPDX-License-Identifier: MIT
// The second filter family (P8): Sallen-Key low-pass / high-pass with diode feedback clipping, the OTA state-variable
// filter, the four-pole OTA with partial compensation and the passive high-pass, measured with the P1 harness against
// the rows of docs/plans/ENGINE-COMMUNITY-PLAN.md, sections 2.6, 2.7 and 3.1: self-oscillation tracking, harmonics of the
// Sallen-Key limit cycle, thresholds (the SVF never oscillates), bass loss, slopes, level dependence, nonlinear aliasing,
// 60 s of random modulation, the four-lane path, CPU, and the engine parameters `filterN_character` / `filterN_hp`.

#include "AllocCounter.h"
#include "AnalogMeasure.h"
#include "TestSupport.h"

#include <crucible/EngineCore.h>
#include <crucible/Filters.h>
#include <crucible/Simd.h>
#include <crucible/filters/FilterModel.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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

std::string num(double v, int decimals = 3)
{
    return String(v, decimals).str();
}

enum class Kind
{
    SallenKeyLp,
    SallenKeyHp,
    Svf,
    Ota4,
    PassiveHp
};

struct ModelCase
{
    const char* name;
    FilterModel model;
    Kind kind;
    std::unique_ptr<AnalogFilter> (*make)();
    double slopeDb; // expected slope with resonance 0: 2 to 4 octaves above the cutoff (low-pass), below it (high-pass)
    double slopeTolerance; // and its tolerance
    bool highpass;         // the slope is measured below the cutoff
    double cpuTarget;      // plan target in percent of one core per voice at Standard (processQuad)
};

const std::array<ModelCase, 5>& models()
{
    static const std::array<ModelCase, 5> cases = {{
        {"Sallen-Key LP", FilterModel::SallenKeyLowpass, Kind::SallenKeyLp, makeSallenKeyLowpass, -12.0, 1.0, false, 0.08},
        {"Sallen-Key HP", FilterModel::SallenKeyHighpass, Kind::SallenKeyHp, makeSallenKeyHighpass, 12.0, 1.0, true, 0.08},
        {"OTA SVF", FilterModel::SvfOta, Kind::Svf, makeSvfOta, -12.0, 1.0, false, 0.08},
        {"OTA 4-pole", FilterModel::Ota4Pole, Kind::Ota4, makeOta4Pole, -24.0, 1.5, false, 0.12},
        {"Passive HP", FilterModel::PassiveHighpass, Kind::PassiveHp, makePassiveHp, 6.0, 0.5, true, 0.08},
    }};
    return cases;
}

const ModelCase& modelCase(Kind k)
{
    for (const auto& c : models())
        if (c.kind == k)
            return c;
    return models()[0];
}

std::unique_ptr<AnalogFilter> build(const ModelCase& c, Quality q, int lanes = 1, int maxBlock = 128)
{
    auto f = c.make();
    f->prepare(kFs, q, lanes, maxBlock);
    return f;
}

void run(AnalogFilter& f, std::vector<float>& x, int block = 128, const std::vector<float>* mod = nullptr)
{
    for (size_t off = 0; off < x.size(); off += static_cast<size_t>(block))
    {
        const int n = static_cast<int>(std::min<size_t>(static_cast<size_t>(block), x.size() - off));
        f.process(x.data() + off, n, mod != nullptr ? mod->data() + off : nullptr);
    }
}

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

// The resonance setting whose loop quantity is k (feedbackForResonance is monotonic).
float resonanceForK(const AnalogFilter& f, double k)
{
    float lo = 0.0f, hi = 1.0f;
    for (int i = 0; i < 40; ++i)
    {
        const float mid = 0.5f * (lo + hi);
        if (f.feedbackForResonance(mid) < k)
            lo = mid;
        else
            hi = mid;
    }
    return 0.5f * (lo + hi);
}

struct Oscillation
{
    double freqHz = 0.0;
    double levelDbfs = -300.0;
    double variationDb = 0.0;
    m::HarmonicProfile harmonics;
};

// Full resonance on silence with the start-up noise on; the last 40 % is analysed.
Oscillation oscillate(const ModelCase& c, Quality q, float fc, double seconds, float tolerance = 0.0f,
                      std::uint32_t seed = 1, float res = 1.0f, float character = 0.0f)
{
    auto f = build(c, q);
    f->setSeed(seed);
    f->setControls(fc, res, 0.0f, character, tolerance);
    f->setStartupNoise(true);
    std::vector<float> x(static_cast<size_t>(seconds * kFs), 0.0f);
    x[0] = 1.0e-3f;
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
    {
        o.freqHz = m::estimateFrequency(tail, kFs, fc);
        o.harmonics = m::harmonicProfile(tail.data(), tail.size(), kFs, o.freqHz, 8);
    }
    return o;
}

// Level change of a build-up / ring-down between two windows after a small kick, in dB.
double growthDb(const ModelCase& c, double k, double kick, double t1, double t2)
{
    auto f = build(c, Quality::Standard);
    f->setControls(1000.0f, resonanceForK(*f, k), 0.0f, 0.0f, 0.0f);
    std::vector<float> x(static_cast<size_t>((t2 + 0.1) * kFs), 0.0f);
    x[0] = static_cast<float>(kick);
    run(*f, x);
    const auto rms = [&](double t0)
    {
        const auto a = static_cast<size_t>(t0 * kFs);
        return m::rmsDbfs(m::toDouble(x, a, static_cast<size_t>(0.05 * kFs)));
    };
    return rms(t2) - rms(t1);
}

double gainDb(const ModelCase& c, Quality q, float fc, float res, float character, double hz, double amplitude = 0.01,
              int hpStep = 0)
{
    const auto make = [&]
    {
        auto f = std::shared_ptr<AnalogFilter>(build(c, q));
        f->setControls(fc, res, 0.0f, character, 0.0f);
        f->setHighpassStep(hpStep);
        return Sample{f};
    };
    return m::steppedGainDb(make, hz, kFs, amplitude, 0.4, 0.2);
}

double peakGainDb(const ModelCase& c, double levelDbfs, float res)
{
    double best = -300.0;
    for (double hz = 640.0; hz <= 1250.0; hz *= 1.07)
        best = std::max(best, gainDb(c, Quality::Standard, 1000.0f, res, 0.0f, hz, std::pow(10.0, levelDbfs / 20.0)));
    return best;
}

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

struct StressPeaks
{
    float output = 0.0f;
    float input = 0.0f;
    bool finite = true;
};

StressPeaks stress(const ModelCase& c, Quality q, double seconds, bool quad, std::uint64_t seed)
{
    m::Random rng(seed);
    const int lanes = quad ? 4 : 1;
    auto f = build(c, q, lanes, 128);
    std::vector<std::vector<float>> io(4, std::vector<float>(128)), mod(4, std::vector<float>(128));
    StressPeaks peaks;
    const int blocks = static_cast<int>(seconds * kFs / 128.0);
    for (int b = 0; b < blocks; ++b)
    {
        if (b % 64 == 0)
            f->setHighpassStep(static_cast<int>(rng.uniform() * 3.99));
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
        for (int l = 0; l < lanes; ++l)
            for (const float v : io[static_cast<size_t>(l)])
                peaks.input = std::max(peaks.input, std::abs(v));
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
                {
                    peaks.finite = false;
                    return peaks;
                }
                peaks.output = std::max(peaks.output, std::abs(v));
            }
    }
    return peaks;
}

// Percent of one core per voice for 128-sample blocks of noise at 48 kHz, the best of three runs.
double cpuPercent(const ModelCase& c, Quality q, bool quad, float drive = 0.2f)
{
    const int blocks = static_cast<int>(3.0 * kFs / 128.0);
    double best = 1.0e9;
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        auto f = build(c, q, quad ? 4 : 1, 128);
        for (int l = 0; l < (quad ? 4 : 1); ++l)
            f->setLaneControls(l, 2000.0f, 0.7f, drive, 0.0f, 0.0f);
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

double rmsAfter(const std::vector<float>& x, double fromSeconds)
{
    return m::rmsDbfs(m::toDouble(x, static_cast<size_t>(fromSeconds * kFs)));
}

struct EngineLevels
{
    double totalDb = -300.0;
    double bassDb = -300.0; // 20 to 150 Hz
};

// A held note through the engine's first filter; the last 2^14 samples of the left channel are analysed.
EngineLevels engineNote(int type, float res, float character, int hpStep, int note, float drive)
{
    CRUCIBLE_HEAP(EngineCore, core);
    core.prepare(kRate);
    auto v = ParameterMap::defaultValues();
    setValue(v, "filter_on", 1.0f);
    setValue(v, "filter_type", static_cast<float>(type));
    setValue(v, "filter_cutoff", 12000.0f);
    setValue(v, "filter_res", res);
    setValue(v, "filter_drive", drive);
    setValue(v, "filter_character", character);
    setValue(v, "filter_hp", static_cast<float>(hpStep));
    std::vector<float> l(256), r(256), all;
    MidiInput midi[1] = {{0, 0x90, static_cast<std::uint8_t>(note), 110}};
    core.process(l.data(), r.data(), 256, v.data(), midi, 1, {});
    for (int i = 0; i < 120; ++i)
    {
        core.process(l.data(), r.data(), 256, v.data(), nullptr, 0, {});
        all.insert(all.end(), l.begin(), l.end());
    }
    const auto sig = m::toDouble(all, all.size() - (1u << 14));
    EngineLevels e;
    e.totalDb = m::rmsDbfs(sig);
    e.bassDb = m::bandRmsDbfs(sig.data(), sig.size(), kRate, 20.0, 150.0);
    return e;
}
} // namespace

class AnalogFilterSkTests : public UnitTest
{
public:
    AnalogFilterSkTests() : UnitTest("AnalogFilterSk", "Filters") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        const auto& sklp = modelCase(Kind::SallenKeyLp);
        const auto& skhp = modelCase(Kind::SallenKeyHp);
        const auto& svf = modelCase(Kind::Svf);
        const auto& ota4 = modelCase(Kind::Ota4);
        const auto& passive = modelCase(Kind::PassiveHp);

        beginTest("the new models are appended after the first family: indices 20 to 24, legacy 0 to 19 unchanged");
        {
            expectEquals(static_cast<int>(FilterModel::CoupledDiodeLadder), 19);
            expectEquals(static_cast<int>(FilterModel::SallenKeyLowpass), 20);
            expectEquals(static_cast<int>(FilterModel::SallenKeyHighpass), 21);
            expectEquals(static_cast<int>(FilterModel::SvfOta), 22);
            expectEquals(static_cast<int>(FilterModel::Ota4Pole), 23);
            expectEquals(static_cast<int>(FilterModel::PassiveHighpass), 24);
            expectEquals(static_cast<int>(FilterModel::Count), 25);
            const auto& names = filterModelNames();
            expectEquals(static_cast<int>(names.size()), 25);
            expectEquals(names[19], std::string("Coupled Diode Ladder"));
            expectEquals(names[20], std::string("Sallen-Key LP"));
            expectEquals(names[24], std::string("Passive HP"));
            expect(FilterChannel::isAnalogModel(20) && FilterChannel::isAnalogModel(24) &&
                   !FilterChannel::isAnalogModel(16));
        }

        beginTest("self-oscillation tracks the cutoff: 5 cents at Standard and High, 15 cents at Draft, steady amplitude");
        {
            for (const auto* c : {&sklp, &skhp, &ota4})
                for (const float fc : {40.0f, 100.0f, 250.0f, 1000.0f, 4000.0f, 8000.0f})
                {
                    const double seconds = fc < 60.0f ? 8.0 : (fc < 300.0f ? 4.0 : 2.5);
                    for (const Quality q : {Quality::Standard, Quality::High})
                    {
                        if (q == Quality::High && fc < 100.0f)
                            continue;
                        const auto o = oscillate(*c, q, fc, seconds);
                        const double cents = m::centsBetween(o.freqHz, fc);
                        expect(o.levelDbfs > -40.0 && std::abs(cents) <= 5.0 && o.variationDb < 3.0,
                               std::string(c->name) + (q == Quality::High ? " High " : " Standard ") + num(fc) +
                                   " Hz: " + num(cents, 2) + " cents, " + num(o.levelDbfs, 1) + " dBFS, steadiness " +
                                   num(o.variationDb, 2) + " dB");
                    }
                    if (fc >= 100.0f)
                    {
                        const auto d = oscillate(*c, Quality::Draft, fc, seconds);
                        const double cents = m::centsBetween(d.freqHz, fc);
                        expect(d.levelDbfs > -40.0 && std::abs(cents) <= 15.0 && d.variationDb < 3.0,
                               std::string(c->name) + " Draft " + num(fc) + " Hz: " + num(cents, 2) + " cents");
                    }
                }
            // The clipper hardness (character) moves the frequency of the Sallen-Key limit cycle: the table is measured
            // at both ends of the dial and interpolated, so the whole dial tracks.
            for (const auto* c : {&sklp, &skhp})
                for (const float chr : {0.25f, 0.5f, 1.0f})
                {
                    const auto o = oscillate(*c, Quality::Standard, 1000.0f, 2.0, 0.0f, 1, 1.0f, chr);
                    expect(std::abs(m::centsBetween(o.freqHz, 1000.0)) <= 5.0 && o.levelDbfs > -40.0,
                           std::string(c->name) + " character " + num(chr) + ": " +
                               num(m::centsBetween(o.freqHz, 1000.0), 2) + " cents");
                }
        }

        beginTest("Sallen-Key self-oscillation carries even and odd harmonics (asymmetric diode clipper); the OTA 4-pole is "
                  "odd only");
        {
            for (const auto* c : {&sklp, &skhp})
                for (const Quality q : {Quality::Standard, Quality::High})
                {
                    const auto o = oscillate(*c, q, 1000.0f, 2.0);
                    const double h2 = o.harmonics.harmonicDb(2), h3 = o.harmonics.harmonicDb(3);
                    logMessage(std::string(c->name) + (q == Quality::High ? " High" : " Standard") + ": " +
                               num(o.levelDbfs, 1) + " dBFS, H2 " + num(h2, 1) + " dB, H3 " + num(h3, 1) + " dB, H4 " +
                               num(o.harmonics.harmonicDb(4), 1) + " dB, even " + num(o.harmonics.evenDb, 1) + " dB, odd " +
                               num(o.harmonics.oddDb, 1) + " dB");
                    expect(h2 > -45.0 && h2 < -10.0 && h3 > -45.0 && h3 < -10.0,
                           std::string(c->name) + " H2 " + num(h2, 1) + " dB, H3 " + num(h3, 1) + " dB");
                    expect(o.harmonics.harmonicDb(4) > -80.0,
                           std::string(c->name) + " H4 " + num(o.harmonics.harmonicDb(4), 1));
                }
            const auto o = oscillate(ota4, Quality::Standard, 1000.0f, 2.0);
            expect(o.harmonics.harmonicDb(2) < -80.0 && o.harmonics.harmonicDb(3) > -70.0,
                   "OTA 4-pole: H2 " + num(o.harmonics.harmonicDb(2), 1) + ", H3 " + num(o.harmonics.harmonicDb(3), 1));
        }

        beginTest("self-oscillation threshold: Sallen-Key at Ka = 3, OTA 4-pole at k = 4.0 to 4.3, the SVF never builds up");
        {
            struct Band
            {
                const ModelCase* c;
                double step, band, kick, t1, t2;
            };
            for (const Band b : {Band{&sklp, 0.01, 0.06, 1.0e-7, 0.05, 0.12}, Band{&skhp, 0.01, 0.06, 1.0e-7, 0.05, 0.12},
                                 Band{&ota4, 0.05, 0.3, 1.0e-4, 0.10, 0.40}})
            {
                const double kc = build(*b.c, Quality::Standard)->selfOscillationGain();
                const double kMax = build(*b.c, Quality::Standard)->feedbackForResonance(1.0f);
                double onset = 0.0;
                bool grewBelow = false;
                for (double k = kc - 6.0 * b.step; k <= std::min(kc + 12.0 * b.step, kMax); k += b.step)
                {
                    const double g = growthDb(*b.c, k, b.kick, b.t1, b.t2);
                    if (k <= kc - 0.5 * b.step && g > 0.3)
                        grewBelow = true;
                    if (onset == 0.0 && g > 0.3)
                        onset = k;
                }
                expect(!grewBelow, std::string(b.c->name) + " builds up below its linear threshold");
                expect(onset >= kc - 1.0e-6 && onset <= kc + b.band,
                       std::string(b.c->name) + " onset k = " + num(onset, 3) + ", linear threshold " + num(kc, 2));
            }
            // The SVF: Q is capped at 20, so every resonance setting rings down, with and without the start-up noise.
            expect(!std::isfinite(build(svf, Quality::Standard)->selfOscillationGain()), "the SVF reports no threshold");
            for (const float fc : {40.0f, 200.0f, 1000.0f, 5000.0f, 12000.0f})
                for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
                {
                    auto f = build(svf, q);
                    f->setControls(fc, 1.0f, 0.0f, 0.0f, 0.0f);
                    f->setStartupNoise(true);
                    std::vector<float> x(static_cast<size_t>(3.0 * kFs), 0.0f);
                    x[0] = 0.05f;
                    run(*f, x);
                    const double early = m::rmsDbfs(m::toDouble(x, 0, static_cast<size_t>(0.005 * kFs)));
                    const double late = rmsAfter(x, 2.0);
                    expect(late < -90.0 && late < early - 30.0, std::string("SVF ") + num(fc) + " Hz tier " +
                                                                    num(static_cast<int>(q), 0) + ": early " +
                                                                    num(early, 1) + " dBFS, late " + num(late, 1) + " dBFS");
                }
        }

        beginTest("bass loss: the SVF and Sallen-Key low-pass keep unity gain at any resonance; the 4-pole sits between "
                  "ladder and cascade");
        {
            for (const auto* c : {&sklp, &svf})
                for (const float res : {0.0f, 0.25f, 0.5f, 0.75f, 0.95f})
                    for (const float chr : {0.0f, 1.0f})
                    {
                        if (c->kind == Kind::Svf && chr > 0.0f)
                            continue; // character is the morph of the SVF
                        const double expected = 20.0 * std::log10(build(*c, Quality::Standard)->dcGain(res, chr));
                        expectWithinAbsoluteError(gainDb(*c, Quality::Standard, 4000.0f, res, chr, 25.0), expected, 0.5,
                                                  std::string(c->name) + " res " + num(res) + " character " + num(chr));
                        expectWithinAbsoluteError(expected, 0.0, 0.01, std::string(c->name) + " dcGain");
                    }
            // High-pass counterpart: no loss in the pass band, 16 times the cutoff.
            for (const float res : {0.0f, 0.5f, 0.9f})
                expectWithinAbsoluteError(gainDb(skhp, Quality::Standard, 500.0f, res, 0.0f, 8000.0), 0.0, 0.5,
                                          "Sallen-Key high-pass pass band, res " + num(res));
            // OTA 4-pole: its own law, within 0.5 dB, and compensated with character 1.
            for (const float res : {0.0f, 0.25f, 0.5f, 0.75f, 0.95f})
            {
                auto probe = build(ota4, Quality::Standard);
                expectWithinAbsoluteError(gainDb(ota4, Quality::Standard, 4000.0f, res, 0.0f, 25.0),
                                          20.0 * std::log10(probe->dcGain(res, 0.0f)), 0.5, "OTA 4-pole res " + num(res));
                expectWithinAbsoluteError(gainDb(ota4, Quality::Standard, 4000.0f, res, 1.0f, 25.0), 0.0, 0.5,
                                          "OTA 4-pole compensated, res " + num(res));
            }
            const double between = gainDb(ota4, Quality::Standard, 4000.0f, 0.9f, 0.0f, 25.0);
            const auto measureOther = [&](std::unique_ptr<AnalogFilter> f)
            {
                f->prepare(kFs, Quality::Standard, 1, 128);
                f->setControls(4000.0f, 0.9f, 0.0f, 0.0f, 0.0f);
                std::shared_ptr<AnalogFilter> shared = std::move(f);
                return m::steppedGainDb([&] { return Sample{shared}; }, 25.0, kFs, 0.01, 0.4, 0.2);
            };
            const double lad = measureOther(makeTransistorLadder()), cas = measureOther(makeOtaCascade());
            expect(lad + 2.0 < between && between + 1.0 < cas, "ladder " + num(lad, 2) + " dB < OTA 4-pole " +
                                                                   num(between, 2) + " dB < OTA cascade " + num(cas, 2) +
                                                                   " dB");
        }

        beginTest("slopes: 12 +-1 dB/oct (Sallen-Key, SVF, both ways), 24 +-1.5 (OTA 4-pole), passive high-pass 6 and 12 "
                  "within 0.5");
        {
            for (const auto& c : models())
            {
                if (c.kind == Kind::PassiveHp)
                    continue;
                for (const Quality q : {Quality::Standard, Quality::High})
                {
                    const auto grid = c.highpass ? m::logSpacedFrequencies(250.0, 1000.0, 12)
                                                 : m::logSpacedFrequencies(1000.0, 4000.0, 12);
                    const float fc = c.highpass ? 4000.0f : 250.0f;
                    const auto make = [&]
                    {
                        auto f = std::shared_ptr<AnalogFilter>(build(c, q));
                        f->setControls(fc, 0.0f, 0.0f, 0.0f, 0.0f);
                        return Sample{f};
                    };
                    const auto curve = m::steppedResponse(make, grid, kFs, 0.01, 0.4, 0.2);
                    const double slope =
                        m::slopeDbPerOctave(curve, c.highpass ? 250.0 : 1000.0, c.highpass ? 1000.0 : 4000.0);
                    expectWithinAbsoluteError(slope, c.slopeDb, c.slopeTolerance,
                                              std::string(c.name) + (q == Quality::High ? " High" : " Standard"));
                }
            }
            // The SVF morphed to the high-pass.
            for (const Quality q : {Quality::Standard, Quality::High})
            {
                const auto grid = m::logSpacedFrequencies(250.0, 1000.0, 12);
                const auto make = [&]
                {
                    auto f = std::shared_ptr<AnalogFilter>(build(svf, q));
                    f->setControls(4000.0f, 0.0f, 0.0f, 1.0f, 0.0f);
                    return Sample{f};
                };
                const double slope = m::slopeDbPerOctave(m::steppedResponse(make, grid, kFs, 0.01, 0.4, 0.2), 250.0, 1000.0);
                expectWithinAbsoluteError(slope, 12.0, 1.0, "SVF morphed to the high-pass");
            }
            // Passive high-pass: one section (6 dB/oct) and two (12 dB/oct), well inside 0.5 dB.
            for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
                for (const float res : {0.0f, 1.0f})
                {
                    const auto grid = m::logSpacedFrequencies(250.0, 1000.0, 12);
                    const auto make = [&]
                    {
                        auto f = std::shared_ptr<AnalogFilter>(build(passive, q));
                        f->setControls(4000.0f, res, 0.0f, 0.0f, 0.0f);
                        return Sample{f};
                    };
                    const double slope =
                        m::slopeDbPerOctave(m::steppedResponse(make, grid, kFs, 0.01, 0.4, 0.2), 250.0, 1000.0);
                    const double want = res > 0.5f ? 12.0 : 6.0;
                    logMessage(std::string("passive high-pass tier ") + num(static_cast<int>(q), 0) + " resonance " +
                               num(res, 0) + ": " + num(slope, 3) + " dB/oct");
                    expectWithinAbsoluteError(slope, want, 0.5, "passive high-pass slope, resonance " + num(res, 0));
                }
        }

        beginTest("passive high-pass: -3.01 dB at the corner with one section, -6.02 dB with two, flat above, no resonance");
        {
            for (const float fc : {200.0f, 1000.0f, 6000.0f})
            {
                expectWithinAbsoluteError(gainDb(passive, Quality::Standard, fc, 0.0f, 0.0f, fc), -3.01, 0.3,
                                          "one section at " + num(fc));
                expectWithinAbsoluteError(gainDb(passive, Quality::Standard, fc, 1.0f, 0.0f, fc), -6.02, 0.3,
                                          "two sections at " + num(fc));
                if (8.0 * fc <= 12000.0)
                    expectWithinAbsoluteError(gainDb(passive, Quality::Standard, fc, 1.0f, 0.0f, 8.0 * fc), 0.0, 0.4,
                                              "pass band at " + num(fc));
            }
            double worst = -300.0;
            for (double hz = 100.0; hz < 15000.0; hz *= 1.3)
                worst = std::max(worst, gainDb(passive, Quality::Standard, 1500.0f, 1.0f, 0.0f, hz));
            expect(worst < 0.1, "no peak, highest gain " + num(worst, 3) + " dB");
            auto probe = build(passive, Quality::Standard);
            expect(!std::isfinite(probe->selfOscillationGain()) && probe->dcGain(1.0f, 0.0f) == 0.0f,
                   "reports no oscillation and zero DC gain");
        }

        beginTest("the OTA SVF morphs low-pass (0), notch (0.5), high-pass (1) with character; the notch cuts the centre by "
                  "40 dB");
        {
            const double lp = gainDb(svf, Quality::Standard, 1000.0f, 0.3f, 0.0f, 250.0);
            const double hpLow = gainDb(svf, Quality::Standard, 1000.0f, 0.3f, 1.0f, 250.0);
            expect(lp > -1.0 && hpLow < -20.0,
                   "low-pass keeps 250 Hz (" + num(lp, 2) + " dB), high-pass rejects it (" + num(hpLow, 2) + " dB)");
            const double notchAt = gainDb(svf, Quality::Standard, 1000.0f, 0.3f, 0.5f, 1000.0);
            const double notchBelow = gainDb(svf, Quality::Standard, 1000.0f, 0.3f, 0.5f, 200.0);
            const double notchAbove = gainDb(svf, Quality::Standard, 1000.0f, 0.3f, 0.5f, 8000.0);
            expect(notchAt < -40.0 && std::abs(notchBelow) < 1.0 && std::abs(notchAbove) < 1.0,
                   "notch: centre " + num(notchAt, 1) + " dB, 200 Hz " + num(notchBelow, 2) + " dB, 8 kHz " +
                       num(notchAbove, 2) + " dB");
            auto probe = build(svf, Quality::Standard);
            expectWithinAbsoluteError(static_cast<double>(probe->dcGain(0.3f, 0.5f)), 1.0, 1.0e-6, "dcGain at the notch");
            expectWithinAbsoluteError(static_cast<double>(probe->dcGain(0.3f, 1.0f)), 0.0, 1.0e-6,
                                      "dcGain at the high-pass");
            const double peakLow = gainDb(svf, Quality::Standard, 1000.0f, 0.2f, 0.0f, 1000.0);
            const double peakHigh = gainDb(svf, Quality::Standard, 1000.0f, 0.8f, 0.0f, 1000.0);
            expect(peakHigh > peakLow + 8.0,
                   "peak at resonance 0.8 is " + num(peakHigh - peakLow, 1) + " dB above resonance 0.2");
        }

        beginTest("the passive high-pass step ahead of the model: 60 / 180 / 540 Hz, -3 dB at the corner, one pole; models "
                  "without it ignore it");
        {
            const double corner[4] = {0.0, 60.0, 180.0, 540.0};
            for (const auto* c : {&sklp, &svf, &ota4})
            {
                expectWithinAbsoluteError(gainDb(*c, Quality::Standard, 8000.0f, 0.0f, 0.0f, 100.0, 0.01, 0), 0.0, 0.3,
                                          std::string(c->name) + " off");
                for (int step = 1; step <= 3; ++step)
                {
                    expectWithinAbsoluteError(gainDb(*c, Quality::Standard, 8000.0f, 0.0f, 0.0f, corner[step], 0.01, step),
                                              -3.01, 0.5, std::string(c->name) + " corner of step " + num(step, 0));
                    expectWithinAbsoluteError(
                        gainDb(*c, Quality::Standard, 8000.0f, 0.0f, 0.0f, corner[step] / 8.0, 0.01, step), -18.1, 1.0,
                        std::string(c->name) + " one pole below the corner, step " + num(step, 0));
                }
            }
            expectWithinAbsoluteError(gainDb(skhp, Quality::Standard, 8000.0f, 0.0f, 0.0f, 100.0, 0.01, 3),
                                      gainDb(skhp, Quality::Standard, 8000.0f, 0.0f, 0.0f, 100.0, 0.01, 0), 0.01,
                                      "Sallen-Key high-pass");
            expectWithinAbsoluteError(gainDb(passive, Quality::Standard, 8000.0f, 0.0f, 0.0f, 100.0, 0.01, 3),
                                      gainDb(passive, Quality::Standard, 8000.0f, 0.0f, 0.0f, 100.0, 0.01, 0), 0.01,
                                      "passive high-pass");
            // Switching the step during playback does not click: the output stays continuous.
            auto f = build(sklp, Quality::Standard);
            f->setControls(2000.0f, 0.3f, 0.0f, 0.0f, 0.0f);
            std::vector<float> x(48000);
            for (size_t i = 0; i < x.size(); ++i)
                x[i] = 0.3f * static_cast<float>(std::sin(0.02 * static_cast<double>(i)));
            float maxStep = 0.0f, last = 0.0f;
            for (size_t off = 0; off < x.size(); off += 128)
            {
                f->setHighpassStep(static_cast<int>((off / 128) % 4));
                f->process(x.data() + off, 128);
                for (size_t i = off; i < off + 128; ++i)
                {
                    maxStep = std::max(maxStep, std::abs(x[i] - last));
                    last = x[i];
                }
            }
            expect(maxStep < 0.1f, "largest sample step while switching the high-pass " + num(maxStep, 3));
        }

        beginTest("level dependence: the resonant peak compresses monotonically from -40 to +6 dBFS");
        {
            struct Level
            {
                const ModelCase* c;
                float res;
            };
            for (const Level l : {Level{&sklp, 0.85f}, Level{&skhp, 0.85f}, Level{&svf, 0.9f}, Level{&ota4, 0.9f}})
            {
                const double a = peakGainDb(*l.c, -40.0, l.res), b = peakGainDb(*l.c, -12.0, l.res),
                             d = peakGainDb(*l.c, 0.0, l.res), e = peakGainDb(*l.c, 6.0, l.res);
                expect(a > b + 0.3 && b > d + 0.3 && d > e + 0.3, std::string(l.c->name) + ": " + num(a, 2) + ", " +
                                                                      num(b, 2) + ", " + num(d, 2) + ", " + num(e, 2) +
                                                                      " dB");
                expect(a - e > 6.0, std::string(l.c->name) + " total compression " + num(a - e, 2) + " dB");
            }
            // The passive high-pass is linear.
            expectWithinAbsoluteError(gainDb(passive, Quality::Standard, 1000.0f, 1.0f, 0.0f, 700.0, 0.01),
                                      gainDb(passive, Quality::Standard, 1000.0f, 1.0f, 0.0f, 700.0, 2.0), 0.05,
                                      "passive high-pass level independence");
        }

        beginTest("nonlinear aliasing at 5 kHz: -90 dB Standard, -100 dB High through the default settings (0 and -6 dBFS), "
                  "full drive reported");
        {
            for (const auto& c : models())
            {
                const double plain6 = aliasing(c, Quality::Standard, 0.5f, 0.0f);
                const double plain0 = aliasing(c, Quality::Standard, 1.0f, 0.0f);
                const double plainHigh6 = aliasing(c, Quality::High, 0.5f, 0.0f);
                const double plainHigh0 = aliasing(c, Quality::High, 1.0f, 0.0f);
                const double drive6 = aliasing(c, Quality::Standard, 0.5f, 1.0f);
                const double drive0 = aliasing(c, Quality::Standard, 1.0f, 1.0f);
                const double driveHigh0 = aliasing(c, Quality::High, 1.0f, 1.0f);
                logMessage(std::string(c.name) + ": default settings Standard " + num(plain6, 1) + " dB at -6 dBFS, " +
                           num(plain0, 1) + " dB at 0 dBFS; High " + num(plainHigh6, 1) + " / " + num(plainHigh0, 1) +
                           " dB; full drive Standard " + num(drive6, 1) + " dB at -6 dBFS, " + num(drive0, 1) +
                           " dB at 0 dBFS; High 0 dBFS " + num(driveHigh0, 1) + " dB");
                expect(plain6 <= -90.0 && plain0 <= -90.0,
                       std::string(c.name) + " Standard default: " + num(plain6, 1) + " / " + num(plain0, 1) + " dB");
                expect(plainHigh6 <= -100.0 && plainHigh0 <= -100.0,
                       std::string(c.name) + " High default: " + num(plainHigh6, 1) + " / " + num(plainHigh0, 1) + " dB");
                // A high-pass keeps the harmonics of the hard-driven input that a low-pass removes before they can fold, so
                // at full drive its floor is the shaper's own: recorded (docs/MEASUREMENTS.md), not asserted.
                if (!c.highpass)
                {
                    expect(drive6 <= -90.0,
                           std::string(c.name) + " Standard full drive at -6 dBFS: " + num(drive6, 1) + " dB");
                    expect(driveHigh0 <= -100.0,
                           std::string(c.name) + " High full drive at 0 dBFS: " + num(driveHigh0, 1) + " dB");
                }
            }
        }

        beginTest("60 s of random cutoff, resonance, drive, character, tolerance, high-pass and audio activity stays finite "
                  "and bounded");
        {
            const double seconds = CRUCIBLE_TEST_SANITIZED ? 6.0 : 60.0;
            for (const auto& c : models())
            {
                // Finite, and bounded: 100 for the models whose resonance is limited; for the SVF 200 times the largest
                // input (its Q of 20, the drive stage's small-signal gain of 2.45 and a factor 4 for the swept cutoff: a
                // transient of the time-varying resonator, while steady signals compress, see the level dependence row).
                const auto bound = [&](const StressPeaks& p)
                { return c.kind == Kind::Svf ? std::max(100.0f, 200.0f * p.input) : 100.0f; };
                for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
                {
                    const auto p = stress(c, q, q == Quality::High ? seconds * 0.5 : seconds, false, 21);
                    expect(p.finite && p.output < bound(p), std::string(c.name) + " tier " + num(static_cast<int>(q), 0) +
                                                                " peak " + num(p.output) + " for input peak " +
                                                                num(p.input));
                    if (c.kind == Kind::Svf)
                        logMessage(std::string("SVF tier ") + num(static_cast<int>(q), 0) + ": output peak " +
                                   num(p.output, 1) + " for input peak " + num(p.input, 1));
                }
                const auto p = stress(c, Quality::Standard, seconds, true, 22);
                expect(p.finite && p.output < bound(p),
                       std::string(c.name) + " four lanes, peak " + num(p.output) + " for input peak " + num(p.input));
            }
        }

        beginTest(
            "processQuad equals four mono filters with per-lane controls; the output does not depend on the block size");
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

                const auto render = [&](int block)
                {
                    auto f = build(c, Quality::Standard, 1, 128);
                    f->setControls(900.0f, 0.8f, 0.3f, 0.4f, 0.5f);
                    std::vector<float> x(4096);
                    for (size_t i = 0; i < x.size(); ++i)
                        x[i] = 0.4f * static_cast<float>(std::sin(0.03 * static_cast<double>(i)) +
                                                         0.5 * std::sin(0.4 * static_cast<double>(i)));
                    run(*f, x, block);
                    return x;
                };
                const auto whole = render(128);
                double blockWorst = 0.0;
                for (const int block : {1, 7})
                {
                    const auto cut = render(block);
                    for (size_t i = 0; i < whole.size(); ++i)
                        blockWorst = std::max(blockWorst, static_cast<double>(std::abs(whole[i] - cut[i])));
                }
                expect(blockWorst < 1.0e-5, std::string(c.name) + " block-size dependence " + num(blockWorst));
            }
        }

        beginTest("processQuad is at least 2.5 times faster than four scalar calls (the nonlinear models)");
        {
#if defined(NDEBUG) && !CRUCIBLE_TEST_SANITIZED && CRUCIBLE_SIMD_NATIVE
            for (const auto& c : models())
            {
                if (c.kind == Kind::PassiveHp)
                    continue; // linear: the resampling chain dominates and scales less
                double speedup = 0.0;
                for (int attempt = 0; attempt < 3 && speedup < 2.5; ++attempt)
                {
                    const double mono = cpuPercent(c, Quality::Standard, false);
                    const double quad = cpuPercent(c, Quality::Standard, true);
                    speedup = mono / quad;
                }
                expectCpu(speedup >= 2.5, std::string(c.name) + " speed-up " + num(speedup, 2));
            }
#endif
        }

        beginTest("CPU per voice at Standard (48 kHz, 128-sample blocks): processQuad within 20 % of the plan's target");
        {
#if defined(NDEBUG) && !CRUCIBLE_TEST_SANITIZED && CRUCIBLE_SIMD_NEON
            // Plan targets (section 2.6): 2-pole models 0.08 %, ladder-class (the 4-pole OTA) 0.12 %; the bench fails
            // at target + 20 %. The four-lane path is the one that meets them; one voice alone is latency bound (a
            // chain of dependent divisions per oversampled sample, plus the resampling chain, 0.06 % by itself) and is
            // recorded with a regression bound; see docs/MEASUREMENTS.md section 11.
            for (const auto& c : models())
            {
                double mono = 1.0e9, quad = 1.0e9;
                for (int attempt = 0; attempt < 3 && quad > c.cpuTarget * 1.2; ++attempt)
                {
                    mono = std::min(mono, cpuPercent(c, Quality::Standard, false));
                    quad = std::min(quad, cpuPercent(c, Quality::Standard, true));
                }
                const double mono0 = cpuPercent(c, Quality::Standard, false, 0.0f);
                const double quad0 = cpuPercent(c, Quality::Standard, true, 0.0f);
                logMessage(std::string(c.name) + ": one voice " + num(mono, 3) + " % (drive 0: " + num(mono0, 3) +
                           "), processQuad " + num(quad, 3) + " % per voice (drive 0: " + num(quad0, 3) + "), plan target " +
                           num(c.cpuTarget, 2) + " %");
                expectCpu(quad <= c.cpuTarget * 1.2, std::string(c.name) + " processQuad " + num(quad, 3) + " % per voice");
                expectCpu(mono <= 0.6, std::string(c.name) + " one voice " + num(mono, 3) + " %");
            }
#endif
        }

        beginTest("tolerance: seeded, deterministic, bounded, zero-mean; zero tolerance ignores the seed");
        {
            std::vector<double> cents;
            for (std::uint32_t seed = 1; seed <= 32; ++seed)
            {
                const auto o = oscillate(sklp, Quality::Standard, 1000.0f, 1.2, 1.0f, seed);
                cents.push_back(m::centsBetween(o.freqHz, 1000.0));
            }
            double sum = 0.0, worst = 0.0;
            for (const double v : cents)
            {
                sum += v;
                worst = std::max(worst, std::abs(v));
            }
            const double mean = sum / static_cast<double>(cents.size());
            expect(worst <= 100.0 && worst > 5.0, "largest deviation " + num(worst, 1) + " cents");
            expect(std::abs(mean) < 20.0, "mean " + num(mean, 1) + " cents");
            for (const auto& c : models())
            {
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
                expect(render(7, 1.0f) == render(7, 1.0f), std::string(c.name) + " same seed, same output");
                expect(render(7, 1.0f) != render(8, 1.0f), std::string(c.name) + " different seeds differ");
                expect(render(7, 0.0f) == render(8, 0.0f), std::string(c.name) + " zero tolerance ignores the seed");
            }
        }

        beginTest("start-up noise: off, silence stays silent; on, the oscillating models build up and the SVF stays quiet");
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
            }
            for (const auto* c : {&sklp, &skhp, &ota4})
                expect(oscillate(*c, Quality::Standard, 1000.0f, 2.5).levelDbfs > -40.0,
                       std::string(c->name) + " builds up with the noise on");
            expect(oscillate(svf, Quality::Standard, 1000.0f, 2.5).levelDbfs < -90.0, "the SVF does not");
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

        beginTest("the legacy FilterChannel and FilterUnit run the new models like the interface does, character and "
                  "high-pass included");
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
                s.highpass = 2;
                ch.configure(s, kFs);
                auto f = build(c, Quality::Standard, 1, 32);
                f->setControls(700.0f, 0.8f, 0.4f, 0.3f, 0.0f);
                f->setHighpassStep(2);
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
                // Antisymmetric input stays antisymmetric through the symmetric models; the Sallen-Key clipper is not
                // symmetric.
                const bool symmetric = c.kind != Kind::SallenKeyLp && c.kind != Kind::SallenKeyHp;
                expect(std::isfinite(l) && std::isfinite(r) && (!symmetric || std::abs(l + r) < 1.0e-3f),
                       std::string(c.name) + " stereo unit");
            }
        }

        beginTest("no allocation while the models run, are switched and are retuned (interface, four lanes, FilterChannel)");
        {
            for (const auto& c : models())
            {
                auto mono = build(c, Quality::Standard, 1, 256);
                auto quad = build(c, Quality::High, 4, 256);
                FilterChannel ch;
                ch.prepare(kFs, Quality::Standard);
                std::vector<float> a(256, 0.1f), b(256, 0.1f), cc(256, 0.1f), d(256, 0.1f), mod(256, 0.5f);
                float* lanes[4] = {a.data(), b.data(), cc.data(), d.data()};
                const float* mods[4] = {mod.data(), nullptr, mod.data(), nullptr};
                gAllocations = 0;
                gTrackAllocations = true;
                for (int i = 0; i < 20; ++i)
                {
                    mono->setControls(500.0f + static_cast<float>(i), 0.9f, 0.3f, 0.2f, 0.5f);
                    mono->setHighpassStep(i % 4);
                    mono->process(a.data(), 256, i % 2 == 0 ? mod.data() : nullptr);
                    quad->setLaneControls(i % 4, 800.0f, 0.7f, 0.4f, 0.1f, 0.2f);
                    quad->setHighpassStep((i + 1) % 4);
                    quad->processQuad(lanes, 256, mods);
                    FilterSettings s;
                    s.model = static_cast<int>(c.model);
                    s.cutoffHz = 400.0f + 30.0f * static_cast<float>(i);
                    s.resonance = 0.6f;
                    s.character = 0.5f;
                    s.highpass = i % 4;
                    ch.configure(s, kFs);
                    for (int k = 0; k < 8; ++k)
                        ch.process(0.2f);
                }
                gTrackAllocations = false;
                expectEquals(static_cast<int>(gAllocations), 0, std::string(c.name) + ": allocations on the audio path");
            }
        }

        beginTest("engine: filterN_character and filterN_hp exist for every filter slot (their place in the order is frozen "
                  "in tests/golden/param_order.txt), the new filter types are selectable and audible");
        {
            const int character = ParameterMap::indexOf("filter_character"), hp = ParameterMap::indexOf("filter_hp");
            expect(character >= 0 && hp >= 0 && ParameterMap::indexOf("filter2_character") >= 0 &&
                   ParameterMap::indexOf("filter2_hp") >= 0);
            int slots = 0;
            for (;;)
            {
                const std::string prefix = slots == 0 ? std::string("filter") : "filter" + std::to_string(slots + 1);
                if (ParameterMap::indexOf(prefix + "_character") < 0)
                    break;
                ++slots;
            }
            expect(slots >= 2, "filter slots with character: " + num(slots, 0));
            const auto& types = ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf("filter_type"))];
            expect(types.choices.size() == 25 && types.choices[20] == "Sallen-Key LP" && types.choices[24] == "Passive HP",
                   "filter type choices");
            expect(ParameterMap::params()[static_cast<size_t>(hp)].choices.size() == 4);

            // A low note through the Sallen-Key low-pass: the high-pass step removes bass; the SVF morph does too.
            const auto off = engineNote(20, 0.3f, 0.0f, 0, 33, 0.0f);
            const auto high = engineNote(20, 0.3f, 0.0f, 3, 33, 0.0f);
            expect(std::isfinite(off.totalDb) && off.totalDb > -60.0,
                   "Sallen-Key through the engine is audible: " + num(off.totalDb, 1) + " dBFS");
            expect(high.bassDb < off.bassDb - 6.0, "the 540 Hz high-pass step takes bass out of a 55 Hz note: " +
                                                       num(off.bassDb, 1) + " -> " + num(high.bassDb, 1) + " dB");
            const auto morphLp = engineNote(22, 0.3f, 0.0f, 0, 45, 0.0f);
            const auto morphHp = engineNote(22, 0.3f, 1.0f, 0, 45, 0.0f);
            expect(morphHp.bassDb < morphLp.bassDb - 10.0,
                   "SVF character morph to high-pass takes the bass out: " + num(morphLp.bassDb, 1) + " -> " +
                       num(morphHp.bassDb, 1) + " dB");
            for (int type = 20; type <= 24; ++type)
            {
                const auto e = engineNote(type, 1.0f, 0.5f, 1, 48, 1.0f);
                expect(std::isfinite(e.totalDb) && e.totalDb < 6.0,
                       "type " + num(type, 0) + ": " + num(e.totalDb, 1) + " dBFS");
            }
        }
    }
};

static AnalogFilterSkTests analogFilterSkTests;
