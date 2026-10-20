// SPDX-License-Identifier: MIT
#include <crucible/Filters.h>
#include <crucible/Engine.h>
#include <crucible/Patch.h>

#include <atomic>
#include <thread>
#include "TestSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
constexpr int kFrame = 8192;
constexpr int kBins = kFrame / 2;

// The hardware-style vowels: A E I O U, four formants each (freq Hz, bandwidth Hz, gain dB).
const std::array<std::array<FormantBand, 4>, 5> kHwVowels = {{
    {{{730, 80, 0}, {1090, 90, -4}, {2440, 120, -14}, {3400, 150, -20}}},
    {{{530, 70, 0}, {1840, 100, -6}, {2480, 120, -12}, {3400, 150, -20}}},
    {{{270, 60, 0}, {2290, 100, -10}, {3010, 120, -12}, {3500, 150, -20}}},
    {{{450, 70, 0}, {800, 80, -5}, {2830, 120, -18}, {3500, 150, -24}}},
    {{{300, 60, 0}, {870, 80, -8}, {2240, 120, -20}, {3400, 150, -26}}},
}};

FormantTable hwVowelTable(float outputGain)
{
    FormantBand flat[20];
    for (int v = 0; v < 5; ++v)
        for (int b = 0; b < 4; ++b)
            flat[v * 4 + b] = kHwVowels[static_cast<size_t>(v)][static_cast<size_t>(b)];
    FormantTable t;
    std::string error;
    const bool ok = makeFormantTable(flat, 5, 4, outputGain, t, error);
    if (!ok)
        t.vowels = 0;
    return t;
}

// Cutoff that puts the position exactly on vowel v of n (the law m = (n - 1) log(fc / 20) / log(1000)).
float cutoffForPosition(double m, int vowels)
{
    return static_cast<float>(20.0 * std::pow(1000.0, m / (vowels - 1)));
}

// Average power spectrum (dB per bin, relative to the power of the white input per bin) of uniform noise through the
// filter: the measured |H|^2.
std::vector<double> measuredResponseDb(const FilterSettings& s)
{
    FilterChannel c;
    c.configure(s, kRate);
    const Fft fft(13);
    std::uint32_t state = 12345u;
    const auto noise = [&state]
    {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) / 8388608.0f - 1.0f;
    };
    for (int i = 0; i < 8192; ++i)
        c.process(noise());
    std::vector<double> acc(kBins + 1, 0.0), ref(kBins + 1, 0.0);
    std::vector<float> in(kFrame), out(2 * kFrame), inSpec(2 * kFrame);
    const int frames = 96;
    for (int f = 0; f < frames; ++f)
    {
        for (int i = 0; i < kFrame; ++i)
        {
            in[static_cast<size_t>(i)] = noise();
            const double w = 0.5 - 0.5 * std::cos(2.0 * 3.14159265358979 * i / kFrame);
            out[static_cast<size_t>(i)] = c.process(in[static_cast<size_t>(i)]) * static_cast<float>(w);
            inSpec[static_cast<size_t>(i)] = in[static_cast<size_t>(i)] * static_cast<float>(w);
        }
        std::fill(out.begin() + kFrame, out.end(), 0.0f);
        std::fill(inSpec.begin() + kFrame, inSpec.end(), 0.0f);
        fft.forward(out.data());
        fft.forward(inSpec.data());
        for (int b = 0; b <= kBins; ++b)
        {
            const size_t k = static_cast<size_t>(b);
            acc[k] += out[2 * k] * out[2 * k] + out[2 * k + 1] * out[2 * k + 1];
            ref[k] += inSpec[2 * k] * inSpec[2 * k] + inSpec[2 * k + 1] * inSpec[2 * k + 1];
        }
    }
    double refMean = 0.0;
    for (int b = 100; b < kBins - 100; ++b)
        refMean += ref[static_cast<size_t>(b)];
    refMean /= (kBins - 200);
    std::vector<double> db(kBins + 1);
    for (int b = 0; b <= kBins; ++b)
        db[static_cast<size_t>(b)] = 10.0 * std::log10(std::max(acc[static_cast<size_t>(b)], 1.0e-20) / refMean);
    return db;
}

double binHz(int b) { return b * kRate / kFrame; }

// The highest level within +-6 % of hz: position (Hz) and dB.
std::pair<double, double> peakNear(const std::vector<double>& db, double hz)
{
    int best = -1;
    for (int b = 1; b < kBins; ++b)
        if (binHz(b) >= hz * 0.94 && binHz(b) <= hz * 1.06 && (best < 0 || db[static_cast<size_t>(b)] > db[static_cast<size_t>(best)]))
            best = b;
    return {binHz(best), db[static_cast<size_t>(best)]};
}
// Noise through filter 1 of a whole engine, as the host hears it: averaged power spectrum (dB per bin) of the left channel.
std::vector<double> engineSpectrumDb(Engine& e, float cutoff, float res, std::vector<float>* raw = nullptr)
{
    e.resetParamsToDefaults();
    e.setParam("osc1_level", 0.0f);
    e.setParam("osc2_level", 0.0f);
    e.setParam("osc3_level", 0.0f);
    e.setParam("sub_level", 0.0f);
    e.setParam("noise_level", 1.0f);
    e.setParam("noise_route", 0.0f);
    e.setParam("filter_on", 1.0f);
    e.setParam("filter_type", static_cast<float>(FilterModel::Formant));
    e.setParam("filter_cutoff", cutoff);
    e.setParam("filter_res", res);
    e.setParam("filter2_on", 0.0f);
    e.reset();
    e.noteOn(60, 1.0f);
    std::vector<float> l(512), r(512), all;
    for (int i = 0; i < 400; ++i)
    {
        e.process(l.data(), r.data(), 512);
        if (i >= 40)
            all.insert(all.end(), l.begin(), l.end());
    }
    e.noteOff(60);
    if (raw != nullptr)
        *raw = all;
    const Fft fft(13);
    std::vector<double> acc(kBins + 1, 0.0);
    std::vector<float> buf(2 * kFrame);
    int frames = 0;
    for (size_t at = 0; at + kFrame <= all.size(); at += kFrame, ++frames)
    {
        for (int i = 0; i < kFrame; ++i)
            buf[static_cast<size_t>(i)] = all[at + static_cast<size_t>(i)] *
                                          static_cast<float>(0.5 - 0.5 * std::cos(2.0 * 3.14159265358979 * i / kFrame));
        std::fill(buf.begin() + kFrame, buf.end(), 0.0f);
        fft.forward(buf.data());
        for (int b = 0; b <= kBins; ++b)
            acc[static_cast<size_t>(b)] += buf[2 * static_cast<size_t>(b)] * buf[2 * static_cast<size_t>(b)] +
                                           buf[2 * static_cast<size_t>(b) + 1] * buf[2 * static_cast<size_t>(b) + 1];
    }
    std::vector<double> db(kBins + 1);
    for (int b = 0; b <= kBins; ++b)
        db[static_cast<size_t>(b)] = 10.0 * std::log10(std::max(acc[static_cast<size_t>(b)] / std::max(frames, 1), 1.0e-20));
    return db;
}
} // namespace

class FormantTableEngineTests : public UnitTest
{
public:
    FormantTableEngineTests() : UnitTest("FormantTableEngine", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        const float cutoff = cutoffForPosition(2, 5); // vowel I
        beginTest("engine: noise through filter 1 peaks at the user vowel; reset restores the built-in output bit for bit");
        {
            std::vector<float> before, after, custom;
            {
                Engine never;
                never.prepare(48000.0, 512);
                engineSpectrumDb(never, cutoff, 0.25f, &before);
            }
            FormantBand flat[20];
            for (int v = 0; v < 5; ++v)
                for (int b = 0; b < 4; ++b)
                    flat[v * 4 + b] = kHwVowels[static_cast<size_t>(v)][static_cast<size_t>(b)];
            std::string error;
            Engine e;
            e.prepare(48000.0, 512);
            expect(e.setFormantTable(0, flat, 5, 4, 4.5f, error), error);
            const auto db = engineSpectrumDb(e, cutoff, 0.25f, &custom);
            for (const FormantBand& fb : kHwVowels[2])
            {
                const auto p = peakNear(db, fb.freqHz);
                expectWithinAbsoluteError(p.first, fb.freqHz, 0.05 * fb.freqHz, "engine peak " + String(fb.freqHz));
            }
            expect(custom != before, "the table changes the output");
            FormantTable probe;
            expect(e.formantTable(0, probe) && probe.vowels == 5 && probe.bands == 4);
            expect(e.setFormantTable(0, nullptr, 0, 0, 1.0f, error));
            expect(!e.formantTable(0, probe));
            Engine reset;
            reset.prepare(48000.0, 512);
            expect(reset.setFormantTable(0, flat, 5, 4, 4.5f, error));
            expect(reset.setFormantTable(0, nullptr, 0, 0, 1.0f, error));
            engineSpectrumDb(reset, cutoff, 0.25f, &after);
            expect(before.size() == after.size() &&
                       std::memcmp(before.data(), after.data(), before.size() * sizeof(float)) == 0,
                   "reset output equals the never-set output");
            expect(!e.setFormantTable(-1, flat, 5, 4, 4.5f, error) && !error.empty());
            expect(!e.setFormantTable(99, flat, 5, 4, 4.5f, error));
            expect(!e.setFormantTable(0, flat, 5, 4, 0.0f, error));
            expect(!e.formantTable(0, probe), "a refused table leaves the slot alone");
        }

        beginTest("engine: publishing tables while rendering is race free and finite");
        {
            Engine e;
            e.prepare(48000.0, 512);
            e.setParam("noise_level", 1.0f);
            e.setParam("filter_type", static_cast<float>(FilterModel::Formant));
            e.noteOn(60, 1.0f);
            FormantBand flat[20];
            for (int v = 0; v < 5; ++v)
                for (int b = 0; b < 4; ++b)
                    flat[v * 4 + b] = kHwVowels[static_cast<size_t>(v)][static_cast<size_t>(b)];
            std::atomic<bool> stop{false};
            std::thread writer(
                [&]
                {
                    std::string err;
                    for (int i = 0; !stop.load(); ++i)
                    {
                        e.setFormantTable(0, (i & 1) != 0 ? flat : nullptr, 5, 4, 4.5f, err);
                        e.setFormantSaturation(0, (i & 2) != 0, err);
                    }
                });
            std::vector<float> l(512), r(512);
            bool finite = true;
            for (int i = 0; i < 300; ++i)
            {
                e.process(l.data(), r.data(), 512);
                for (const float x : l)
                    finite = finite && std::isfinite(x);
            }
            stop = true;
            writer.join();
            expect(finite);
        }

        beginTest("engine: the filter1/response view follows the table");
        {
            Engine e;
            e.prepare(48000.0, 512);
            e.setParam("filter_type", static_cast<float>(FilterModel::Formant));
            e.setParam("filter_cutoff", cutoff);
            e.setParam("filter_res", 0.25f);
            {
                std::vector<float> l(512), r(512);
                e.process(l.data(), r.data(), 512); // applies the queued parameter changes
            }
            const int view = e.views().find("filter", "response");
            expect(view >= 0);
            std::vector<float> builtin(256), user(256), restored(256);
            expectEquals(e.views().read(view, builtin.data(), 256, nullptr), 256);
            std::string error;
            FormantBand flat[20];
            for (int v = 0; v < 5; ++v)
                for (int b = 0; b < 4; ++b)
                    flat[v * 4 + b] = kHwVowels[static_cast<size_t>(v)][static_cast<size_t>(b)];
            expect(e.setFormantTable(0, flat, 5, 4, 4.5f, error));
            expectEquals(e.views().read(view, user.data(), 256, nullptr), 256);
            FormantTable t;
            expect(makeFormantTable(flat, 5, 4, 4.5f, t, error));
            std::vector<float> grid(256), expected(256);
            for (int i = 0; i < 256; ++i)
                grid[static_cast<size_t>(i)] = static_cast<float>(20.0 * std::pow(1000.0, i / 255.0));
            FilterSettings fs;
            fs.model = static_cast<int>(FilterModel::Formant);
            fs.cutoffHz = e.getParam("filter_cutoff");
            fs.resonance = e.getParam("filter_res");
            fs.formants = &t;
            filterResponseDb(fs, e.getParam("filter_drive"), 48000.0, Quality::Standard, grid.data(), 256, expected.data());
            double worst = 0.0;
            for (size_t i = 0; i < 256; ++i)
                worst = std::max(worst, static_cast<double>(std::abs(user[i] - expected[i])));
            expect(worst < 0.05, "view equals filterResponseDb with the table: " + String(worst, 3) + " dB");
            expect(user != builtin, "user differs from built-in");
            expect(e.setFormantTable(0, nullptr, 0, 0, 1.0f, error));
            e.views().read(view, restored.data(), 256, nullptr);
            expect(restored == builtin, "restored equals built-in");
        }
    }
};

static FormantTableEngineTests formantTableEngineTests;

class FormantTableTests : public UnitTest
{
public:
    FormantTableTests() : UnitTest("FormantTable", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("makeFormantTable refuses bad input with a reason");
        {
            FormantTable t;
            std::string e;
            FormantBand ok[8];
            for (auto& b : ok)
                b = {1000.0f, 100.0f, 0.0f};
            expect(makeFormantTable(ok, 2, 4, 1.0f, t, e));
            expect(!makeFormantTable(ok, 1, 4, 1.0f, t, e) && !e.empty(), "one vowel");
            expect(!makeFormantTable(ok, 9, 1, 1.0f, t, e), "nine vowels");
            expect(!makeFormantTable(ok, 2, 0, 1.0f, t, e), "no formants");
            expect(!makeFormantTable(ok, 2, 5, 1.0f, t, e), "five formants");
            expect(!makeFormantTable(nullptr, 2, 4, 1.0f, t, e), "null");
            expect(!makeFormantTable(ok, 2, 4, 0.0f, t, e), "gain 0");
            expect(!makeFormantTable(ok, 2, 4, std::nanf(""), t, e), "gain nan");
            FormantBand bad[8];
            std::copy(ok, ok + 8, bad);
            bad[3].freqHz = 10.0f;
            expect(!makeFormantTable(bad, 2, 4, 1.0f, t, e), "freq 10");
            bad[3] = ok[3];
            bad[3].freqHz = 30000.0f;
            expect(!makeFormantTable(bad, 2, 4, 1.0f, t, e), "freq 30000");
            bad[3] = ok[3];
            bad[3].bandwidthHz = -1.0f;
            expect(!makeFormantTable(bad, 2, 4, 1.0f, t, e), "bandwidth < 0");
            bad[3] = ok[3];
            bad[3].gainDb = 24.5f;
            expect(!makeFormantTable(bad, 2, 4, 1.0f, t, e), "gain above 24 dB");
            bad[3] = ok[3];
            bad[3].gainDb = INFINITY;
            expect(!makeFormantTable(bad, 2, 4, 1.0f, t, e), "gain inf");
        }

        beginTest("a null table is the built-in vowels, bit for bit");
        {
            const FormantTable t = hwVowelTable(4.5f);
            FilterChannel a, b;
            FilterSettings s;
            s.model = static_cast<int>(FilterModel::Formant);
            s.cutoffHz = 700.0f;
            s.resonance = 0.4f;
            a.configure(s, kRate);
            FilterSettings u = s;
            u.formants = &t;
            b.configure(u, kRate); // a user table first, then back to the built-in one
            b.configure(s, kRate);
            b.reset();
            a.reset();
            bool same = true;
            std::uint32_t st = 7;
            for (int i = 0; i < 20000; ++i)
            {
                st = st * 1664525u + 1013904223u;
                const float x = static_cast<float>(st >> 8) / 8388608.0f - 1.0f;
                float ya = a.process(x), yb = b.process(x);
                same = same && std::memcmp(&ya, &yb, sizeof(float)) == 0;
            }
            expect(same);
        }

        beginTest("Hardware-style table: rendered peaks sit on the vowel's formants with the gains in dB");
        {
            const FormantTable t = hwVowelTable(4.5f);
            expectEquals(t.vowels, 5);
            for (int v = 0; v < 5; ++v)
            {
                FilterSettings s;
                s.model = static_cast<int>(FilterModel::Formant);
                s.cutoffHz = cutoffForPosition(v, 5);
                s.resonance = 0.25f;
                s.formants = &t;
                const auto db = measuredResponseDb(s);
                const double first = peakNear(db, kHwVowels[static_cast<size_t>(v)][0].freqHz).second;
                expectWithinAbsoluteError(first, 20.0 * std::log10(4.5), 1.0, "level of formant 1 of vowel " + String(v));
                for (int b = 0; b < 4; ++b)
                {
                    const FormantBand& fb = kHwVowels[static_cast<size_t>(v)][static_cast<size_t>(b)];
                    const auto p = peakNear(db, fb.freqHz);
                    expectWithinAbsoluteError(p.first, fb.freqHz, 0.03 * fb.freqHz,
                                              "peak " + String(b) + " of vowel " + String(v));
                    // The skirts of the lower formants add in quadrature to the weak upper ones (up to about +3 dB for
                    // the -20 dB formants of this table; the response view test pins the exact sum).
                    expectWithinAbsoluteError(p.second - first, fb.gainDb, b < 2 ? 1.0 : 3.5,
                                              "relative level " + String(b) + " of vowel " + String(v));
                }
            }
        }

        beginTest("well separated formants show their gains in dB within 1 dB");
        {
            FormantBand fb[4] = {{500.0f, 100.0f, 0.0f}, {4000.0f, 100.0f, -10.0f}, {600.0f, 100.0f, 0.0f}, {3000.0f, 100.0f, -20.0f}};
            FormantTable t;
            std::string e;
            expect(makeFormantTable(fb, 2, 2, 1.0f, t, e));
            FilterSettings s;
            s.model = static_cast<int>(FilterModel::Formant);
            s.cutoffHz = 20.0f;
            s.resonance = 0.25f;
            s.formants = &t;
            const auto db = measuredResponseDb(s);
            const auto p0 = peakNear(db, 500.0), p1 = peakNear(db, 4000.0);
            expectWithinAbsoluteError(p0.first, 500.0, 15.0);
            expectWithinAbsoluteError(p1.first, 4000.0, 120.0);
            expectWithinAbsoluteError(p1.second - p0.second, -10.0, 1.0);
        }

        beginTest("between vowels the formants glide geometrically");
        {
            const FormantTable t = hwVowelTable(1.0f);
            FilterSettings s;
            s.model = static_cast<int>(FilterModel::Formant);
            s.cutoffHz = cutoffForPosition(0.5, 5);
            s.resonance = 0.25f;
            s.formants = &t;
            const auto db = measuredResponseDb(s);
            const double f1 = std::sqrt(730.0 * 530.0);
            const double f2 = std::sqrt(1090.0 * 1840.0);
            expectWithinAbsoluteError(peakNear(db, f1).first, f1, 0.03 * f1, "formant 1 midway");
            expectWithinAbsoluteError(peakNear(db, f2).first, f2, 0.03 * f2, "formant 2 midway");
            expectWithinAbsoluteError(peakNear(db, f2).second - peakNear(db, f1).second, -5.0, 1.5, "mid gain in dB");
        }

        beginTest("the response view equals the rendered transfer");
        {
            const FormantTable t = hwVowelTable(4.5f);
            for (const double m : {0.0, 1.0, 2.5, 3.0})
                for (const float res : {0.25f, 0.8f})
                {
                    FilterSettings s;
                    s.model = static_cast<int>(FilterModel::Formant);
                    s.cutoffHz = cutoffForPosition(m, 5);
                    s.resonance = res;
                    s.formants = &t;
                    const auto db = measuredResponseDb(s);
                    std::vector<float> hz, resp(kBins);
                    for (int b = 0; b < kBins; ++b)
                        hz.push_back(static_cast<float>(binHz(b)));
                    filterResponseDb(s, 0.0f, kRate, Quality::Standard, hz.data(), kBins, resp.data());
                    double worst = 0.0;
                    for (int b = 40; b < 2400; ++b)
                        if (resp[static_cast<size_t>(b)] > -20.0f)
                            worst = std::max(worst, std::abs(db[static_cast<size_t>(b)] - resp[static_cast<size_t>(b)]));
                    expect(worst < 1.5, "view vs render at position " + String(m) + " res " + String(res) + ": " +
                                            String(worst, 2) + " dB");
                }
        }

        beginTest("a bandwidth of 0 keeps the resonance-derived damping");
        {
            FormantBand fb[6];
            fb[0] = {500.0f, 0.0f, 0.0f};
            fb[1] = {500.0f, 0.0f, 0.0f};
            fb[2] = {1500.0f, 0.0f, 0.0f};
            fb[3] = {1500.0f, 0.0f, 0.0f};
            fb[4] = {2500.0f, 0.0f, 0.0f};
            fb[5] = {2500.0f, 0.0f, 0.0f};
            FormantTable t;
            std::string e;
            expect(makeFormantTable(fb, 2, 1, 1.0f, t, e));
            FilterSettings s;
            s.model = static_cast<int>(FilterModel::Formant);
            s.cutoffHz = 20.0f;
            s.formants = &t;
            s.resonance = 0.0f;
            const auto lo = measuredResponseDb(s);
            s.resonance = 0.9f;
            const auto hi = measuredResponseDb(s);
            const int centre = static_cast<int>(std::lround(500.0 / (kRate / kFrame)));
            // Unity-peak band-pass: the centre level does not move with resonance, the skirts narrow.
            expectWithinAbsoluteError(lo[static_cast<size_t>(centre)], hi[static_cast<size_t>(centre)], 1.5);
            expect(lo[static_cast<size_t>(centre + 60)] > hi[static_cast<size_t>(centre + 60)] + 6.0);
        }
    }
};

static FormantTableTests formantTableTests;

class FormantTablePatchTests : public UnitTest
{
public:
    FormantTablePatchTests() : UnitTest("FormantTablePatch", "Engine") {}

    static bool same(const FormantTable& a, const FormantTable& b)
    {
        return a.vowels == b.vowels && a.bands == b.bands && std::memcmp(&a.outputGain, &b.outputGain, sizeof(float)) == 0 &&
               std::memcmp(a.band.data(), b.band.data(), sizeof(FormantBand) * a.band.size()) == 0;
    }

    void runTest() override
    {
        beginTest("patches save and load user vowel tables; save -> load -> save is bit-stable");
        {
            Engine a;
            a.prepare(kRate, 256);
            FormantBand flat[20];
            for (int v = 0; v < 5; ++v)
                for (int b = 0; b < 4; ++b)
                    flat[v * 4 + b] = kHwVowels[static_cast<size_t>(v)][static_cast<size_t>(b)];
            FormantBand small[6] = {{333.3f, 0.0f, 0.1f}, {777.7f, 55.5f, -3.3f}, {1234.5f, 0.0f, 24.0f},
                                    {222.2f, 33.3f, 0.0f}, {3333.3f, 99.9f, -1.1f}, {5555.5f, 0.0f, -40.0f}};
            std::string error;
            expect(a.setFormantTable(0, flat, 5, 4, 4.5f, error), error);
            expect(a.setFormantTable(1, small, 3, 2, 0.123f, error), error);
            const std::string text = patchToJson(capturePatch(a));
            expect(text.find("formantTables") != std::string::npos);
            Patch p;
            expect(patchFromJson(text, p, error), error);
            Engine b;
            b.prepare(kRate, 256);
            applyPatch(b, p);
            FormantTable ta, tb;
            for (int slot = 0; slot < 2; ++slot)
            {
                expect(a.formantTable(slot, ta) && b.formantTable(slot, tb), "slot " + String(slot));
                expect(same(ta, tb), "slot " + String(slot) + " identical");
            }
            expect(!b.formantTable(2, tb));
            Patch p2;
            expect(patchFromJson(patchToJson(capturePatch(b)), p2, error), error);
            expect(patchToJson(capturePatch(b)) == text, "save -> load -> save is identical");
        }

        beginTest("old patches load unchanged: no key written without tables, and loading one clears the tables");
        {
            Engine a;
            a.prepare(kRate, 256);
            const std::string text = patchToJson(capturePatch(a));
            expect(text.find("formantTables") == std::string::npos);
            FormantBand two[2] = {{500.0f, 50.0f, 0.0f}, {1500.0f, 50.0f, 0.0f}};
            std::string error;
            Engine b;
            b.prepare(kRate, 256);
            expect(b.setFormantTable(0, two, 2, 1, 1.0f, error));
            Patch p;
            expect(patchFromJson(text, p, error), error);
            applyPatch(b, p);
            FormantTable t;
            expect(!b.formantTable(0, t), "a patch without tables restores the built-in vowels");
        }

        beginTest("malformed table entries are skipped");
        {
            Engine a;
            a.prepare(kRate, 256);
            FormantBand two[2] = {{500.0f, 50.0f, 0.0f}, {1500.0f, 50.0f, 0.0f}};
            std::string error;
            expect(a.setFormantTable(0, two, 2, 1, 1.0f, error));
            std::string text = patchToJson(capturePatch(a));
            const auto at = text.find("\"formantTables\"");
            expect(at != std::string::npos);
            // Break the payload: the entry must be dropped rather than crash or half-apply.
            const auto data = text.find("\"data\": \"", at);
            expect(data != std::string::npos);
            text.insert(data + 9, "!!!");
            Patch p;
            expect(patchFromJson(text, p, error), error);
            expect(p.formants.empty());
        }
    }
};

static FormantTablePatchTests formantTablePatchTests;

namespace
{
// Reference of the hardware-style formant filter, independent of FilterChannel: per formant a unity-peak two-pole zero-delay
// band-pass (peak = gain at the centre), four in parallel, summed, times the output gain. Q = max(f / bw * 0.5 * 16^res,
// 0.3); vowel position m = (n - 1) log(fc / 20) / log(1000), geometric freq / bw and dB-linear gain between vowels.
struct LegacyFormantRef
{
    struct Band
    {
        double a1, a2, a3, k, gain, ic1 = 0.0, ic2 = 0.0;
    };
    std::vector<Band> bands;
    double outGain = 1.0;
    bool saturate = false;

    LegacyFormantRef(const FormantBand* flat, int vowels, int nBands, float outputGain, double cutoff, double res,
                     bool sat = false)
    {
        outGain = outputGain;
        saturate = sat;
        const double pos = std::clamp(std::log(std::clamp(cutoff, 20.0, 20000.0) / 20.0) / std::log(1000.0), 0.0, 1.0) *
                           (vowels - 1);
        const int i = std::min(static_cast<int>(pos), vowels - 2);
        const double f = pos - i;
        for (int b = 0; b < nBands; ++b)
        {
            const FormantBand& x = flat[i * nBands + b];
            const FormantBand& y = flat[(i + 1) * nBands + b];
            const double freq = x.freqHz * std::pow(static_cast<double>(y.freqHz) / x.freqHz, f);
            const double bw = x.bandwidthHz * std::pow(static_cast<double>(y.bandwidthHz) / x.bandwidthHz, f);
            const double q = std::max(freq / bw * 0.5 * std::pow(16.0, res), 0.3);
            Band bd;
            bd.k = 1.0 / q;
            const double g = std::tan(3.14159265358979323846 * freq / kRate);
            bd.a1 = 1.0 / (1.0 + g * (g + bd.k));
            bd.a2 = g * bd.a1;
            bd.a3 = g * bd.a2;
            bd.gain = std::pow(10.0, (x.gainDb + (y.gainDb - x.gainDb) * f) / 20.0);
            bands.push_back(bd);
        }
    }
    double process(double x)
    {
        double sum = 0.0;
        for (auto& b : bands)
        {
            const double v3 = x - b.ic2;
            const double v1 = b.a1 * b.ic1 + b.a2 * v3;
            const double v2 = b.ic2 + b.a2 * b.ic1 + b.a3 * v3;
            if (saturate)
            {
                const double u = std::clamp(0.5 * (2.0 * v1 - b.ic1), -3.0, 3.0);
                b.ic1 = 2.0 * (u * (27.0 + u * u) / (27.0 + 9.0 * u * u));
                b.ic2 = std::clamp(2.0 * v2 - b.ic2, -8.0, 8.0);
            }
            else
            {
                b.ic1 = 2.0 * v1 - b.ic1;
                b.ic2 = 2.0 * v2 - b.ic2;
            }
            sum += b.k * v1 * b.gain;
        }
        return sum * outGain;
    }
};

double rmsDb(const std::vector<double>& v, size_t from)
{
    double s = 0.0;
    for (size_t i = from; i < v.size(); ++i)
        s += v[i] * v[i];
    return 10.0 * std::log10(std::max(s / static_cast<double>(v.size() - from), 1.0e-30));
}
} // namespace

class FormantLevelTests : public UnitTest
{
public:
    FormantLevelTests() : UnitTest("FormantLevel", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        FormantBand flat[20];
        for (int v = 0; v < 5; ++v)
            for (int b = 0; b < 4; ++b)
                flat[v * 4 + b] = kHwVowels[static_cast<size_t>(v)][static_cast<size_t>(b)];
        const double pi = 3.14159265358979323846;

        beginTest("FilterChannel: a sine on a single formant's centre passes at 0 dB (unity-peak band-pass)");
        for (const float res : {0.35f, 0.55f})
            for (const float hz : {300.0f, 800.0f, 2400.0f})
            {
                const FormantBand one[2] = {{hz, 100.0f, 0.0f}, {hz, 100.0f, 0.0f}};
                FormantTable t;
                std::string error;
                expect(makeFormantTable(one, 2, 1, 1.0f, t, error), error);
                FilterSettings s;
                s.model = static_cast<int>(FilterModel::Formant);
                s.cutoffHz = 500.0f;
                s.resonance = res;
                s.formants = &t;
                FilterChannel c;
                c.configure(s, kRate);
                std::vector<double> y;
                for (int i = 0; i < 48000; ++i)
                    y.push_back(c.process(static_cast<float>(std::sin(2.0 * pi * hz * i / kRate))));
                expectWithinAbsoluteError(rmsDb(y, 24000) - rmsDb({std::sqrt(0.5)}, 0), 0.0, 0.1,
                                          "centre gain, hz " + String(hz) + " res " + String(res));
            }

        beginTest("FilterChannel and engine voice: the 5x4 legacy table on a saw matches the in-test legacy reference");
        const auto sawLevelDelta = [&](int note, float cutoff, float res)
        {
            const double f0 = 440.0 * std::pow(2.0, (note - 69) / 12.0);
            FormantTable t = hwVowelTable(4.5f);
            FilterSettings s;
            s.model = static_cast<int>(FilterModel::Formant);
            s.cutoffHz = cutoff;
            s.resonance = res;
            s.formants = &t;
            FilterChannel c;
            c.configure(s, kRate);
            LegacyFormantRef ref(flat, 5, 4, 4.5f, cutoff, res);
            std::vector<double> in, got, want;
            double ph = 0.0;
            for (int i = 0; i < 96000; ++i)
            {
                const double x = 0.5 * (2.0 * ph - 1.0);
                ph += f0 / kRate;
                ph -= std::floor(ph);
                in.push_back(x);
                got.push_back(c.process(static_cast<float>(x)));
                want.push_back(ref.process(x));
            }
            return std::array<double, 2>{rmsDb(got, 48000) - rmsDb(in, 48000), rmsDb(want, 48000) - rmsDb(in, 48000)};
        };
        for (const int note : {36, 48, 60})
            for (const float cutoff : {110.0f, 160.0f, 632.0f})
                for (const float res : {0.35f, 0.55f})
                {
                    const auto d = sawLevelDelta(note, cutoff, res);
                    expectWithinAbsoluteError(d[0], d[1], 1.0,
                                              "note " + String(note) + " cutoff " + String(cutoff) + " res " + String(res) +
                                                  " (channel " + String(d[0]) + " ref " + String(d[1]) + " dB)");
                }

        beginTest("engine voice: filter-on level against the legacy reference run on the voice's own unfiltered output");
        {
            FormantBand f20[20];
            std::copy(flat, flat + 20, f20);
            for (const int note : {36, 48, 60})
                for (const float cutoff : {110.0f, 160.0f, 632.0f})
                    for (const float res : {0.35f, 0.55f})
                    {
                        std::vector<double> off, on;
                        for (const bool filterOn : {false, true})
                        {
                            Engine e;
                            e.prepare(kRate, 512);
                            std::string error;
                            expect(e.setFormantTable(0, f20, 5, 4, 4.5f, error), error);
                            e.resetParamsToDefaults();
                            e.setParam("filter_on", filterOn ? 1.0f : 0.0f);
                            e.setParam("filter_type", static_cast<float>(FilterModel::Formant));
                            e.setParam("filter_cutoff", cutoff);
                            e.setParam("filter_res", res);
                            e.setParam("filter_mix", 1.0f);
                            e.setParam("filter2_on", 0.0f);
                            e.reset();
                            e.noteOn(note, 1.0f);
                            std::vector<float> l(512), r(512);
                            for (int i = 0; i < 188; ++i)
                            {
                                e.process(l.data(), r.data(), 512);
                                if (i >= 94)
                                    for (const float x : l)
                                        (filterOn ? on : off).push_back(x);
                            }
                        }
                        LegacyFormantRef ref(flat, 5, 4, 4.5f, cutoff, res);
                        std::vector<double> want;
                        for (const double x : off)
                            want.push_back(ref.process(x));
                        const double eng = rmsDb(on, 0) - rmsDb(off, 0);
                        const double leg = rmsDb(want, 0) - rmsDb(off, 0);
                        expectWithinAbsoluteError(eng, leg, 1.0,
                                                  "engine note " + String(note) + " cutoff " + String(cutoff) + " res " +
                                                      String(res) + " (engine " + String(eng) + " ref " + String(leg) + " dB)");
                    }
        }
    }
};

static FormantLevelTests formantLevelTests;

class FormantSaturationTests : public UnitTest
{
public:
    FormantSaturationTests() : UnitTest("FormantSaturation", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        FormantBand flat[20];
        for (int v = 0; v < 5; ++v)
            for (int b = 0; b < 4; ++b)
                flat[v * 4 + b] = kHwVowels[static_cast<size_t>(v)][static_cast<size_t>(b)];

        beginTest("FilterChannel with saturation: the 5x4 legacy table on a saw is within 0.5 dB of the saturating reference");
        for (const int note : {36, 48, 60})
            for (const float cutoff : {110.0f, 160.0f, 632.0f})
                for (const float res : {0.35f, 0.55f})
                {
                    const double f0 = 440.0 * std::pow(2.0, (note - 69) / 12.0);
                    FormantTable t = hwVowelTable(4.5f);
                    t.saturate = true;
                    FilterSettings s;
                    s.model = static_cast<int>(FilterModel::Formant);
                    s.cutoffHz = cutoff;
                    s.resonance = res;
                    s.formants = &t;
                    FilterChannel c;
                    c.configure(s, kRate);
                    LegacyFormantRef ref(flat, 5, 4, 4.5f, cutoff, res, true);
                    LegacyFormantRef lin(flat, 5, 4, 4.5f, cutoff, res, false);
                    std::vector<double> in, got, want, linear;
                    double ph = 0.0;
                    for (int i = 0; i < 96000; ++i)
                    {
                        const double x = 0.5 * (2.0 * ph - 1.0);
                        ph += f0 / kRate;
                        ph -= std::floor(ph);
                        in.push_back(x);
                        got.push_back(c.process(static_cast<float>(x)));
                        want.push_back(ref.process(x));
                        linear.push_back(lin.process(x));
                    }
                    const double g = rmsDb(got, 48000) - rmsDb(in, 48000);
                    const double w = rmsDb(want, 48000) - rmsDb(in, 48000);
                    const double l = rmsDb(linear, 48000) - rmsDb(in, 48000);
                    if (note == 60 && cutoff == 632.0f)
                        expectWithinAbsoluteError(g, res < 0.45f ? 0.4 : -4.0, 0.6, "legacy plug-in level");
                    if (note == 36 && cutoff == 110.0f && res < 0.45f)
                        expectWithinAbsoluteError(g, -9.8, 1.6, "legacy plug-in level");
                    expectWithinAbsoluteError(g, w, 0.5,
                                              "note " + String(note) + " cutoff " + String(cutoff) + " res " + String(res) +
                                                  " (channel " + String(g, 2) + " ref " + String(w, 2) + " linear " +
                                                  String(l, 2) + " dB)");
                }

        beginTest("engine voice with saturation: level against the saturating reference on the voice's own unfiltered output");
        for (const int note : {36, 60})
            for (const float cutoff : {110.0f, 632.0f})
                for (const float res : {0.35f, 0.55f})
                {
                    std::vector<double> off, on;
                    for (const bool filterOn : {false, true})
                    {
                        Engine e;
                        e.prepare(kRate, 512);
                        std::string error;
                        expect(e.setFormantTable(0, flat, 5, 4, 4.5f, error), error);
                        expect(e.setFormantSaturation(0, true, error), error);
                        e.resetParamsToDefaults();
                        e.setParam("filter_on", filterOn ? 1.0f : 0.0f);
                        e.setParam("filter_type", static_cast<float>(FilterModel::Formant));
                        e.setParam("filter_cutoff", cutoff);
                        e.setParam("filter_res", res);
                        e.setParam("filter_mix", 1.0f);
                        e.setParam("filter2_on", 0.0f);
                        // The amplifier follows the filter: a constant gain of 1 keeps the filter input equal to the
                        // unfiltered output, which the level-dependent saturation needs.
                        e.setParam("amp_attack", 0.001f);
                        e.setParam("amp_sustain", 1.0f);
                        e.setParam("master_gain", 0.0f);
                        e.reset();
                        e.noteOn(note, 1.0f);
                        std::vector<float> l(512), r(512);
                        for (int i = 0; i < 188; ++i)
                        {
                            e.process(l.data(), r.data(), 512);
                            if (i >= 94)
                                for (const float x : l)
                                    (filterOn ? on : off).push_back(x);
                        }
                    }
                    LegacyFormantRef ref(flat, 5, 4, 4.5f, cutoff, res, true);
                    std::vector<double> want;
                    for (const double x : off)
                        want.push_back(ref.process(x));
                    const double eng = rmsDb(on, 0) - rmsDb(off, 0);
                    const double leg = rmsDb(want, 0) - rmsDb(off, 0);
                    expectWithinAbsoluteError(eng, leg, 0.5,
                                              "engine note " + String(note) + " cutoff " + String(cutoff) + " res " +
                                                  String(res) + " (engine " + String(eng, 2) + " ref " + String(leg, 2) + " dB)");
                }

        beginTest("saturation is off by default (bit for bit) and compresses loud input while leaving quiet input alone");
        {
            FormantTable lin = hwVowelTable(4.5f), sat = lin, dflt = lin;
            sat.saturate = true;
            expect(!dflt.saturate);
            const auto run = [&](const FormantTable& t, float amp)
            {
                FilterSettings s;
                s.model = static_cast<int>(FilterModel::Formant);
                s.cutoffHz = 300.0f;
                s.resonance = 0.55f;
                s.formants = &t;
                FilterChannel c;
                c.configure(s, kRate);
                std::vector<float> y;
                for (int i = 0; i < 24000; ++i)
                    y.push_back(c.process(amp * static_cast<float>(std::sin(2.0 * 3.14159265358979 * 300.0 * i / kRate))));
                return y;
            };
            const auto a = run(lin, 0.5f), b = run(dflt, 0.5f);
            expect(std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0, "default table equals linear");
            const auto qLin = run(lin, 0.001f), qSat = run(sat, 0.001f);
            double dq = 0.0;
            for (size_t i = 12000; i < qLin.size(); ++i)
                dq = std::max(dq, static_cast<double>(std::abs(qLin[i] - qSat[i])));
            expect(dq < 0.002 * 4.5 * 0.01, "quiet input is linear: max diff " + String(dq, 6));
            const auto lLin = run(lin, 1.0f), lSat = run(sat, 1.0f);
            std::vector<double> dl(lLin.begin(), lLin.end()), ds(lSat.begin(), lSat.end());
            expect(rmsDb(ds, 12000) < rmsDb(dl, 12000) - 0.5, "loud input is compressed");
            expect(std::all_of(lSat.begin(), lSat.end(), [](float x) { return std::isfinite(x); }));
        }

        beginTest("engine: needs a user table, resets with it, and is saved in patches");
        {
            Engine e;
            e.prepare(kRate, 256);
            std::string error;
            expect(!e.setFormantSaturation(0, true, error) && !error.empty(), "no table");
            expect(!e.setFormantSaturation(-1, true, error) && !e.setFormantSaturation(99, true, error), "bad slot");
            expect(e.setFormantTable(0, flat, 5, 4, 4.5f, error), error);
            FormantTable t;
            expect(e.formantTable(0, t) && !t.saturate);
            expect(e.setFormantSaturation(0, true, error), error);
            expect(e.formantTable(0, t) && t.saturate);
            const std::string text = patchToJson(capturePatch(e));
            expect(text.find("\"saturate\"") != std::string::npos);
            Patch p;
            expect(patchFromJson(text, p, error), error);
            Engine b;
            b.prepare(kRate, 256);
            applyPatch(b, p);
            expect(b.formantTable(0, t) && t.saturate, "loaded");
            expect(patchToJson(capturePatch(b)) == text, "save -> load -> save is identical");
            expect(e.setFormantTable(0, flat, 5, 4, 4.5f, error));
            expect(e.formantTable(0, t) && !t.saturate, "setting a table turns it off");
            expect(patchToJson(capturePatch(e)).find("saturate") == std::string::npos, "off is not written");
        }
    }
};

static FormantSaturationTests formantSaturationTests;
