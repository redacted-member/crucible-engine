// SPDX-License-Identifier: MIT
// The filter response views ("filter" / "filterN", "response" and "response_mod") and the magnitude function behind them
// (filterResponseDb): the linear models against stepped-sine measurements of the filter itself, the nonlinear analog
// models against small-signal measurements, the frequency of the cutoff / peak feature, and the view wiring.

#include "AnalogMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Filters.h>
#include <crucible/Views.h>

#include <atomic>
#include <memory>
#include <thread>

using namespace crucible;
using namespace crucible::testing;
namespace m = crucible::measure;

namespace
{
constexpr int kPoints = 256;

std::vector<float> responseGrid()
{
    std::vector<float> hz(kPoints);
    for (int i = 0; i < kPoints; ++i)
        hz[static_cast<size_t>(i)] = static_cast<float>(20.0 * std::pow(1000.0, static_cast<double>(i) / (kPoints - 1)));
    return hz;
}

std::vector<float> analytic(const FilterSettings& s, float drive, const std::vector<float>& hz)
{
    std::vector<float> db(hz.size());
    filterResponseDb(s, drive, kRate, Quality::Standard, hz.data(), static_cast<int>(hz.size()), db.data());
    return db;
}

// The filter as a sample processor, built the way the voices configure it.
auto unitFactory(const FilterSettings& s, float drive)
{
    return [=]()
    {
        auto unit = std::make_shared<FilterUnit>();
        unit->prepare(kRate, Quality::Standard);
        unit->configure(s, drive, kRate);
        return [unit](double x)
        {
            float l = static_cast<float>(x), r = l;
            unit->process(l, r);
            return static_cast<double>(l);
        };
    };
}

m::ResponseCurve sweepCurve(const FilterSettings& s, float drive, const std::vector<double>& freqs)
{
    const auto ir = m::sweepImpulseResponse(unitFactory(s, drive), kRate, 20.0, 0.0, 3.0, 0.01, 0.25, 0.4);
    return m::responseFromImpulse(ir, kRate, freqs);
}

double interpolate(const std::vector<float>& db, const std::vector<float>& hz, double f)
{
    size_t i = 1;
    while (i + 1 < hz.size() && hz[i] < f)
        ++i;
    const double t = (std::log(f) - std::log(hz[i - 1])) / (std::log(hz[i]) - std::log(hz[i - 1]));
    return db[i - 1] + (db[i] - db[i - 1]) * t;
}

// The frequency of the response's feature: the peak when it stands at least 3 dB above the pass band (the end the filter
// passes), else the -3 dB crossing of the pass band level.
double featureHz(const std::vector<double>& hz, const std::vector<double>& db, bool lowpass)
{
    const double pass = lowpass ? db.front() : db.back();
    size_t top = 0;
    for (size_t i = 1; i < db.size(); ++i)
        if (db[i] > db[top])
            top = i;
    if (db[top] > pass + 3.0)
    {
        // Parabolic refinement on the log axis.
        if (top > 0 && top + 1 < db.size())
        {
            const double a = db[top - 1], b = db[top], c = db[top + 1];
            const double denom = a - 2.0 * b + c;
            const double offset = std::abs(denom) > 1.0e-9 ? 0.5 * (a - c) / denom : 0.0;
            return hz[top] * std::pow(hz[top + 1] / hz[top], offset);
        }
        return hz[top];
    }
    if (lowpass)
    {
        for (size_t i = 1; i < db.size(); ++i)
            if (db[i] <= pass - 3.0)
                return hz[i - 1] * std::pow(hz[i] / hz[i - 1], (db[i - 1] - (pass - 3.0)) / (db[i - 1] - db[i]));
    }
    else
    {
        for (size_t i = db.size() - 1; i > 0; --i)
            if (db[i - 1] <= pass - 3.0)
                return hz[i - 1] * std::pow(hz[i] / hz[i - 1], ((pass - 3.0) - db[i - 1]) / (db[i] - db[i - 1]));
    }
    return 0.0;
}

std::string modelName(int model)
{
    return filterModelNames()[static_cast<size_t>(model)];
}

bool isLowpassLike(int model)
{
    const auto f = static_cast<FilterModel>(model);
    return f == FilterModel::Lowpass12 || f == FilterModel::Lowpass24 || f == FilterModel::LadderLowpass ||
           f == FilterModel::DiodeLowpass || f == FilterModel::DirtyLowpass || f == FilterModel::TransistorLadder ||
           f == FilterModel::OtaCascade || f == FilterModel::CoupledDiodeLadder || f == FilterModel::SallenKeyLowpass ||
           f == FilterModel::Ota4Pole;
}

bool isHighpassLike(int model)
{
    const auto f = static_cast<FilterModel>(model);
    return f == FilterModel::Highpass12 || f == FilterModel::Highpass24 || f == FilterModel::LadderHighpass ||
           f == FilterModel::SallenKeyHighpass || f == FilterModel::PassiveHighpass;
}

// The values of two curves agree to the float noise of the parameter smoothing (0.01 dB).
bool near(const std::vector<float>& a, const std::vector<float>& b)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (!(std::abs(a[i] - b[i]) <= 0.01f))
            return false;
    return true;
}

void run(Engine& e, int blocks, int size = 256)
{
    std::vector<float> l(static_cast<size_t>(size)), r(static_cast<size_t>(size));
    for (int b = 0; b < blocks; ++b)
        e.process(l.data(), r.data(), size);
}

std::vector<float> readFull(const Engine& e, int view)
{
    std::vector<float> buffer(static_cast<size_t>(e.views().info(view).maxElements));
    const int n = e.views().read(view, buffer.data(), static_cast<int>(buffer.size()), nullptr);
    buffer.resize(static_cast<size_t>(std::max(n, 0)));
    return buffer;
}
} // namespace

class FilterResponseTests : public UnitTest
{
public:
    FilterResponseTests() : UnitTest("Views filter response", "Engine") {}

    void runTest() override
    {
        const auto grid = responseGrid();

        beginTest("linear models: the response matches the stepped-sine magnitude of the filter within 0.5 dB up to 10 kHz");
        {
            const auto freqs = m::logSpacedFrequencies(60.0, 10000.0, 2);
            struct Case
            {
                float cutoff, res, drive;
            };
            const std::array<Case, 3> cases = {{{1200.0f, 0.35f, 0.0f}, {300.0f, 0.6f, 0.0f}, {2500.0f, 0.2f, 0.4f}}};
            double worst = 0.0;
            for (int model = 0; model <= static_cast<int>(FilterModel::DirtyLowpass); ++model)
                for (const auto& c : cases)
                {
                    FilterSettings s;
                    s.model = model;
                    s.cutoffHz = c.cutoff;
                    s.resonance = c.res;
                    std::vector<float> hz(freqs.begin(), freqs.end());
                    const auto db = analytic(s, c.drive, hz);
                    float top = -200.0f;
                    for (const float v : db)
                        top = std::max(top, v);
                    for (size_t i = 0; i < freqs.size(); ++i)
                    {
                        if (db[i] < top - 40.0f)
                            continue; // a null: the measurement's own floor decides there
                        const double got = m::steppedGainDb(unitFactory(s, c.drive), freqs[i], kRate, 0.004, 0.25, 0.15);
                        const double err = std::abs(got - db[i]);
                        worst = std::max(worst, err);
                        if (err > 0.5)
                            expect(false, modelName(model) + " cutoff " + String(c.cutoff, 0).str() + " res " +
                                              String(c.res, 2).str() + " drive " + String(c.drive, 1).str() + " at " +
                                              String(freqs[i], 0).str() + " Hz: measured " + String(got, 2).str() +
                                              " dB, response " + String(db[i], 2).str() + " dB");
                    }
                }
            std::printf("    linear models: worst deviation %.3f dB\n", worst);
            expect(worst <= 0.5, "linear models within 0.5 dB");
        }

        beginTest("analog models: the response matches the small-signal stepped-sine magnitude within 1.5 dB up to 10 kHz");
        {
            const auto freqs = m::logSpacedFrequencies(80.0, 10000.0, 2);
            struct Case
            {
                float cutoff, res, character, drive;
                int highpass;
            };
            const std::array<Case, 4> cases = {{{1000.0f, 0.3f, 0.0f, 0.0f, 0},
                                                {2000.0f, 0.6f, 1.0f, 0.0f, 0},
                                                {600.0f, 0.8f, 0.5f, 0.0f, 0},
                                                {1500.0f, 0.4f, 0.3f, 0.5f, 2}}};
            double worst = 0.0;
            for (int model = static_cast<int>(FilterModel::TransistorLadder); model < static_cast<int>(FilterModel::Count);
                 ++model)
                for (const auto& c : cases)
                {
                    FilterSettings s;
                    s.model = model;
                    s.cutoffHz = c.cutoff;
                    s.resonance = c.res;
                    s.character = c.character;
                    s.highpass = c.highpass;
                    std::vector<float> hz(freqs.begin(), freqs.end());
                    const auto db = analytic(s, c.drive, hz);
                    float top = -200.0f;
                    for (const float v : db)
                        top = std::max(top, v);
                    double caseWorst = 0.0;
                    for (size_t i = 0; i < freqs.size(); ++i)
                    {
                        if (db[i] < top - 45.0f)
                            continue;
                        const double got = m::steppedGainDb(unitFactory(s, c.drive), freqs[i], kRate, 0.002, 0.2, 0.1);
                        caseWorst = std::max(caseWorst, std::abs(got - db[i]));
                    }
                    worst = std::max(worst, caseWorst);
                    if (caseWorst > 1.5)
                        expect(false, modelName(model) + " cutoff " + String(c.cutoff, 0).str() + " res " +
                                          String(c.res, 2).str() + " char " + String(c.character, 2).str() + " drive " +
                                          String(c.drive, 1).str() + ": deviation " + String(caseWorst, 2).str() + " dB");
                }
            std::printf("    analog models: worst deviation %.3f dB\n", worst);
            expect(worst <= 1.5, "analog models within 1.5 dB");
        }

        beginTest("cutoff or peak frequency of the response matches the measured one within 3 %");
        {
            const auto freqs = m::logSpacedFrequencies(80.0, 9000.0, 24);
            std::vector<float> hz(freqs.begin(), freqs.end());
            double worst = 0.0;
            int checked = 0;
            for (int model = 0; model < static_cast<int>(FilterModel::Count); ++model)
            {
                const bool lowpass = isLowpassLike(model), highpass = isHighpassLike(model);
                if (!lowpass && !highpass)
                    continue;
                for (const float res : {0.0f, 0.7f})
                {
                    FilterSettings s;
                    s.model = model;
                    s.cutoffHz = 1500.0f;
                    s.resonance = res;
                    const auto db = analytic(s, 0.0f, hz);
                    std::vector<double> modelDb(db.begin(), db.end());
                    const auto measured = sweepCurve(s, 0.0f, freqs);
                    const double fa = featureHz(freqs, modelDb, lowpass);
                    const double fm = featureHz(freqs, measured.gainDb, lowpass);
                    if (fa <= 0.0 || fm <= 0.0)
                    {
                        expect(false, modelName(model) + " res " + String(res, 1).str() + ": no feature found");
                        continue;
                    }
                    ++checked;
                    const double err = std::abs(fa / fm - 1.0);
                    worst = std::max(worst, err);
                    if (err > 0.03)
                        expect(false, modelName(model) + " res " + String(res, 1).str() + ": feature at " +
                                          String(fa, 1).str() + " Hz in the response, " + String(fm, 1).str() +
                                          " Hz measured");
                }
            }
            std::printf("    feature frequency: %d curves, worst deviation %.2f %%\n", checked, worst * 100.0);
            expect(checked >= 24 && worst <= 0.03, "features within 3 %");
        }

        beginTest("the passive high-pass sits at -3 dB on the dial frequency; the low-pass at full resonance peaks on it");
        {
            std::vector<float> hz = {1000.0f};
            FilterSettings s;
            s.model = static_cast<int>(FilterModel::PassiveHighpass);
            s.cutoffHz = 1000.0f;
            s.resonance = 0.0f;
            const auto db = analytic(s, 0.0f, hz);
            expect(std::abs(db[0] + 3.01f) < 0.05f, "one section: " + String(db[0], 3).str() + " dB at the corner");
            s.model = static_cast<int>(FilterModel::Lowpass12);
            s.resonance = 0.95f;
            std::vector<float> fine;
            for (int i = 0; i < 200; ++i)
                fine.push_back(static_cast<float>(700.0 * std::pow(1.0 / 0.7 * 1.3, i / 199.0)));
            const auto curve = analytic(s, 0.0f, fine);
            size_t top = 0;
            for (size_t i = 1; i < curve.size(); ++i)
                if (curve[i] > curve[top])
                    top = i;
            expect(std::abs(fine[top] / 1000.0f - 1.0f) < 0.03f && curve[top] > 10.0f,
                   "peak " + String(fine[top], 1).str() + " Hz, " + String(curve[top], 1).str() + " dB");
        }

        beginTest("engine: filter and filterN publish response and response_mod as pulled 256-point views");
        {
            Engine e;
            e.prepare(kRate, 256);
            expect(e.views().find("filter", "response") >= 0 && e.views().find("filter", "response_mod") >= 0, "filter 1");
            expect(e.views().find("filter2", "response") >= 0 && e.views().find("filter2", "response_mod") >= 0, "filter 2");
            const int v = e.views().find("filter", "response");
            const auto& info = e.views().info(v);
            expect(info.kind == ViewKind::Vector && info.maxElements == kPoints && !info.live,
                   "pulled vector of 256 points");

            e.setParam("filter_type", static_cast<float>(FilterModel::LadderLowpass));
            e.setParam("filter_cutoff", 900.0f);
            e.setParam("filter_res", 0.5f);
            e.setParam("filter2_type", static_cast<float>(FilterModel::SvfOta));
            e.setParam("filter2_cutoff", 3000.0f);
            e.setParam("filter2_res", 0.4f);
            e.setParam("filter2_character", 0.5f);
            run(e, 4);

            FilterSettings s1;
            s1.model = static_cast<int>(FilterModel::LadderLowpass);
            s1.cutoffHz = 900.0f;
            s1.resonance = 0.5f;
            const auto expected1 = analytic(s1, 0.0f, grid);
            const auto got1 = readFull(e, v);
            expect(got1.size() == static_cast<size_t>(kPoints) && near(got1, expected1),
                   "filter response is the response of its parameters");
            FilterSettings s2;
            s2.model = static_cast<int>(FilterModel::SvfOta);
            s2.cutoffHz = 3000.0f;
            s2.resonance = 0.4f;
            s2.character = 0.5f;
            const auto got2 = readFull(e, e.views().find("filter2", "response"));
            expect(near(got2, analytic(s2, 0.0f, grid)), "filter 2 reads its own parameters");
            expect(got1 != got2, "the two filters differ");
            expect(near(readFull(e, e.views().find("filter", "response_mod")), got1),
                   "unmodulated: response_mod equals response");

            // The sequence number advances only when the content changes.
            std::uint64_t before = 0, same = 0, after = 0;
            std::vector<float> buffer(kPoints);
            e.views().read(v, buffer.data(), kPoints, &before);
            e.views().read(v, buffer.data(), kPoints, &same);
            e.setParam("filter_cutoff", 1400.0f);
            run(e, 2);
            e.views().read(v, buffer.data(), kPoints, &after);
            expect(before == same && after != before, "sequence follows the content");

            // Every model, including the nonlinear ones, gives finite values at the extremes of the controls.
            bool finite = true;
            for (int model = 0; model < static_cast<int>(FilterModel::Count); ++model)
                for (const float res : {0.0f, 1.0f})
                    for (const float cutoff : {20.0f, 20000.0f})
                    {
                        e.setParam("filter_type", static_cast<float>(model));
                        e.setParam("filter_res", res);
                        e.setParam("filter_cutoff", cutoff);
                        e.setParam("filter_drive", 1.0f);
                        run(e, 1);
                        for (const float x : readFull(e, v))
                            finite = finite && std::isfinite(x);
                    }
            expect(finite, "finite at the extremes");
        }

        beginTest("engine: response_mod follows the modulated values; reading while the audio thread renders is safe");
        {
            Engine e;
            e.prepare(kRate, 256);
            e.setParam("filter_type", static_cast<float>(FilterModel::TransistorLadder));
            e.setParam("filter_cutoff", 2000.0f);
            e.setParam("filter_res", 0.3f);
            run(e, 2);
            const int plain = e.views().find("filter", "response");
            const int mod = e.views().find("filter", "response_mod");
            const auto& synth = e.coreEngine().engine();
            FilterSettings s;
            s.model = static_cast<int>(synth.modulatedValue(ParameterMap::indexOf("filter_type")));
            s.cutoffHz = synth.modulatedValue(ParameterMap::indexOf("filter_cutoff"));
            s.resonance = synth.modulatedValue(ParameterMap::indexOf("filter_res"));
            expect(near(readFull(e, mod), analytic(s, synth.modulatedValue(ParameterMap::indexOf("filter_drive")), grid)),
                   "response_mod is computed from the .mod values");

            std::atomic<bool> stop{false};
            std::atomic<int> reads{0};
            std::thread reader(
                [&]
                {
                    std::vector<float> buffer(kPoints);
                    while (!stop.load())
                    {
                        e.views().read(plain, buffer.data(), kPoints, nullptr);
                        e.views().read(mod, buffer.data(), kPoints, nullptr);
                        reads.fetch_add(1);
                    }
                });
            for (int i = 0; i < 40; ++i)
            {
                e.setParam("filter_cutoff", 300.0f + 40.0f * static_cast<float>(i));
                e.setParam("filter_type", static_cast<float>(i % 25));
                run(e, 4);
            }
            stop = true;
            reader.join();
            expect(reads.load() > 0, "the reader ran alongside the render");
        }
    }
};

static FilterResponseTests filterResponseTests;
