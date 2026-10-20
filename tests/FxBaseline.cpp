// SPDX-License-Identifier: MIT
// Baseline measurement of the rack effects that remain (limiter, multiband, ring / shift, EQ cramping, delay line) against the quality bar of docs/plans/ENGINE-FX-PLAN.md (section 2),
// with the primitives of FxMeasure.h. Prints Markdown tables (docs/MEASUREMENTS.md, "Effects", embeds the output).
//
//   crucible_engine_fx_baseline [all|peak|eq|dynamics|bugs|reverb|cpu]   print the tables
//   crucible_engine_fx_baseline check                                            guard: exit 1 when a row leaves its band
//
// Every row has the number the plan states and the number measured now; `check` fails when they differ by more than
// 0.5 dB (levels) or 5 % (times, ratios). CPU rows depend on the machine and are printed, not checked. Everything is
// rendered offline; nothing is played.

#include "FxMeasure.h"

#include <crucible/DelayLine.h>
#include <crucible/Fx.h>
#include <crucible/FxExtra.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace crucible;
using namespace crucible::measure;

namespace
{
constexpr double kRate = 48000.0;

// Runs a stereo effect on a mono signal (the same signal on both channels) in blocks of 32 and returns the left channel.
template <class FxType, class Params> Signal runFx(FxType& fx, const Params& p, const Signal& in, int block = 32)
{
    std::vector<float> l(in.begin(), in.end()), r = l;
    for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
    {
        const int n = static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - pos));
        fx.process(l.data() + pos, r.data() + pos, n, p);
    }
    return Signal(l.begin(), l.end());
}

struct Row
{
    std::string group, what, unit;
    double plan; // the number the plan states (NaN: none)
    double measured;
    bool relative; // compare within 5 % instead of 0.5 (dB)
    bool checked;
    int bound = 0; // 1: measured must be at least `limit`, 2: at most `limit` (rows that depend on the exact level)
    double limit = 0.0;
};

std::vector<Row> g_rows;

void add(const char* group, const std::string& what, const char* unit, double plan, double measured, bool relative = false,
         bool checked = true)
{
    g_rows.push_back({group, what, unit, plan, measured, relative, checked, 0, 0.0});
}

// A row whose number depends on the exact input level (folding and hard clipping at low drive alias chaotically): the
// plan quotes one value; the guard is a bound that every level in the neighbourhood satisfies.
void addBound(const char* group, const std::string& what, const char* unit, double plan, double measured, int bound,
              double limit)
{
    g_rows.push_back({group, what, unit, plan, measured, false, true, bound, limit});
}

bool rowOk(const Row& r)
{
    if (r.bound == 1)
        return r.measured >= r.limit;
    if (r.bound == 2)
        return r.measured <= r.limit;
    if (!r.checked || std::isnan(r.plan))
        return true;
    return r.relative ? std::abs(r.measured / r.plan - 1.0) <= 0.05 : std::abs(r.measured - r.plan) <= 0.5;
}

// ---------------------------------------------------------------------------------------------------------------
// True peak of the limiter

void peaks()
{
    const char* g = "Limiter (ceiling -0.3 dB, fs/4 sine at 45 degrees)";
    FxLimiter lim;
    lim.prepare(kRate);
    FxLimiterParams p;
    p.on = true;
    p.ceilingDb = -0.3f;
    Signal x(static_cast<size_t>(0.5 * kRate));
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = 2.0 * std::sin(kPi / 2.0 * static_cast<double>(i) + kPi / 4.0);
    const Signal y = runFx(lim, p, x);
    const size_t from = y.size() / 2;
    double samplePk = 0.0;
    for (size_t i = from; i < y.size(); ++i)
        samplePk = std::max(samplePk, std::abs(y[i]));
    const double tp = truePeak(y.data() + from, y.size() - from, 8, 64);
    add(g, "sample peak", "dBFS", -0.3, amplitudeToDb(samplePk));
    add(g, "true peak", "dBTP", 2.71, amplitudeToDb(tp));
    auto render = [&](const Signal& in)
    {
        FxLimiter l;
        l.prepare(kRate);
        FxLimiterParams q;
        q.on = true;
        q.ceilingDb = -0.3f;
        return runFx(l, q, in);
    };
    const auto h = thdCurve(render, kRate, 60.0, {6.0}, 0.5, 0.5);
    add("Limiter THD", "60 Hz sine at +6 dBFS (ceiling -0.3 dB)", "dB", -39.0, h[0].thdDb);
}

// ---------------------------------------------------------------------------------------------------------------
// EQ cramping

double analogBellDb(double f, double f0, double gainDb, double q)
{
    const double a = std::pow(10.0, gainDb / 40.0), w = f / f0;
    const Cx s(0.0, w);
    const Cx h = (s * s + s * (a / q) + 1.0) / (s * s + s / (a * q) + 1.0);
    return amplitudeToDb(std::abs(h));
}

void eq()
{
    const char* g = "EQ cramping (RBJ bells, +12 dB, Q 1, 48 kHz): response at 20 kHz";
    for (const double f0 : {16000.0, 12000.0})
    {
        const BiquadCoefs c = BiquadCoefs::design(BiquadType::Peak, f0, 12.0, 1.0, kRate);
        const double digital = c.magnitudeDb(20000.0, kRate), analog = analogBellDb(20000.0, f0, 12.0, 1.0);
        char label[96];
        std::snprintf(label, sizeof label, "%.0f kHz bell: digital minus analog", f0 / 1000.0);
        add(g, label, "dB", f0 > 14000.0 ? -6.2 : -4.5, digital - analog);
        if (f0 > 14000.0)
        {
            add(g, "16 kHz bell: digital", "dB", 3.42, digital);
            add(g, "16 kHz bell: analog prototype", "dB", 9.65, analog);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Multiband band sum (BFX2)

double bandSumWorstDb(float low, float high, double& atHz)
{
    double worst = 0.0;
    atHz = 0.0;
    auto render = [&](const Signal& x)
    {
        FxMultiband m;
        m.prepare(kRate);
        FxMultibandParams p;
        p.on = true;
        p.lowCrossHz = low;
        p.highCrossHz = high;
        p.ratio = 1.0f;
        p.thresholdDb = {0.0f, 0.0f, 0.0f};
        return runFx(m, p, x);
    };
    for (const double f : logSpacedFrequencies(50.0, 12000.0, 24))
    {
        const double g = staticCurve(render, kRate, {-30.0}, f, 0.5, 0.2)[0].gainDb;
        if (std::abs(g) > std::abs(worst))
        {
            worst = g;
            atHz = f;
        }
    }
    return worst;
}

void multiband()
{
    const char* g = "Multiband band sum with the dynamics neutral (BFX2): worst deviation";
    double at = 0.0;
    double v = bandSumWorstDb(500.0f, 750.0f, at);
    add(g, "crossovers 500 / 750 Hz", "dB", 0.0, v);
    add(g, "crossovers 500 / 750 Hz: at", "Hz", std::nan(""), at, false, false);
    v = bandSumWorstDb(300.0f, 1200.0f, at);
    add(g, "crossovers 300 / 1200 Hz", "dB", 0.0, v);
    add(g, "crossovers 300 / 1200 Hz: at", "Hz", std::nan(""), at, false, false);
    v = bandSumWorstDb(200.0f, 3000.0f, at);
    add(g, "defaults 200 / 3000 Hz", "dB", 0.0, v);
}

// ---------------------------------------------------------------------------------------------------------------
// Bugs that are audible as a step or a replay

void bugs()
{
    {
        // BFX5: a lookahead change clears the limiter.
        FxLimiter lim;
        lim.prepare(kRate);
        FxLimiterParams p;
        p.on = true;
        p.lookaheadMs = 1.5f;
        const size_t n = static_cast<size_t>(1.0 * kRate), change = n / 2;
        std::vector<float> l(n), r;
        for (size_t i = 0; i < n; ++i)
            l[i] = static_cast<float>(0.4 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(i) / kRate));
        r = l;
        for (size_t pos = 0; pos < n; pos += 32)
        {
            if (pos == change - change % 32)
                p.lookaheadMs = 3.0f;
            lim.process(l.data() + pos, r.data() + pos, 32, p);
        }
        const ClickResult c = clickTest(Signal(l.begin(), l.end()), 1000.0, kRate, 4000);
        add("Bugs found by the baseline", "BFX5 limiter lookahead 1.5 -> 3 ms: largest step", "dB", std::nan(""), c.stepDb,
            false, false);
    }
    {
        // BFX6: the multiband crossover coefficients jump.
        FxMultiband m;
        m.prepare(kRate);
        FxMultibandParams p;
        p.on = true;
        p.ratio = 1.0f;
        p.thresholdDb = {0.0f, 0.0f, 0.0f};
        p.lowCrossHz = 400.0f;
        const size_t n = static_cast<size_t>(1.0 * kRate), change = n / 2;
        std::vector<float> l(n), r;
        for (size_t i = 0; i < n; ++i)
            l[i] = static_cast<float>(0.4 * std::sin(2.0 * kPi * 500.0 * static_cast<double>(i) / kRate));
        r = l;
        for (size_t pos = 0; pos < n; pos += 32)
        {
            if (pos == change - change % 32)
                p.lowCrossHz = 800.0f;
            m.process(l.data() + pos, r.data() + pos, 32, p);
        }
        const ClickResult c = clickTest(Signal(l.begin(), l.end()), 500.0, kRate, 4000);
        add("Bugs found by the baseline", "BFX6 multiband crossover 400 -> 800 Hz: largest step", "dB", std::nan(""),
            c.stepDb, false, false);
    }
    for (const double rate : {48000.0, 192000.0})
    {
        // BFX7: frequency shifter image rejection of a 30 Hz tone shifted up by 100 Hz.
        FxRingShift s;
        s.prepare(rate);
        FxRingShiftParams p;
        p.on = true;
        p.mix = 1.0f;
        p.mode = 1;
        p.freqHz = 100.0f;
        const Signal x = sineWave(static_cast<size_t>(4.0 * rate), 30.0, rate, 0.3);
        const Signal y = runFx(s, p, x);
        const size_t from = static_cast<size_t>(2.0 * rate), n = static_cast<size_t>(2.0 * rate);
        const double wanted = toneAmplitude(y.data() + from, n, 130.0, rate),
                     image = toneAmplitude(y.data() + from, n, 70.0, rate);
        char label[128];
        std::snprintf(label, sizeof label,
                      "BFX7 frequency shifter, 30 Hz up by 100 Hz at %.0f kHz: image against the wanted tone",
                      rate / 1000.0);
        add("Bugs found by the baseline", label, "dB", std::nan(""), amplitudeToDb(image / wanted), false, false);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Delay line

void delayRead()
{
    // Half-sample read of the cubic delay line: a sine goes in, the gain at the half-sample position is read.
    for (const double hz : {16000.0, 20000.0})
    {
        DelayLine line;
        line.allocate(64);
        const size_t n = 8192;
        Signal y(n);
        for (size_t i = 0; i < n; ++i)
        {
            line.write(static_cast<float>(std::sin(2.0 * kPi * hz * static_cast<double>(i) / kRate)));
            y[i] = line.read(10.5f);
        }
        const double a = toneAmplitude(y.data() + 1024, 4096, hz, kRate);
        char label[64];
        std::snprintf(label, sizeof label, "cubic fractional read at half a sample: gain at %.0f kHz", hz / 1000.0);
        add("Delay line", label, "dB", hz < 18000.0 ? -3.25 : -8.4, amplitudeToDb(a));
    }
}

// ---------------------------------------------------------------------------------------------------------------
// CPU: percent of one core per stereo instance at 48 kHz (machine dependent, printed only)

template <class FxType, class Params> double cpuPercent(FxType& fx, const Params& p)
{
    const size_t n = static_cast<size_t>(10.0 * kRate), warm = 48000;
    Random rng(1);
    std::vector<float> l(n), r;
    for (auto& v : l)
        v = static_cast<float>(0.25 * rng.gaussian());
    r = l;
    for (size_t pos = 0; pos < warm; pos += 32)
        fx.process(l.data() + pos, r.data() + pos, 32, p);
    const auto t0 = std::chrono::steady_clock::now();
    for (size_t pos = warm; pos + 32 <= n; pos += 32)
        fx.process(l.data() + pos, r.data() + pos, 32, p);
    const auto t1 = std::chrono::steady_clock::now();
    return 100.0 * std::chrono::duration<double>(t1 - t0).count() / (static_cast<double>(n - warm) / kRate);
}

void cpu()
{
    const char* g = "CPU per instance (percent of one core, stereo, 48 kHz; machine dependent, not checked)";
    {
        FxLimiter f;
        f.prepare(kRate);
        FxLimiterParams p;
        p.on = true;
        add(g, "limiter", "%", 0.33, cpuPercent(f, p), true, false);
    }
    {
        FxMultiband f;
        f.prepare(kRate);
        FxMultibandParams p;
        p.on = true;
        add(g, "multiband", "%", 0.18, cpuPercent(f, p), true, false);
    }
}

// ---------------------------------------------------------------------------------------------------------------
void print()
{
    std::string group;
    for (const Row& r : g_rows)
    {
        if (r.group != group)
        {
            group = r.group;
            std::printf("\n**%s**\n\n| quantity | unit | plan | measured |\n| --- | --- | --- | --- |\n", group.c_str());
        }
        char plan[32] = "-";
        if (!std::isnan(r.plan))
            std::snprintf(plan, sizeof plan, "%.2f", r.plan);
        std::printf("| %s | %s | %s | %.2f%s |\n", r.what.c_str(), r.unit.c_str(), plan, r.measured,
                    rowOk(r) ? "" : " (off)");
    }
}
} // namespace

int main(int argc, char** argv)
{
    const std::string mode = argc > 1 ? argv[1] : "all";
    const bool all = mode == "all" || mode == "check";
    if (all || mode == "bugs")
        bugs();
    if (all || mode == "peak")
        peaks();
    if (all || mode == "eq")
        eq();
    if (all || mode == "dynamics")
    {
        multiband();
    }
    if (all || mode == "reverb")
        delayRead();
    if (mode == "all" || mode == "cpu")
        cpu();
    print();
    if (mode == "check")
    {
        int off = 0;
        for (const Row& r : g_rows)
            if (!rowOk(r))
            {
                std::printf("OFF: %s / %s: plan %.2f, measured %.2f\n", r.group.c_str(), r.what.c_str(), r.plan, r.measured);
                ++off;
            }
        std::printf("%s\n", off == 0 ? "all checked rows within 0.5 dB / 5 %" : "baseline drifted");
        return off == 0 ? 0 : 1;
    }
    return 0;
}
