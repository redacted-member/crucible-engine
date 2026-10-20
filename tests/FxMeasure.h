// SPDX-License-Identifier: MIT
#pragma once

// Measurement primitives for effects (docs/plans/ENGINE-FX-PLAN.md, section 5.1), built on AnalogMeasure.h. Header
// only, double precision, tests and offline tools only; nothing here runs on the audio thread. Each primitive has a
// self-test with a synthetic truth in tests/FxMeasureTests.cpp.
//
//   dynamics   staticCurve, stepTimes, releaseFamily (all through gainTrace)
//   distortion thdCurve, harmonicCurve, imd (SMPTE and CCIF), aliasAtDrive
//   alignment  nullTest, measureLatency
//   smoothness clickTest, zipperTest (a step estimator that ignores the carrier and any smooth modulation)
//   rooms      energyDecay (Schroeder curve, EDT / T20 / T30 per octave band, curvature, echo density, IACC,
//              tail spectral flatness)
//   modulation modulationDepth, wowFlutter
//   loudness   measureLoudness (ITU-R BS.1770-4 K-weighting and gating, momentary / short-term / integrated, range)
//   stereo     stereoStats, monoSumPerBand
//
// A processor under test is a renderer: a callable Signal(const Signal& in) that makes a fresh instance, runs the whole
// input through it and returns the output (zipperTest also hands over a per-sample automation signal). Level
// arguments are dB relative to a full-scale sine's peak.

#include "AnalogMeasure.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace crucible::measure
{

// ---------------------------------------------------------------------------------------------------------------
// Signal helpers

struct Segment
{
    double levelDb;
    double seconds;
};

// Phase-continuous sine whose amplitude jumps from segment to segment; starts[i] is the first sample of segment i.
inline Signal programSine(const std::vector<Segment>& program, double hz, double rate, std::vector<size_t>* starts = nullptr)
{
    Signal x;
    if (starts != nullptr)
        starts->clear();
    size_t n = 0;
    for (const Segment& s : program)
    {
        if (starts != nullptr)
            starts->push_back(n);
        const size_t count = static_cast<size_t>(std::llround(s.seconds * rate));
        const double a = dbToAmplitude(s.levelDb);
        for (size_t i = 0; i < count; ++i, ++n)
            x.push_back(a * std::sin(2.0 * kPi * hz * static_cast<double>(n) / rate));
    }
    return x;
}

// Amplitude of the component at `hz`, from a window of whole periods ending anywhere (Blackman-Harris DTFT).
inline double toneAmplitude(const double* x, size_t n, double hz, double rate)
{
    const Signal w = makeWindow(Window::BlackmanHarris, n);
    double sumW = 0.0;
    for (const double v : w)
        sumW += v;
    return 2.0 * std::abs(windowedDtft(x, w.data(), n, hz, rate)) / sumW;
}

// Envelope of a tone: quadrature demodulation followed by a centred box of `periods` whole periods (so no group
// delay). hz should divide the rate into a whole number of samples per period (rate / 12 does).
inline Signal amplitudeTrace(const Signal& x, double hz, double rate, int periods = 1)
{
    const size_t n = x.size();
    const size_t w = std::max<size_t>(2, static_cast<size_t>(std::llround(periods * rate / hz)));
    std::vector<double> re(n + 1, 0.0), im(n + 1, 0.0); // prefix sums
    for (size_t i = 0; i < n; ++i)
    {
        const double ph = 2.0 * kPi * hz * static_cast<double>(i) / rate;
        re[i + 1] = re[i] + x[i] * std::cos(ph);
        im[i + 1] = im[i] - x[i] * std::sin(ph);
    }
    Signal a(n, 0.0);
    for (size_t i = 0; i < n; ++i)
    {
        size_t lo = i >= w / 2 ? i - w / 2 : 0;
        size_t hi = std::min(n, lo + w);
        lo = hi >= w ? hi - w : 0; // keep a full window at the edges
        const double cr = (re[hi] - re[lo]) / static_cast<double>(hi - lo);
        const double ci = (im[hi] - im[lo]) / static_cast<double>(hi - lo);
        a[i] = 2.0 * std::hypot(cr, ci);
    }
    return a;
}

// Gain of the output against the input, in dB per sample, from their tone envelopes.
inline Signal gainTrace(const Signal& in, const Signal& out, double hz, double rate)
{
    const Signal ai = amplitudeTrace(in, hz, rate), ao = amplitudeTrace(out, hz, rate);
    Signal g(std::min(ai.size(), ao.size()));
    for (size_t i = 0; i < g.size(); ++i)
        g[i] = amplitudeToDb(ao[i]) - amplitudeToDb(ai[i]);
    return g;
}

// ---------------------------------------------------------------------------------------------------------------
// Dynamics

struct StaticPoint
{
    double inDb, outDb, gainDb;
};

// Gain against input level: stepped sines at `hz`, the output's fundamental read after the settle time.
template <class Render>
std::vector<StaticPoint> staticCurve(Render&& render, double rate, const std::vector<double>& levelsDb, double hz = 1000.0,
                                     double settleSeconds = 0.6, double measureSeconds = 0.2)
{
    std::vector<StaticPoint> curve;
    const size_t settle = static_cast<size_t>(settleSeconds * rate), count = static_cast<size_t>(measureSeconds * rate);
    for (const double level : levelsDb)
    {
        const Signal in = sineWave(settle + count, hz, rate, dbToAmplitude(level));
        const Signal out = render(in);
        const double a = toneAmplitude(out.data() + settle, count, hz, rate);
        curve.push_back({level, amplitudeToDb(a), amplitudeToDb(a) - level});
    }
    return curve;
}

struct StepMetrics
{
    double startDb = 0.0, endDb = 0.0; // gain before the event and settled gain after it
    double t63Ms = -1.0;               // time from the event to 63.2 % of the change
    double t10Ms = -1.0, t90Ms = -1.0;
    double riseMs() const { return t90Ms - t10Ms; } // 10 - 90 % time
    double overshootDb = 0.0;                       // excursion beyond the settled value, in the direction of change
    bool valid() const { return t63Ms >= 0.0; }
};

// Time constants of the gain trace after an event at sample `event` (the level changed there).
inline StepMetrics stepMetrics(const Signal& gainDb, double rate, size_t event, size_t end, double hz)
{
    StepMetrics m;
    const size_t w = static_cast<size_t>(std::llround(rate / hz));
    end = std::min(end, gainDb.size());
    if (event < 2 * w || end <= event + 8 * w)
        return m;
    m.startDb = gainDb[event - w];
    const size_t tail = std::max<size_t>(w, (end - event) / 20);
    double s = 0.0;
    for (size_t i = end - tail; i < end; ++i)
        s += gainDb[i];
    m.endDb = s / static_cast<double>(tail);
    const double total = m.endDb - m.startDb;
    if (std::abs(total) < 0.05)
        return m;
    const double sign = total > 0 ? 1.0 : -1.0;
    auto crossing = [&](double fraction)
    {
        const double target = m.startDb + fraction * total;
        for (size_t i = event; i + 1 < end; ++i)
            if (sign * (gainDb[i + 1] - target) >= 0.0 && sign * (gainDb[i] - target) < 0.0)
            {
                const double frac = (target - gainDb[i]) / (gainDb[i + 1] - gainDb[i]);
                return (static_cast<double>(i - event) + frac) / rate * 1000.0;
            }
        return -1.0;
    };
    m.t63Ms = crossing(1.0 - std::exp(-1.0));
    m.t10Ms = crossing(0.1);
    m.t90Ms = crossing(0.9);
    double over = 0.0;
    for (size_t i = event; i < end; ++i)
        over = std::max(over, sign * (gainDb[i] - m.endDb));
    m.overshootDb = over;
    return m;
}

// Response to a level step from fromDb to toDb (attack when the level rises, release when it falls).
template <class Render>
StepMetrics stepTimes(Render&& render, double rate, double fromDb, double toDb, double hz = 0.0, double preSeconds = 1.0,
                      double postSeconds = 1.5)
{
    if (hz <= 0.0)
        hz = rate / 12.0;
    std::vector<size_t> starts;
    const Signal in = programSine({{fromDb, preSeconds}, {toDb, postSeconds}}, hz, rate, &starts);
    const Signal out = render(in);
    const Signal g = gainTrace(in, out, hz, rate);
    return stepMetrics(g, rate, starts[1], g.size(), hz);
}

struct ReleasePoint
{
    double burstSeconds, burstDb;
    StepMetrics release; // recovery after the burst ends
};

// Release time against burst length and depth: base level, a burst, base level again.
template <class Render>
std::vector<ReleasePoint> releaseFamily(Render&& render, double rate, double baseDb, const std::vector<double>& burstDb,
                                        const std::vector<double>& burstSeconds, double hz = 0.0, double preSeconds = 1.0,
                                        double postSeconds = 3.0)
{
    if (hz <= 0.0)
        hz = rate / 12.0;
    std::vector<ReleasePoint> out;
    for (const double depth : burstDb)
        for (const double len : burstSeconds)
        {
            std::vector<size_t> starts;
            const Signal in = programSine({{baseDb, preSeconds}, {depth, len}, {baseDb, postSeconds}}, hz, rate, &starts);
            const Signal y = render(in);
            const Signal g = gainTrace(in, y, hz, rate);
            out.push_back({len, depth, stepMetrics(g, rate, starts[2], g.size(), hz)});
        }
    return out;
}

// ---------------------------------------------------------------------------------------------------------------
// Distortion

struct HarmonicPoint
{
    double x = 0.0; // level in dB (thdCurve) or frequency in Hz (harmonicCurve)
    double fundamentalGainDb = 0.0;
    double thdDb = -300.0, thdPlusNoiseDb = -300.0;
    double h2Db = -300.0, h3Db = -300.0, h4Db = -300.0, h5Db = -300.0; // relative to the fundamental
};

template <class Render>
HarmonicPoint harmonicsAt(Render&& render, double rate, double hz, double levelDb, double settleSeconds,
                          double measureSeconds)
{
    const size_t settle = static_cast<size_t>(settleSeconds * rate), count = static_cast<size_t>(measureSeconds * rate);
    const Signal out = render(sineWave(settle + count, hz, rate, dbToAmplitude(levelDb)));
    const HarmonicProfile p = harmonicProfile(out.data() + settle, count, rate, hz, 10);
    HarmonicPoint h;
    h.fundamentalGainDb = amplitudeToDb(p.fundamentalAmplitude) - levelDb;
    h.thdDb = p.thdDb;
    h.thdPlusNoiseDb = p.thdPlusNoiseDb;
    auto at = [&](int k) { return p.count() >= k ? p.harmonicDb(k) : -300.0; };
    h.h2Db = at(2);
    h.h3Db = at(3);
    h.h4Db = at(4);
    h.h5Db = at(5);
    return h;
}

// THD and H2..H5 against level at one frequency.
template <class Render>
std::vector<HarmonicPoint> thdCurve(Render&& render, double rate, double hz, const std::vector<double>& levelsDb,
                                    double settleSeconds = 0.3, double measureSeconds = 0.3)
{
    std::vector<HarmonicPoint> r;
    for (const double level : levelsDb)
    {
        HarmonicPoint h = harmonicsAt(render, rate, hz, level, settleSeconds, measureSeconds);
        h.x = level;
        r.push_back(h);
    }
    return r;
}

// THD and H2..H5 against frequency at one level.
template <class Render>
std::vector<HarmonicPoint> harmonicCurve(Render&& render, double rate, double levelDb, const std::vector<double>& freqsHz,
                                         double settleSeconds = 0.3, double measureSeconds = 0.3)
{
    std::vector<HarmonicPoint> r;
    for (const double f : freqsHz)
    {
        HarmonicPoint h = harmonicsAt(render, rate, f, levelDb, settleSeconds, measureSeconds);
        h.x = f;
        r.push_back(h);
    }
    return r;
}

struct ImdLine
{
    double hz;
    double db; // against the reference tone
};

struct ImdResult
{
    double totalDb = -300.0; // root-sum-square of the lines against the reference (SMPTE: f2; CCIF: f1 + f2)
    std::vector<ImdLine> lines;
};

namespace detail
{
inline double lineAmplitude(const Signal& y, const Signal& w, double sumW, double hz, double rate)
{
    return 2.0 * std::abs(windowedDtft(y.data(), w.data(), y.size(), hz, rate)) / sumW;
}
} // namespace detail

// SMPTE / DIN: f1 (large) and f2 (small) at ratio:1 amplitudes, total peak at levelDb. The lines are f2 +- k f1.
template <class Render>
ImdResult imdSmpte(Render&& render, double rate, double levelDb = -6.0, double f1 = 60.0, double f2 = 7000.0,
                   double ratio = 4.0, int orders = 4, double settleSeconds = 0.3, double measureSeconds = 0.5)
{
    const size_t settle = static_cast<size_t>(settleSeconds * rate), count = static_cast<size_t>(measureSeconds * rate);
    const double peak = dbToAmplitude(levelDb), a1 = peak * ratio / (ratio + 1.0), a2 = peak / (ratio + 1.0);
    Signal in(settle + count);
    for (size_t i = 0; i < in.size(); ++i)
    {
        const double t = static_cast<double>(i) / rate;
        in[i] = a1 * std::sin(2.0 * kPi * f1 * t) + a2 * std::sin(2.0 * kPi * f2 * t + 0.7);
    }
    const Signal out = render(in);
    const Signal y(out.begin() + static_cast<long>(settle), out.end());
    const Signal w = makeWindow(Window::Harris7, y.size());
    double sumW = 0.0;
    for (const double v : w)
        sumW += v;
    const double ref = detail::lineAmplitude(y, w, sumW, f2, rate);
    ImdResult r;
    double power = 0.0;
    for (int k = 1; k <= orders; ++k)
        for (const double sign : {-1.0, 1.0})
        {
            const double hz = f2 + sign * k * f1;
            if (hz <= 0.0 || hz >= 0.5 * rate)
                continue;
            const double a = detail::lineAmplitude(y, w, sumW, hz, rate);
            r.lines.push_back({hz, amplitudeToDb(a / ref)});
            power += a * a;
        }
    r.totalDb = amplitudeToDb(std::sqrt(power) / ref);
    return r;
}

// CCIF twin tone: equal tones at f1 and f2 (peak of the sum at levelDb). The lines are the difference and
// low-order products below Nyquist; the total is against the sum of the two tone amplitudes.
template <class Render>
ImdResult imdCcif(Render&& render, double rate, double levelDb = -6.0, double f1 = 19000.0, double f2 = 20000.0,
                  double settleSeconds = 0.3, double measureSeconds = 0.5)
{
    const size_t settle = static_cast<size_t>(settleSeconds * rate), count = static_cast<size_t>(measureSeconds * rate);
    const double a = 0.5 * dbToAmplitude(levelDb);
    Signal in(settle + count);
    for (size_t i = 0; i < in.size(); ++i)
    {
        const double t = static_cast<double>(i) / rate;
        in[i] = a * std::sin(2.0 * kPi * f1 * t) + a * std::sin(2.0 * kPi * f2 * t + 0.4);
    }
    const Signal out = render(in);
    const Signal y(out.begin() + static_cast<long>(settle), out.end());
    const Signal w = makeWindow(Window::Harris7, y.size());
    double sumW = 0.0;
    for (const double v : w)
        sumW += v;
    const double tones = detail::lineAmplitude(y, w, sumW, f1, rate) + detail::lineAmplitude(y, w, sumW, f2, rate);
    ImdResult r;
    double power = 0.0;
    const double candidates[] = {f2 - f1, 2 * f1 - f2, 2 * f2 - f1, 3 * f1 - 2 * f2, 3 * f2 - 2 * f1};
    for (const double hz : candidates)
    {
        if (hz <= 0.0 || hz >= 0.5 * rate)
            continue;
        const double l = detail::lineAmplitude(y, w, sumW, hz, rate);
        r.lines.push_back({hz, amplitudeToDb(l / tones)});
        power += l * l;
    }
    r.totalDb = amplitudeToDb(std::sqrt(power) / tones);
    return r;
}

struct AliasPoint
{
    double levelDb = 0.0;
    double aliasDb = -300.0;     // power outside the harmonics below 20 kHz against the fundamental
    double worstSpurDb = -300.0; // strongest single non-harmonic bin against the fundamental
    double worstSpurHz = 0.0;
    double thdDb = -300.0;
};

// Alias power and worst spur of a tone at `hz` for each level (P1 measureAliasing on the settled output).
template <class Render>
std::vector<AliasPoint> aliasAtDrive(Render&& render, double rate, double hz, const std::vector<double>& levelsDb,
                                     double settleSeconds = 0.3, size_t window = 1 << 15)
{
    std::vector<AliasPoint> r;
    const size_t settle = static_cast<size_t>(settleSeconds * rate);
    for (const double level : levelsDb)
    {
        const Signal out = render(sineWave(settle + window, hz, rate, dbToAmplitude(level)));
        const AliasResult a = measureAliasing(out.data() + settle, window, rate, hz);
        const HarmonicProfile p = harmonicProfile(out.data() + settle, window, rate, hz, 10);
        r.push_back({level, a.aliasDb, a.worstSpurDb, a.worstSpurHz, p.thdDb});
    }
    return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Alignment: delay, null, latency

namespace detail
{
inline size_t pow2AtLeast(size_t n)
{
    size_t p = 1;
    while (p < n)
        p <<= 1;
    return p;
}
} // namespace detail

// x delayed by `delay` samples (fractions by a phase shift of the zero-padded spectrum; exact for band-limited data
// away from the ends). Samples before the start read zero. With circular set, a fraction is applied to the signal as
// one period of a periodic signal (n a power of two), which is exact for bin-centred multisines.
inline Signal delaySignal(const Signal& x, double delay, bool circular = false)
{
    const size_t n = x.size();
    const double whole = std::round(delay);
    if (std::abs(delay - whole) < 1.0e-12)
    {
        Signal y(n, 0.0);
        const long d = static_cast<long>(whole);
        for (size_t i = 0; i < n; ++i)
        {
            const long j = static_cast<long>(i) - d;
            if (j >= 0 && j < static_cast<long>(n))
                y[i] = x[static_cast<size_t>(j)];
        }
        return y;
    }
    const size_t size = circular ? n : detail::pow2AtLeast(2 * n + 64);
    std::vector<Cx> a(size, Cx(0.0, 0.0));
    for (size_t i = 0; i < n; ++i)
        a[i] = x[i];
    fft(a);
    for (size_t k = 0; k < size; ++k)
    {
        const double f = k <= size / 2 ? static_cast<double>(k) : static_cast<double>(k) - static_cast<double>(size);
        a[k] *= std::polar(1.0, -2.0 * kPi * f * delay / static_cast<double>(size));
    }
    a[size / 2] = Cx(0.0, 0.0);
    fft(a, true);
    Signal y(n);
    for (size_t i = 0; i < n; ++i)
        y[i] = a[i].real();
    return y;
}

// Residual of `test` against `reference` delayed by `delaySamples`, in dB relative to the reference's RMS, over the
// interior (guard samples are skipped at both ends). A pure delay reads below -150 dB.
inline double nullTest(const Signal& reference, const Signal& test, double delaySamples, size_t guard = 0,
                       bool circular = false)
{
    const Signal ref = delaySignal(reference, delaySamples, circular);
    const size_t n = std::min(ref.size(), test.size());
    double num = 0.0, den = 0.0;
    for (size_t i = guard; i + guard < n; ++i)
    {
        const double d = test[i] - ref[i];
        num += d * d;
        den += ref[i] * ref[i];
    }
    return powerToDb(num / std::max(den, 1.0e-300));
}

// Delay of `out` against `in` in samples. Coarse lag from the cross-correlation peak, then a magnitude-weighted
// least-squares fit of the cross-spectrum phase slope between loFraction and hiFraction of Nyquist, which is exact
// for a pure (fractional) delay and gives the average delay of a system whose delay varies with frequency.
inline double measureLatency(const Signal& in, const Signal& out, double loFraction = 0.02, double hiFraction = 0.4)
{
    const size_t n = std::min(in.size(), out.size());
    const size_t size = detail::pow2AtLeast(2 * n);
    std::vector<Cx> x(size, Cx(0, 0)), y(size, Cx(0, 0));
    for (size_t i = 0; i < n; ++i)
    {
        x[i] = in[i];
        y[i] = out[i];
    }
    fft(x);
    fft(y);
    std::vector<Cx> c(size);
    for (size_t k = 0; k < size; ++k)
        c[k] = y[k] * std::conj(x[k]);
    std::vector<Cx> r = c;
    fft(r, true);
    size_t best = 0;
    double bestV = -1.0;
    for (size_t lag = 0; lag < n / 2; ++lag)
        if (std::abs(r[lag].real()) > bestV)
        {
            bestV = std::abs(r[lag].real());
            best = lag;
        }
    double num = 0.0, den = 0.0;
    for (size_t k = 1; k < size / 2; ++k)
    {
        const double w = 2.0 * kPi * static_cast<double>(k) / static_cast<double>(size); // rad / sample
        const double fraction = w / kPi;
        if (fraction < loFraction || fraction > hiFraction)
            continue;
        const Cx v = c[k] * std::polar(1.0, w * static_cast<double>(best));
        const double weight = std::norm(v);
        num += weight * w * std::arg(v);
        den += weight * w * w;
    }
    return static_cast<double>(best) - (den > 0.0 ? num / den : 0.0);
}

// ---------------------------------------------------------------------------------------------------------------
// Smoothness: click and zipper detection

namespace detail
{
// Step estimator for a signal that is a carrier at `hz` with slowly varying amplitude and phase. The operator
// D2 = (1 - 2 cos w z^-1 + z^-2)^2 annihilates the carrier and any linearly varying amplitude. A step that adds a
// sinusoid Re(S e^{jwi}) from sample n0 on (i counted from n0) appears in D2's output as Re(S c_j) at n0 + j for
// j = 0..3, with c_j fixed by w. Its real part u_j = Re c_j is the response to a pure jump of the sample value (S real:
// the added sinusoid starts at S); the imaginary part is the response to a jump of slope only, which is smaller by a
// factor of about w. The estimate is the least-squares height of the value jump, d = sum(o_j u_j) / sum(u_j^2).
struct StepEstimator
{
    double taps[5];
    double u[4], uu;

    StepEstimator(double hz, double rate)
    {
        const double w = 2.0 * kPi * hz / rate, c = std::cos(w);
        const double h[5] = {1.0, -4.0 * c, 4.0 * c * c + 2.0, -4.0 * c, 1.0};
        for (int i = 0; i < 5; ++i)
            taps[i] = h[i];
        uu = 0.0;
        for (int j = 0; j < 4; ++j)
        {
            Cx cj(0, 0);
            for (int k = 0; k <= j; ++k)
                cj += h[k] * std::polar(1.0, w * (j - k));
            u[j] = cj.real();
            uu += u[j] * u[j];
        }
    }

    // Estimated jump height at each onset index (size y.size() - 8; entry i belongs to sample i + 4).
    Signal profile(const Signal& y) const
    {
        const size_t n = y.size();
        Signal o(n, 0.0);
        for (size_t i = 4; i < n; ++i)
            o[i] = taps[0] * y[i] + taps[1] * y[i - 1] + taps[2] * y[i - 2] + taps[3] * y[i - 3] + taps[4] * y[i - 4];
        Signal p(n > 8 ? n - 8 : 0, 0.0);
        for (size_t i = 0; i < p.size(); ++i)
        {
            const size_t s = i + 4;
            double bu = 0.0;
            for (int j = 0; j < 4; ++j)
                bu += o[s + static_cast<size_t>(j)] * u[j];
            p[i] = std::abs(bu) / uu;
        }
        return p;
    }
};
} // namespace detail

struct ClickResult
{
    double stepDb = -300.0; // largest step against full scale
    size_t at = 0;          // sample where it starts
};

// Largest jump of the sample value (a click) in y[from, to) for a carrier at hz. Smooth modulation of amplitude or phase
// does not register. A gain step e on a carrier of amplitude A that happens at carrier phase p jumps the value by
// A e sin p, so test at a phase near a crest (or take the largest of several phases); a step at a zero crossing changes
// only the slope and reads smaller by about w = 2 pi hz / rate.
inline ClickResult clickTest(const Signal& y, double hz, double rate, size_t from = 0, size_t to = 0)
{
    if (to == 0 || to > y.size())
        to = y.size();
    ClickResult r;
    if (to < from + 16)
        return r;
    const detail::StepEstimator est(hz, rate);
    const Signal sub(y.begin() + static_cast<long>(from), y.begin() + static_cast<long>(to));
    const Signal p = est.profile(sub);
    double best = 0.0;
    for (size_t i = 0; i < p.size(); ++i)
        if (p[i] > best)
        {
            best = p[i];
            r.at = from + i + 4;
        }
    r.stepDb = amplitudeToDb(best);
    return r;
}

struct ZipperResult
{
    double rampDb = -300.0;   // largest step while a parameter ramps 0 -> 1 over 50 ms
    double squareDb = -300.0; // largest step while a 2 Hz square wave switches it
};

// Quintic-smoothed edge: 0 -> 1 over `edgeSamples`.
inline double smootherEdge(double t)
{
    t = std::min(1.0, std::max(0.0, t));
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

// Automation test of one continuous parameter. render(in, automation) runs a fresh instance on the sine `in` while
// `automation` (one value in [0, 1] per sample, already smoothed the way the engine hands it to a module: a quintic
// 50 ms ramp, quintic edges of automationEdgeMs for the square wave) drives the parameter.
template <class Render>
ZipperResult zipperTest(Render&& render, double rate, double carrierHz = 1000.0, double levelDb = -6.0,
                        double automationEdgeMs = 20.0)
{
    ZipperResult z;
    const double amp = dbToAmplitude(levelDb);
    {
        const size_t pre = static_cast<size_t>(0.3 * rate), ramp = static_cast<size_t>(0.05 * rate),
                     post = static_cast<size_t>(0.3 * rate);
        const size_t total = pre + ramp + post;
        Signal a(total);
        for (size_t i = 0; i < total; ++i)
            a[i] = i < pre
                       ? 0.0
                       : (i >= pre + ramp ? 1.0 : smootherEdge(static_cast<double>(i - pre) / static_cast<double>(ramp)));
        const Signal out = render(sineWave(total, carrierHz, rate, amp), a);
        z.rampDb = clickTest(out, carrierHz, rate, static_cast<size_t>(0.1 * rate)).stepDb;
    }
    {
        const size_t total = static_cast<size_t>(2.0 * rate), half = static_cast<size_t>(0.25 * rate);
        const double edge = automationEdgeMs * 1.0e-3 * rate;
        Signal a(total);
        for (size_t i = 0; i < total; ++i)
        {
            const size_t k = i / half; // 2 Hz square: toggles every quarter second... half period = 0.25 s
            const double into = static_cast<double>(i - k * half);
            const double level = (k % 2 == 1) ? 1.0 : 0.0, prev = (k % 2 == 1) ? 0.0 : 1.0;
            a[i] = k == 0 ? 0.0 : prev + (level - prev) * smootherEdge(into / edge);
        }
        const Signal out = render(sineWave(total, carrierHz, rate, amp), a);
        z.squareDb = clickTest(out, carrierHz, rate, static_cast<size_t>(0.1 * rate)).stepDb;
    }
    return z;
}

// ---------------------------------------------------------------------------------------------------------------
// Rooms: energy decay

struct DecayBand
{
    double centreHz = 0.0;
    double edtS = 0.0, t20S = 0.0, t30S = 0.0; // reverberation time extrapolated to 60 dB from each fit
    double curvaturePct = 0.0;                 // 100 (T30 / T20 - 1)
};

namespace detail
{
// Zero-phase band-pass: smooth bell in log frequency (sigma in octaves).
inline Signal bandFilter(const Signal& x, double rate, double centreHz, double sigmaOctaves = 0.35)
{
    const size_t size = pow2AtLeast(x.size());
    std::vector<Cx> a(size, Cx(0, 0));
    for (size_t i = 0; i < x.size(); ++i)
        a[i] = x[i];
    fft(a);
    for (size_t k = 0; k <= size / 2; ++k)
    {
        const double f = static_cast<double>(k) * rate / static_cast<double>(size);
        const double g = f <= 0.0 ? 0.0 : std::exp(-0.5 * std::pow(std::log2(f / centreHz) / sigmaOctaves, 2.0));
        a[k] *= g;
        if (k > 0 && k < size / 2)
            a[size - k] *= g;
    }
    fft(a, true);
    Signal y(x.size());
    for (size_t i = 0; i < x.size(); ++i)
        y[i] = a[i].real();
    return y;
}

// Schroeder backward integral in dB (0 at the start).
inline Signal schroederDb(const Signal& ir)
{
    Signal e(ir.size());
    double acc = 0.0;
    for (size_t i = ir.size(); i-- > 0;)
    {
        acc += ir[i] * ir[i];
        e[i] = acc;
    }
    const double total = std::max(e.empty() ? 1.0 : e[0], 1.0e-300);
    for (double& v : e)
        v = powerToDb(v / total);
    return e;
}

// Reverberation time (60 dB extrapolation) from a least-squares line through the decay between two levels.
inline double decayTime(const Signal& edcDb, double rate, double fromDb, double toDb, double scaleTo60)
{
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    size_t n = 0;
    for (size_t i = 0; i < edcDb.size(); ++i)
        if (edcDb[i] <= fromDb && edcDb[i] >= toDb)
        {
            const double t = static_cast<double>(i) / rate;
            sx += t;
            sy += edcDb[i];
            sxx += t * t;
            sxy += t * edcDb[i];
            ++n;
        }
    if (n < 4)
        return 0.0;
    const double dn = static_cast<double>(n);
    const double slope = (dn * sxy - sx * sy) / (dn * sxx - sx * sx);
    return slope < 0.0 ? scaleTo60 * (-60.0 / slope) / scaleTo60 : 0.0;
}
} // namespace detail

// Octave-band decay times of an impulse response.
inline std::vector<DecayBand> energyDecay(const Signal& ir, double rate,
                                          const std::vector<double>& centresHz = {125, 250, 500, 1000, 2000, 4000, 8000})
{
    std::vector<DecayBand> bands;
    for (const double fc : centresHz)
    {
        const Signal edc = detail::schroederDb(detail::bandFilter(ir, rate, fc));
        DecayBand b;
        b.centreHz = fc;
        b.edtS = detail::decayTime(edc, rate, 0.0, -10.0, 1.0);
        b.t20S = detail::decayTime(edc, rate, -5.0, -25.0, 1.0);
        b.t30S = detail::decayTime(edc, rate, -5.0, -35.0, 1.0);
        b.curvaturePct = b.t20S > 0.0 ? 100.0 * (b.t30S / b.t20S - 1.0) : 0.0;
        bands.push_back(b);
    }
    return bands;
}

// Broadband T30 of an impulse response.
inline double reverbTimeT30(const Signal& ir, double rate)
{
    return detail::decayTime(detail::schroederDb(ir), rate, -5.0, -35.0, 1.0);
}

// Normalised echo density (Abel and Huang): the fraction of samples in a sliding window that lie outside one
// standard deviation, divided by the value for Gaussian noise (0.3173). Returns one value per sample from the start
// of the impulse response; the first sample with a value of 1 or more is echoDensityTime().
inline Signal echoDensity(const Signal& ir, double rate, double windowSeconds = 0.020)
{
    const size_t w = std::max<size_t>(8, static_cast<size_t>(windowSeconds * rate));
    const size_t n = ir.size();
    Signal d(n, 0.0);
    std::vector<double> prefix(n + 1, 0.0);
    for (size_t i = 0; i < n; ++i)
        prefix[i + 1] = prefix[i] + ir[i] * ir[i];
    const double gaussian = std::erfc(1.0 / std::sqrt(2.0));
    for (size_t i = 0; i < n; ++i)
    {
        const size_t lo = i >= w / 2 ? i - w / 2 : 0, hi = std::min(n, lo + w);
        const double sigma = std::sqrt((prefix[hi] - prefix[lo]) / static_cast<double>(hi - lo));
        size_t outside = 0;
        for (size_t k = lo; k < hi; ++k)
            if (std::abs(ir[k]) > sigma)
                ++outside;
        d[i] = sigma > 0.0 ? static_cast<double>(outside) / static_cast<double>(hi - lo) / gaussian : 0.0;
    }
    return d;
}

inline double echoDensityTime(const Signal& density, double rate)
{
    for (size_t i = 0; i < density.size(); ++i)
        if (density[i] >= 1.0)
            return static_cast<double>(i) / rate;
    return -1.0;
}

// Interaural cross-correlation coefficient: largest normalised correlation of two channels within +- maxLagMs over
// the first earlySeconds.
inline double iacc(const Signal& l, const Signal& r, double rate, double earlySeconds = 0.080, double maxLagMs = 1.0)
{
    const size_t n = std::min({l.size(), r.size(), static_cast<size_t>(earlySeconds * rate)});
    const long maxLag = static_cast<long>(maxLagMs * 1.0e-3 * rate);
    double el = 0.0, er = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        el += l[i] * l[i];
        er += r[i] * r[i];
    }
    const double norm = std::sqrt(el * er);
    if (norm <= 0.0)
        return 0.0;
    double best = 0.0;
    for (long lag = -maxLag; lag <= maxLag; ++lag)
    {
        double s = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            const long j = static_cast<long>(i) + lag;
            if (j >= 0 && j < static_cast<long>(n))
                s += l[i] * r[static_cast<size_t>(j)];
        }
        best = std::max(best, std::abs(s) / norm);
    }
    return best;
}

// Standard deviation (dB) of the tail's 1/3-octave band levels, each divided by its bandwidth, between loHz and hiHz,
// for the part of the impulse response from `startSeconds` on: 0 for a flat (white) tail, larger for ringing.
inline double tailSpectralFlatness(const Signal& ir, double rate, double startSeconds, double loHz = 100.0,
                                   double hiHz = 10000.0)
{
    const size_t start = static_cast<size_t>(startSeconds * rate);
    if (start >= ir.size())
        return 0.0;
    const size_t size = detail::pow2AtLeast(ir.size() - start);
    std::vector<Cx> a(size, Cx(0, 0));
    for (size_t i = start; i < ir.size(); ++i)
        a[i - start] = ir[i];
    fft(a);
    Signal levels;
    const double step = std::pow(2.0, 1.0 / 3.0);
    for (double lo = loHz; lo * step <= hiHz * 1.0001; lo *= step)
    {
        const double hi = lo * step;
        double e = 0.0;
        size_t bins = 0;
        for (size_t k = 1; k < size / 2; ++k)
        {
            const double f = static_cast<double>(k) * rate / static_cast<double>(size);
            if (f >= lo && f < hi)
            {
                e += std::norm(a[k]);
                ++bins;
            }
        }
        if (bins > 0)
            levels.push_back(powerToDb(e / static_cast<double>(bins)));
    }
    if (levels.empty())
        return 0.0;
    const double mean = std::accumulate(levels.begin(), levels.end(), 0.0) / static_cast<double>(levels.size());
    double var = 0.0;
    for (const double v : levels)
        var += (v - mean) * (v - mean);
    return std::sqrt(var / static_cast<double>(levels.size()));
}

// ---------------------------------------------------------------------------------------------------------------
// Modulation

struct ModulationResult
{
    double peakCents = 0.0; // peak deviation from the mean frequency
    double rateHz = 0.0;    // dominant modulation rate
    double meanHz = 0.0;
};

namespace detail
{
// Instantaneous frequency (Hz) of a narrow-band signal from its analytic signal (FFT Hilbert transform). The ends are
// tapered by a raised cosine of taperSeconds first: an abrupt start leaves a 1/t tail in the Hilbert transform that is
// worth 1 % of a flutter reading a second in; with the taper the result is clean beyond taper + 0.05 s from either end.
inline Signal instantaneousFrequency(const Signal& x, double rate, double taperSeconds = 0.1)
{
    const size_t n = x.size();
    const size_t size = pow2AtLeast(n);
    const size_t taper = std::min(n / 2, static_cast<size_t>(taperSeconds * rate));
    std::vector<Cx> a(size, Cx(0, 0));
    for (size_t i = 0; i < n; ++i)
    {
        double w = 1.0;
        const size_t edge = std::min(i, n - 1 - i);
        if (edge < taper)
            w = 0.5 - 0.5 * std::cos(kPi * static_cast<double>(edge) / static_cast<double>(taper));
        a[i] = x[i] * w;
    }
    fft(a);
    for (size_t k = 1; k < size / 2; ++k)
        a[k] *= 2.0;
    for (size_t k = size / 2 + 1; k < size; ++k)
        a[k] = Cx(0, 0);
    fft(a, true);
    Signal f(n, 0.0);
    for (size_t i = 1; i + 1 < n; ++i)
    {
        const double d = std::arg(a[i + 1] * std::conj(a[i - 1])); // phase advance over two samples
        f[i] = d * rate / (4.0 * kPi);
    }
    f[0] = f[1];
    f[n - 1] = f[n - 2];
    return f;
}
} // namespace detail

// Pitch deviation and rate of a frequency-modulated sine (edges of `guardSeconds`, at least 0.15 s, are ignored).
inline ModulationResult modulationDepth(const Signal& x, double rate, double guardSeconds = 0.15)
{
    const Signal f = detail::instantaneousFrequency(x, rate);
    const size_t guard = static_cast<size_t>(guardSeconds * rate);
    ModulationResult r;
    if (f.size() < 4 * guard + 64)
        return r;
    const Signal t(f.begin() + static_cast<long>(guard), f.end() - static_cast<long>(guard));
    r.meanHz = std::accumulate(t.begin(), t.end(), 0.0) / static_cast<double>(t.size());
    Signal cents(t.size());
    for (size_t i = 0; i < t.size(); ++i)
        cents[i] = centsBetween(t[i], r.meanHz);
    // Dominant rate: spectral peak of the deviation, refined by the windowed DTFT.
    const size_t n = floorPow2(cents.size());
    Signal d(cents.begin(), cents.begin() + static_cast<long>(n));
    const double mean = std::accumulate(d.begin(), d.end(), 0.0) / static_cast<double>(n);
    for (double& v : d)
        v -= mean;
    r.rateHz = estimateFrequency(d.data(), n, rate);
    // Peak: amplitude of the sinusoid at that rate over whole cycles (least squares), robust to residual ripple.
    const size_t cycles =
        std::max<size_t>(1, static_cast<size_t>(std::floor(static_cast<double>(cents.size()) / rate * r.rateHz)));
    const size_t len =
        std::min(cents.size(), static_cast<size_t>(std::llround(static_cast<double>(cycles) * rate / r.rateHz)));
    double sc = 0.0, ss = 0.0;
    for (size_t i = 0; i < len; ++i)
    {
        const double ph = 2.0 * kPi * r.rateHz * static_cast<double>(i) / rate;
        sc += cents[i] * std::cos(ph);
        ss += cents[i] * std::sin(ph);
    }
    r.peakCents = 2.0 * std::hypot(sc, ss) / static_cast<double>(len);
    return r;
}

// Weighting curve of the wow and flutter meter: first-order high-pass at 1 Hz, first-order low-pass at 8 Hz and a
// second-order roll-off above, scaled to 0 dB at 4 Hz. An approximation of the shape of the IEC 60386 weighting (peak
// near 4 Hz); the tabulated values of the standard are not reproduced here.
inline double wowFlutterWeight(double hz)
{
    auto raw = [](double f)
    {
        const double hp = (f / 1.0) / std::sqrt(1.0 + (f / 1.0) * (f / 1.0));
        const double lp = 1.0 / std::sqrt(1.0 + std::pow(f / 8.0, 4.0));
        return hp * lp;
    };
    return raw(hz) / raw(4.0);
}

struct WowFlutterResult
{
    double unweightedPeakPct = 0.0;
    double weightedPeakPct = 0.0;
    double weightedRmsPct = 0.0;
};

// Wow and flutter of a recorded tone near 3.15 kHz: the frequency deviation (percent of the mean) is weighted and its
// peak and RMS read.
inline WowFlutterResult wowFlutter(const Signal& x, double rate, double guardSeconds = 0.15)
{
    const Signal f = detail::instantaneousFrequency(x, rate);
    const size_t guard = static_cast<size_t>(guardSeconds * rate);
    WowFlutterResult r;
    if (f.size() < 4 * guard + 64)
        return r;
    const size_t n = f.size() - 2 * guard;
    const size_t size = detail::pow2AtLeast(n);
    const double mean = std::accumulate(f.begin() + static_cast<long>(guard), f.end() - static_cast<long>(guard), 0.0) /
                        static_cast<double>(n);
    // The deviation trace is tapered (raised cosine over up to a second at each end) before the weighting is applied in
    // the frequency domain, so that the abrupt ends do not leak into the pass band; readings come from the middle.
    const size_t taper = std::min(n / 6, static_cast<size_t>(rate));
    std::vector<Cx> a(size, Cx(0, 0)), u(size, Cx(0, 0));
    for (size_t i = 0; i < n; ++i)
    {
        double w = 1.0;
        const size_t edge = std::min(i, n - 1 - i);
        if (edge < taper)
            w = 0.5 - 0.5 * std::cos(kPi * static_cast<double>(edge) / static_cast<double>(taper));
        a[i] = 100.0 * (f[guard + i] - mean) / mean * w;
    }
    u = a;
    fft(a);
    for (size_t k = 0; k <= size / 2; ++k)
    {
        const double g = wowFlutterWeight(static_cast<double>(k) * rate / static_cast<double>(size));
        a[k] *= g;
        if (k > 0 && k < size / 2)
            a[size - k] *= g;
    }
    fft(a, true);
    double sum = 0.0;
    size_t count = 0;
    for (size_t i = taper; i + taper < n; ++i, ++count)
    {
        r.unweightedPeakPct = std::max(r.unweightedPeakPct, std::abs(u[i].real()));
        r.weightedPeakPct = std::max(r.weightedPeakPct, std::abs(a[i].real()));
        sum += a[i].real() * a[i].real();
    }
    r.weightedRmsPct = std::sqrt(sum / static_cast<double>(std::max<size_t>(1, count)));
    return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Loudness: ITU-R BS.1770-4 (offline, double precision, every block kept). The real-time meter (fx/Loudness.h) is checked
// against this and against libebur128; this stays as the independent offline reference for effects tests.

struct KBiquad
{
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
    double operator()(double x)
    {
        const double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
};

// The two K-weighting stages for any sample rate, from the analog prototypes of the recommendation (shelf: 3.9998 dB
// at 1681.97 Hz, Q 0.7072; high-pass: 38.1355 Hz, Q 0.5003) through the bilinear transform. At 48 kHz they reproduce the
// coefficients printed in the recommendation.
inline std::pair<KBiquad, KBiquad> kWeighting(double rate)
{
    KBiquad shelf, hp;
    {
        const double g = 3.999843853973347, q = 0.7071752369554196, fc = 1681.9744509555319;
        const double k = std::tan(kPi * fc / rate), vh = std::pow(10.0, g / 20.0), vb = std::pow(vh, 0.4996667741545416);
        const double a0 = 1.0 + k / q + k * k;
        shelf.b0 = (vh + vb * k / q + k * k) / a0;
        shelf.b1 = 2.0 * (k * k - vh) / a0;
        shelf.b2 = (vh - vb * k / q + k * k) / a0;
        shelf.a1 = 2.0 * (k * k - 1.0) / a0;
        shelf.a2 = (1.0 - k / q + k * k) / a0;
    }
    {
        const double q = 0.5003270373238773, fc = 38.13547087602444;
        const double k = std::tan(kPi * fc / rate), a0 = 1.0 + k / q + k * k;
        hp.b0 = 1.0;
        hp.b1 = -2.0;
        hp.b2 = 1.0;
        hp.a1 = 2.0 * (k * k - 1.0) / a0;
        hp.a2 = (1.0 - k / q + k * k) / a0;
    }
    return {shelf, hp};
}

struct LoudnessResult
{
    double integratedLufs = -300.0;
    double momentaryMaxLufs = -300.0;
    double shortTermMaxLufs = -300.0;
    double rangeLu = 0.0;
};

namespace detail
{
inline double lufsOf(double meanSquareSum)
{
    return meanSquareSum > 0.0 ? -0.691 + 10.0 * std::log10(meanSquareSum) : -300.0;
}

// Energy mean of block loudness values above a threshold.
inline double gatedMeanSquare(const Signal& z, double thresholdLufs, size_t* count = nullptr)
{
    double s = 0.0;
    size_t n = 0;
    for (const double v : z)
        if (lufsOf(v) > thresholdLufs)
        {
            s += v;
            ++n;
        }
    if (count != nullptr)
        *count = n;
    return n > 0 ? s / static_cast<double>(n) : 0.0;
}
} // namespace detail

// Channels are left, right (weight 1 each) or a single mono channel. Momentary: 400 ms, short term: 3 s, both
// sampled every 100 ms; integrated: 400 ms blocks with 75 % overlap, absolute gate -70 LUFS, relative gate -10 LU;
// range: short-term values with absolute gate -70 and relative gate -20 LU, 95th minus 10th percentile.
inline LoudnessResult measureLoudness(const std::vector<Signal>& channels, double rate)
{
    LoudnessResult r;
    if (channels.empty() || channels[0].empty())
        return r;
    const size_t n = channels[0].size();
    std::vector<Signal> sq(channels.size(), Signal(n)); // K-weighted squares, summed over channels below
    Signal power(n, 0.0);
    for (size_t c = 0; c < channels.size(); ++c)
    {
        auto [shelf, hp] = kWeighting(rate);
        for (size_t i = 0; i < n; ++i)
        {
            const double y = hp(shelf(channels[c][i]));
            power[i] += y * y;
        }
    }
    std::vector<double> prefix(n + 1, 0.0);
    for (size_t i = 0; i < n; ++i)
        prefix[i + 1] = prefix[i] + power[i];
    const size_t hop = static_cast<size_t>(0.1 * rate);
    auto windows = [&](double seconds)
    {
        Signal z;
        const size_t w = static_cast<size_t>(seconds * rate);
        for (size_t start = 0; start + w <= n; start += hop)
            z.push_back((prefix[start + w] - prefix[start]) / static_cast<double>(w));
        return z;
    };
    const Signal momentary = windows(0.4), shortTerm = windows(3.0);
    for (const double v : momentary)
        r.momentaryMaxLufs = std::max(r.momentaryMaxLufs, detail::lufsOf(v));
    for (const double v : shortTerm)
        r.shortTermMaxLufs = std::max(r.shortTermMaxLufs, detail::lufsOf(v));
    const double absMean = detail::gatedMeanSquare(momentary, -70.0);
    if (absMean > 0.0)
    {
        const double rel = detail::lufsOf(absMean) - 10.0;
        r.integratedLufs = detail::lufsOf(detail::gatedMeanSquare(momentary, std::max(-70.0, rel)));
    }
    std::vector<double> st;
    const double stAbs = detail::gatedMeanSquare(shortTerm, -70.0);
    if (stAbs > 0.0)
    {
        const double rel = detail::lufsOf(stAbs) - 20.0;
        for (const double v : shortTerm)
            if (detail::lufsOf(v) > std::max(-70.0, rel))
                st.push_back(detail::lufsOf(v));
        if (st.size() > 1)
        {
            std::sort(st.begin(), st.end());
            auto percentile = [&](double p)
            {
                const double pos = p * static_cast<double>(st.size() - 1);
                const size_t i = static_cast<size_t>(pos);
                const double f = pos - static_cast<double>(i);
                return i + 1 < st.size() ? st[i] * (1.0 - f) + st[i + 1] * f : st[i];
            };
            r.rangeLu = percentile(0.95) - percentile(0.10);
        }
    }
    return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Stereo

struct StereoStats
{
    double correlation = 0.0; // Pearson correlation of left and right
    double midDb = -300.0;    // RMS of (L + R) / 2
    double sideDb = -300.0;   // RMS of (L - R) / 2
    double midSideDb = 0.0;   // mid against side, dB (positive: mostly centre)
};

inline StereoStats stereoStats(const Signal& l, const Signal& r)
{
    const size_t n = std::min(l.size(), r.size());
    double sl = 0, sr = 0, sll = 0, srr = 0, slr = 0, mid = 0, side = 0;
    for (size_t i = 0; i < n; ++i)
    {
        sl += l[i];
        sr += r[i];
        sll += l[i] * l[i];
        srr += r[i] * r[i];
        slr += l[i] * r[i];
        mid += 0.25 * (l[i] + r[i]) * (l[i] + r[i]);
        side += 0.25 * (l[i] - r[i]) * (l[i] - r[i]);
    }
    const double dn = static_cast<double>(n);
    StereoStats s;
    const double cov = slr / dn - sl * sr / (dn * dn);
    const double vl = sll / dn - sl * sl / (dn * dn), vr = srr / dn - sr * sr / (dn * dn);
    s.correlation = vl > 0.0 && vr > 0.0 ? cov / std::sqrt(vl * vr) : 0.0;
    s.midDb = powerToDb(mid / dn);
    s.sideDb = powerToDb(side / dn);
    s.midSideDb = s.midDb - s.sideDb;
    return s;
}

// Energy of the mono sum (L + R) / 2 against the mean energy of the two channels, per octave band. 0 dB: fully
// correlated; -3 dB: independent; very negative: out of phase.
inline Signal monoSumPerBand(const Signal& l, const Signal& r, double rate, const std::vector<double>& centresHz)
{
    Signal out;
    Signal mono(std::min(l.size(), r.size()));
    for (size_t i = 0; i < mono.size(); ++i)
        mono[i] = 0.5 * (l[i] + r[i]);
    for (const double fc : centresHz)
    {
        const Signal ml = detail::bandFilter(mono, rate, fc), bl = detail::bandFilter(l, rate, fc),
                     br = detail::bandFilter(r, rate, fc);
        double em = 0, el = 0, er = 0;
        for (size_t i = 0; i < ml.size(); ++i)
        {
            em += ml[i] * ml[i];
            el += bl[i] * bl[i];
            er += br[i] * br[i];
        }
        out.push_back(powerToDb(em / std::max(0.5 * (el + er), 1.0e-300)));
    }
    return out;
}

} // namespace crucible::measure
