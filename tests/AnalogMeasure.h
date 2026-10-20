// SPDX-License-Identifier: MIT
#pragma once

// Measurement primitives for checking DSP by numbers instead of by ear (docs/plans/ENGINE-COMMUNITY-PLAN.md, section 3).
//
// Everything works in double precision on plain vectors, allocates freely and is meant for tests and offline tools,
// never for the audio thread. The functions are small and independent:
//
//   spectra         computeSpectrum, estimateFrequency, harmonicProfile (levels, THD, THD+N), measureAliasing
//   levels          rmsDbfs, bandRmsDbfs, dcOffset, truePeak
//   filters         steppedGainDb, steppedResponse, sweepImpulseResponse, responseFromImpulse, slopeDbPerOctave,
//                   resonancePeak, crossingBelow, measureSelfOscillation
//   statistics      ouStats (std, autocorrelation time, PSD slope), welchPsd, generateOu, pitchTraceCents
//   envelopes       fitExponential, timeToLevel, rmsEnvelope
//
// A processor under test is a callable double(double) (one sample in, one sample out); the functions that need a
// fresh instance per measurement take a factory returning one. Windows are the cosine-sum family: Blackman-Harris
// (the plan's default, -92 dB sidelobes) and a 7-term window (-180 dB sidelobes) for floor measurements.

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace crucible::measure
{

constexpr double kPi = 3.14159265358979323846;
using Signal = std::vector<double>;
using Cx = std::complex<double>;

// ---------------------------------------------------------------------------------------------------------------
// Small helpers

inline Signal toDouble(const std::vector<float>& x, size_t from = 0, size_t count = std::numeric_limits<size_t>::max())
{
    if (from >= x.size())
        return {};
    const size_t n = std::min(count, x.size() - from);
    return Signal(x.begin() + static_cast<std::ptrdiff_t>(from), x.begin() + static_cast<std::ptrdiff_t>(from + n));
}

inline double amplitudeToDb(double a)
{
    return 20.0 * std::log10(std::max(a, 1.0e-300));
}
inline double powerToDb(double p)
{
    return 10.0 * std::log10(std::max(p, 1.0e-300));
}
inline double dbToAmplitude(double db)
{
    return std::pow(10.0, db / 20.0);
}
inline double centsBetween(double hz, double referenceHz)
{
    return 1200.0 * std::log2(hz / referenceHz);
}

inline size_t floorPow2(size_t n)
{
    size_t p = 1;
    while (p * 2 <= n)
        p *= 2;
    return p;
}

// Deterministic generator for synthetic test signals (splitmix64).
class Random
{
public:
    explicit Random(std::uint64_t seed = 1) : state(seed) {}
    std::uint64_t nextBits()
    {
        state += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    // Uniform in (0, 1).
    double uniform() { return (static_cast<double>(nextBits() >> 11) + 0.5) * (1.0 / 9007199254740992.0); }
    // Standard normal (Box-Muller).
    double gaussian()
    {
        const double u = uniform(), v = uniform();
        return std::sqrt(-2.0 * std::log(u)) * std::cos(2.0 * kPi * v);
    }

private:
    std::uint64_t state;
};

inline Signal sineWave(size_t n, double hz, double rate, double amplitude = 1.0, double phase = 0.0)
{
    Signal s(n);
    for (size_t i = 0; i < n; ++i)
        s[i] = amplitude * std::sin(2.0 * kPi * hz * static_cast<double>(i) / rate + phase);
    return s;
}

// ---------------------------------------------------------------------------------------------------------------
// FFT and windows

// In-place radix-2 FFT (size must be a power of two); the inverse is scaled by 1 / n.
inline void fft(std::vector<Cx>& a, bool inverse = false)
{
    const size_t n = a.size();
    if (n < 2)
        return;
    for (size_t i = 1, j = 0; i < n; ++i)
    {
        size_t bit = n >> 1;
        for (; (j & bit) != 0; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(a[i], a[j]);
    }
    std::vector<Cx> w(n / 2);
    const double sign = inverse ? 2.0 : -2.0;
    for (size_t k = 0; k < n / 2; ++k)
    {
        const double angle = sign * kPi * static_cast<double>(k) / static_cast<double>(n);
        w[k] = Cx(std::cos(angle), std::sin(angle));
    }
    for (size_t len = 2; len <= n; len <<= 1)
    {
        const size_t step = n / len, half = len / 2;
        for (size_t i = 0; i < n; i += len)
            for (size_t j = 0; j < half; ++j)
            {
                const Cx u = a[i + j];
                const Cx v = a[i + j + half] * w[j * step];
                a[i + j] = u + v;
                a[i + j + half] = u - v;
            }
    }
    if (inverse)
        for (auto& v : a)
            v /= static_cast<double>(n);
}

enum class Window
{
    Rectangular,
    Hann,
    BlackmanHarris, // 4-term, sidelobes -92 dB, main lobe +-4 bins
    Harris7         // 7-term (Albrecht), sidelobes below -180 dB, main lobe +-7 bins
};

// Periodic (DFT-even) window of n points.
inline Signal makeWindow(Window type, size_t n)
{
    static const double hann[] = {0.5, 0.5};
    static const double bh4[] = {0.35875, 0.48829, 0.14128, 0.01168};
    static const double h7[] = {0.2712203606, 0.4334446123, 0.2180041228, 0.0657853433,
                                0.0107618673, 0.0007700127, 0.0000136808};
    const double* c = nullptr;
    int terms = 1;
    switch (type)
    {
    case Window::Rectangular:
        break;
    case Window::Hann:
        c = hann;
        terms = 2;
        break;
    case Window::BlackmanHarris:
        c = bh4;
        terms = 4;
        break;
    case Window::Harris7:
        c = h7;
        terms = 7;
        break;
    }
    Signal w(n, 1.0);
    if (c == nullptr)
        return w;
    for (size_t i = 0; i < n; ++i)
    {
        const double x = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(n);
        double v = 0.0;
        for (int k = 0; k < terms; ++k)
            v += ((k & 1) != 0 ? -1.0 : 1.0) * c[k] * std::cos(static_cast<double>(k) * x);
        w[i] = v;
    }
    return w;
}

// Bins either side of a tone that its main lobe covers.
inline int windowHalfWidthBins(Window type)
{
    switch (type)
    {
    case Window::Rectangular:
        return 1;
    case Window::Hann:
        return 2;
    case Window::BlackmanHarris:
        return 4;
    case Window::Harris7:
        return 7;
    }
    return 4;
}

struct Spectrum
{
    size_t n = 0;
    double rate = 0.0;
    double binHz = 0.0;
    Window window = Window::Harris7;
    double sumW = 0.0;         // sum of the window
    double sumW2 = 0.0;        // sum of the squared window
    std::vector<double> power; // |X[k]|^2 for k = 0..n/2, unnormalised

    // Amplitude of a sinusoid centred on bin k (a sine of amplitude A reads A).
    double amplitude(size_t k) const
    {
        const double scale = (k == 0 || k == n / 2) ? 1.0 : 2.0;
        return scale * std::sqrt(power[k]) / sumW;
    }
    size_t bins() const { return power.size(); }
};

// Spectrum of x[0..n) (n must be a power of two).
inline Spectrum computeSpectrum(const double* x, size_t n, double rate, Window type = Window::Harris7)
{
    Spectrum s;
    s.n = n;
    s.rate = rate;
    s.binHz = rate / static_cast<double>(n);
    s.window = type;
    const Signal w = makeWindow(type, n);
    std::vector<Cx> a(n);
    for (size_t i = 0; i < n; ++i)
    {
        a[i] = Cx(x[i] * w[i], 0.0);
        s.sumW += w[i];
        s.sumW2 += w[i] * w[i];
    }
    fft(a);
    s.power.resize(n / 2 + 1);
    for (size_t k = 0; k <= n / 2; ++k)
        s.power[k] = std::norm(a[k]);
    return s;
}

inline Spectrum computeSpectrum(const Signal& x, size_t start = 0, size_t n = 0, double rate = 48000.0,
                                Window type = Window::Harris7)
{
    if (start >= x.size())
        return {};
    if (n == 0)
        n = floorPow2(x.size() - start);
    return computeSpectrum(x.data() + start, n, rate, type);
}

// Windowed DTFT of x[0..n) at an arbitrary frequency: sum x[i] w[i] exp(-j 2 pi f i / rate).
inline Cx windowedDtft(const double* x, const double* w, size_t n, double hz, double rate)
{
    const double step = -2.0 * kPi * hz / rate;
    const Cx rot(std::cos(step), std::sin(step));
    Cx z(1.0, 0.0), acc(0.0, 0.0);
    for (size_t i = 0; i < n; ++i)
    {
        acc += (x[i] * w[i]) * z;
        z *= rot;
        if ((i & 1023U) == 1023U)
            z = std::polar(1.0, step * static_cast<double>(i + 1));
    }
    return acc;
}

// Frequency of the strongest sinusoid in x[0..n): spectral peak, then a golden-section search of the windowed DTFT
// magnitude. hintHz > 0 restricts the coarse search to +-30 % around it. Accurate to well below 0.01 cent for a
// clean tone of a few hundred periods.
inline double estimateFrequency(const double* x, size_t n, double rate, double hintHz = 0.0)
{
    const size_t m = floorPow2(n);
    const Spectrum s = computeSpectrum(x, m, rate, Window::BlackmanHarris);
    size_t lo = 3, hi = m / 2 - 2;
    if (hintHz > 0.0)
    {
        lo = std::max<size_t>(lo, static_cast<size_t>(hintHz / 1.3 / s.binHz));
        hi = std::min(hi, static_cast<size_t>(hintHz * 1.3 / s.binHz) + 1);
    }
    size_t peak = lo;
    for (size_t k = lo; k <= hi; ++k)
        if (s.power[k] > s.power[peak])
            peak = k;
    const auto lg = [&](size_t k) { return std::log(std::max(s.power[k], 1.0e-300)); };
    const double a = lg(peak - 1), b = lg(peak), c = lg(peak + 1);
    const double denom = a - 2.0 * b + c;
    const double delta = denom != 0.0 ? 0.5 * (a - c) / denom : 0.0;
    const double coarse = (static_cast<double>(peak) + std::clamp(delta, -1.0, 1.0)) * s.binHz;

    const Signal w = makeWindow(Window::BlackmanHarris, n);
    const auto magnitude = [&](double f) { return std::abs(windowedDtft(x, w.data(), n, f, rate)); };
    const double span = rate / static_cast<double>(n);
    double left = coarse - span, right = coarse + span;
    const double golden = 0.6180339887498949;
    double c1 = right - golden * (right - left), c2 = left + golden * (right - left);
    double m1 = magnitude(c1), m2 = magnitude(c2);
    while (right - left > span * 1.0e-9)
    {
        if (m1 < m2)
        {
            left = c1;
            c1 = c2;
            m1 = m2;
            c2 = left + golden * (right - left);
            m2 = magnitude(c2);
        }
        else
        {
            right = c2;
            c2 = c1;
            m2 = m1;
            c1 = right - golden * (right - left);
            m1 = magnitude(c1);
        }
    }
    return 0.5 * (left + right);
}

inline double estimateFrequency(const Signal& x, double rate, double hintHz = 0.0)
{
    return estimateFrequency(x.data(), x.size(), rate, hintHz);
}

// ---------------------------------------------------------------------------------------------------------------
// Levels

inline double rmsOf(const double* x, size_t n)
{
    double s = 0.0;
    for (size_t i = 0; i < n; ++i)
        s += x[i] * x[i];
    return n > 0 ? std::sqrt(s / static_cast<double>(n)) : 0.0;
}

// RMS relative to 1.0 (a full-scale sine reads -3.01 dB).
inline double rmsDbfs(const double* x, size_t n)
{
    return amplitudeToDb(rmsOf(x, n));
}
inline double rmsDbfs(const Signal& x)
{
    return rmsDbfs(x.data(), x.size());
}

inline double dcOffset(const double* x, size_t n)
{
    double s = 0.0;
    for (size_t i = 0; i < n; ++i)
        s += x[i];
    return n > 0 ? s / static_cast<double>(n) : 0.0;
}
inline double dcOffset(const Signal& x)
{
    return dcOffset(x.data(), x.size());
}
inline double dcDbfs(const Signal& x)
{
    return amplitudeToDb(std::abs(dcOffset(x)));
}

// DC offset of a periodic signal that does not hold a whole number of periods: the mean under a 7-term window,
// whose sidelobes leave no trace of a tone more than a few bins from DC (so the fundamental should be at least
// ten bins, i.e. ten divided by the length in seconds, above 0 Hz).
inline double dcOffsetWindowed(const double* x, size_t n)
{
    const Signal w = makeWindow(Window::Harris7, n);
    double s = 0.0, sw = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        s += x[i] * w[i];
        sw += w[i];
    }
    return s / sw;
}

// RMS of the signal inside a frequency band, in dB relative to 1.0 (x[0..n), n a power of two).
inline double bandRmsDbfs(const double* x, size_t n, double rate, double loHz, double hiHz, Window type = Window::Harris7)
{
    const Spectrum s = computeSpectrum(x, n, rate, type);
    const size_t kLo = std::max<size_t>(1, static_cast<size_t>(std::ceil(loHz / s.binHz)));
    const size_t kHi = std::min(s.bins() - 2, static_cast<size_t>(std::floor(hiHz / s.binHz)));
    double sum = 0.0;
    for (size_t k = kLo; k <= kHi; ++k)
        sum += s.power[k];
    const double variance = 2.0 * sum / (static_cast<double>(n) * s.sumW2);
    return powerToDb(variance); // 10 log10 of the variance is 20 log10 of the RMS
}

// Inter-sample peak (linear) by windowed-sinc oversampling, as ITU-R BS.1770 describes it. The first and last
// `taps` samples are not interpolated.
inline double truePeak(const double* x, size_t n, int oversample = 4, int taps = 32)
{
    double peak = 0.0;
    for (size_t i = 0; i < n; ++i)
        peak = std::max(peak, std::abs(x[i]));
    if (oversample < 2 || n < static_cast<size_t>(2 * taps + 2))
        return peak;
    std::vector<Signal> kernel(static_cast<size_t>(oversample));
    for (int p = 1; p < oversample; ++p)
    {
        const double frac = static_cast<double>(p) / oversample;
        Signal& k = kernel[static_cast<size_t>(p)];
        k.resize(static_cast<size_t>(2 * taps));
        double sum = 0.0;
        for (int m = -taps + 1; m <= taps; ++m)
        {
            const double t = static_cast<double>(m) - frac; // distance from the interpolation point
            const double sinc = std::sin(kPi * t) / (kPi * t);
            const double u = t / static_cast<double>(taps); // |u| < 1
            const double win = 0.35875 + 0.48829 * std::cos(kPi * u) + 0.14128 * std::cos(2.0 * kPi * u) +
                               0.01168 * std::cos(3.0 * kPi * u);
            k[static_cast<size_t>(m + taps - 1)] = sinc * win;
            sum += sinc * win;
        }
        for (auto& v : k)
            v /= sum;
    }
    for (size_t i = static_cast<size_t>(taps); i + static_cast<size_t>(taps) < n; ++i)
        for (int p = 1; p < oversample; ++p)
        {
            double acc = 0.0;
            const Signal& k = kernel[static_cast<size_t>(p)];
            // The point lies between x[i] and x[i+1]; m = 0 is x[i], m = 1 is x[i+1].
            for (int m = -taps + 1; m <= taps; ++m)
                acc += k[static_cast<size_t>(m + taps - 1)] * x[static_cast<size_t>(static_cast<std::ptrdiff_t>(i) + m)];
            peak = std::max(peak, std::abs(acc));
        }
    return peak;
}
inline double truePeak(const Signal& x, int oversample = 4)
{
    return truePeak(x.data(), x.size(), oversample);
}

// ---------------------------------------------------------------------------------------------------------------
// Harmonics, THD and aliasing

struct HarmonicProfile
{
    double f0Hz = 0.0;
    double fundamentalAmplitude = 0.0;
    std::vector<double> amplitude;  // index k - 1 = harmonic k, absolute amplitude
    std::vector<double> db;         // relative to the fundamental; db[0] = 0
    double thdDb = -300.0;          // harmonics 2.. against the fundamental
    double thdPlusNoiseDb = -300.0; // everything but the fundamental against the fundamental
    double evenDb = -300.0;         // power of the even harmonics against the fundamental
    double oddDb = -300.0;          // power of the odd harmonics 3.. against the fundamental
    int count() const { return static_cast<int>(amplitude.size()); }
    double harmonicDb(int k) const { return db[static_cast<size_t>(k - 1)]; }
};

// Levels of the harmonics of f0Hz in x[0..n) (any n). f0Hz <= 0 estimates it. Harmonics up to maxHarmonics and
// below maxHz (0 = 0.49 x rate) are measured with the windowed DTFT at their exact frequency, so the signal does
// not have to contain whole periods.
inline HarmonicProfile harmonicProfile(const double* x, size_t n, double rate, double f0Hz = 0.0, int maxHarmonics = 40,
                                       double maxHz = 0.0, Window type = Window::Harris7)
{
    HarmonicProfile p;
    p.f0Hz = f0Hz > 0.0 ? f0Hz : estimateFrequency(x, n, rate);
    if (maxHz <= 0.0)
        maxHz = 0.49 * rate;
    const Signal w = makeWindow(type, n);
    double sumW = 0.0, sumW2 = 0.0;
    for (const double v : w)
    {
        sumW += v;
        sumW2 += v * v;
    }
    Cx first;
    for (int k = 1; k <= maxHarmonics && k * p.f0Hz < maxHz; ++k)
    {
        const Cx d = windowedDtft(x, w.data(), n, k * p.f0Hz, rate);
        if (k == 1)
            first = 2.0 * d / sumW;
        p.amplitude.push_back(2.0 * std::abs(d) / sumW);
    }
    if (p.amplitude.empty())
        return p;
    p.fundamentalAmplitude = p.amplitude[0];
    double harmonicPower = 0.0, even = 0.0, odd = 0.0;
    for (size_t k = 0; k < p.amplitude.size(); ++k)
    {
        p.db.push_back(amplitudeToDb(p.amplitude[k] / p.fundamentalAmplitude));
        if (k >= 1)
        {
            const double pw = p.amplitude[k] * p.amplitude[k];
            harmonicPower += pw;
            ((k + 1) % 2 == 0 ? even : odd) += pw;
        }
    }
    const double a2 = p.fundamentalAmplitude * p.fundamentalAmplitude;
    p.thdDb = powerToDb(harmonicPower / a2);
    p.evenDb = powerToDb(even / a2);
    p.oddDb = powerToDb(odd / a2);

    // THD+N: remove the fitted fundamental and compare the window-weighted residual power.
    double residual = 0.0;
    const double amp = std::abs(first), phase = std::arg(first);
    const double omega = 2.0 * kPi * p.f0Hz / rate;
    for (size_t i = 0; i < n; ++i)
    {
        const double r = x[i] - amp * std::cos(omega * static_cast<double>(i) + phase);
        residual += w[i] * w[i] * r * r;
    }
    p.thdPlusNoiseDb = powerToDb((residual / sumW2) / (0.5 * a2));
    return p;
}
inline HarmonicProfile harmonicProfile(const Signal& x, double rate, double f0Hz = 0.0, int maxHarmonics = 40)
{
    return harmonicProfile(x.data(), x.size(), rate, f0Hz, maxHarmonics);
}

// Deviation of a measured harmonic series from an ideal one (idealDb(k) in dB relative to the fundamental).
struct ShapeError
{
    double maxAbsDb = 0.0;
    double rmsDb = 0.0;
    int harmonics = 0;
};
inline ShapeError shapeError(const HarmonicProfile& p, const std::function<double(int)>& idealDb, double maxHz,
                             bool (*include)(int) = nullptr)
{
    ShapeError e;
    double sum = 0.0;
    for (int k = 1; k <= p.count(); ++k)
    {
        if (k * p.f0Hz > maxHz || (include != nullptr && !include(k)))
            continue;
        const double d = p.harmonicDb(k) - idealDb(k);
        e.maxAbsDb = std::max(e.maxAbsDb, std::abs(d));
        sum += d * d;
        ++e.harmonics;
    }
    e.rmsDb = e.harmonics > 0 ? std::sqrt(sum / e.harmonics) : 0.0;
    return e;
}

struct AliasResult
{
    double aliasDb = -300.0;     // power outside the harmonics below limitHz against the fundamental's power
    double worstSpurDb = -300.0; // strongest single non-harmonic bin against the fundamental
    double worstSpurHz = 0.0;
    double noiseFloorDb = -300.0; // median non-harmonic bin amplitude against the fundamental (the floor, not spurs)
};

// Aliasing and other non-harmonic energy of a periodic signal of fundamental f0Hz, x[0..n) with n a power of two.
// Bins within the window's main lobe (plus guardBins for pitch movement) of every harmonic are excluded.
inline AliasResult measureAliasing(const double* x, size_t n, double rate, double f0Hz, double limitHz = 20000.0,
                                   int guardBins = 0, Window type = Window::Harris7)
{
    const Spectrum s = computeSpectrum(x, n, rate, type);
    const int half = windowHalfWidthBins(type) + 1 + guardBins;
    std::vector<char> harmonic(s.bins(), 0);
    for (size_t b = 0; b < static_cast<size_t>(half) + 1 && b < s.bins(); ++b)
        harmonic[b] = 1;
    for (int h = 1; h * f0Hz < 0.5 * rate + half * s.binHz; ++h)
    {
        const long centre = std::lround(h * f0Hz / s.binHz);
        for (long b = centre - half; b <= centre + half; ++b)
            if (b >= 0 && b < static_cast<long>(s.bins()))
                harmonic[static_cast<size_t>(b)] = 1;
    }
    double fundamental = 0.0, other = 0.0, spur = 0.0;
    size_t spurBin = 0;
    std::vector<double> floorAmps;
    const long fc = std::lround(f0Hz / s.binHz);
    for (long b = fc - half; b <= fc + half; ++b)
        if (b >= 0 && b < static_cast<long>(s.bins()))
            fundamental += s.power[static_cast<size_t>(b)];
    for (size_t b = 0; b < s.bins(); ++b)
    {
        if (harmonic[b] != 0 || static_cast<double>(b) * s.binHz >= limitHz)
            continue;
        other += s.power[b];
        floorAmps.push_back(s.amplitude(b));
        if (s.power[b] > spur)
        {
            spur = s.power[b];
            spurBin = b;
        }
    }
    // Amplitude of the tone from the power in its main lobe (Parseval).
    const double fundamentalAmp = 2.0 * std::sqrt(fundamental / (static_cast<double>(n) * s.sumW2));
    AliasResult r;
    r.aliasDb = powerToDb(other / std::max(fundamental, 1.0e-300));
    // The spur's amplitude from the power of its whole main lobe, so that its position between bins does not matter.
    double spurLobe = 0.0;
    for (long b = static_cast<long>(spurBin) - half; b <= static_cast<long>(spurBin) + half; ++b)
        if (b >= 0 && b < static_cast<long>(s.bins()) && harmonic[static_cast<size_t>(b)] == 0)
            spurLobe += s.power[static_cast<size_t>(b)];
    r.worstSpurDb =
        amplitudeToDb(2.0 * std::sqrt(spurLobe / (static_cast<double>(n) * s.sumW2)) / std::max(fundamentalAmp, 1.0e-300));
    r.worstSpurHz = static_cast<double>(spurBin) * s.binHz;
    if (!floorAmps.empty())
    {
        std::nth_element(floorAmps.begin(), floorAmps.begin() + static_cast<std::ptrdiff_t>(floorAmps.size() / 2),
                         floorAmps.end());
        r.noiseFloorDb = amplitudeToDb(floorAmps[floorAmps.size() / 2] / std::max(fundamentalAmp, 1.0e-300));
    }
    return r;
}
inline AliasResult measureAliasing(const Signal& x, size_t start, double rate, double f0Hz, double limitHz = 20000.0,
                                   int guardBins = 0, Window type = Window::Harris7)
{
    return measureAliasing(x.data() + start, floorPow2(x.size() - start), rate, f0Hz, limitHz, guardBins, type);
}

// ---------------------------------------------------------------------------------------------------------------
// Filters

// Gain (dB) of a processor at one frequency: a sine of the given amplitude is run through a fresh instance, the
// first settleSeconds are discarded and the complex amplitude at the test frequency is compared with the input's.
template <class Factory>
double steppedGainDb(Factory&& make, double hz, double rate, double amplitude = 0.01, double settleSeconds = 0.4,
                     double measureSeconds = 0.2)
{
    auto proc = make();
    const size_t settle = static_cast<size_t>(settleSeconds * rate);
    const size_t count = std::max<size_t>(64, static_cast<size_t>(measureSeconds * rate));
    Signal out(count);
    const double w = 2.0 * kPi * hz / rate;
    for (size_t i = 0; i < settle + count; ++i)
    {
        const double y = proc(amplitude * std::sin(w * static_cast<double>(i)));
        if (i >= settle)
            out[i - settle] = y;
    }
    const Signal win = makeWindow(Window::BlackmanHarris, count);
    double sumW = 0.0;
    for (const double v : win)
        sumW += v;
    const Cx c = windowedDtft(out.data(), win.data(), count, hz, rate);
    return amplitudeToDb(2.0 * std::abs(c) / sumW / amplitude);
}

struct ResponseCurve
{
    std::vector<double> freqHz;
    std::vector<double> gainDb;
};

inline std::vector<double> logSpacedFrequencies(double loHz, double hiHz, int perOctave)
{
    std::vector<double> f;
    const int count = static_cast<int>(std::ceil(std::log2(hiHz / loHz) * perOctave)) + 1;
    for (int i = 0; i < count; ++i)
        f.push_back(loHz * std::pow(2.0, static_cast<double>(i) / perOctave));
    return f;
}

// Magnitude response by stepped sines (right for nonlinear processors: the level is set, the gain is read at it).
template <class Factory>
ResponseCurve steppedResponse(Factory&& make, const std::vector<double>& freqs, double rate, double amplitude = 0.01,
                              double settleSeconds = 0.4, double measureSeconds = 0.2)
{
    ResponseCurve r;
    for (const double f : freqs)
    {
        r.freqHz.push_back(f);
        r.gainDb.push_back(steppedGainDb(make, f, rate, amplitude, settleSeconds, measureSeconds));
    }
    return r;
}

// Linear impulse response by the exponential-sweep method: the processor runs an exponential sine sweep, the
// output is divided by the sweep's spectrum, and the causal part of the result is kept (harmonic distortion of a
// nonlinear processor lands before t = 0 and is discarded). Returns irSeconds of response.
template <class Factory>
Signal sweepImpulseResponse(Factory&& make, double rate, double f1 = 20.0, double f2 = 0.0, double sweepSeconds = 4.0,
                            double amplitude = 0.01, double irSeconds = 0.25, double tailSeconds = 0.5)
{
    if (f2 <= 0.0)
        f2 = 0.45 * rate;
    const double L = sweepSeconds / std::log(f2 / f1);
    const double K = 2.0 * kPi * f1 * L;
    const size_t sweepLen = static_cast<size_t>(sweepSeconds * rate);
    const size_t total = sweepLen + static_cast<size_t>(tailSeconds * rate);
    size_t size = 1;
    while (size < total)
        size *= 2;
    std::vector<Cx> x(size), y(size);
    auto proc = make();
    for (size_t i = 0; i < total; ++i)
    {
        const double in = i < sweepLen ? amplitude * std::sin(K * (std::exp(static_cast<double>(i) / rate / L) - 1.0)) : 0.0;
        x[i] = in;
        y[i] = proc(in);
    }
    fft(x);
    fft(y);
    double peak = 0.0;
    for (const auto& v : x)
        peak = std::max(peak, std::norm(v));
    const double eps = peak * 1.0e-10;
    std::vector<Cx> h(size);
    for (size_t k = 0; k < size; ++k)
        h[k] = y[k] * std::conj(x[k]) / (std::norm(x[k]) + eps);
    fft(h, true);
    const size_t len = std::min(size, static_cast<size_t>(irSeconds * rate));
    Signal ir(len);
    for (size_t i = 0; i < len; ++i)
    {
        // Half-Hann fade over the last quarter so a truncated tail does not ripple the magnitude.
        const double fadeStart = 0.75 * static_cast<double>(len);
        const double fade =
            static_cast<double>(i) > fadeStart
                ? 0.5 + 0.5 * std::cos(kPi * (static_cast<double>(i) - fadeStart) / (0.25 * static_cast<double>(len)))
                : 1.0;
        ir[i] = h[i].real() * fade;
    }
    return ir;
}

// Magnitude (dB) of an impulse response at the given frequencies.
inline ResponseCurve responseFromImpulse(const Signal& ir, double rate, const std::vector<double>& freqs)
{
    ResponseCurve r;
    const Signal rect(ir.size(), 1.0);
    for (const double f : freqs)
    {
        r.freqHz.push_back(f);
        r.gainDb.push_back(amplitudeToDb(std::abs(windowedDtft(ir.data(), rect.data(), ir.size(), f, rate))));
    }
    return r;
}

// Least-squares slope of gain against log2(frequency) between f1Hz and f2Hz, in dB per octave (negative when the
// response falls).
inline double slopeDbPerOctave(const ResponseCurve& c, double f1Hz, double f2Hz)
{
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    int n = 0;
    for (size_t i = 0; i < c.freqHz.size(); ++i)
    {
        if (c.freqHz[i] < f1Hz || c.freqHz[i] > f2Hz)
            continue;
        const double x = std::log2(c.freqHz[i]), y = c.gainDb[i];
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        ++n;
    }
    const double d = n * sxx - sx * sx;
    return (n >= 2 && d != 0.0) ? (n * sxy - sx * sy) / d : 0.0;
}

struct PeakInfo
{
    double freqHz = 0.0;
    double gainDb = -300.0;
};

// Maximum of a response curve; the position and height are refined by a parabola through the three points around
// the maximum (in log frequency), so they are exact to a fraction of the grid spacing.
inline PeakInfo resonancePeak(const ResponseCurve& c, double loHz = 0.0, double hiHz = 1.0e9)
{
    size_t best = 0;
    bool found = false;
    for (size_t i = 0; i < c.freqHz.size(); ++i)
    {
        if (c.freqHz[i] < loHz || c.freqHz[i] > hiHz)
            continue;
        if (!found || c.gainDb[i] > c.gainDb[best])
        {
            best = i;
            found = true;
        }
    }
    PeakInfo p;
    if (!found)
        return p;
    p.freqHz = c.freqHz[best];
    p.gainDb = c.gainDb[best];
    if (best > 0 && best + 1 < c.freqHz.size())
    {
        const double x0 = std::log2(c.freqHz[best - 1]), x1 = std::log2(c.freqHz[best]), x2 = std::log2(c.freqHz[best + 1]);
        const double y0 = c.gainDb[best - 1], y1 = c.gainDb[best], y2 = c.gainDb[best + 1];
        // Parabola through three points with unequal spacing.
        const double d01 = (y1 - y0) / (x1 - x0), d12 = (y2 - y1) / (x2 - x1);
        const double a = (d12 - d01) / (x2 - x0);
        if (a < 0.0)
        {
            const double b = d01 - a * (x0 + x1);
            const double xp = -b / (2.0 * a);
            if (xp > x0 && xp < x2)
            {
                p.freqHz = std::exp2(xp);
                p.gainDb = y0 + d01 * (xp - x0) + a * (xp - x0) * (xp - x1);
            }
        }
    }
    return p;
}

// First frequency above startHz where the response falls to levelDb or lower (log-linear interpolation); 0 if never.
inline double crossingBelow(const ResponseCurve& c, double levelDb, double startHz = 0.0)
{
    for (size_t i = 1; i < c.freqHz.size(); ++i)
    {
        if (c.freqHz[i] < startHz)
            continue;
        if (c.gainDb[i - 1] > levelDb && c.gainDb[i] <= levelDb)
        {
            const double t = (levelDb - c.gainDb[i - 1]) / (c.gainDb[i] - c.gainDb[i - 1]);
            return std::exp2(std::log2(c.freqHz[i - 1]) + t * (std::log2(c.freqHz[i]) - std::log2(c.freqHz[i - 1])));
        }
    }
    return 0.0;
}

struct SelfOscillation
{
    bool oscillating = false;
    double freqHz = 0.0;
    double levelDbfs = -300.0;         // RMS of the analysed tail
    double amplitudeVariationDb = 0.0; // max - min of the tail's per-segment RMS
};

// Self-oscillation of a processor: an impulse of kickAmplitude starts it, then it runs on silence for `seconds`.
// The last 40 % is analysed: frequency, RMS, level steadiness. "Oscillating" means the tail is above -60 dBFS and
// its last quarter is within 3 dB of its first quarter (a ringing filter that decays fails this).
template <class Factory>
SelfOscillation measureSelfOscillation(Factory&& make, double rate, double seconds = 2.0, double kickAmplitude = 0.01,
                                       double hintHz = 0.0)
{
    auto proc = make();
    const size_t total = static_cast<size_t>(seconds * rate);
    Signal out(total);
    for (size_t i = 0; i < total; ++i)
        out[i] = proc(i == 0 ? kickAmplitude : 0.0);
    const size_t from = static_cast<size_t>(0.6 * static_cast<double>(total));
    const size_t count = total - from;
    SelfOscillation r;
    r.levelDbfs = rmsDbfs(out.data() + from, count);
    const size_t segments = 8, seg = count / segments;
    double lo = 1.0e300, hi = -1.0e300;
    std::vector<double> levels;
    for (size_t s = 0; s < segments; ++s)
    {
        const double db = rmsDbfs(out.data() + from + s * seg, seg);
        levels.push_back(db);
        lo = std::min(lo, db);
        hi = std::max(hi, db);
    }
    r.amplitudeVariationDb = hi - lo;
    r.oscillating = r.levelDbfs > -60.0 && std::abs(levels.back() - levels.front()) < 3.0;
    if (r.levelDbfs > -120.0)
        r.freqHz = estimateFrequency(out.data() + from, count, rate, hintHz);
    return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Statistics: drift and other random processes

// Exact discretisation of an Ornstein-Uhlenbeck process (stationary start), standard deviation sigma, time constant tau.
inline Signal generateOu(size_t n, double dt, double tau, double sigma, std::uint64_t seed)
{
    Random rng(seed);
    const double a = std::exp(-dt / tau);
    const double b = sigma * std::sqrt(1.0 - a * a);
    Signal x(n);
    double v = sigma * rng.gaussian();
    for (size_t i = 0; i < n; ++i)
    {
        x[i] = v;
        v = a * v + b * rng.gaussian();
    }
    return x;
}

struct PsdCurve
{
    std::vector<double> freqHz;
    std::vector<double> power; // per Hz
};

// Welch periodogram (Hann window, 50 % overlap, mean removed); series sampled every dt seconds.
inline PsdCurve welchPsd(const Signal& x, double dt, size_t segment)
{
    PsdCurve out;
    if (x.size() < segment)
        return out;
    const Signal w = makeWindow(Window::Hann, segment);
    double sumW2 = 0.0;
    for (const double v : w)
        sumW2 += v * v;
    const double mean = dcOffset(x);
    std::vector<double> acc(segment / 2 + 1, 0.0);
    int count = 0;
    for (size_t start = 0; start + segment <= x.size(); start += segment / 2)
    {
        std::vector<Cx> a(segment);
        for (size_t i = 0; i < segment; ++i)
            a[i] = Cx((x[start + i] - mean) * w[i], 0.0);
        fft(a);
        for (size_t k = 0; k <= segment / 2; ++k)
            acc[k] += std::norm(a[k]);
        ++count;
    }
    const double fs = 1.0 / dt;
    for (size_t k = 1; k <= segment / 2; ++k)
    {
        out.freqHz.push_back(static_cast<double>(k) * fs / static_cast<double>(segment));
        out.power.push_back(2.0 * acc[k] / count / (fs * sumW2));
    }
    return out;
}

// Slope (log10 power against log10 frequency) between loHz and hiHz, from the PSD averaged in log-spaced bands.
inline double psdSlope(const PsdCurve& c, double loHz, double hiHz, int bandsPerOctave = 3)
{
    const int nb = std::max(2, static_cast<int>(std::log2(hiHz / loHz) * bandsPerOctave));
    std::vector<double> sum(static_cast<size_t>(nb), 0.0), logf(static_cast<size_t>(nb), 0.0);
    std::vector<int> cnt(static_cast<size_t>(nb), 0);
    for (size_t i = 0; i < c.freqHz.size(); ++i)
    {
        const double f = c.freqHz[i];
        if (f < loHz || f >= hiHz)
            continue;
        const int b = std::min(nb - 1, static_cast<int>(std::log2(f / loHz) * bandsPerOctave));
        sum[static_cast<size_t>(b)] += c.power[i];
        logf[static_cast<size_t>(b)] += std::log10(f);
        ++cnt[static_cast<size_t>(b)];
    }
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    int n = 0;
    for (int b = 0; b < nb; ++b)
    {
        if (cnt[static_cast<size_t>(b)] == 0)
            continue;
        const double x = logf[static_cast<size_t>(b)] / cnt[static_cast<size_t>(b)];
        const double y = std::log10(sum[static_cast<size_t>(b)] / cnt[static_cast<size_t>(b)]);
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        ++n;
    }
    const double d = n * sxx - sx * sx;
    return (n >= 2 && d != 0.0) ? (n * sxy - sx * sy) / d : 0.0;
}

struct OuStats
{
    double mean = 0.0;
    double stdDev = 0.0;
    double tauSeconds = 0.0; // from the autocorrelation, fitted over the lags before it falls under 0.3
    double lag1 = 0.0;       // autocorrelation at one sample
    double psdSlope = 0.0;   // log-log slope between slopeLoHz and slopeHiHz
};

// Statistics of drift series sampled every dt seconds. Several runs (one per note) are pooled: one mean, one
// variance, one autocorrelation. The PSD slope is taken over the longest run's Welch estimate.
inline OuStats ouStats(const std::vector<Signal>& runs, double dt, double slopeLoHz, double slopeHiHz)
{
    OuStats r;
    size_t total = 0, longest = 0;
    double sum = 0.0;
    for (size_t i = 0; i < runs.size(); ++i)
    {
        total += runs[i].size();
        for (const double v : runs[i])
            sum += v;
        if (runs[i].size() > runs[longest].size())
            longest = i;
    }
    if (total == 0)
        return r;
    r.mean = sum / static_cast<double>(total);
    const size_t maxLag = std::min<size_t>(runs[longest].size() / 2, 4096);
    std::vector<double> cov(maxLag + 1, 0.0);
    std::vector<double> pairs(maxLag + 1, 0.0);
    for (const auto& run : runs)
        for (size_t lag = 0; lag <= maxLag && lag < run.size(); ++lag)
        {
            double s = 0.0;
            for (size_t i = 0; i + lag < run.size(); ++i)
                s += (run[i] - r.mean) * (run[i + lag] - r.mean);
            cov[lag] += s;
            pairs[lag] += static_cast<double>(run.size() - lag);
        }
    for (size_t lag = 0; lag <= maxLag; ++lag)
        cov[lag] = pairs[lag] > 0.0 ? cov[lag] / pairs[lag] : 0.0;
    r.stdDev = std::sqrt(std::max(cov[0], 0.0));
    if (cov[0] > 0.0)
    {
        r.lag1 = cov[1] / cov[0];
        // Fit ln(rho) = -lag * dt / tau through the origin over the lags while rho stays above 0.3.
        double num = 0.0, den = 0.0;
        for (size_t lag = 1; lag <= maxLag; ++lag)
        {
            const double rho = cov[lag] / cov[0];
            if (rho < 0.3)
                break;
            const double t = static_cast<double>(lag) * dt;
            num += t * t;
            den += -t * std::log(rho);
        }
        r.tauSeconds = den > 0.0 ? num / den : 0.0;
    }
    const size_t seg = std::min<size_t>(floorPow2(runs[longest].size() / 8), 8192);
    if (seg >= 64)
        r.psdSlope = psdSlope(welchPsd(runs[longest], dt, seg), slopeLoHz, slopeHiHz);
    return r;
}
inline OuStats ouStats(const Signal& x, double dt, double slopeLoHz, double slopeHiHz)
{
    return ouStats(std::vector<Signal>{x}, dt, slopeLoHz, slopeHiHz);
}

// Pitch of a tone over time: the frequency of consecutive windows, in cents relative to referenceHz (0 = the
// mean). One value per hopSeconds; the window is windowSeconds rounded down to a power of two of samples.
inline Signal pitchTraceCents(const Signal& x, double rate, double hopSeconds, double windowSeconds,
                              double referenceHz = 0.0)
{
    const size_t window = floorPow2(static_cast<size_t>(windowSeconds * rate));
    const size_t hop = std::max<size_t>(1, static_cast<size_t>(hopSeconds * rate));
    Signal f;
    for (size_t start = 0; start + window <= x.size(); start += hop)
        f.push_back(estimateFrequency(x.data() + start, window, rate));
    if (referenceHz <= 0.0)
    {
        double s = 0.0;
        for (const double v : f)
            s += v;
        referenceHz = f.empty() ? 1.0 : s / static_cast<double>(f.size());
    }
    for (auto& v : f)
        v = centsBetween(v, referenceHz);
    return f;
}

// ---------------------------------------------------------------------------------------------------------------
// Envelopes

struct ExpFit
{
    double tau = 0.0; // time constant of y = offset + amplitude * exp(-t / tau)
    double offset = 0.0;
    double amplitude = 0.0;
    double residualRms = 0.0; // rms of the fit error as a fraction of the data's peak-to-peak range
    double residualMax = 0.0; // largest fit error as a fraction of the range
};

// Least-squares fit of y = offset + amplitude * exp(-t / tau) (decays have amplitude > 0, charging curves < 0):
// a golden-section search over tau with the two linear parameters solved exactly for each candidate.
inline ExpFit fitExponential(const Signal& t, const Signal& y)
{
    ExpFit best;
    const size_t n = std::min(t.size(), y.size());
    if (n < 4)
        return best;
    const double t0 = t[0];
    const double span = t[n - 1] - t0;
    double ymin = y[0], ymax = y[0];
    for (size_t i = 0; i < n; ++i)
    {
        ymin = std::min(ymin, y[i]);
        ymax = std::max(ymax, y[i]);
    }
    const double range = std::max(ymax - ymin, 1.0e-300);
    const auto solve = [&](double tau, double& offset, double& amp)
    {
        double s1 = 0, se = 0, see = 0, sy = 0, sye = 0;
        for (size_t i = 0; i < n; ++i)
        {
            const double e = std::exp(-(t[i] - t0) / tau);
            s1 += 1.0;
            se += e;
            see += e * e;
            sy += y[i];
            sye += y[i] * e;
        }
        const double det = s1 * see - se * se;
        if (std::abs(det) < 1.0e-300)
        {
            offset = sy / s1;
            amp = 0.0;
        }
        else
        {
            offset = (sy * see - sye * se) / det;
            amp = (s1 * sye - se * sy) / det;
        }
        double sse = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            const double d = y[i] - (offset + amp * std::exp(-(t[i] - t0) / tau));
            sse += d * d;
        }
        return sse;
    };
    double off = 0, amp = 0;
    // Coarse scan over log tau, then golden-section refinement around the best cell.
    const int cells = 240;
    const double lo = std::log(span * 1.0e-4), hi = std::log(span * 1.0e3);
    int bestCell = 0;
    double bestSse = 1.0e300;
    for (int c = 0; c <= cells; ++c)
    {
        const double sse = solve(std::exp(lo + (hi - lo) * c / cells), off, amp);
        if (sse < bestSse)
        {
            bestSse = sse;
            bestCell = c;
        }
    }
    double a = lo + (hi - lo) * std::max(0, bestCell - 1) / cells;
    double b = lo + (hi - lo) * std::min(cells, bestCell + 1) / cells;
    const double golden = 0.6180339887498949;
    double c1 = b - golden * (b - a), c2 = a + golden * (b - a);
    double f1 = solve(std::exp(c1), off, amp), f2 = solve(std::exp(c2), off, amp);
    for (int it = 0; it < 80; ++it)
    {
        if (f1 < f2)
        {
            b = c2;
            c2 = c1;
            f2 = f1;
            c1 = b - golden * (b - a);
            f1 = solve(std::exp(c1), off, amp);
        }
        else
        {
            a = c1;
            c1 = c2;
            f1 = f2;
            c2 = a + golden * (b - a);
            f2 = solve(std::exp(c2), off, amp);
        }
    }
    best.tau = std::exp(0.5 * (a + b));
    solve(best.tau, best.offset, best.amplitude);
    double sse = 0.0, worst = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const double d = std::abs(y[i] - (best.offset + best.amplitude * std::exp(-(t[i] - t0) / best.tau)));
        sse += d * d;
        worst = std::max(worst, d);
    }
    best.residualRms = std::sqrt(sse / static_cast<double>(n)) / range;
    best.residualMax = worst / range;
    return best;
}

// First time y crosses level (rising or falling), linear interpolation; negative if it never does.
inline double timeToLevel(const Signal& t, const Signal& y, double level, bool rising = true)
{
    for (size_t i = 1; i < std::min(t.size(), y.size()); ++i)
    {
        const bool crossed = rising ? (y[i - 1] < level && y[i] >= level) : (y[i - 1] > level && y[i] <= level);
        if (crossed)
            return t[i - 1] + (level - y[i - 1]) / (y[i] - y[i - 1]) * (t[i] - t[i - 1]);
    }
    return -1.0;
}

struct EnvelopeTrace
{
    Signal timeSeconds;
    Signal level; // linear
};

// RMS envelope of an audio signal: one point per hop, each from a window centred on it.
inline EnvelopeTrace rmsEnvelope(const Signal& x, double rate, double windowSeconds, double hopSeconds)
{
    EnvelopeTrace e;
    const size_t window = std::max<size_t>(1, static_cast<size_t>(windowSeconds * rate));
    const size_t hop = std::max<size_t>(1, static_cast<size_t>(hopSeconds * rate));
    for (size_t start = 0; start + window <= x.size(); start += hop)
    {
        e.timeSeconds.push_back((static_cast<double>(start) + 0.5 * static_cast<double>(window)) / rate);
        e.level.push_back(rmsOf(x.data() + start, window));
    }
    return e;
}

} // namespace crucible::measure
