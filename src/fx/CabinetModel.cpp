// SPDX-License-Identifier: MIT
#include <crucible/fx/CabinetModel.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>

namespace crucible::fx
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
using Cx = std::complex<double>;

// Radix-2 in-place complex FFT (control thread only; double precision keeps the cepstrum clean).
void fftInPlace(std::vector<Cx>& a, bool inverse)
{
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i)
    {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1)
    {
        const double ang = 2.0 * kPi / static_cast<double>(len) * (inverse ? 1.0 : -1.0);
        const Cx wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len)
        {
            Cx w(1.0, 0.0);
            for (size_t k = 0; k < len / 2; ++k)
            {
                const Cx u = a[i + k], v = a[i + k + len / 2] * w;
                a[i + k] = u + v;
                a[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
    if (inverse)
        for (auto& x : a)
            x /= static_cast<double>(n);
}

const CabinetSpec kCabinets[kCabinetCount] = {
    // name, resonance Hz, Q, ported, rolloff Hz, order, bells {hz, dB, q}, count, comb ms, comb gain, notch Hz
    {"1x12 Open Back", 105.0, 1.1, false, 5200.0, 3, {{2600.0, 4.5, 1.6}, {1500.0, -2.0, 1.2}, {3900.0, 2.0, 2.0}, {}}, 3, 0.62, 0.30, 2300.0},
    {"2x12 Open Back", 92.0, 1.0, false, 5000.0, 3, {{2300.0, 4.0, 1.4}, {1100.0, -2.5, 1.2}, {}, {}}, 2, 0.78, 0.30, 2100.0},
    {"4x12 Closed", 98.0, 1.35, false, 4800.0, 4, {{2200.0, 5.0, 1.3}, {4200.0, 3.0, 1.8}, {1300.0, 2.0, 1.4}, {}}, 3, 0.33, 0.08, 2700.0},
    {"1x10 Combo", 128.0, 0.95, false, 5900.0, 3, {{3200.0, 3.5, 1.6}, {1800.0, -2.0, 1.4}, {}, {}}, 2, 0.5, 0.28, 2900.0},
    {"2x10 Open Back", 118.0, 1.05, false, 5600.0, 3, {{3000.0, 4.0, 1.5}, {1700.0, -1.5, 1.3}, {4300.0, 1.5, 2.0}, {}}, 3, 0.55, 0.30, 2600.0},
    {"1x15 Bass", 55.0, 1.2, true, 3200.0, 2, {{1800.0, 2.0, 1.2}, {900.0, -1.5, 1.0}, {}, {}}, 2, 0.0, 0.0, 1900.0},
    {"4x10 Bass", 64.0, 1.15, true, 4500.0, 3, {{2800.0, 3.0, 1.4}, {1400.0, 1.5, 1.2}, {}, {}}, 2, 0.0, 0.0, 2400.0},
    {"8x10 Bass", 50.0, 1.3, true, 3800.0, 3, {{2400.0, 2.5, 1.3}, {1000.0, -2.0, 1.1}, {4400.0, 1.0, 2.0}, {}}, 3, 0.0, 0.0, 2000.0},
};

double hp2(double f, double f0, double q)
{
    const double x = f / f0, x2 = x * x;
    return x2 / std::sqrt((1.0 - x2) * (1.0 - x2) + x2 / (q * q));
}

double smooth(double t)
{
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double bellDb(double f, double hz, double gainDb, double q)
{
    const double z = std::log(f / hz) / (0.6 / q);
    return gainDb * std::exp(-0.5 * z * z);
}

struct Corners
{
    double highPassHz = 100.0; // corner of the second-order high-pass
    double lowPassHz = 5000.0; // -3 dB point of the roll-off stage
};

double rawMagnitudeAt(const CabinetSettings& s, double f, const Corners& k);

// Power averaged over a one-octave window, in dB (the comb ripple of an open back would otherwise make the crossing of a
// level ambiguous).
double smoothedDb(const CabinetSettings& s, double f, const Corners& k)
{
    double sum = 0.0;
    const int n = 21;
    for (int i = 0; i < n; ++i)
    {
        const double g = f * std::pow(2.0, 0.5 * (2.0 * i / (n - 1) - 1.0));
        const double m = rawMagnitudeAt(s, g, k);
        sum += m * m;
    }
    return 10.0 * std::log10(sum / n);
}

// Finds, once per cabinet and for the default microphone (on the cap, 8 cm), the two corners at which the finished response
// (reflections, bells, proximity shelf and tilt included) shows its designed features: the low-frequency peak at
// resonanceHz, and the top end falling 3 dB below the highest level of the presence region (0.3 * rolloffHz up to
// rolloffHz) at rolloffHz. A few fixed-point steps each.
Corners calibratedCorners(int cabinet)
{
    static const std::array<Corners, kCabinetCount> table = []
    {
        std::array<Corners, kCabinetCount> t{};
        for (int i = 0; i < kCabinetCount; ++i)
        {
            const CabinetSpec& c = kCabinets[i];
            CabinetSettings s;
            s.cabinet = i;
            s.micPosition = 0.0f;
            s.micDistanceCm = 8.0f;
            Corners k;
            k.highPassHz = c.resonanceHz * std::sqrt(1.0 - 1.0 / (2.0 * c.resonanceQ * c.resonanceQ));
            k.lowPassHz = c.rolloffHz;
            for (int round = 0; round < 14; ++round)
            {
                double best = -1.0, bestHz = c.resonanceHz;
                for (double f = c.resonanceHz * 0.5; f < c.resonanceHz * 2.0; f *= 1.0005)
                {
                    const double m = rawMagnitudeAt(s, f, k);
                    if (m > best)
                    {
                        best = m;
                        bestHz = f;
                    }
                }
                k.highPassHz *= c.resonanceHz / bestHz;

                best = -1.0e9;
                double topHz = 0.3 * c.rolloffHz;
                for (double f = 0.3 * c.rolloffHz; f < c.rolloffHz; f *= 1.01)
                {
                    const double m = smoothedDb(s, f, k);
                    if (m > best)
                    {
                        best = m;
                        topHz = f;
                    }
                }
                // The last frequency at which the level is still within 3 dB of the peak (dips in between do not count).
                double crossing = 18000.0;
                while (crossing > topHz && smoothedDb(s, crossing, k) < best - 3.0)
                    crossing /= 1.003;
                k.lowPassHz = std::clamp(k.lowPassHz * std::pow(c.rolloffHz / crossing, 0.8), 0.4 * c.rolloffHz, 2.5 * c.rolloffHz);
            }
            t[static_cast<size_t>(i)] = k;
        }
        return t;
    }();
    return table[static_cast<size_t>(std::clamp(cabinet, 0, kCabinetCount - 1))];
}

double rawMagnitude(const CabinetSettings& s, double f)
{
    return rawMagnitudeAt(s, f, calibratedCorners(s.cabinet));
}

// Magnitude before normalisation.
double rawMagnitudeAt(const CabinetSettings& s, double f, const Corners& k)
{
    const CabinetSpec& c = cabinetSpec(s.cabinet);
    f = std::max(f, 1.0e-3);
    const double pos = std::clamp(static_cast<double>(s.micPosition), 0.0, 1.0);
    const double dist = std::clamp(static_cast<double>(s.micDistanceCm), 0.0, 60.0);

    // Box resonance: a second-order high-pass whose peak is at resonanceHz.
    const double q = c.resonanceQ;
    double mag = hp2(f, k.highPassHz, q);
    if (c.ported)
        mag *= hp2(f, 0.55 * k.highPassHz, 0.6);

    // Roll-off.
    mag /= std::sqrt(1.0 + std::pow(f / k.lowPassHz, 2.0 * c.rolloffOrder));

    double db = 0.0;
    for (int i = 0; i < c.bellCount; ++i)
        db += bellDb(f, c.bells[i].hz, c.bells[i].gainDb, c.bells[i].q);

    // Microphone position: off-axis tilt of the top end, and the dust-cap notch moving down and deepening.
    db -= 9.0 * pos * smooth((std::log2(f / 1500.0)) / 2.0);
    db -= 5.0 * pos * std::exp(-0.5 * std::pow(std::log(f / (c.notchHz * (1.0 - 0.3 * pos))) / 0.15, 2.0));
    // Air loss with distance.
    db -= 0.02 * dist * (f / 5000.0);

    mag *= std::pow(10.0, db / 20.0);

    // Back-wall / baffle reflection.
    if (c.combGain > 0.0)
    {
        // The cone beams its top end forward, so little of it reaches the back wall: the reflection fades above 1.5 kHz.
        const double g = c.combGain / (1.0 + (f / 1500.0) * (f / 1500.0));
        mag *= std::sqrt(1.0 + g * g + 2.0 * g * std::cos(2.0 * kPi * f * c.combDelayMs * 1.0e-3));
    }

    // Proximity effect: a first-order low shelf, up to +5 dB at the grille, gone at about 30 cm.
    const double g = std::pow(10.0, 5.0 * std::exp(-dist / 10.0) / 20.0), fc = 70.0;
    mag *= std::sqrt((f * f + g * g * fc * fc) / (f * f + fc * fc));
    return mag;
}

double normalisation(const CabinetSettings& s)
{
    double sum = 0.0;
    int n = 0;
    for (double f = 400.0; f <= 2500.0; f *= 1.02, ++n)
    {
        const double m = rawMagnitude(s, f);
        sum += m * m;
    }
    return 1.0 / std::sqrt(sum / n);
}
} // namespace

const CabinetSpec& cabinetSpec(int index)
{
    return kCabinets[std::clamp(index, 0, kCabinetCount - 1)];
}

double cabinetResonanceHz(int cabinet)
{
    return cabinetSpec(cabinet).resonanceHz;
}

int cabinetTapCount(double sampleRate)
{
    return std::clamp(static_cast<int>(std::lround(2048.0 * sampleRate / 48000.0)), 256, 8192);
}

double cabinetMagnitude(const CabinetSettings& settings, double hz)
{
    return rawMagnitude(settings, hz) * normalisation(settings);
}

void generateCabinetIr(const CabinetSettings& settings, double rate, std::vector<float>& out)
{
    const int taps = cabinetTapCount(rate);
    size_t n = 1;
    while (n < static_cast<size_t>(taps) * 8)
        n <<= 1;
    const double norm = normalisation(settings);

    // Real cepstrum of the log magnitude, folded to the causal side: the minimum-phase filter with that magnitude.
    std::vector<Cx> a(n);
    for (size_t k = 0; k <= n / 2; ++k)
    {
        const double f = static_cast<double>(k) * rate / static_cast<double>(n);
        const double m = std::max(rawMagnitude(settings, f) * norm, 1.0e-4);
        a[k] = Cx(std::log(m), 0.0);
        if (k > 0 && k < n / 2)
            a[n - k] = a[k];
    }
    fftInPlace(a, true);
    for (size_t i = 1; i < n / 2; ++i)
        a[i] = Cx(2.0 * a[i].real(), 0.0);
    for (size_t i = n / 2 + 1; i < n; ++i)
        a[i] = Cx(0.0, 0.0);
    a[0] = Cx(a[0].real(), 0.0);
    a[n / 2] = Cx(a[n / 2].real(), 0.0);
    fftInPlace(a, false);
    for (auto& x : a)
        x = std::exp(x);
    fftInPlace(a, true);

    std::vector<double> h(static_cast<size_t>(taps));
    for (int i = 0; i < taps; ++i)
        h[static_cast<size_t>(i)] = a[static_cast<size_t>(i)].real();

    // One early reflection at the microphone distance (the room behind and around the cabinet, heard more the farther away).
    const double dist = std::clamp(static_cast<double>(settings.micDistanceCm), 0.0, 60.0);
    const double gRoom = 0.35 * std::pow(dist / 60.0, 1.5);
    const int delay = static_cast<int>(std::lround(rate * (0.0004 + 2.0 * dist * 0.01 / 343.0)));
    if (gRoom > 0.0 && delay < taps)
        for (int i = taps - 1; i >= delay; --i)
            h[static_cast<size_t>(i)] += gRoom * h[static_cast<size_t>(i - delay)];

    // Fade out the last fifth so that the truncation leaves no step.
    const int fadeStart = taps - taps / 5;
    out.assign(static_cast<size_t>(taps), 0.0f);
    for (int i = 0; i < taps; ++i)
    {
        double w = 1.0;
        if (i >= fadeStart)
        {
            const double c = std::cos(0.5 * kPi * static_cast<double>(i - fadeStart) / static_cast<double>(taps - fadeStart));
            w = c * c;
        }
        out[static_cast<size_t>(i)] = static_cast<float>(h[static_cast<size_t>(i)] * w);
    }
}

} // namespace crucible::fx
