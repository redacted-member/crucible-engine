// SPDX-License-Identifier: MIT
// Linkwitz-Riley tree crossover and its linear-phase counterpart. See include/crucible/fx/Crossover.h.

#include <crucible/fx/Crossover.h>

#include <algorithm>
#include <cmath>

namespace crucible::fx
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kChunk = 128;

int butterworthOrder(CrossoverSlope s)
{
    switch (s)
    {
    case CrossoverSlope::Db6:
        return 0;
    case CrossoverSlope::Db12:
        return 1;
    case CrossoverSlope::Db24:
        return 2;
    case CrossoverSlope::Db36:
        return 3;
    default:
        return 4;
    }
}

void adjustFrequencies(int bands, const double* in, double sampleRate, double* out)
{
    double previous = 10.0;
    for (int i = 0; i < bands - 1; ++i)
    {
        double f = in[i];
        f = std::clamp(f, 10.0, 0.45 * sampleRate);
        if (i > 0 && f < previous * 1.01)
            f = previous * 1.01;
        out[i] = std::min(f, 0.46 * sampleRate);
        previous = out[i];
    }
}
} // namespace

// ---------------------------------------------------------------------------------------------------------------
void Crossover::prepare(double sampleRate)
{
    sr = sampleRate;
    reset();
}

void Crossover::reset()
{
    for (auto& s : split)
    {
        s.lp.reset();
        s.hp.reset();
    }
    for (auto& row : comp)
        for (auto& c : row)
            c.reset();
}

void Crossover::designSplit(Split& s, double frequency, CrossoverSlope slope, int ramp)
{
    s.frequency = frequency;
    s.lpDesign = {};
    s.hpDesign = {};
    s.apDesign = {};
    const int n = butterworthOrder(slope);
    if (n == 0)
    {
        s.lpDesign.section[0] = bilinearSection(analogPrototype(SectionKind::LowPass, 0.0, 0.7071, 1), frequency, sr);
        s.lpDesign.count = 1;
        s.hpDesign.section[0] = bilinearSection(analogPrototype(SectionKind::HighPass, 0.0, 0.7071, 1), frequency, sr);
        s.hpDesign.count = 1;
    }
    else
    {
        PolePair poles[8];
        const int count = butterworthPoles(n, poles);
        for (int rep = 0; rep < 2; ++rep)
            for (int i = 0; i < count; ++i)
            {
                const int order = poles[i].real ? 1 : 2;
                s.lpDesign.section[static_cast<size_t>(s.lpDesign.count++)] =
                    bilinearSection(analogPrototype(SectionKind::LowPass, 0.0, poles[i].q, order), frequency, sr);
                s.hpDesign.section[static_cast<size_t>(s.hpDesign.count++)] =
                    bilinearSection(analogPrototype(SectionKind::HighPass, 0.0, poles[i].q, order), frequency, sr);
            }
        if (n % 2 == 1) // the high-pass is inverted so that low-pass + high-pass is an all-pass
        {
            auto& first = s.hpDesign.section[0];
            first.b0 = -first.b0;
            first.b1 = -first.b1;
            first.b2 = -first.b2;
        }
        for (int i = 0; i < count; ++i)
            s.apDesign.section[static_cast<size_t>(s.apDesign.count++)] = bilinearSection(
                analogPrototype(SectionKind::AllPass, 0.0, poles[i].q, poles[i].real ? 1 : 2), frequency, sr);
    }
    s.lp.set(s.lpDesign.section.data(), s.lpDesign.count, ramp);
    s.hp.set(s.hpDesign.section.data(), s.hpDesign.count, ramp);
}

void Crossover::set(int bands, const double* freqsHz, CrossoverSlope slope, int rampSamples)
{
    bands = std::clamp(bands, 2, kMaxCrossoverBands);
    double f[kMaxCrossoverBands];
    adjustFrequencies(bands, freqsHz, sr, f);
    const bool topology = bands != numBands || slope != slopeNow;
    if (topology)
    {
        // A different band count or slope: the states mean nothing any more.
        for (auto& s : split)
        {
            s.lp.reset();
            s.hp.reset();
        }
        for (auto& row : comp)
            for (auto& c : row)
                c.reset();
        rampSamples = 0;
    }
    numBands = bands;
    slopeNow = slope;
    for (int j = 0; j < bands - 1; ++j)
        designSplit(split[static_cast<size_t>(j)], f[j], slope, rampSamples);
    for (int j = 0; j + 2 < bands; ++j)
        for (int m = j + 1; m < bands - 1; ++m)
        {
            const Design& ap = split[static_cast<size_t>(m)].apDesign;
            comp[static_cast<size_t>(j)][static_cast<size_t>(m - j - 1)].set(ap.section.data(), ap.count, rampSamples);
        }
}

void Crossover::process(const float* l, const float* r, int n, float* const* bandL, float* const* bandR) noexcept
{
    double xa[kChunk], xb[kChunk], la[kChunk], lb[kChunk];
    for (int pos = 0; pos < n; pos += kChunk)
    {
        const int m = std::min(kChunk, n - pos);
        for (int i = 0; i < m; ++i)
        {
            xa[i] = l[pos + i];
            xb[i] = r[pos + i];
        }
        for (int j = 0; j < numBands - 1; ++j)
        {
            std::copy(xa, xa + m, la);
            std::copy(xb, xb + m, lb);
            Split& s = split[static_cast<size_t>(j)];
            s.lp.process(la, lb, m);
            s.hp.process(xa, xb, m);
            for (int k = j + 1; k < numBands - 1; ++k)
                comp[static_cast<size_t>(j)][static_cast<size_t>(k - j - 1)].process(la, lb, m);
            for (int i = 0; i < m; ++i)
            {
                bandL[j][pos + i] = static_cast<float>(la[i]);
                bandR[j][pos + i] = static_cast<float>(lb[i]);
            }
        }
        for (int i = 0; i < m; ++i)
        {
            bandL[numBands - 1][pos + i] = static_cast<float>(xa[i]);
            bandR[numBands - 1][pos + i] = static_cast<float>(xb[i]);
        }
    }
}

std::complex<double> Crossover::chainResponse(const Design& d, double omega) const
{
    std::complex<double> h = 1.0;
    for (int i = 0; i < d.count; ++i)
        h *= biquadResponse(d.section[static_cast<size_t>(i)], omega);
    return h;
}

std::complex<double> Crossover::response(int band, double hz) const
{
    const double omega = 2.0 * kPi * hz / sr;
    std::complex<double> h = 1.0;
    for (int m = 0; m < std::min(band, numBands - 1); ++m)
        h *= chainResponse(split[static_cast<size_t>(m)].hpDesign, omega);
    if (band < numBands - 1)
    {
        h *= chainResponse(split[static_cast<size_t>(band)].lpDesign, omega);
        for (int m = band + 1; m < numBands - 1; ++m)
            h *= chainResponse(split[static_cast<size_t>(m)].apDesign, omega);
    }
    return h;
}

std::complex<double> Crossover::sumResponse(double hz) const
{
    std::complex<double> sum = 0.0;
    for (int b = 0; b < numBands; ++b)
        sum += response(b, hz);
    return sum;
}

// ---------------------------------------------------------------------------------------------------------------
void LinearCrossover::prepare(double sampleRate)
{
    sr = sampleRate;
    designer.prepare(kMaxKernelLog2);
    convolver.prepare();
}

void LinearCrossover::set(int bands, const double* freqsHz, int order, int lengthLog2)
{
    bands = std::clamp(bands, 2, kMaxCrossoverBands);
    lengthLog2 = std::clamp(lengthLog2, kMinKernelLog2, kMaxKernelLog2);
    order = std::clamp(order + (order % 2), 2, 16);
    double f[kMaxCrossoverBands];
    adjustFrequencies(bands, freqsHz, sr, f);

    const int points = LinearPhaseDesigner::gridPoints(lengthLog2);
    const int length = kernelTaps(lengthLog2), centre = kernelCentre(lengthLog2);
    magnitude.assign(static_cast<size_t>(points), 0.0f);
    std::vector<std::vector<float>> cumulative(static_cast<size_t>(bands - 1),
                                               std::vector<float>(static_cast<size_t>(length)));
    for (int k = 0; k < bands - 1; ++k)
    {
        for (int j = 0; j < points; ++j)
        {
            const double hz = LinearPhaseDesigner::gridOmega(lengthLog2, j) * sr / (2.0 * kPi);
            magnitude[static_cast<size_t>(j)] =
                static_cast<float>(1.0 / (1.0 + std::pow(hz / f[k], static_cast<double>(order))));
        }
        designer.design(lengthLog2, magnitude.data(), cumulative[static_cast<size_t>(k)].data());
    }
    // Band k is the difference of two cumulative low-passes, so the sum telescopes to the unit impulse at the centre.
    taps.assign(static_cast<size_t>(bands), std::vector<float>(static_cast<size_t>(length), 0.0f));
    for (int b = 0; b < bands; ++b)
        for (int i = 0; i < length; ++i)
        {
            const double upper = b < bands - 1
                                     ? static_cast<double>(cumulative[static_cast<size_t>(b)][static_cast<size_t>(i)])
                                     : (i == centre ? 1.0 : 0.0);
            const double lower =
                b > 0 ? static_cast<double>(cumulative[static_cast<size_t>(b - 1)][static_cast<size_t>(i)]) : 0.0;
            taps[static_cast<size_t>(b)][static_cast<size_t>(i)] = static_cast<float>(upper - lower);
        }

    const float* pointers[2 * kMaxOutputs] = {};
    for (int b = 0; b < bands; ++b)
        for (int ch = 0; ch < 2; ++ch)
            pointers[(2 * b + ch) * 2 + ch] = taps[static_cast<size_t>(b)].data();
    convolver.mailbox().post(builder.build(lengthLog2, 2 * bands, pointers, ++generation));
    convolver.mailbox().collect();
    numBands = bands;
    lengthLog2Now = lengthLog2;
}

void LinearCrossover::process(const float* l, const float* r, int n, float* const* bandL, float* const* bandR) noexcept
{
    float* out[kMaxOutputs];
    const int count = numBands;
    for (int b = 0; b < count; ++b)
    {
        out[2 * b] = bandL[b];
        out[2 * b + 1] = bandR[b];
    }
    convolver.process(l, r, out, 2 * count, n);
}

} // namespace crucible::fx
