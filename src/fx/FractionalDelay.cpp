// SPDX-License-Identifier: MIT
#include <crucible/fx/FractionalDelay.h>

#include <algorithm>
#include <cmath>

namespace crucible::fx
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double besselI0(double x)
{
    double sum = 1.0, term = 1.0;
    const double q = 0.25 * x * x;
    for (int k = 1; k < 80; ++k)
    {
        term *= q / (static_cast<double>(k) * static_cast<double>(k));
        sum += term;
        if (term < 1.0e-17 * sum)
            break;
    }
    return sum;
}

// Kaiser beta for a stopband of about 80 dB over a transition that the tap count allows.
double betaFor(int taps)
{
    return taps <= 16 ? 5.0 : (taps <= 32 ? 6.6 : 8.0);
}
} // namespace

void FractionalDelay::prepare(double maxDelay, int taps)
{
    numTaps = std::clamp(taps & ~1, 8, 64);
    minD = static_cast<double>(numTaps / 2 - 1);
    maxDelaySamples = std::max(maxDelay, minD + 1.0);
    std::size_t need = static_cast<std::size_t>(maxDelaySamples) + static_cast<std::size_t>(numTaps) + 4;
    size = 1;
    while (size < need)
        size <<= 1;
    mask = size - 1;
    maxD = static_cast<double>(size) - static_cast<double>(numTaps) - 2.0;
    maxD = std::min(maxD, maxDelaySamples);
    buffer.assign(size * 2, 0.0f);
    writePos = 0;

    // Phase p holds the kernel for a fractional delay p / kPhases. Tap k (oldest first) sits at lag
    // whole - (numTaps/2 - 1) + (numTaps - 1 - k) ... measured from the position: t = lag - (whole + f).
    const double beta = betaFor(numTaps);
    const double norm = besselI0(beta);
    const double half = static_cast<double>(numTaps) / 2.0;
    table.assign(static_cast<std::size_t>(kPhases + 1) * static_cast<std::size_t>(numTaps), 0.0f);
    std::vector<double> h(static_cast<std::size_t>(numTaps));
    for (int p = 0; p <= kPhases; ++p)
    {
        const double f = static_cast<double>(p) / kPhases;
        double sum = 0.0;
        for (int k = 0; k < numTaps; ++k)
        {
            // Oldest sample first: tap k is at lag whole + numTaps/2 - k, so its distance from the position is
            // (numTaps/2 - k) - f.
            const double t = (static_cast<double>(numTaps / 2 - k)) - f;
            const double s = std::abs(t) < 1.0e-12 ? 1.0 : std::sin(kPi * t) / (kPi * t);
            const double r = t / half;
            const double w = std::abs(r) >= 1.0 ? 0.0 : besselI0(beta * std::sqrt(1.0 - r * r)) / norm;
            h[static_cast<std::size_t>(k)] = s * w;
            sum += s * w;
        }
        for (int k = 0; k < numTaps; ++k)
            table[static_cast<std::size_t>(p) * static_cast<std::size_t>(numTaps) + static_cast<std::size_t>(k)] =
                static_cast<float>(h[static_cast<std::size_t>(k)] / sum);
    }
}

void FractionalDelay::reset()
{
    std::fill(buffer.begin(), buffer.end(), 0.0f);
    writePos = 0;
}

} // namespace crucible::fx
