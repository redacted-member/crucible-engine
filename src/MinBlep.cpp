// SPDX-License-Identifier: MIT
#include <crucible/MinBlep.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

namespace crucible
{
namespace
{
using Complex = std::complex<double>;
constexpr double kPi = 3.14159265358979323846;

void fft(std::vector<Complex>& a, bool inverse)
{
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i)
    {
        size_t bit = n >> 1U;
        for (; j & bit; bit >>= 1U)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1U)
    {
        const double ang = 2.0 * kPi / static_cast<double>(len) * (inverse ? 1.0 : -1.0);
        const Complex wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len)
        {
            Complex w(1.0, 0.0);
            for (size_t k = 0; k < len / 2; ++k)
            {
                const Complex u = a[i + k];
                const Complex v = a[i + k + len / 2] * w;
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
} // namespace

MinBlep::MinBlep()
{
    constexpr int zeroCrossings = 32;
    constexpr int kernelLength = 2 * zeroCrossings * oversample + 1;
    constexpr size_t fftSize = 1U << 15U; // plenty of room for the cepstrum (no time aliasing)

    // Blackman-Harris windowed sinc at the output Nyquist.
    std::vector<Complex> h(fftSize, Complex(0.0, 0.0));
    double dc = 0.0;
    for (int n = 0; n < kernelLength; ++n)
    {
        const double x = static_cast<double>(n - kernelLength / 2) / oversample;
        const double sinc = std::abs(x) < 1.0e-12 ? 1.0 : std::sin(kPi * x) / (kPi * x);
        const double r = static_cast<double>(n) / (kernelLength - 1);
        const double w =
            0.35875 - 0.48829 * std::cos(2 * kPi * r) + 0.14128 * std::cos(4 * kPi * r) - 0.01168 * std::cos(6 * kPi * r);
        h[static_cast<size_t>(n)] = sinc * w;
        dc += sinc * w;
    }

    // Minimum phase by the real cepstrum.
    fft(h, false);
    std::vector<Complex> cep(fftSize);
    for (size_t i = 0; i < fftSize; ++i)
        cep[i] = Complex(std::log(std::max(std::abs(h[i]), 1.0e-12)), 0.0);
    fft(cep, true);
    std::vector<Complex> folded(fftSize, Complex(0.0, 0.0));
    folded[0] = cep[0];
    for (size_t i = 1; i < fftSize / 2; ++i)
        folded[i] = 2.0 * cep[i];
    folded[fftSize / 2] = cep[fftSize / 2];
    fft(folded, false);
    for (auto& v : folded)
        v = std::exp(v);
    fft(folded, true);

    // Integrate to a step, normalise to end at one, keep the first `length` samples as the correction (step - 1).
    const size_t keep = static_cast<size_t>(length * oversample) + 1;
    std::vector<double> step(fftSize, 0.0);
    double sum = 0.0;
    for (size_t i = 0; i < fftSize; ++i)
    {
        sum += folded[i].real();
        step[i] = sum;
    }
    // The DC gain of the kernel (identical for the minimum-phase version) normalises the step.
    const double total = dc;
    std::vector<double> c(keep);
    for (size_t i = 0; i < keep; ++i)
        c[i] = step[i] / total - 1.0;
    // Integrated correction (minBLAMP): C(x) = integral of c, minus its tail times the step so it settles at zero.
    std::vector<double> integral(keep, 0.0);
    for (size_t i = 1; i < keep; ++i)
        integral[i] = integral[i - 1] + 0.5 * (c[i] + c[i - 1]) / oversample;
    const double tail = integral[keep - 1];
    for (size_t i = 0; i < keep; ++i)
        ramp[i] = static_cast<float>(integral[i] - tail * (c[i] + 1.0));
    ramp[keep - 1] = 0.0f;
    ramp[keep] = 0.0f;
    for (size_t i = 0; i < keep; ++i)
        table[i] = static_cast<float>(c[i]);
    // Force the tail to zero smoothly over the last sample so truncation cannot leave a step.
    const size_t fade = static_cast<size_t>(oversample);
    for (size_t i = 0; i < fade; ++i)
    {
        const double w = static_cast<double>(i) / static_cast<double>(fade);
        const size_t k = keep - fade + i;
        table[k] = static_cast<float>(table[k] * (0.5 + 0.5 * std::cos(kPi * w)));
    }
    table[keep - 1] = 0.0f;
    table[keep] = 0.0f;
}

const MinBlep& MinBlep::instance()
{
    static const MinBlep m;
    return m;
}

} // namespace crucible
