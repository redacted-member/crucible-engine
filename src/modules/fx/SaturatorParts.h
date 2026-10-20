// SPDX-License-Identifier: MIT
#pragma once

// Pieces of the oversampled saturation path shared by fx.saturator and fx.multiband_drive: the flattening filter that
// undoes the low-pass of second-order antiderivative anti-aliasing, and the whole-sample padding that brings every model
// to the same latency.

#include <algorithm>
#include <array>
#include <cmath>

namespace crucible::modules::fx::detail
{
constexpr double kPiD = 3.14159265358979323846;

// Second-order ADAA turns a linear curve into the mean of three samples at the oversampled rate, a gentle low-pass
// (-0.3 dB at 20 kHz when the shaper runs at 384 kHz). A symmetric seven-tap filter at the base rate, three samples of
// delay, undoes it up to 20 kHz: its three free taps are fitted by least squares to the inverse of that response (unity
// at DC), which leaves 0.04 dB at 4x, 0.01 dB at 8x and 0.002 dB at 16x between 0 and 20 kHz.
struct Flatten
{
    static constexpr int kTaps = 3; // taps on each side of the centre
    std::array<double, kTaps + 1> g{1.0, 0.0, 0.0, 0.0};
    std::array<float, 2 * kTaps + 520> work{};

    void design(double baseRate, double osRate)
    {
        constexpr int points = 256;
        double m[kTaps][kTaps + 1] = {};
        for (int i = 0; i < points; ++i)
        {
            const double f = 20000.0 * (i + 0.5) / points;
            const double h = (1.0 + 2.0 * std::cos(2.0 * kPiD * f / osRate)) / 3.0;
            const double wb = 2.0 * kPiD * f / baseRate;
            double basis[kTaps];
            for (int k = 0; k < kTaps; ++k)
                basis[k] = 2.0 * (std::cos((k + 1) * wb) - 1.0) * h;
            const double target = (1.0 / h - 1.0) * h;
            for (int r = 0; r < kTaps; ++r)
            {
                for (int c = 0; c < kTaps; ++c)
                    m[r][c] += basis[r] * basis[c];
                m[r][kTaps] += basis[r] * target;
            }
        }
        for (int c = 0; c < kTaps; ++c) // Gauss-Jordan on the normal equations
        {
            const double pivot = m[c][c];
            for (int k = c; k <= kTaps; ++k)
                m[c][k] /= pivot;
            for (int r = 0; r < kTaps; ++r)
                if (r != c)
                {
                    const double f = m[r][c];
                    for (int k = c; k <= kTaps; ++k)
                        m[r][k] -= f * m[c][k];
                }
        }
        double sum = 0.0;
        for (int k = 1; k <= kTaps; ++k)
        {
            g[static_cast<size_t>(k)] = m[k - 1][kTaps];
            sum += 2.0 * g[static_cast<size_t>(k)];
        }
        g[0] = 1.0 - sum;
        work.fill(0.0f);
    }
    void clear() { work.fill(0.0f); }
    void run(float* p, int n)
    {
        constexpr int h = 2 * kTaps;
        float* w = work.data();
        std::copy(p, p + n, w + h);
        for (int i = 0; i < n; ++i)
        {
            const int c = i + kTaps;
            double acc = g[0] * static_cast<double>(w[c]);
            for (int k = 1; k <= kTaps; ++k)
                acc += g[static_cast<size_t>(k)] * (static_cast<double>(w[c - k]) + static_cast<double>(w[c + k]));
            p[i] = static_cast<float>(acc);
        }
        std::copy(w + n, w + n + h, w);
    }
};

// Whole-sample delay at the oversampled rate (up to 64 samples), to bring every model to the same latency.
struct Pad
{
    static constexpr int kMax = 64;
    std::array<float, kMax> hist{};
    void clear() { hist.fill(0.0f); }
    // The same delay without moving the block: p has kMax readable and writable floats in front of it (the buffer's
    // history room). The returned pointer, delay samples before p, holds the delayed block (n samples, the history
    // first); p itself is not touched.
    const float* delayed(float* p, int n, int delay)
    {
        std::copy(hist.begin(), hist.begin() + delay, p - delay);
        std::copy(p + n - delay, p + n, hist.begin());
        return p - delay;
    }
    void run(float* p, int n, int delay)
    {
        std::array<float, 2 * kMax> tmp;
        if (n >= delay)
        {
            std::copy(p + n - delay, p + n, tmp.begin());
            std::copy_backward(p, p + n - delay, p + n);
            std::copy(hist.begin(), hist.begin() + delay, p);
            std::copy(tmp.begin(), tmp.begin() + delay, hist.begin());
        }
        else
        {
            std::copy(hist.begin(), hist.begin() + delay, tmp.begin());
            std::copy(p, p + n, tmp.begin() + delay);
            std::copy(tmp.begin(), tmp.begin() + n, p);
            std::copy(tmp.begin() + n, tmp.begin() + n + delay, hist.begin());
        }
    }
};

} // namespace crucible::modules::fx::detail
