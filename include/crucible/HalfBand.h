// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <crucible/Simd.h>

namespace crucible
{

// Linear-phase half-band FIR (47 taps, Kaiser window) used for 2x oversampling.
// Group delay is kHalfBandDelay samples at the oversampled rate.
struct HalfBandKernel
{
    static constexpr int taps = 47;
    static constexpr int centre = 23;
    static constexpr int pairs = 12; // non-zero off-centre taps on each side

    // odd[j] is the tap at distance 2j + 1 from the centre; centreTap is 0.5.
    std::array<float, pairs> odd{};

    static double besselI0(double x)
    {
        double sum = 1.0;
        double term = 1.0;
        for (int k = 1; k < 40; ++k)
        {
            term *= (x / (2.0 * k)) * (x / (2.0 * k));
            sum += term;
        }
        return sum;
    }

    HalfBandKernel()
    {
        constexpr double pi = 3.14159265358979323846;
        constexpr double beta = 7.0;
        std::array<double, taps> h{};
        double total = 0.0;
        for (int k = 0; k < taps; ++k)
        {
            const int n = k - centre;
            double v = 0.0;
            if (n == 0)
                v = 0.5;
            else if (n % 2 != 0)
                v = std::sin(pi * n / 2.0) / (pi * n);
            const double r = static_cast<double>(n) / centre;
            const double w = besselI0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / besselI0(beta);
            h[static_cast<size_t>(k)] = v * w;
            total += v * w;
        }
        for (int j = 0; j < pairs; ++j)
            odd[static_cast<size_t>(j)] = static_cast<float>(h[static_cast<size_t>(centre + 2 * j + 1)] / total);
    }

    static const HalfBandKernel& instance()
    {
        static const HalfBandKernel k;
        return k;
    }

    static constexpr float centreTap = 0.5f;
};

constexpr int kHalfBandDelay = HalfBandKernel::centre;

// Decimate by two: feed two oversampled samples, get one output.
class HalfBandDecimator
{
public:
    void reset()
    {
        buffer.fill(0.0f);
        pos = 0;
    }

    float process(float first, float second)
    {
        push(first);
        push(second);
        // Newest sample sits at pos - 1; the centre tap is centre samples older.
        const auto& k = HalfBandKernel::instance();
        const int c = pos - 1 - HalfBandKernel::centre;
        float acc = HalfBandKernel::centreTap * at(c);
        for (int j = 0; j < HalfBandKernel::pairs; ++j)
            acc += k.odd[static_cast<size_t>(j)] * (at(c - (2 * j + 1)) + at(c + (2 * j + 1)));
        return acc;
    }

    // Decimates 2 * count input samples to count outputs, with the same result as count calls of process(). The
    // taps are read from de-interleaved contiguous streams and four outputs are computed at once.
    void processBlock(const float* in, float* out, int count)
    {
        constexpr int history = HalfBandKernel::taps - 1; // 46 samples of past input
        constexpr int maxCount = 256;
        if (count > maxCount || !simd::kernelsEnabled())
        {
            for (int k = 0; k < count; ++k)
                out[k] = process(in[2 * k], in[2 * k + 1]);
            return;
        }
        // ext holds the history followed by the new input; even / odd samples of it go to two streams.
        constexpr int half = history / 2 + maxCount + 4;
        alignas(16) float even[half], odd[half];
        for (int i = 0; i < history; ++i)
        {
            const float v = at(pos - history + i);
            if (i & 1)
                odd[i >> 1] = v;
            else
                even[i >> 1] = v;
        }
        for (int k = 0; k < count; ++k)
        {
            even[history / 2 + k] = in[2 * k];
            odd[history / 2 + k] = in[2 * k + 1];
        }
        const auto& kern = HalfBandKernel::instance();
        // out[k] = 0.5 * E[k + 12] + sum_j odd[j] * (O[k - j + 11] + O[k + j + 12])
        int k = 0;
        for (; k + 4 <= count; k += 4)
        {
            using simd::Float4;
            Float4 acc = Float4::splat(HalfBandKernel::centreTap) * Float4::load(even + k + 12);
            for (int j = 0; j < HalfBandKernel::pairs; ++j)
                acc = Float4::mulAdd(acc, Float4::splat(kern.odd[static_cast<size_t>(j)]),
                                     Float4::load(odd + k - j + 11) + Float4::load(odd + k + j + 12));
            acc.store(out + k);
        }
        for (; k < count; ++k)
        {
            float acc = HalfBandKernel::centreTap * even[k + 12];
            for (int j = 0; j < HalfBandKernel::pairs; ++j)
                acc += kern.odd[static_cast<size_t>(j)] * (odd[k - j + 11] + odd[k + j + 12]);
            out[k] = acc;
        }
        for (int i = 0; i < 2 * count; ++i)
            push(in[i]);
    }

private:
    static constexpr int size = 64;
    void push(float v)
    {
        buffer[static_cast<size_t>(pos & (size - 1))] = v;
        ++pos;
    }
    float at(int i) const { return buffer[static_cast<size_t>(i & (size - 1))]; }

    std::array<float, size> buffer{};
    int pos = 0;
};

// Interpolate by two: one input sample produces two outputs.
class HalfBandInterpolator
{
public:
    void reset()
    {
        buffer.fill(0.0f);
        pos = 0;
    }

    void process(float x, float& out0, float& out1)
    {
        buffer[static_cast<size_t>(pos & (size - 1))] = x;
        ++pos;
        // Output phase 1 is the centre tap (a pure delay), phase 0 uses the odd taps.
        const auto& k = HalfBandKernel::instance();
        const int newest = pos - 1;
        const int d = HalfBandKernel::centre / 2; // 11
        out1 = at(newest - d);
        float acc = 0.0f;
        for (int j = 0; j < HalfBandKernel::pairs; ++j)
            acc += k.odd[static_cast<size_t>(j)] * (at(newest - d + j) + at(newest - d - 1 - j));
        out0 = acc * 2.0f;
    }

private:
    static constexpr int size = 64;
    float at(int i) const { return buffer[static_cast<size_t>(i & (size - 1))]; }
    std::array<float, size> buffer{};
    int pos = 0;
};

} // namespace crucible
