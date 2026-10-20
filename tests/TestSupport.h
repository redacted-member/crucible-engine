// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/ParameterMap.h>
#include <crucible/SynthEngine.h>

#include "TestFramework.h"

#include <crucible/Denormals.h>
#include <crucible/Fft.h>

#include <cassert>

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace crucible::testing
{

using crucible::test::String;
using crucible::test::UnitTest;

constexpr double kRate = 48000.0;

inline void setValue(std::vector<float>& values, const std::string& id, float v)
{
    const int i = ParameterMap::indexOf(id);
    assert(i >= 0);
    values[static_cast<size_t>(i)] = v;
}

struct Rendered
{
    float peak = 0.0f;
    bool finite = true;
    std::vector<float> left;
    std::vector<float> right;
};

inline Rendered renderBlocks(SynthEngine& engine, int totalSamples, int blockSize = 512, bool keep = false)
{
    Rendered r;
    std::vector<float> l(static_cast<size_t>(blockSize)), rt(static_cast<size_t>(blockSize));
    for (int done = 0; done < totalSamples; done += blockSize)
    {
        const int n = std::min(blockSize, totalSamples - done);
        engine.render(l.data(), rt.data(), n);
        for (int i = 0; i < n; ++i)
        {
            for (const float s : {l[static_cast<size_t>(i)], rt[static_cast<size_t>(i)]})
            {
                if (!std::isfinite(s))
                    r.finite = false;
                r.peak = std::max(r.peak, std::abs(s));
            }
            if (keep)
            {
                r.left.push_back(l[static_cast<size_t>(i)]);
                r.right.push_back(rt[static_cast<size_t>(i)]);
            }
        }
    }
    return r;
}

inline void prepare(SynthEngine& engine, const std::vector<float>& values)
{
    engine.prepare(kRate);
    engine.ensureSelectedTables(values.data(), static_cast<int>(values.size()));
    engine.setParameterValues(values.data(), static_cast<int>(values.size()));
}

// Share of spectral energy that is not within a few bins of a harmonic of f0
// (aliases, noise and distortion products), in dB relative to the total.
inline double inharmonicEnergyDb(const std::vector<float>& signal, double f0, double sampleRate, int skip = 0)
{
    constexpr int order = 14;
    constexpr int n = 1 << order;
    const Fft fft(order);
    std::vector<float> data(static_cast<size_t>(2 * n), 0.0f);
    // 4-term Blackman-Harris window.
    for (int i = 0; i < n; ++i)
    {
        const double x = 2.0 * 3.14159265358979 * i / (n - 1);
        const double w = 0.35875 - 0.48829 * std::cos(x) + 0.14128 * std::cos(2 * x) - 0.01168 * std::cos(3 * x);
        data[static_cast<size_t>(i)] = signal[static_cast<size_t>(skip + i)] * static_cast<float>(w);
    }
    fft.forward(data.data());

    const double binHz = sampleRate / n;
    std::vector<char> harmonic(static_cast<size_t>(n / 2 + 1), 0);
    for (int h = 1; h * f0 < sampleRate * 0.5 + 4 * binHz; ++h)
    {
        const int centre = static_cast<int>(std::lround(h * f0 / binHz));
        for (int b = centre - 6; b <= centre + 6; ++b)
            if (b >= 0 && b <= n / 2)
                harmonic[static_cast<size_t>(b)] = 1;
    }
    for (int b = 0; b < 4; ++b)
        harmonic[static_cast<size_t>(b)] = 1;

    double total = 0.0, other = 0.0;
    for (int b = 0; b <= n / 2; ++b)
    {
        const double re = data[static_cast<size_t>(2 * b)];
        const double im = data[static_cast<size_t>(2 * b + 1)];
        const double e = re * re + im * im;
        total += e;
        if (harmonic[static_cast<size_t>(b)] == 0)
            other += e;
    }
    return 10.0 * std::log10(std::max(other, 1.0e-30) / std::max(total, 1.0e-30));
}

// Aliasing as the reference bench measures it: energy below limitHz that is not within a few bins of a harmonic
// of f0 (fourteen bins each side, beyond the window leakage), in dB relative to the energy at the harmonics. Blackman-Harris
// window over 2^14 samples starting at `skip`.
inline double aliasingBelowDb(const std::vector<float>& signal, double f0, double sampleRate, double limitHz = 20000.0,
                              int skip = 0)
{
    constexpr int order = 14;
    constexpr int n = 1 << order;
    const Fft fft(order);
    std::vector<float> data(static_cast<size_t>(2 * n), 0.0f);
    for (int i = 0; i < n; ++i)
    {
        const double x = 2.0 * 3.14159265358979 * i / (n - 1);
        const double w = 0.35875 - 0.48829 * std::cos(x) + 0.14128 * std::cos(2 * x) - 0.01168 * std::cos(3 * x);
        data[static_cast<size_t>(i)] = signal[static_cast<size_t>(skip + i)] * static_cast<float>(w);
    }
    fft.forward(data.data());
    const double binHz = sampleRate / n;
    std::vector<char> harmonic(static_cast<size_t>(n / 2 + 1), 0);
    for (int h = 1; h * f0 < sampleRate * 0.5 + 4 * binHz; ++h)
    {
        const int centre = static_cast<int>(std::lround(h * f0 / binHz));
        for (int b = centre - 14; b <= centre + 14; ++b)
            if (b >= 0 && b <= n / 2)
                harmonic[static_cast<size_t>(b)] = 1;
    }
    for (int b = 0; b < 4; ++b)
        harmonic[static_cast<size_t>(b)] = 1;
    double harmonics = 0.0, other = 0.0;
    for (int b = 0; b <= n / 2; ++b)
    {
        const double re = data[static_cast<size_t>(2 * b)];
        const double im = data[static_cast<size_t>(2 * b + 1)];
        const double e = re * re + im * im;
        if (harmonic[static_cast<size_t>(b)] != 0)
            harmonics += e;
        else if (b * binHz < limitHz)
            other += e;
    }
    return 10.0 * std::log10(std::max(other, 1.0e-30) / std::max(harmonics, 1.0e-30));
}

inline double midiToHz(double note)
{
    return 440.0 * std::pow(2.0, (note - 69.0) / 12.0);
}

} // namespace crucible::testing
