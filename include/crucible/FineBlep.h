// SPDX-License-Identifier: MIT
#pragma once

// The fine minBLEP / minBLAMP corrections used by the band-limited oscillators (the analog oscillator and the audio-rate
// LFO). Only the part for the lagged naive signal is here; see the comments below.

#include <crucible/MinBlep.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

namespace crucible
{

// Samples the output lags the oscillator's own timeline by. The min-phase minBLEP rises over a centroid delay of 3.2
// samples; computing the naive waveform kBlepLag samples late and stretching the table in time until its centroid is exactly
// kBlepLag makes every edge land on its exact position and keeps the corrections free of DC (a plain minBLEP saw carries
// an offset of 2 x 3.2 x f0 / fs).
constexpr int kBlepLag = 4;

// The minBLEP / minBLAMP corrections on a 256-entries-per-sample grid, for the lagged naive signal. MinBlep.h tabulates
// 32 entries per sample and its linear interpolation limits a saw above 1 kHz to about -85 dB, its integrated (minBLAMP)
// table a triangle to about -87 dB at C8. Here the 32-entry step is upsampled once (cubic), stretched in time to a
// centroid of kBlepLag samples, and the corner correction is its trapezoid integral. A correction is
//     step:   S(x) - [x >= kBlepLag]        corner: R(x) - max(0, x - kBlepLag)        (x = samples since the event)
// where S is the stretched min-phase step and R its integral; the tables hold S and R for x < kBlepLag and S - 1 and
// R - (x - kBlepLag) beyond, so the interpolation never straddles the lag. Built on the control thread.
class FineBlep
{
public:
    static constexpr int length = MinBlep::length;
    static constexpr int over = 256;

    static const FineBlep& instance()
    {
        static const FineBlep f;
        return f;
    }
    const float* stepEarly() const { return stepLo.data(); }
    const float* stepLate() const { return stepHi.data(); }
    const float* rampEarly() const { return rampLo.data(); }
    const float* rampLate() const { return rampHi.data(); }

private:
    FineBlep()
    {
        const auto& mb = MinBlep::instance();
        constexpr int coarseOver = MinBlep::oversample;
        const int n = length * coarseOver + 2;
        std::vector<double> g(static_cast<size_t>(n) + 4, 0.0);
        for (int i = 0; i < n; ++i)
            g[static_cast<size_t>(i) + 1] =
                static_cast<double>(mb.at(static_cast<float>(i) / static_cast<float>(coarseOver))) + 1.0;
        g[0] = g[1];
        for (size_t i = static_cast<size_t>(n) + 1; i < g.size(); ++i)
            g[i] = 1.0;
        // The min-phase step on the fine grid (cubic), then its centroid delay.
        const auto stepAt = [&g](double x)
        {
            const double pos = x * coarseOver;
            const int i = static_cast<int>(pos);
            const double f = pos - i;
            const double p0 = g[static_cast<size_t>(i)], p1 = g[static_cast<size_t>(i) + 1],
                         p2 = g[static_cast<size_t>(i) + 2], p3 = g[static_cast<size_t>(i) + 3];
            return 0.5 * ((2.0 * p1) + (-p0 + p2) * f + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * f * f +
                          (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * f * f * f);
        };
        const int fineCount = length * over;
        double centroid = 0.0;
        for (int j = 0; j < fineCount; ++j)
        {
            const double x0 = static_cast<double>(j) / over, x1 = static_cast<double>(j + 1) / over;
            centroid +=
                0.5 * ((1.0 - stepAt(x0)) + (1.0 - stepAt(std::min(x1, static_cast<double>(length) - 1.0e-9)))) / over;
        }
        const double stretch = centroid / kBlepLag; // S'(x) = S(stretch x) has centroid kBlepLag
        const size_t size = static_cast<size_t>(fineCount) + 2;
        std::vector<double> step(size, 1.0), integral(size, 0.0);
        for (int j = 0; j <= fineCount; ++j)
        {
            const double x = std::min(static_cast<double>(j) / over * stretch, static_cast<double>(length) - 1.0e-9);
            step[static_cast<size_t>(j)] = stepAt(x);
        }
        for (size_t j = 1; j <= static_cast<size_t>(fineCount); ++j)
            integral[j] = integral[j - 1] + 0.5 * (step[j] + step[j - 1]) / over;
        stepLo.resize(size);
        stepHi.resize(size);
        rampLo.resize(size);
        rampHi.resize(size);
        for (size_t j = 0; j < size; ++j)
        {
            const double x = static_cast<double>(j) / over;
            stepLo[j] = static_cast<float>(step[j]);
            stepHi[j] = static_cast<float>(step[j] - 1.0);
            rampLo[j] = static_cast<float>(integral[j]);
            rampHi[j] = static_cast<float>(integral[j] - (x - kBlepLag));
        }
        // The tails settle to zero (the stretch makes the centroid exact); remove the rounding residue.
        for (auto* t : {&stepHi, &rampHi})
        {
            (*t)[static_cast<size_t>(fineCount)] = 0.0f;
            (*t)[static_cast<size_t>(fineCount) + 1] = 0.0f;
        }
    }

    std::vector<float> stepLo, stepHi, rampLo, rampHi;
};

// Like MinBlepBuffer, on the fine tables and for the lagged naive signal.
class BlepBuffer
{
public:
    void reset()
    {
        buf.fill(0.0f);
        pos = 0;
    }
    void addStep(float jump, float d)
    {
        const auto& f = FineBlep::instance();
        add(jump, d, f.stepEarly(), f.stepLate());
    }
    void addKink(float slopeChange, float d)
    {
        const auto& f = FineBlep::instance();
        add(slopeChange, d, f.rampEarly(), f.rampLate());
    }
    float tick()
    {
        const float v = buf[static_cast<size_t>(pos)];
        buf[static_cast<size_t>(pos)] = 0.0f;
        pos = (pos + 1) & (FineBlep::length - 1);
        return v;
    }

private:
    void add(float amount, float d, const float* early, const float* late)
    {
        // An event exactly on the sample boundary (a frequency that is a rational share of the sample rate lands there)
        // stays in this sample: d stays below 1, and the lagged naive step arrives kBlepLag samples after this one.
        d = std::clamp(d, 1.0e-4f, 0.99999994f);
        const float p = d * static_cast<float>(FineBlep::over);
        const int i0 = static_cast<int>(p);
        const float f = p - static_cast<float>(i0);
        // Samples j < split are earlier than kBlepLag samples after the event (the lagged naive step has not arrived yet).
        // d is below 1 after the clamp, so this is kBlepLag; written as a constant, the compiler unrolls the two loops into
        // slower code (Apple clang: +9 % on the analog oscillator), the expression keeps their structure.
        const int split = d >= 1.0f ? kBlepLag - 1 : kBlepLag;
        int j = 0;
        for (; j < split; ++j)
        {
            const int idx = i0 + j * FineBlep::over;
            buf[static_cast<size_t>((pos + j) & (FineBlep::length - 1))] +=
                amount * (early[idx] + (early[idx + 1] - early[idx]) * f);
        }
        for (; j < FineBlep::length; ++j)
        {
            const int idx = i0 + j * FineBlep::over;
            buf[static_cast<size_t>((pos + j) & (FineBlep::length - 1))] +=
                amount * (late[idx] + (late[idx + 1] - late[idx]) * f);
        }
    }

    std::array<float, FineBlep::length> buf{};
    int pos = 0;
};

} // namespace crucible
