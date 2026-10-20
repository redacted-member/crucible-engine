// SPDX-License-Identifier: MIT
#pragma once

#include <array>
#include <cstddef>

namespace crucible
{

// Minimum-phase band-limited step (minBLEP, after Brandt): the correction that turns a naive discontinuity into
// a band-limited one. The table is the integrated minimum-phase impulse response of a 32-zero-crossing windowed
// sinc, minus the ideal step, sampled 32 times per output sample. Built once from a double-precision cepstral
// design; call MinBlep::instance() on a control thread before the first use on an audio thread.
class MinBlep
{
public:
    static constexpr int length = 64;     // samples the correction rings for
    static constexpr int oversample = 32; // table entries per sample

    static const MinBlep& instance();
    // Correction at distance x samples after the discontinuity, 0 <= x <= length (linear between entries).
    float at(float x) const
    {
        const float p = x * static_cast<float>(oversample);
        const int i = static_cast<int>(p);
        const float f = p - static_cast<float>(i);
        return table[static_cast<std::size_t>(i)] +
               (table[static_cast<std::size_t>(i) + 1] - table[static_cast<std::size_t>(i)]) * f;
    }

    // Correction of a slope break (a kink: the signal continues with a different slope), per unit of slope change in
    // value per sample: the integrated step correction minus its constant tail times the step, so it starts and
    // ends at zero (minBLAMP, delay compensated).
    float rampAt(float x) const
    {
        const float p = x * static_cast<float>(oversample);
        const int i = static_cast<int>(p);
        const float f = p - static_cast<float>(i);
        return ramp[static_cast<std::size_t>(i)] +
               (ramp[static_cast<std::size_t>(i) + 1] - ramp[static_cast<std::size_t>(i)]) * f;
    }

private:
    MinBlep();
    std::array<float, length * oversample + 2> table{};
    std::array<float, length * oversample + 2> ramp{};
};

// Accumulates the corrections of the discontinuities of one signal and releases them one sample at a time.
class MinBlepBuffer
{
public:
    void reset()
    {
        buf.fill(0.0f);
        pos = 0;
    }

    // A step of `jump` (value after minus value before) happened `d` samples (0 < d <= 1) before the sample that
    // the next tick() call returns the correction for.
    void addStep(float jump, float d)
    {
        const auto& m = MinBlep::instance();
        if (d < 1.0e-4f)
            d = 1.0e-4f;
        if (d > 1.0f)
            d = 1.0f;
        for (int j = 0; j < MinBlep::length; ++j)
            buf[static_cast<std::size_t>((pos + j) & (MinBlep::length - 1))] += jump * m.at(d + static_cast<float>(j));
    }

    // A change of slope by `slopeChange` (value per sample) at the same point in time.
    void addKink(float slopeChange, float d)
    {
        const auto& m = MinBlep::instance();
        if (d < 1.0e-4f)
            d = 1.0e-4f;
        if (d > 1.0f)
            d = 1.0f;
        for (int j = 0; j < MinBlep::length; ++j)
            buf[static_cast<std::size_t>((pos + j) & (MinBlep::length - 1))] +=
                slopeChange * m.rampAt(d + static_cast<float>(j));
    }

    // Correction to add to the next output sample.
    float tick()
    {
        const float v = buf[static_cast<std::size_t>(pos)];
        buf[static_cast<std::size_t>(pos)] = 0.0f;
        pos = (pos + 1) & (MinBlep::length - 1);
        return v;
    }

private:
    std::array<float, MinBlep::length> buf{};
    int pos = 0;
};

} // namespace crucible
