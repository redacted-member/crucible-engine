// SPDX-License-Identifier: MIT
#pragma once

// A band-limited audio-rate oscillator that plays the shape of an LFO (lfoN_audio): sine, triangle, saw up, square, saw down
// and 25 % pulse. Every step and every slope break of the naive waveform is located at its sub-sample position and
// corrected with the fine minBLEP / minBLAMP of FineBlep.h (the naive waveform runs kBlepLag samples late, as in the analog
// oscillator), so the output stays clean up to a few kHz and rolls off above; the sine has no discontinuity and needs
// nothing. Phase is kept in double precision. No allocation; FineBlep::instance() builds its tables on first use, so call
// it on a control thread first (Voice::prepare does).

#include <crucible/FineBlep.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace crucible
{

class AudioLfo
{
public:
    void reset(double startPhase = 0.0)
    {
        phase = startPhase - std::floor(startPhase);
        blep.reset();
        fresh = true;
    }

    // Shape indices are those of Lfo::shapeValue; shapes without a band-limited form play as sine.
    // inc: cycles per sample (clamped below 0.45).
    void render(int shape, double inc, float amount, int n, float* out)
    {
        inc = std::clamp(inc, 0.0, 0.45);
        // The discontinuity lies `fromPhase / inc` of a sample before the end of this sample. An event exactly on the
        // sample boundary (a rational frequency lands there) stays in this sample: d must stay below 1.
        const auto delay = [&](double fromPhase) { return static_cast<float>(std::min(fromPhase / inc, 1.0 - 1.0e-6)); };
        const auto step = [&](float jump, double fromPhase) { blep.addStep(jump, delay(fromPhase)); };
        const auto kink = [&](float slope, double fromPhase)
        { blep.addKink(slope * static_cast<float>(inc), delay(fromPhase)); };
        for (int i = 0; i < n; ++i)
        {
            const double before = phase;
            const double unwrapped = phase + inc;
            // The position, in cycles, past an event at phase `at` (0 when the sample does not contain it); the event sits
            // that far before the end of the sample. A step of 25 % pulse can fall in the sample of the wrap.
            const auto passed = [&](double at)
            {
                const double crossings = std::floor(unwrapped - at) - std::floor(before - at);
                return crossings > 0.0 ? unwrapped - at - std::floor(unwrapped - at) : -1.0;
            };
            phase = unwrapped - std::floor(unwrapped);
            const double p = phase;
            float v;
            switch (shape)
            {
            case 1: // triangle: corners at 0 (slope -4 -> +4) and 0.5 (+4 -> -4)
                if (const double at = passed(0.0); at >= 0.0)
                    kink(8.0f, at);
                if (const double at = passed(0.5); at >= 0.0)
                    kink(-8.0f, at);
                v = p < 0.5 ? static_cast<float>(4.0 * p - 1.0) : static_cast<float>(3.0 - 4.0 * p);
                break;
            case 2: // saw up
                if (const double at = passed(0.0); at >= 0.0)
                    step(-2.0f, at);
                v = static_cast<float>(2.0 * p - 1.0);
                break;
            case 3: // square
                if (const double at = passed(0.0); at >= 0.0)
                    step(2.0f, at);
                if (const double at = passed(0.5); at >= 0.0)
                    step(-2.0f, at);
                v = p < 0.5 ? 1.0f : -1.0f;
                break;
            case 5: // saw down
                if (const double at = passed(0.0); at >= 0.0)
                    step(2.0f, at);
                v = static_cast<float>(1.0 - 2.0 * p);
                break;
            case 7: // 25 % pulse
                if (const double at = passed(0.0); at >= 0.0)
                    step(2.0f, at);
                if (const double at = passed(0.25); at >= 0.0)
                    step(-2.0f, at);
                v = p < 0.25 ? 1.0f : -1.0f;
                break;
            default:
                v = static_cast<float>(std::sin(6.283185307179586 * p));
                break;
            }
            if (fresh)
            {
                ring.fill(v);
                fresh = false;
            }
            const float old = ring[static_cast<size_t>(pos)];
            ring[static_cast<size_t>(pos)] = v;
            pos = (pos + 1) & (kBlepLag - 1);
            out[i] = (old + blep.tick()) * amount;
        }
    }

    double currentPhase() const { return phase; }

private:
    double phase = 0.0;
    BlepBuffer blep;
    std::array<float, kBlepLag> ring{};
    int pos = 0;
    bool fresh = true;
};

} // namespace crucible
