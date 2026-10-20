// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/fx/IrImport.h>

namespace crucible
{

// Edits applied to an impulse response before it becomes a convolution kernel (fx.convolution). All of them run on a
// non-audio thread and allocate. Order of application: trim start, length, reverse, stretch, fade in / out, early / late
// balance, low / high cut, predelay, width.
struct IrEditSettings
{
    double trimMs = 0.0;          // samples dropped from the start
    double lengthPercent = 100.0; // part of the (trimmed) response that is kept, from its start
    bool reverse = false;
    double stretch = 1.0; // time scale by resampling: 2 doubles the length and lowers the pitch an octave
    double fadeInMs = 0.0;
    double fadeOutMs = 0.0;
    double balanceDb = 0.0;     // late level minus early level, split at 80 ms with a smooth transition from 60 to 100 ms
    double lowCutHz = 20.0;     // 24 dB per octave Butterworth; off at or below 20.5 Hz
    double highCutHz = 20000.0; // 24 dB per octave Butterworth; off at or above 19999.5 Hz or 0.49 of the sample rate
    double predelayMs = 0.0;    // silence inserted before the response
    double width = 1.0;         // side level of every left / right pair of kernels (0 mono, 1 unchanged, 2 double)

    bool operator==(const IrEditSettings& o) const noexcept
    {
        return trimMs == o.trimMs && lengthPercent == o.lengthPercent && reverse == o.reverse && stretch == o.stretch &&
               fadeInMs == o.fadeInMs && fadeOutMs == o.fadeOutMs && balanceDb == o.balanceDb && lowCutHz == o.lowCutHz &&
               highCutHz == o.highCutHz && predelayMs == o.predelayMs && width == o.width;
    }
    bool operator!=(const IrEditSettings& o) const noexcept { return !(*this == o); }
};

// Longest edited response, in seconds (the import limit plus room for the predelay and a stretch).
constexpr double kMaxEditedImpulseResponseSeconds = 60.0;

// Applies the edits to `in` (kernels at in.sampleRate) and returns the result with the same layout and rate; never empty.
ImpulseResponse applyIrEdit(const ImpulseResponse& in, const IrEditSettings& edit);

} // namespace crucible
