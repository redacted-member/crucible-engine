// SPDX-License-Identifier: MIT
#pragma once

// Capability interface for effects that measure the signal passing through them (the loudness readout of the maximiser,
// the meter module). The engine publishes what they report as views (<module id>/loudness, <module id>/levels). Both
// calls are made on the audio thread, from the view producer, so an implementation reads its own state without locks.

namespace crucible
{

class MeterReadout
{
public:
    // Values of loudness(): momentary LUFS, short-term LUFS, integrated LUFS, loudness range LU, true peak dB (largest since
    // reset), largest momentary LUFS, lower and upper percentile levels of the loudness range (LUFS). -120 means nothing
    // yet.
    static constexpr int kLoudnessValues = 8;
    // Values of levels(): sample peak dB (left, right; largest since reset), true peak dB (left, right; decaying, 1.5 s),
    // RMS dB (left, right; 300 ms), PPM dB (left, right), VU dB (left, right), correlation (-1 to 1).
    static constexpr int kLevelValues = 11;

    virtual ~MeterReadout() = default;
    virtual void loudness(float* out) const = 0;
    // False when the module has no level meters (only loudness).
    virtual bool levels(float* out) const
    {
        (void)out;
        return false;
    }
    // Gain in dB that would bring the integrated loudness to targetLufs (a suggestion for the control thread to apply; never
    // applied on the audio thread). 0 until an integrated reading exists.
    virtual float suggestedGainDb(float targetLufs) const
    {
        float v[kLoudnessValues];
        loudness(v);
        return v[2] > -119.0f ? targetLufs - v[2] : 0.0f;
    }
};

} // namespace crucible
