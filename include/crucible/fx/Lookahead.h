// SPDX-License-Identifier: MIT
#pragma once

// Lookahead delay whose length can change while audio runs (docs/plans/ENGINE-FX-PLAN.md, sections 3.1 and 4.5). A
// buffer that is cleared when the lookahead changes drops out for that many samples (the legacy limiter's fault,
// BFX5). Here two read taps share one buffer: on a change the output crossfades from the old tap to the new one over
// 10 ms with a quintic curve whose first and second derivatives vanish at both ends, so a sine keeps its shape and
// the change leaves no step above -90 dB. The delay is also the module's reported latency.
//
// Requests made while a fade runs are queued (the latest wins) and start when it ends. Before the first sample is
// processed after prepare() or reset(), setDelay() applies at once.

#include <cstddef>
#include <vector>

namespace crucible::fx
{

class Lookahead
{
public:
    static constexpr int kMaxChannels = 2;

    // Control thread. maxDelaySamples bounds setDelay().
    void prepare(int numChannels, int maxDelaySamples, double sampleRate, double fadeSeconds = 0.010);
    void reset();

    // Target delay in samples (clamped to the prepared maximum). Any thread that is also the audio thread, or the
    // control thread while audio is stopped.
    void setDelay(int samples);
    // The target delay, which is what a module reports as latency.
    int delay() const { return target; }
    // The delay of the tap that is audible now (differs from delay() only during a fade).
    int activeDelay() const { return fadeFrom; }
    // The delay the running fade is heading to (equals activeDelay() when none runs).
    int fadeTargetDelay() const { return fadeTo; }
    bool fading() const { return fadePosition < fadeLength; }
    int maxDelay() const { return maximum; }

    // In place on `channels` channels of n samples each.
    void process(float* const* io, int n);
    // Single-channel convenience (channel 0).
    float processSample(float x);

private:
    void advanceFade();

    std::vector<float> buffer[kMaxChannels];
    size_t mask = 0, writePos = 0;
    int channels = 1, maximum = 0;
    int target = 0, fadeFrom = 0, fadeTo = 0, pending = 0;
    bool hasPending = false, started = false;
    int fadeLength = 480, fadePosition = 480;
};

} // namespace crucible::fx
