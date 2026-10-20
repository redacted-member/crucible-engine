// SPDX-License-Identifier: MIT
#pragma once

// Per-voice shaper: a stereo pair of library models behind the oversampler, sized for the voice path (one step more
// oversampling than the effects, as the plan's table for the shaper node). It is a self-contained stage with no
// parameter table of its own; the voice graph (P4) wraps it as a node and owns the parameters.

#include <crucible/Saturation.h>

namespace crucible::modules
{

class VoiceShaper
{
public:
    void prepare(double sampleRate, Quality quality, OversamplingFilter filter, int maxBlock);
    void reset();
    void set(const saturation::Settings& s);
    // In place; n <= maxBlock given to prepare().
    void process(float* left, float* right, int n);
    // Whole-sample latency of the oversampler path (0 when the model is not oversampled).
    float latency() const { return chain[0].latency(); }

private:
    saturation::OversampledSaturator chain[2];
};

} // namespace crucible::modules
