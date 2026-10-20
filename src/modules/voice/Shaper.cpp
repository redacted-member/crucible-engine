// SPDX-License-Identifier: MIT
#include "Shaper.h"

namespace crucible::modules
{

void VoiceShaper::prepare(double sampleRate, Quality quality, OversamplingFilter filter, int maxBlock)
{
    const int factor = saturation::recommendedOversampling(saturation::Model::Soft, quality, true);
    for (auto& c : chain)
        c.prepare(sampleRate, factor, filter, quality, maxBlock);
}

void VoiceShaper::reset()
{
    for (auto& c : chain)
        c.reset();
}

void VoiceShaper::set(const saturation::Settings& s)
{
    for (auto& c : chain)
        c.set(s);
}

void VoiceShaper::process(float* left, float* right, int n)
{
    chain[0].process(left, n);
    chain[1].process(right, n);
}

} // namespace crucible::modules
