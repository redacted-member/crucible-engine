// SPDX-License-Identifier: MIT
#pragma once

#include "TestSupport.h"

namespace crucible::testing
{

// Plays one note on a SynthEngine built from the config and returns the stereo render.
inline Rendered playNote(const EngineConfig& config, const std::vector<float>& values, int note, int samples,
                         bool keep = true)
{
    SynthEngine engine(config);
    prepare(engine, values);
    engine.noteOn(note, 1.0f);
    return renderBlocks(engine, samples, 512, keep);
}

inline double energy(const std::vector<float>& s, size_t from = 0)
{
    double e = 0.0;
    for (size_t i = from; i < s.size(); ++i)
        e += static_cast<double>(s[i]) * s[i];
    return e;
}

// Energy of the third difference: a crude measure of high-frequency content.
inline double brightness(const std::vector<float>& s, size_t from = 0)
{
    double e = 0.0;
    for (size_t i = from + 3; i < s.size(); ++i)
    {
        const double d = static_cast<double>(s[i]) - 3.0 * s[i - 1] + 3.0 * s[i - 2] - s[i - 3];
        e += d * d;
    }
    return e;
}

inline std::vector<float> graphPatch()
{
    auto v = ParameterMap::defaultValues();
    setValue(v, "amp_attack", 0.001f);
    setValue(v, "amp_sustain", 1.0f);
    setValue(v, "master_gain", 0.0f);
    setValue(v, "osc1_table", 0.0f);
    setValue(v, "osc1_pos", 1.0f);
    setValue(v, "filter_cutoff", 20000.0f);
    setValue(v, "filter_res", 0.0f);
    return v;
}

} // namespace crucible::testing
