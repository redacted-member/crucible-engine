// SPDX-License-Identifier: MIT
#pragma once

// Helpers for declaring module parameters.

#include <crucible/ParameterMap.h>

#include <string>
#include <vector>

namespace crucible::params
{

ParamDef floating(std::string id, std::string name, std::string pageId, float min, float max, float def,
                  std::string unit = {}, float skew = 1.0f, Smoothing smoothing = Smoothing::Linear);
// Frequencies and other strictly positive, exponential quantities: log-domain smoothing.
ParamDef frequency(std::string id, std::string name, std::string pageId, float min, float max, float def, float skew = 0.3f);
ParamDef integer(std::string id, std::string name, std::string pageId, int min, int max, int def, std::string unit = {});
ParamDef choice(std::string id, std::string name, std::string pageId, std::vector<std::string> choices, int def);
ParamDef toggle(std::string id, std::string name, std::string pageId, bool def);

// Marks the parameter as a target of the generic modulation system.
inline ParamDef modulatable(ParamDef p)
{
    p.modulatable = true;
    return p;
}

// Modulation target that the voice evaluates per voice (see ParamDef::voiceMod).
inline ParamDef voiceModulatable(ParamDef p)
{
    p.modulatable = true;
    p.voiceMod = true;
    return p;
}

// Shared choice lists.
const std::vector<std::string>& tempoSyncNames(); // "Off", "4 Bar" ... "1/32" (see lfoSyncBeats())
const std::vector<std::string>& lfoShapeNames();

} // namespace crucible::params
