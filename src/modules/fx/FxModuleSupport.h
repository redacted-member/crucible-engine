// SPDX-License-Identifier: MIT
#pragma once

// Shared helpers for the effect modules in this folder.

#include "../Modules.h"

#include <crucible/Dsp.h>
#include <crucible/Fx.h>
#include <crucible/FxExtra.h>
#include <crucible/module/ParamBuilder.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace crucible::modules::fx
{

// "fx_<id>_on" (and "fx_<id>_mix") on the effect's main page.
inline void addOn(std::vector<ParamDef>& p, const std::string& id, const std::string& name, const std::string& page,
                  float mixDefault, bool withMix = true)
{
    p.push_back(params::toggle("fx_" + id + "_on", name + " On", page, false));
    if (withMix)
        p.push_back(params::modulatable(params::floating("fx_" + id + "_mix", name + " Mix", page, 0.0f, 1.0f, mixDefault)));
}

// "fx_<id>" continuous parameter (modulatable).
inline void addF(std::vector<ParamDef>& p, const std::string& id, const std::string& name, const std::string& page, float lo,
                 float hi, float def, const std::string& unit = {}, float skew = 1.0f,
                 Smoothing smoothing = Smoothing::Linear)
{
    p.push_back(params::modulatable(params::floating("fx_" + id, name, page, lo, hi, def, unit, skew, smoothing)));
}

inline void addHz(std::vector<ParamDef>& p, const std::string& id, const std::string& name, const std::string& page,
                  float lo, float hi, float def, float skew = 0.3f)
{
    p.push_back(params::modulatable(params::frequency("fx_" + id, name, page, lo, hi, def, skew)));
}

// Chain position "fx_pos_<id>" on one of the FX order pages.
inline void addPosition(std::vector<ParamDef>& p, const std::string& id, const std::string& name, const std::string& page,
                        int defaultSlot)
{
    p.push_back(params::integer("fx_pos_" + id, name + " Slot", page, 1, kNumFx, defaultSlot));
}

// Local index of a parameter id within a descriptor (resolved once when an effect instance is created).
int localIndex(const ModuleDescriptor& d, const std::string& id);

// Fills the common descriptor fields of an effect module.
ModuleDescriptor effectDescriptor(std::string id, std::string name, std::string category, CpuClass cpu,
                                  std::string description);

} // namespace crucible::modules::fx
