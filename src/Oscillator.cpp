// SPDX-License-Identifier: MIT
#include <crucible/module/Oscillator.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace crucible
{

int SlotParams::bind(const std::string& suffix)
{
    const std::string id = "osc" + std::to_string(slot + 1) + "_" + suffix;
    const int g = model->indexOf(id);
    if (g < 0)
        throw std::logic_error("no oscillator parameter " + id);
    global.push_back(g);
    voiceTarget.push_back(model->voiceTargetOfParam(g));
    return static_cast<int>(global.size()) - 1;
}

float SlotParams::value(const float* values, const float* targetMod, int local) const
{
    const auto l = static_cast<size_t>(local);
    const int g = global[l];
    float v = values[g];
    const int t = voiceTarget[l];
    if (targetMod != nullptr && t >= 0)
    {
        const float m = targetMod[t];
        if (m != 0.0f)
        {
            const auto& def = model->params()[static_cast<size_t>(g)];
            v = def.smoothing == Smoothing::Log ? v * std::exp2(5.0f * m) : v + m * (def.max - def.min);
            v = std::clamp(v, def.min, def.max);
        }
    }
    return v;
}

} // namespace crucible
