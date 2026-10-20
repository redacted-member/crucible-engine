// SPDX-License-Identifier: MIT
#include <crucible/Scenes.h>

#include <crucible/ParameterMap.h>
#include <crucible/module/Module.h>

#include "TextFormat.h"

#include <algorithm>
#include <cmath>

namespace crucible
{
namespace
{
float sceneNormalised(const ParamDef& d, float v)
{
    const float range = d.max - d.min;
    if (range <= 0.0f)
        return 0.0f;
    const float p = std::clamp((v - d.min) / range, 0.0f, 1.0f);
    return std::pow(p, d.skew);
}

float sceneDenormalised(const ParamDef& d, float n)
{
    return d.min + (d.max - d.min) * std::pow(std::clamp(n, 0.0f, 1.0f), 1.0f / d.skew);
}
} // namespace

SceneBank::SceneBank(const ParameterModel* model)
    : params(model != nullptr ? model : &ModuleRegistry::instance().parameters())
{
    std::lock_guard<std::mutex> lock(mutex);
    publishLocked();
}

void SceneBank::publishLocked()
{
    auto snap = std::make_unique<SceneSnapshot>();
    snap->generation = ++generation;
    const auto& defs = params->params();
    for (size_t s = 0; s < data.size(); ++s)
    {
        if (data[s].empty())
            continue;
        snap->filled[s] = true;
        snap->natural[s] = data[s];
        snap->normalised[s].resize(defs.size());
        for (size_t i = 0; i < defs.size() && i < data[s].size(); ++i)
            snap->normalised[s][i] = defs[i].type == ParamType::Float ? sceneNormalised(defs[i], data[s][i]) : 0.0f;
    }
    shared.publish(std::move(snap));
}

void SceneBank::store(int slot, const std::vector<float>& values)
{
    if (slot < 0 || slot >= kNumScenes)
        return;
    std::lock_guard<std::mutex> lock(mutex);
    data[static_cast<size_t>(slot)] = values;
    if (storedHook)
        storedHook(data[static_cast<size_t>(slot)]);
    publishLocked();
}

void SceneBank::setStoredHook(std::function<void(const std::vector<float>&)> hook)
{
    std::lock_guard<std::mutex> lock(mutex);
    storedHook = std::move(hook);
}

bool SceneBank::isFilled(int slot) const
{
    if (slot < 0 || slot >= kNumScenes)
        return false;
    std::lock_guard<std::mutex> lock(mutex);
    return !data[static_cast<size_t>(slot)].empty();
}

void SceneBank::clear(int slot)
{
    if (slot < 0 || slot >= kNumScenes)
        return;
    std::lock_guard<std::mutex> lock(mutex);
    data[static_cast<size_t>(slot)].clear();
    publishLocked();
}

bool SceneBank::recall(int slot, std::vector<float>& out) const
{
    if (slot < 0 || slot >= kNumScenes)
        return false;
    std::lock_guard<std::mutex> lock(mutex);
    const auto& s = data[static_cast<size_t>(slot)];
    const auto& defs = params->params();
    if (s.empty() || out.size() < defs.size())
        return false;
    for (size_t i = 0; i < defs.size(); ++i)
        if (defs[i].scene)
            out[i] = s[i];
    return true;
}

bool SceneBank::morph(int a, int b, float t, std::vector<float>& out) const
{
    if (a < 0 || a >= kNumScenes || b < 0 || b >= kNumScenes)
        return false;
    std::lock_guard<std::mutex> lock(mutex);
    const auto& sa = data[static_cast<size_t>(a)];
    const auto& sb = data[static_cast<size_t>(b)];
    const auto& defs = params->params();
    if (sa.empty() || sb.empty() || out.size() < defs.size())
        return false;
    t = std::clamp(t, 0.0f, 1.0f);
    for (size_t i = 0; i < defs.size(); ++i)
    {
        const auto& d = defs[i];
        if (!d.scene)
            continue;
        if (d.type == ParamType::Float)
        {
            const float na = sceneNormalised(d, sa[i]);
            const float nb = sceneNormalised(d, sb[i]);
            out[i] = sceneDenormalised(d, na + (nb - na) * t);
        }
        else
        {
            out[i] = t < 0.5f ? sa[i] : sb[i];
        }
    }
    return true;
}

VectorWeights vectorWeights(float x, float y)
{
    x = std::clamp(x, 0.0f, 1.0f);
    y = std::clamp(y, 0.0f, 1.0f);
    VectorWeights v;
    v.w = {(1.0f - x) * y, x * y, (1.0f - x) * (1.0f - y), x * (1.0f - y)};
    return v;
}

float vectorMorphValue(const ParamDef& d, size_t i, const SceneSnapshot& scenes, const std::array<int, 4>& slots,
                       const VectorWeights& weights, float hostValue)
{
    std::array<float, 4> natural{};
    std::array<bool, 4> stored{};
    bool same = true;
    for (size_t k = 0; k < 4; ++k)
    {
        const int s = slots[k];
        stored[k] = s >= 0 && s < kNumScenes && scenes.filled[static_cast<size_t>(s)] &&
                    i < scenes.natural[static_cast<size_t>(s)].size();
        natural[k] = stored[k] ? scenes.natural[static_cast<size_t>(s)][i] : hostValue;
        same = same && natural[k] == natural[0];
    }
    if (same)
        return natural[0];
    size_t heaviest = 0;
    for (size_t k = 0; k < 4; ++k)
    {
        if (weights.w[k] >= 1.0f)
            return natural[k];
        if (weights.w[k] > weights.w[heaviest])
            heaviest = k;
    }
    if (d.type != ParamType::Float)
        return natural[heaviest];
    float n = 0.0f;
    for (size_t k = 0; k < 4; ++k)
        if (weights.w[k] > 0.0f)
            n += weights.w[k] *
                 (stored[k] ? scenes.normalised[static_cast<size_t>(slots[k])][i] : sceneNormalised(d, hostValue));
    return sceneDenormalised(d, n);
}

bool SceneBank::vectorMorph(const std::array<int, 4>& slots, float x, float y, std::vector<float>& out) const
{
    const auto& defs = params->params();
    if (out.size() < defs.size())
        return false;
    SceneSnapshot snap;
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto* latest = shared.latest();
        if (latest == nullptr)
            return false;
        snap = *latest;
    }
    bool any = false;
    for (const int s : slots)
        any = any || (s >= 0 && s < kNumScenes && snap.filled[static_cast<size_t>(s)]);
    if (!any)
        return false;
    const auto w = vectorWeights(x, y);
    for (size_t i = 0; i < defs.size(); ++i)
        if (defs[i].scene)
            out[i] = vectorMorphValue(defs[i], i, snap, slots, w, out[i]);
    return true;
}

std::vector<SceneRecord> SceneBank::serialise() const
{
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<SceneRecord> out;
    const auto& defs = params->params();
    for (int slot = 0; slot < kNumScenes; ++slot)
    {
        const auto& s = data[static_cast<size_t>(slot)];
        if (s.empty())
            continue;
        std::vector<std::string> pairs;
        for (size_t i = 0; i < defs.size(); ++i)
            if (defs[i].scene)
                pairs.push_back(defs[i].id + "=" + text::fixed(s[i], 6));
        out.push_back({slot, text::join(pairs, ';')});
    }
    return out;
}

void SceneBank::deserialise(const std::vector<SceneRecord>& records)
{
    std::lock_guard<std::mutex> lock(mutex);
    for (auto& s : data)
        s.clear();
    const auto defaults = params->defaultValues();
    for (const auto& record : records)
    {
        const int slot = record.slot;
        if (slot < 0 || slot >= kNumScenes)
            continue;
        auto values = defaults;
        for (const auto& pair : text::split(record.values, ';'))
        {
            const auto eq = pair.find('=');
            if (eq == std::string::npos)
                continue;
            const int idx = params->indexOf(pair.substr(0, eq));
            if (idx >= 0)
            {
                const auto& d = params->params()[static_cast<size_t>(idx)];
                values[static_cast<size_t>(idx)] = std::clamp(text::toFloat(pair.substr(eq + 1)), d.min, d.max);
            }
        }
        data[static_cast<size_t>(slot)] = std::move(values);
        if (storedHook)
            storedHook(data[static_cast<size_t>(slot)]);
    }
    publishLocked();
}

} // namespace crucible
