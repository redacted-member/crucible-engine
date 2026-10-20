// SPDX-License-Identifier: MIT
#include <crucible/Modulation.h>

#include "TextFormat.h"

#include <crucible/module/Module.h>

#include <algorithm>
#include <bitset>
#include <cmath>
#include <unordered_map>

namespace crucible
{

namespace
{
const ParameterModel& modelOf(const ParameterModel* m)
{
    return m != nullptr ? *m : ModuleRegistry::instance().parameters();
}

std::string audioSourceName(int a)
{
    if (a >= kAudioLfoFirst && a < kAudioLfoFirst + kNumLfos)
        return "LFO " + std::to_string(a - kAudioLfoFirst + 1) + " Audio";
    if (a >= kAudioOscFirst && a < kAudioOscFirst + kMaxOscSlots)
        return "Osc " + std::to_string(a - kAudioOscFirst + 1) + " Out";
    static const char* envs[kNumAudioEnvs] = {"Mod Env Audio", "Env 3 Audio", "Env 4 Audio"};
    if (a >= kAudioEnvFirst && a < kAudioEnvFirst + kNumAudioEnvs)
        return envs[a - kAudioEnvFirst];
    return "Audio " + std::to_string(a + 1);
}

// Parses "<prefix><digits><suffix>"; returns the number (>= 0) or -1.
int numberedId(const std::string& id, const char* prefix, const char* suffix)
{
    const std::string p = prefix, s = suffix;
    if (id.size() <= p.size() + s.size() || id.compare(0, p.size(), p) != 0 ||
        id.compare(id.size() - s.size(), s.size(), s) != 0)
        return -1;
    const std::string digits = id.substr(p.size(), id.size() - p.size() - s.size());
    if (digits.empty() || digits.size() > 3)
        return -1;
    int n = 0;
    for (const char c : digits)
    {
        if (c < '0' || c > '9')
            return -1;
        n = n * 10 + (c - '0');
    }
    return n;
}

// The audio-rate kind and slot of a route target, if it is one.
bool classifyAudioTarget(const std::string& id, AudioTarget& kind, int& slot)
{
    if (id == kTargetVoiceAmp)
    {
        kind = AudioTarget::Amp;
        slot = -1;
        return true;
    }
    if (id == "voice_transpose")
    {
        kind = AudioTarget::OscPitch;
        slot = -1;
        return true;
    }
    if (id == "filter_cutoff")
    {
        kind = AudioTarget::FilterCutoff;
        slot = 0;
        return true;
    }
    int n = numberedId(id, "filter", "_cutoff");
    if (n >= 2 && n <= kMaxFilterSlots)
    {
        kind = AudioTarget::FilterCutoff;
        slot = n - 1;
        return true;
    }
    struct Suffix
    {
        const char* text;
        AudioTarget kind;
    };
    static const Suffix suffixes[] = {{"_semi", AudioTarget::OscPitch},
                                      {"_xmod_amt", AudioTarget::OscCrossAmount},
                                      {"_an_xmod_amt", AudioTarget::OscFmAmount},
                                      {"_an_pw", AudioTarget::OscPulseWidth},
                                      {"_pos", AudioTarget::OscPosition}};
    for (const auto& sfx : suffixes)
    {
        n = numberedId(id, "osc", sfx.text);
        if (n >= 1 && n <= kMaxOscSlots)
        {
            kind = sfx.kind;
            slot = n - 1;
            return true;
        }
    }
    return false;
}

// True when the routes' amount-modulation edges (route -> target route) contain a cycle.
bool amountGraphCyclic(const std::vector<ModRoute>& routes)
{
    const int n = static_cast<int>(routes.size());
    std::vector<int> next(static_cast<size_t>(n), -1), indegree(static_cast<size_t>(n), 0);
    for (int i = 0; i < n; ++i)
    {
        const int k = routeAmountTarget(routes[static_cast<size_t>(i)].target);
        if (k == i)
            return true;
        if (k >= 0 && k < n)
        {
            next[static_cast<size_t>(i)] = k;
            ++indegree[static_cast<size_t>(k)];
        }
    }
    std::vector<int> stack;
    for (int i = 0; i < n; ++i)
        if (indegree[static_cast<size_t>(i)] == 0)
            stack.push_back(i);
    int visited = 0;
    while (!stack.empty())
    {
        const int i = stack.back();
        stack.pop_back();
        ++visited;
        const int k = next[static_cast<size_t>(i)];
        if (k >= 0 && --indegree[static_cast<size_t>(k)] == 0)
            stack.push_back(k);
    }
    return visited != n;
}

bool amountSourceAllowed(int source)
{
    return (source > 0 && source < kNumModSources) || isModulatorSource(source) || isAudioSource(source);
}
} // namespace

std::string routeAmountTargetId(int route)
{
    return "route:" + std::to_string(route) + ":amount";
}

int routeAmountTarget(const std::string& target)
{
    return numberedId(target, "route:", ":amount");
}

std::string routeSourceName(int source)
{
    return routeSourceName(source, ModuleRegistry::instance().parameters());
}

std::string routeSourceName(int source, const ParameterModel& model)
{
    if (source >= kSourceCC && source < kSourceCC + 128)
        return "CC " + std::to_string(source - kSourceCC);
    if (isMpeSource(source))
    {
        static const char* names[] = {"MPE Pressure", "MPE Slide", "MPE Bend"};
        return names[source - kSourceMpe];
    }
    if (isClipSource(source))
    {
        const int k = source - kSourceClip;
        return "Clip " + std::to_string(k / kMaxClipLanes + 1) + " Lane " + std::to_string(k % kMaxClipLanes + 1);
    }
    if (isModulatorSource(source))
        return "Modulator " + std::to_string(source - kSourceModulator + 1);
    if (isAudioSource(source))
        return audioSourceName(source - kSourceAudio);
    const auto& names = model.modSourceNames();
    if (source >= 0 && source < static_cast<int>(names.size()))
        return names[static_cast<size_t>(source)];
    return "Unknown";
}

std::string routeSourceName(int source, const ClipBank& clips, const ParameterModel* model)
{
    const ClipLane* lane = clips.lane(source);
    const Clip* clip = clips.clipOf(source);
    if (lane == nullptr || clip == nullptr)
        return routeSourceName(source, modelOf(model));
    const int k = source - kSourceClip;
    const std::string clipName = clip->name.empty() ? "Clip " + std::to_string(k / kMaxClipLanes + 1) : clip->name;
    return clipName + " / " + (lane->name.empty() ? "Lane " + std::to_string(k % kMaxClipLanes + 1) : lane->name);
}

std::vector<RouteSourceInfo> routeSourceList(const ClipBank& clips, const ParameterModel* modelPtr)
{
    const ParameterModel& model = modelOf(modelPtr);
    std::vector<RouteSourceInfo> out;
    const int active = static_cast<int>(model.modSourceNames().size());
    for (int i = 1; i < active; ++i)
    {
        const auto s = static_cast<ModSource>(i);
        const bool voiceOnly = s == ModSource::ModEnv || s == ModSource::AmpEnv || s == ModSource::Env3 ||
                               s == ModSource::Env4 || s == ModSource::Velocity || s == ModSource::Key ||
                               s == ModSource::Random;
        out.push_back({i, routeSourceName(i, model), isBipolarSource(s), voiceOnly});
    }
    for (int cc = 0; cc < 128; ++cc)
        out.push_back({kSourceCC + cc, routeSourceName(kSourceCC + cc, model), false, false});
    for (int m = kSourceMpePressure; m <= kSourceMpeBend; ++m)
        out.push_back({m, routeSourceName(m, model), m == kSourceMpeBend, true});
    for (size_t c = 0; c < clips.clips.size(); ++c)
        for (size_t l = 0; l < clips.clips[c].lanes.size(); ++l)
        {
            const int id = clipLaneSource(static_cast<int>(c), static_cast<int>(l));
            out.push_back({id, routeSourceName(id, clips, &model), false, false});
        }
    // The bank's range depends on the modulator type: native bipolar here is the common case, ModulatorSet::bipolar the
    // truth.
    for (int n = 0; n < kNumBankModulators; ++n)
        out.push_back({kSourceModulator + n, routeSourceName(kSourceModulator + n, model), true, false});
    for (int a = 0; a < kNumAudioSources; ++a)
        out.push_back({kSourceAudio + a, routeSourceName(kSourceAudio + a, model), a < kAudioEnvFirst, true});
    return out;
}

bool isRouteSource(int source, const ParameterModel* model)
{
    return (source > 0 && source < static_cast<int>(modelOf(model).modSourceNames().size())) ||
           (source >= kSourceCC && source < kSourceCC + 128) || isClipSource(source) || isMpeSource(source) ||
           isModulatorSource(source) || isAudioSource(source);
}

ModDest routeVoiceDestination(const std::string& paramId)
{
    static const std::unordered_map<std::string, ModDest> map = {
        {"osc1_pos", ModDest::Osc1Position},
        {"osc2_pos", ModDest::Osc2Position},
        {"osc3_pos", ModDest::Osc3Position},
        {"osc1_semi", ModDest::Osc1Pitch},
        {"osc2_semi", ModDest::Osc2Pitch},
        {"osc3_semi", ModDest::Osc3Pitch},
        {"osc1_level", ModDest::Osc1Level},
        {"osc2_level", ModDest::Osc2Level},
        {"osc3_level", ModDest::Osc3Level},
        {"osc1_warp_a", ModDest::Osc1WarpA},
        {"osc1_warp_b", ModDest::Osc1WarpB},
        {"osc2_warp_a", ModDest::Osc2WarpA},
        {"osc2_warp_b", ModDest::Osc2WarpB},
        {"osc3_warp_a", ModDest::Osc3WarpA},
        {"osc3_warp_b", ModDest::Osc3WarpB},
        {"osc1_pan", ModDest::Osc1Pan},
        {"osc2_pan", ModDest::Osc2Pan},
        {"osc3_pan", ModDest::Osc3Pan},
        {"filter_cutoff", ModDest::Cutoff},
        {"filter_res", ModDest::Resonance},
        {"filter_drive", ModDest::Drive},
        {"filter2_cutoff", ModDest::Cutoff2},
        {"filter2_res", ModDest::Resonance2},
        {"filter2_drive", ModDest::Drive2},
        {"filter_mix", ModDest::Filter1Mix},
        {"filter2_mix", ModDest::Filter2Mix},
        {"filter_pan", ModDest::Filter1Pan},
        {"filter2_pan", ModDest::Filter2Pan},
        {"sub_level", ModDest::SubLevel},
        {"noise_level", ModDest::NoiseLevel},
        {"osc1_uni_detune", ModDest::UnisonDetune},
        {"voice_transpose", ModDest::Pitch},
        {"lfo1_rate", ModDest::Lfo1Rate},
        {"lfo2_rate", ModDest::Lfo2Rate},
        {"lfo3_rate", ModDest::Lfo3Rate},
        {"lfo4_rate", ModDest::Lfo4Rate},
        {"master_gain", ModDest::MasterGain},
        {kTargetVoiceAmp, ModDest::AmpLevel},
    };
    const auto it = map.find(paramId);
    return it == map.end() ? ModDest::None : it->second;
}

bool ModRouteStore::accepts(const std::string& paramId) const
{
    if (const int n = routeAmountTarget(paramId); n >= 0)
        return n < kMaxRoutes;
    if (routeVoiceDestination(paramId) != ModDest::None)
        return true;
    const int i = params->indexOf(paramId);
    return i >= 0 && params->params()[static_cast<size_t>(i)].modulatable;
}

bool ModRouteStore::canTarget(const std::string& paramId)
{
    return ModRouteStore(nullptr).accepts(paramId);
}

ModRouteStore::ModRouteStore(const ParameterModel* model) : params(&modelOf(model))
{
    shared.publish(std::make_unique<RouteSet>());
}

void ModRouteStore::publishLocked()
{
    auto set = std::make_unique<RouteSet>();
    set->routes = list;
    const auto& targets = params->modTargets();
    std::vector<AmountMod> mods;
    for (size_t i = 0; i < list.size(); ++i)
    {
        const auto& r = list[i];
        const int ext = extSourceIndex(r.source);
        const int amountTarget = routeAmountTarget(r.target);
        const auto dest = amountTarget >= 0 ? ModDest::None : routeVoiceDestination(r.target);
        int global = -1;
        int voiceTarget = -1;
        if (amountTarget < 0 && dest == ModDest::None)
        {
            const int idx = params->indexOf(r.target);
            voiceTarget = params->voiceTargetOfParam(idx);
            const auto it = std::find(targets.begin(), targets.end(), idx);
            global = it == targets.end() || voiceTarget >= 0 ? -1 : static_cast<int>(it - targets.begin());
        }
        if (isModulatorSource(r.source))
            set->modulatorMask |= 1U << (r.source - kSourceModulator);
        if (isAudioSource(r.source))
            set->audioMask |= 1U << (r.source - kSourceAudio);
        set->voiceTarget.push_back(voiceTarget);
        if (isClipSource(r.source) || isMpeSource(r.source))
            set->dynamicRoutes.push_back(static_cast<int>(set->voiceDest.size()));
        set->voiceDest.push_back(static_cast<int>(dest));
        set->globalTarget.push_back(global);
        const int route = static_cast<int>(i);
        if (amountTarget >= 0)
        {
            // CC, clip lanes and expression have no extended index: such a route stays inert.
            // A target past the list (or past kMaxRoutes) is inert: the audio thread indexes its delta arrays with it.
            if (ext > 0 && amountSourceAllowed(r.source) && amountTarget != route && amountTarget < kMaxRoutes &&
                amountTarget < static_cast<int>(list.size()))
                mods.push_back({route, amountTarget, {ext, amountTarget, r.amount, r.polarity, r.curve}});
            continue;
        }
        AudioTarget kind{};
        int slot = -1;
        if (isAudioSource(r.source) && classifyAudioTarget(r.target, kind, slot))
        {
            set->audioRoutes.push_back(
                {r.source - kSourceAudio, kind, slot, r.amount, route, static_cast<int>(dest), voiceTarget});
            continue;
        }
        if (voiceTarget >= 0 && ext > 0)
        {
            set->voiceTargetSlots.push_back({ext, voiceTarget, r.amount, r.polarity, r.curve});
            set->voiceTargetSlotRoute.push_back(route);
        }
        if (dest != ModDest::None && ext > 0)
        {
            set->voiceSlots.push_back({ext, static_cast<int>(dest), r.amount, r.polarity, r.curve});
            set->voiceSlotRoute.push_back(route);
        }
    }
    // Order the amount modulations so that a route moves another's amount only after its own amount is settled; entries
    // that sit on a cycle (possible after a removal shifted the positions) are dropped.
    {
        std::vector<int> indegree(mods.size(), 0);
        for (size_t a = 0; a < mods.size(); ++a)
            for (size_t b = 0; b < mods.size(); ++b)
                if (mods[a].target == mods[b].route)
                    ++indegree[b];
        std::vector<size_t> ready;
        for (size_t a = 0; a < mods.size(); ++a)
            if (indegree[a] == 0)
                ready.push_back(a);
        while (!ready.empty())
        {
            const size_t a = ready.back();
            ready.pop_back();
            set->amountMods.push_back(mods[a]);
            for (size_t b = 0; b < mods.size(); ++b)
                if (mods[a].target == mods[b].route && --indegree[b] == 0)
                    ready.push_back(b);
        }
    }
    shared.publish(std::move(set));
}

namespace
{
bool sameRoute(const ModRoute& a, const ModRoute& b)
{
    return a.source == b.source && a.target == b.target && a.amount == b.amount && a.polarity == b.polarity &&
           a.curve == b.curve && a.smoothingMs == b.smoothingMs;
}
} // namespace

bool ModRouteStore::add(const ModRoute& route)
{
    if (!isRouteSource(route.source, params) || !accepts(route.target))
        return false;
    std::lock_guard<std::mutex> lock(mutex);
    ModRoute r = route;
    // std::clamp passes NaN through, so non-finite values become 0 first.
    if (!std::isfinite(r.amount))
        r.amount = 0.0f;
    if (!std::isfinite(r.curve))
        r.curve = 0.0f;
    if (!std::isfinite(r.smoothingMs))
        r.smoothingMs = 0.0f;
    r.amount = std::clamp(r.amount, -1.0f, 1.0f);
    r.polarity = std::clamp(r.polarity, 0, 2);
    r.curve = std::clamp(r.curve, -1.0f, 1.0f);
    r.smoothingMs = std::clamp(r.smoothingMs, 0.0f, 2000.0f);
    // The list as it would be: refuse a route that closes a cycle of amount modulation, or needs a ninth audio buffer.
    std::vector<ModRoute> trial = list;
    size_t at = trial.size();
    for (size_t i = 0; i < trial.size(); ++i)
        if (trial[i].source == r.source && trial[i].target == r.target)
            at = i;
    if (at == trial.size())
    {
        if (static_cast<int>(trial.size()) >= kMaxRoutes)
            return false;
        trial.push_back(r);
    }
    else
        trial[at] = r;
    if (amountGraphCyclic(trial))
        return false;
    std::uint32_t audio = 0;
    for (const auto& t : trial)
        if (isAudioSource(t.source))
            audio |= 1U << (t.source - kSourceAudio);
    if (std::bitset<32>(audio).count() > static_cast<size_t>(kAudioBusBuffers))
        return false;
    const bool same = at < list.size() && sameRoute(list[at], r);
    list = std::move(trial);
    publishLocked();
    if (!same)
        changes.fetch_add(1, std::memory_order_release);
    return true;
}

bool ModRouteStore::remove(int source, const std::string& target)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto before = list.size();
    list.erase(std::remove_if(list.begin(), list.end(),
                              [&](const ModRoute& r) { return r.source == source && r.target == target; }),
               list.end());
    if (list.size() == before)
        return false;
    publishLocked();
    changes.fetch_add(1, std::memory_order_release);
    return true;
}

void ModRouteStore::clear()
{
    std::lock_guard<std::mutex> lock(mutex);
    const bool had = !list.empty();
    list.clear();
    publishLocked();
    if (had)
        changes.fetch_add(1, std::memory_order_release);
}

std::vector<ModRoute> ModRouteStore::routes() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return list;
}

std::vector<ModRoute> ModRouteStore::routesFor(const std::string& target) const
{
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<ModRoute> out;
    for (const auto& r : list)
        if (r.target == target)
            out.push_back(r);
    return out;
}

bool ModRouteStore::routeAt(int k, ModRoute& out) const
{
    std::lock_guard<std::mutex> lock(mutex);
    if (k < 0 || k >= static_cast<int>(list.size()))
        return false;
    out = list[static_cast<size_t>(k)];
    return true;
}

int ModRouteStore::routeCount() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return static_cast<int>(list.size());
}

void ModRouteStore::replaceAll(const std::vector<ModRoute>& routes)
{
    const auto before = this->routes();
    const std::uint32_t generationBefore = changes.load(std::memory_order_acquire);
    clear();
    for (const auto& r : routes)
        add(r);
    const auto after = this->routes();
    bool same = before.size() == after.size();
    for (size_t i = 0; same && i < before.size(); ++i)
        same = sameRoute(before[i], after[i]);
    changes.store(generationBefore + (same ? 0U : 1U), std::memory_order_release);
}

std::string ModRouteStore::serialise() const
{
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<std::string> rows;
    for (const auto& r : list)
        rows.push_back(std::to_string(r.source) + "," + r.target + "," + text::fixed(r.amount, 6) + "," +
                       std::to_string(r.polarity) + "," + text::fixed(r.curve, 6) + "," + text::fixed(r.smoothingMs, 3));
    return text::join(rows, ';');
}

bool ModRouteStore::deserialise(const std::string& textIn)
{
    std::vector<ModRoute> routes;
    for (const auto& row : text::split(textIn, ';'))
    {
        const auto f = text::split(row, ',');
        if (f.size() < 6)
            continue;
        ModRoute r;
        r.source = text::toInt(f[0]);
        r.target = f[1];
        r.amount = text::toFloat(f[2]);
        r.polarity = text::toInt(f[3]);
        r.curve = text::toFloat(f[4]);
        r.smoothingMs = text::toFloat(f[5]);
        routes.push_back(r);
    }
    replaceAll(routes);
    return true;
}

} // namespace crucible
