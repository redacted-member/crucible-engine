// SPDX-License-Identifier: MIT
#include <crucible/ParameterMap.h>

#include <crucible/Modulation.h>
#include <crucible/ParamText.h>
#include <crucible/module/Module.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

// The parameter model is assembled from the module registry: the product layout gives the sections and the
// explicit page order, every registered module contributes its parameters in registration order, and pages a
// module uses that the layout does not list are appended (named after the module).

namespace crucible
{
namespace
{
std::string jsonEscape(const std::string& s)
{
    std::string out;
    for (const char c : s)
    {
        if (c == '"' || c == '\\')
            out += '\\';
        out += c;
    }
    return out;
}

std::string num(float v)
{
    std::ostringstream os;
    os << std::setprecision(7) << v;
    return os.str();
}

const char* typeName(ParamType t)
{
    switch (t)
    {
    case ParamType::Float:
        return "float";
    case ParamType::Int:
        return "int";
    case ParamType::Choice:
        return "choice";
    case ParamType::Bool:
        return "bool";
    }
    return "float";
}

const char* smoothingName(Smoothing s)
{
    switch (s)
    {
    case Smoothing::None:
        return "none";
    case Smoothing::Linear:
        return "linear";
    case Smoothing::Log:
        return "log";
    }
    return "none";
}

std::string idPath(const std::vector<SectionDef>& sections, const PageDef& page)
{
    const std::string section = page.section >= 0 && static_cast<size_t>(page.section) < sections.size()
                                    ? sections[static_cast<size_t>(page.section)].id
                                    : std::string("misc");
    return section + "/" + page.id;
}
} // namespace

std::vector<std::string> modSourceNamesFor(int macros)
{
    std::vector<std::string> n = {
        "None",    "LFO 1",   "LFO 2",   "Mod Env",   "Velocity",   "Key",     "Amp Env",     "Env 3",
        "Env 4",   "LFO 3",   "LFO 4",   "Mod Wheel", "Aftertouch", "Random",  "Macro 1",     "Macro 2",
        "Macro 3", "Macro 4", "Macro 5", "Macro 6",   "Strip L",    "Strip R", "Scene Morph", "Pitch Bend",
    };
    if (macros > kDefaultMacros)
    {
        n.push_back("Macro 7");
        n.push_back("Macro 8");
    }
    return n;
}

const std::vector<std::string>& legacyModDestNames()
{
    static const std::vector<std::string> names = {
        "None",          "Osc 1 Position", "Osc 2 Position", "Osc 1 Pitch",     "Osc 2 Pitch",   "Osc 1 Level",
        "Osc 2 Level",   "Cutoff",         "Resonance",      "Drive",           "Sub Level",     "Noise Level",
        "Unison Detune", "Amp Level",      "Osc 3 Position", "Osc 3 Pitch",     "Osc 3 Level",   "Osc 1 Warp A",
        "Osc 1 Warp B",  "Osc 2 Warp A",   "Osc 2 Warp B",   "Osc 3 Warp A",    "Osc 3 Warp B",  "Osc 1 Pan",
        "Osc 2 Pan",     "Osc 3 Pan",      "Cutoff 2",       "Resonance 2",     "Drive 2",       "Filter 1 Mix",
        "Filter 2 Mix",  "Filter 1 Pan",   "Filter 2 Pan",   "Pitch",           "LFO 1 Rate",    "LFO 2 Rate",
        "LFO 3 Rate",    "LFO 4 Rate",     "Master Gain",
    };
    return names;
}

ParameterModel::ParameterModel(const ModuleRegistry& registry)
{
    macroCount = registry.layout().macros;
    sourceNames = modSourceNamesFor(macroCount);
    sectionList = registry.layout().sections;
    pageList = registry.layout().pages;
    std::unordered_map<std::string, int> pageIndex;
    for (size_t i = 0; i < pageList.size(); ++i)
        pageIndex.emplace(pageList[i].id, static_cast<int>(i));
    const auto sectionOf = [&](const std::string& id)
    {
        for (size_t i = 0; i < sectionList.size(); ++i)
            if (sectionList[i].id == id)
                return static_cast<int>(i);
        return std::max(static_cast<int>(sectionList.size()) - 1, 0);
    };
    for (const auto& module : registry.modules())
        for (const auto& mp : module.pages)
            if (pageIndex.find(mp.id) == pageIndex.end())
            {
                PageDef page;
                page.id = mp.id;
                page.name = mp.name;
                page.section = sectionOf(mp.sectionId);
                page.layoutTitle = mp.title;
                page.layoutDescription = mp.description;
                pageList.push_back(page);
                pageIndex.emplace(mp.id, static_cast<int>(pageList.size()) - 1);
            }
    std::unordered_map<std::string, std::string> placed;
    for (const auto& [paramId, pageId] : registry.layout().pageOf)
        if (pageIndex.find(pageId) != pageIndex.end())
            placed.emplace(paramId, pageId);
    std::vector<std::pair<size_t, size_t>> moduleRange; // [first, end) of each module's regular parameters
    std::vector<size_t> appendedIdx;                    // parameters with an append sequence, ordered by (sequence, module order)
    int lastSequence = 0;
    for (const auto& module : registry.modules())
        for (const auto& mp : module.params)
            lastSequence = std::max(lastSequence, mp.appended);
    // Pass 0 lists the regular parameters in module order; pass n lists the parameters of append sequence n (module order
    // within a sequence), so a later addition never moves an earlier one.
    for (int pass = 0; pass <= lastSequence; ++pass)
    for (const auto& module : registry.modules())
    {
        const size_t first = paramList.size();
        for (auto p : module.params)
        {
            if (p.appended != pass)
                continue;
            if (!placed.empty())
            {
                const auto at = placed.find(p.id);
                if (at != placed.end())
                    p.pageId = at->second;
            }
            auto it = pageIndex.find(p.pageId);
            if (it == pageIndex.end())
            {
                PageDef page;
                page.id = p.pageId;
                page.name = module.name;
                page.section = sectionOf(module.sectionHint);
                page.layoutTitle = module.name;
                page.layoutDescription = module.description.empty() ? module.name : module.description;
                pageList.push_back(page);
                it = pageIndex.emplace(p.pageId, static_cast<int>(pageList.size()) - 1).first;
            }
            p.page = it->second;
            if (pass >= 1)
                appendedIdx.push_back(paramList.size());
            paramList.push_back(std::move(p));
        }
        if (pass == 0)
            moduleRange.emplace_back(first, paramList.size());
    }
    if (registry.layout().dropEmptyPages)
    {
        // The surface's own pages and sections stay; any other page stays only while a parameter is on it, and a section
        // only while a kept page is in it.
        const size_t keepPages = registry.layout().keepPages;
        const size_t keepSections = registry.layout().keepSections;
        std::vector<bool> keptPage(pageList.size(), false);
        for (size_t i = 0; i < keepPages && i < keptPage.size(); ++i)
            keptPage[i] = true;
        for (const auto& p : paramList)
            keptPage[static_cast<size_t>(p.page)] = true;
        std::vector<bool> keptSection(sectionList.size(), false);
        for (size_t i = 0; i < keepSections && i < keptSection.size(); ++i)
            keptSection[i] = true;
        for (size_t i = 0; i < pageList.size(); ++i)
            if (keptPage[i] && pageList[i].section >= 0 && static_cast<size_t>(pageList[i].section) < keptSection.size())
                keptSection[static_cast<size_t>(pageList[i].section)] = true;
        std::vector<int> sectionMap(sectionList.size(), 0);
        std::vector<SectionDef> sections;
        for (size_t i = 0; i < sectionList.size(); ++i)
            if (keptSection[i])
            {
                sectionMap[i] = static_cast<int>(sections.size());
                sections.push_back(sectionList[i]);
            }
        std::vector<int> pageMap(pageList.size(), -1);
        std::vector<PageDef> pages;
        for (size_t i = 0; i < pageList.size(); ++i)
            if (keptPage[i])
            {
                pageMap[i] = static_cast<int>(pages.size());
                pages.push_back(pageList[i]);
                auto& section = pages.back().section;
                section = section >= 0 && static_cast<size_t>(section) < sectionMap.size()
                              ? sectionMap[static_cast<size_t>(section)]
                              : 0;
            }
        for (auto& p : paramList)
            p.page = pageMap[static_cast<size_t>(p.page)];
        pageList = std::move(pages);
        sectionList = std::move(sections);
    }
    // Generic modulation targets, then the destination lists of the matrix slots. Targets are numbered module by module in
    // registration order; parameters that a module turns into per-voice targets after the fact (lateVoiceMod) follow that
    // module's own targets, so a module registered later never moves an earlier destination number.
    destNames = legacyModDestNames();
    voiceOfParam.assign(paramList.size(), -1);
    const auto addTarget = [&](size_t i)
    {
        targets.push_back(static_cast<int>(i));
        destNames.push_back(paramList[i].name);
        int voiceIndex = -1;
        if (paramList[i].voiceMod)
        {
            voiceIndex = static_cast<int>(voiceTargetList.size());
            voiceTargetList.push_back(
                {static_cast<int>(i), paramList[i].max - paramList[i].min, paramList[i].smoothing == Smoothing::Log});
            voiceOfParam[i] = voiceIndex;
        }
        voiceOfModTarget.push_back(voiceIndex);
    };
    for (size_t m = 0; m < registry.modules().size(); ++m)
    {
        const auto& module = registry.modules()[m];
        for (size_t i = moduleRange[m].first; i < moduleRange[m].second; ++i)
            if (paramList[i].modulatable)
                addTarget(i);
        for (const auto& id : module.lateVoiceMod)
        {
            const auto it = std::find_if(paramList.begin(), paramList.end(), [&](const ParamDef& d) { return d.id == id; });
            if (it == paramList.end() || it->modulatable)
                continue;
            it->modulatable = true;
            it->voiceMod = true;
            addTarget(static_cast<size_t>(it - paramList.begin()));
        }
    }
    for (const size_t i : appendedIdx)
        if (paramList[i].modulatable)
            addTarget(i);
    if (voiceTargetList.size() > static_cast<size_t>(ParameterMap::kMaxVoiceTargets))
        throw std::logic_error("too many per-voice modulation targets: " + std::to_string(voiceTargetList.size()) +
                               " (limit " + std::to_string(ParameterMap::kMaxVoiceTargets) + ")");
    for (auto& p : paramList)
        if (p.destinationChoice)
        {
            p.choices = destNames;
            p.max = static_cast<float>(p.choices.size() - 1);
        }
    // Resolve the self-description defaults, so that a model's ParamDef is complete.
    for (auto& p : paramList)
    {
        if (p.shortName.empty())
            p.shortName = ParamText::shortNameOf(p.name);
        if (p.group.empty())
            p.group = idPath(sectionList, pageList[static_cast<size_t>(p.page)]);
        if (p.polarity == Polarity::Auto)
            p.polarity = p.min < 0.0f && p.max > 0.0f ? Polarity::Bipolar : Polarity::Unipolar;
        p.textFormat = ParamText::resolveFormat(p);
        p.routable = p.modulatable || routeVoiceDestination(p.id) != ModDest::None;
        if (p.uiHint == UiHint::Auto)
            p.uiHint =
                p.type == ParamType::Choice ? UiHint::Menu : (p.type == ParamType::Bool ? UiHint::Switch : UiHint::Knob);
        switch (p.type)
        {
        case ParamType::Choice:
            p.steps = static_cast<int>(p.choices.size());
            break;
        case ParamType::Bool:
            p.steps = 2;
            break;
        case ParamType::Int:
            p.steps = static_cast<int>(std::lround(p.max - p.min)) + 1;
            break;
        case ParamType::Float:
            p.steps = p.interval > 0.0f ? static_cast<int>(std::floor((p.max - p.min) / p.interval + 0.5f)) + 1 : 0;
            break;
        }
    }
    sortedIds.reserve(paramList.size());
    for (size_t i = 0; i < paramList.size(); ++i)
        sortedIds.emplace_back(paramList[i].id, static_cast<int>(i));
    std::stable_sort(sortedIds.begin(), sortedIds.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    sortedHostIds.reserve(paramList.size());
    for (size_t i = 0; i < paramList.size(); ++i)
        sortedHostIds.emplace_back(hostIdOf(paramList[i].id), static_cast<int>(i));
    std::sort(sortedHostIds.begin(), sortedHostIds.end());
    // The host surface. Unlisted parameters of a "listed" product stay in the model (patches load them, the matrix can
    // target them) but are not automatable.
    if (registry.layout().exposeListed)
    {
        for (auto& p : paramList)
            p.exposed = false;
        for (const auto& id : registry.layout().surface)
        {
            const int i = indexOf(id);
            if (i < 0 || paramList[static_cast<size_t>(i)].exposed)
                continue;
            paramList[static_cast<size_t>(i)].exposed = true;
            surfaceList.push_back(i);
        }
        for (auto& p : paramList)
            if (!p.exposed)
                p.automatable = false;
    }
    else
        for (size_t i = 0; i < paramList.size(); ++i)
        {
            paramList[i].exposed = paramList[i].automatable;
            if (paramList[i].exposed)
                surfaceList.push_back(static_cast<int>(i));
        }
}

int ParameterModel::indexOf(std::string_view id) const
{
    const auto it = std::lower_bound(sortedIds.begin(), sortedIds.end(), id,
                                     [](const auto& e, std::string_view k) { return std::string_view(e.first) < k; });
    return it != sortedIds.end() && it->first == id ? it->second : -1;
}

int ParameterModel::indexOfHostId(std::uint32_t hostId) const
{
    const auto it = std::lower_bound(sortedHostIds.begin(), sortedHostIds.end(), hostId,
                                     [](const auto& e, std::uint32_t k) { return e.first < k; });
    return it != sortedHostIds.end() && it->first == hostId ? it->second : -1;
}

std::vector<int> ParameterModel::paramsOnPage(int page) const
{
    std::vector<int> out;
    for (size_t i = 0; i < paramList.size(); ++i)
        if (paramList[i].page == page)
            out.push_back(static_cast<int>(i));
    return out;
}

int ParameterModel::voiceTargetOfParam(int paramIndex) const
{
    return paramIndex >= 0 && static_cast<size_t>(paramIndex) < voiceOfParam.size()
               ? voiceOfParam[static_cast<size_t>(paramIndex)]
               : -1;
}

int ParameterModel::voiceTargetOfModTarget(int modTarget) const
{
    return modTarget >= 0 && static_cast<size_t>(modTarget) < voiceOfModTarget.size()
               ? voiceOfModTarget[static_cast<size_t>(modTarget)]
               : -1;
}

std::vector<float> ParameterModel::defaultValues() const
{
    std::vector<float> v;
    v.reserve(paramList.size());
    for (const auto& p : paramList)
        v.push_back(p.defaultValue);
    return v;
}

std::string ParameterModel::valueToText(int paramIndex, float value) const
{
    if (paramIndex < 0 || paramIndex >= size())
        return {};
    return ParamText::format(paramList[static_cast<size_t>(paramIndex)], value);
}

bool ParameterModel::choiceAvailable(int paramIndex, int choice) const
{
    if (paramIndex < 0 || paramIndex >= size())
        return false;
    const auto& p = paramList[static_cast<size_t>(paramIndex)];
    if (p.type != ParamType::Choice || choice < 0 || static_cast<size_t>(choice) >= p.choices.size())
        return false;
    return std::find(p.unavailable.begin(), p.unavailable.end(), choice) == p.unavailable.end();
}

bool ParameterModel::textToValue(int paramIndex, const std::string& text, float& out) const
{
    if (paramIndex < 0 || paramIndex >= size())
        return false;
    return ParamText::parse(paramList[static_cast<size_t>(paramIndex)], text, out);
}

float toNormalised(const ParamDef& def, float natural)
{
    const float range = def.max - def.min;
    if (range <= 0.0f)
        return 0.0f;
    const float p = std::clamp((natural - def.min) / range, 0.0f, 1.0f);
    return def.skew == 1.0f ? p : std::pow(p, def.skew);
}

float fromNormalised(const ParamDef& def, float normalised)
{
    const float n = std::clamp(normalised, 0.0f, 1.0f);
    float v = def.min + (def.max - def.min) * (def.skew == 1.0f ? n : std::pow(n, 1.0f / def.skew));
    float interval = def.interval;
    if (def.type != ParamType::Float)
        interval = std::max(interval, 1.0f);
    if (interval > 0.0f)
        v = def.min + interval * std::round((v - def.min) / interval);
    return std::clamp(v, def.min, def.max);
}

std::string ParameterModel::toJson(const JsonExtras& extras) const
{
    std::ostringstream os;
    os << "{\n";
    os << "  \"schema\": 1,\n";
    os << "  \"plugin\": \"" << jsonEscape(extras.pluginName) << "\",\n";
    os << "  \"ParameterMap::knobsPerPage\": " << ParameterMap::knobsPerPage << ",\n";
    os << "  \"valueMapping\": \"natural = min + (max - min) * normalised ^ (1 / skew); choices and ints are "
          "indices/steps\",\n";

    os << "  \"sections\": [\n";
    const auto& secs = sections();
    for (size_t i = 0; i < secs.size(); ++i)
        os << "    {\"index\": " << i << ", \"id\": \"" << secs[i].id << "\", \"name\": \"" << jsonEscape(secs[i].name)
           << "\"}" << (i + 1 < secs.size() ? "," : "") << "\n";
    os << "  ],\n";

    os << "  \"pages\": [\n";
    const auto& pg = pages();
    for (size_t pi = 0; pi < pg.size(); ++pi)
    {
        os << "    {\n";
        os << "      \"index\": " << pi << ",\n";
        os << "      \"id\": \"" << pg[pi].id << "\",\n";
        os << "      \"name\": \"" << jsonEscape(pg[pi].name) << "\",\n";
        os << "      \"section\": " << pg[pi].section << ",\n";
        os << "      \"layout\": {\"title\": \"" << jsonEscape(pg[pi].layoutTitle) << "\", \"description\": \""
           << jsonEscape(pg[pi].layoutDescription) << "\"},\n";
        if (extras.pageBlock)
            os << "      \"" << extras.blockKey << "\": " << extras.pageBlock(static_cast<int>(pi), 6) << ",\n";
        os << "      \"params\": [\n";
        const auto idx = paramsOnPage(static_cast<int>(pi));
        for (size_t k = 0; k < idx.size(); ++k)
        {
            const auto& p = params()[static_cast<size_t>(idx[k])];
            os << "        {\"knob\": " << k << ", \"id\": \"" << p.id << "\", \"name\": \"" << jsonEscape(p.name)
               << "\", \"type\": \"" << typeName(p.type) << "\", \"min\": " << num(p.min) << ", \"max\": " << num(p.max)
               << ", \"default\": " << num(p.defaultValue) << ", \"interval\": " << num(p.interval)
               << ", \"skew\": " << num(p.skew) << ", \"unit\": \"" << jsonEscape(p.unit) << "\", \"smoothing\": \""
               << smoothingName(p.smoothing) << "\"";
            if (!p.choices.empty())
            {
                os << ", \"choices\": [";
                for (size_t c = 0; c < p.choices.size(); ++c)
                    os << (c ? ", " : "") << "\"" << jsonEscape(p.choices[c]) << "\"";
                os << "]";
            }
            os << "}" << (k + 1 < idx.size() ? "," : "") << "\n";
        }
        os << "      ]\n";
        os << "    }" << (pi + 1 < pg.size() ? "," : "") << "\n";
    }
    if (extras.globalBlock)
    {
        os << "  ],\n";
        os << "  \"" << extras.blockKey << "\": " << extras.globalBlock(2) << "\n";
    }
    else
    {
        os << "  ]\n";
    }
    os << "}\n";
    return os.str();
}

namespace ParameterMap
{
namespace
{
const ParameterModel& model()
{
    return ModuleRegistry::instance().parameters();
}
} // namespace
const std::vector<SectionDef>& sections()
{
    return model().sections();
}
const std::vector<PageDef>& pages()
{
    return model().pages();
}
const std::vector<ParamDef>& params()
{
    return model().params();
}
const std::vector<std::string>& modSourceNames()
{
    return model().modSourceNames();
}
const std::vector<std::string>& modDestNames()
{
    return model().modDestNames();
}
const std::vector<std::string>& legacyModDestNames()
{
    return crucible::legacyModDestNames();
}
int indexOf(std::string_view id)
{
    return model().indexOf(id);
}
std::vector<int> paramsOnPage(int page)
{
    return model().paramsOnPage(page);
}
const std::vector<int>& modTargets()
{
    return model().modTargets();
}
const std::vector<VoiceTarget>& voiceTargets()
{
    return model().voiceTargets();
}
int voiceTargetOfParam(int paramIndex)
{
    return model().voiceTargetOfParam(paramIndex);
}
int voiceTargetOfModTarget(int modTarget)
{
    return model().voiceTargetOfModTarget(modTarget);
}
std::vector<float> defaultValues()
{
    return model().defaultValues();
}
std::string toJson(const JsonExtras& extras)
{
    return model().toJson(extras);
}
} // namespace ParameterMap

} // namespace crucible
