// SPDX-License-Identifier: MIT
#include <crucible/graph/Compiler.h>

#include <crucible/module/ModuleRack.h>
#include <crucible/sdk/Host.h>

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace crucible::graph
{
namespace
{
// The registry of the neutral product with every built-in module: the reference for what a built-in effect is.
const ModuleRegistry& referenceRegistry()
{
    static const std::shared_ptr<const ModuleRegistry> registry = ModuleRegistry::build(ModuleRegistry::neutralLayout());
    return *registry;
}

bool validName(const std::string& s)
{
    if (s.empty() || !(s[0] >= 'a' && s[0] <= 'z'))
        return false;
    return std::all_of(s.begin(), s.end(),
                       [](char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'; });
}

bool validPinned(const std::string& s)
{
    if (s.empty())
        return false;
    return std::all_of(s.begin(), s.end(), [](char c)
                       { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_'; });
}

struct PortRef
{
    std::string node, port;
};

bool splitPort(const std::string& s, PortRef& out)
{
    const auto dot = s.rfind('.');
    if (dot == std::string::npos || dot == 0 || dot + 1 >= s.size())
        return false;
    out.node = s.substr(0, dot);
    out.port = s.substr(dot + 1);
    return true;
}

bool isAudio(PortType t)
{
    return t == PortType::AudioMono || t == PortType::AudioStereo;
}

bool isRate(PortType t)
{
    return t == PortType::AudioRate;
}

// Ports of one class are numbered separately in the plan: 0 audio, 1 audio-rate, 2 modulation and trigger.
int portClass(PortType t)
{
    return isAudio(t) ? 0 : (isRate(t) ? 1 : 2);
}

// One voice graph (a layer, or the whole voice).
struct Group
{
    std::string path;
    const std::vector<NodeDef>* nodes;
    const std::vector<EdgeDef>* edges;
};

std::vector<Group> groupsOf(const ProductDef& p)
{
    std::vector<Group> out;
    if (p.voice.kind != VoiceKind::Graph)
        return out;
    if (p.voice.layers.empty())
        out.push_back({"voice", &p.voice.nodes, &p.voice.edges});
    for (size_t i = 0; i < p.voice.layers.size(); ++i)
        out.push_back({"voice.layers[" + std::to_string(i) + "]", &p.voice.layers[i].nodes, &p.voice.layers[i].edges});
    return out;
}

// Kahn's algorithm over indices; returns false on a cycle. Ties keep definition order.
bool topoSort(size_t count, const std::vector<std::pair<size_t, size_t>>& edges, std::vector<size_t>& order)
{
    std::vector<int> indegree(count, 0);
    std::vector<std::vector<size_t>> out(count);
    for (const auto& [a, b] : edges)
    {
        out[a].push_back(b);
        ++indegree[b];
    }
    order.clear();
    std::vector<size_t> ready;
    for (size_t i = 0; i < count; ++i)
        if (indegree[i] == 0)
            ready.push_back(i);
    while (!ready.empty())
    {
        // Smallest index first, for a deterministic schedule.
        const auto it = std::min_element(ready.begin(), ready.end());
        const size_t n = *it;
        ready.erase(it);
        order.push_back(n);
        for (const size_t m : out[n])
            if (--indegree[m] == 0)
                ready.push_back(m);
    }
    return order.size() == count;
}

std::string fxGeneratedId(const std::string& baseId, int k)
{
    if (k <= 1)
        return baseId;
    const std::string prefix = "fx_";
    const std::string tag = "fx" + std::to_string(k) + "_";
    if (baseId.compare(0, prefix.size(), prefix) == 0)
        return tag + baseId.substr(prefix.size());
    return tag + baseId;
}

// An effect module a product can instantiate: a built-in one, or one the module SDK added to the node registry.
const ModuleDescriptor* findFxType(const std::string& type, const NodeRegistry& nodes)
{
    const auto* d = referenceRegistry().find(type);
    if (d != nullptr && d->kind == ModuleKind::Effect && d->createEffect != nullptr)
        return d;
    d = nodes.findEffect(type);
    return d != nullptr && d->kind == ModuleKind::Effect && d->createEffect != nullptr ? d : nullptr;
}

void validateGroup(const Group& g, const NodeRegistry& nodes, const ProductLimits& limits, ValidationResult& res,
                   std::set<std::string>& usedIds, int& modulators)
{
    const auto err = [&](const std::string& path, const std::string& m) { res.errors.push_back({path, m}); };
    if (static_cast<int>(g.nodes->size()) > limits.nodesPerVoice)
        err(g.path + ".nodes", "more than " + std::to_string(limits.nodesPerVoice) + " nodes in one voice");
    std::unordered_map<std::string, const NodeType*> byId;
    int sinks = 0;
    for (size_t i = 0; i < g.nodes->size(); ++i)
    {
        const NodeDef& n = (*g.nodes)[i];
        const std::string path = g.path + ".nodes[" + std::to_string(i) + "]";
        if (!validName(n.id))
        {
            err(path, "node id \"" + n.id + "\" must be lower case letters, digits and underscores, starting with a letter");
            continue;
        }
        if (!usedIds.insert(n.id).second)
            err(path, "id \"" + n.id + "\" is used twice in the product");
        const NodeType* t = nodes.find(n.type);
        if (t == nullptr)
        {
            err(path, "unknown node type \"" + n.type + "\"");
            continue;
        }
        byId[n.id] = t;
        if (n.type == "classic.voice")
            err(path, "classic.voice is the whole voice; use \"voice\": \"classic\" instead of mixing it into a graph");
        if (n.type == "mix.out")
            ++sinks;
        if (t->kind == ModuleKind::Modulator && !t->outputs.empty() && t->outputs[0].type == PortType::Modulation)
            ++modulators;
        if (t->usesSlot)
        {
            if (n.slot < 1 || n.slot > kMaxOscSlots)
                err(path, "\"slot\" must be an oscillator slot 1.." + std::to_string(kMaxOscSlots));
        }
        else if (n.slot != 0)
            err(path, "\"slot\" only applies to oscillator nodes");
        if (n.scope != "voice" && n.scope != "layer")
            err(path, "scope \"" + n.scope + "\" is not supported (voice or layer)");
        for (const auto& [local, value] : n.params)
        {
            const auto it =
                std::find_if(t->params.begin(), t->params.end(), [&](const ParamDef& p) { return p.id == local; });
            if (it == t->params.end())
                err(path + ".params", "\"" + local + "\" is not a parameter of " + n.type);
            else if (static_cast<float>(value) < it->min || static_cast<float>(value) > it->max)
                err(path + ".params", "\"" + local + "\" is out of range");
        }
    }
    if (sinks != 1)
        err(g.path, "a voice graph needs exactly one mix.out node (found " + std::to_string(sinks) + ")");

    std::vector<std::pair<size_t, size_t>> order;
    std::unordered_map<std::string, size_t> index;
    for (size_t i = 0; i < g.nodes->size(); ++i)
        index[(*g.nodes)[i].id] = i;
    for (size_t i = 0; i < g.edges->size(); ++i)
    {
        const EdgeDef& e = (*g.edges)[i];
        const std::string path = g.path + ".edges[" + std::to_string(i) + "]";
        PortRef from, to;
        if (!splitPort(e.from, from) || !splitPort(e.to, to))
        {
            err(path, "endpoints are written \"node.port\"");
            continue;
        }
        const auto fn = byId.find(from.node);
        const auto tn = byId.find(to.node);
        if (fn == byId.end() || tn == byId.end())
        {
            err(path, "unknown node \"" + (fn == byId.end() ? from.node : to.node) + "\"");
            continue;
        }
        const int fp = portIndex(fn->second->outputs, from.port);
        const int tp = portIndex(tn->second->inputs, to.port);
        if (fp < 0)
        {
            err(path, from.node + " has no output \"" + from.port + "\"");
            continue;
        }
        if (tp < 0)
        {
            err(path, to.node + " has no input \"" + to.port + "\"");
            continue;
        }
        const PortType fromType = fn->second->outputs[static_cast<size_t>(fp)].type;
        const PortType toType = tn->second->inputs[static_cast<size_t>(tp)].type;
        if (isRate(toType))
        {
            if (!isAudio(fromType))
            {
                err(path, "an audio-rate input takes an audio output (" + from.node + "." + from.port + " is not one)");
                continue;
            }
        }
        else if (isAudio(fromType) != isAudio(toType))
        {
            err(path, "audio and modulation ports cannot be connected");
            continue;
        }
        order.emplace_back(index[from.node], index[to.node]);
    }
    std::vector<size_t> sorted;
    if (!topoSort(g.nodes->size(), order, sorted))
        err(g.path + ".edges",
            "the graph has a cycle (feedback needs an explicit feedback node, which this build does not have)");
}
} // namespace

std::string ValidationResult::text() const
{
    std::string out;
    for (const auto& e : errors)
        out += e.path + ": " + e.message + "\n";
    return out;
}

ValidationResult validate(const ProductDef& p, const NodeRegistry& nodes, const ProductLimits& limits)
{
    ValidationResult res;
    const auto err = [&](const std::string& path, const std::string& m) { res.errors.push_back({path, m}); };
    if (p.version != kProductFormatVersion)
        err("version", "unsupported product format version");
    if (p.id.empty())
        err("id", "a product needs an id");
    if (p.oscillators < 0 || p.oscillators > kMaxOscSlots)
        err("layout.oscillators", "must be 0.." + std::to_string(kMaxOscSlots));
    if (p.filters < 0 || p.filters > kMaxFilterSlots)
        err("layout.filters", "must be 0.." + std::to_string(kMaxFilterSlots));
    if (p.macros < 0 || p.macros > kNumMacros)
        err("layout.macros", "must be 0.." + std::to_string(kNumMacros));

    {
        std::set<std::string> sectionIds, pageIds;
        for (size_t i = 0; i < p.surface.sections.size(); ++i)
            if (p.surface.sections[i].id.empty() || !sectionIds.insert(p.surface.sections[i].id).second)
                err("surface.sections[" + std::to_string(i) + "]",
                    "section id \"" + p.surface.sections[i].id + "\" is empty or used twice");
        for (size_t i = 0; i < p.surface.pages.size(); ++i)
        {
            const auto& page = p.surface.pages[i];
            const std::string here = "surface.pages[" + std::to_string(i) + "]";
            if (page.id.empty() || !pageIds.insert(page.id).second)
                err(here, "page id \"" + page.id + "\" is empty or used twice");
            if (!page.section.empty() && !p.surface.sections.empty() && sectionIds.count(page.section) == 0)
                err(here, "unknown section \"" + page.section + "\"");
        }
    }

    std::set<std::string> usedIds;
    int modulators = 0;
    const auto groups = groupsOf(p);
    if (p.voice.kind == VoiceKind::Graph)
    {
        if (!p.voice.nodes.empty() && !p.voice.layers.empty())
            err("voice", "use either voice.nodes or voice.layers, not both");
        if (p.voice.nodes.empty() && p.voice.layers.empty())
            err("voice", "a graph voice needs nodes or layers");
        if (static_cast<int>(p.voice.layers.size()) > limits.layers)
            err("voice.layers", "more than " + std::to_string(limits.layers) + " layers");
        std::set<std::string> layerIds;
        for (size_t i = 0; i < p.voice.layers.size(); ++i)
        {
            const LayerDef& l = p.voice.layers[i];
            const std::string path = "voice.layers[" + std::to_string(i) + "]";
            if (!validName(l.id) || !layerIds.insert(l.id).second)
                err(path, "layer id \"" + l.id + "\" must be unique lower case letters, digits and underscores");
            if (l.noteLo < 0 || l.noteHi > 127 || l.noteLo > l.noteHi)
                err(path + ".notes", "must be [low, high] within 0..127");
            if (l.velLo < 1 || l.velHi > 127 || l.velLo > l.velHi)
                err(path + ".velocity", "must be [low, high] within 1..127");
            if (l.voices < 1 || l.voices > 32)
                err(path + ".voices", "must be 1..32");
            if (!l.chokeGroup.empty() && !validName(l.chokeGroup))
                err(path + ".choke", "group names are lower case letters, digits and underscores");
        }
        for (const auto& g : groups)
            validateGroup(g, nodes, limits, res, usedIds, modulators);
        if (modulators > limits.modulators)
            err("voice", "more than " + std::to_string(limits.modulators) + " modulators");
    }

    if (p.fx.present)
    {
        std::unordered_map<std::string, const FxInstanceDef*> inst;
        if (static_cast<int>(p.fx.instances.size()) > limits.fxInstances)
            err("fx.instances", "more than " + std::to_string(limits.fxInstances) + " instances");
        for (size_t i = 0; i < p.fx.instances.size(); ++i)
        {
            const auto& d = p.fx.instances[i];
            const std::string path = "fx.instances[" + std::to_string(i) + "]";
            if (!validName(d.id))
                err(path, "instance id \"" + d.id + "\" must be lower case letters, digits and underscores");
            else if (!usedIds.insert(d.id).second)
                err(path, "id \"" + d.id + "\" is used twice in the product");
            if (findFxType(d.type, nodes) == nullptr)
                err(path, "\"" + d.type + "\" is not an effect module");
            inst[d.id] = &d;
        }
        if (p.fx.lanes.empty())
            err("fx.lanes", "an FX graph needs at least one lane");
        if (p.fx.lanes.size() > static_cast<size_t>(ModuleRack::kMaxLanes))
            err("fx.lanes", "more than " + std::to_string(ModuleRack::kMaxLanes) + " lanes");
        std::unordered_map<std::string, size_t> laneIndex;
        for (size_t i = 0; i < p.fx.lanes.size(); ++i)
        {
            const auto& l = p.fx.lanes[i];
            const std::string path = "fx.lanes[" + std::to_string(i) + "]";
            if (!validName(l.id) || l.id == "voices" || l.id == "aux" || !laneIndex.emplace(l.id, i).second)
                err(path, "lane id \"" + l.id + "\" must be unique and not \"voices\" or \"aux\"");
        }
        std::set<std::string> placed;
        std::vector<std::pair<size_t, size_t>> deps;
        for (size_t i = 0; i < p.fx.lanes.size(); ++i)
        {
            const auto& l = p.fx.lanes[i];
            const std::string path = "fx.lanes[" + std::to_string(i) + "]";
            for (const auto& s : l.inputs)
            {
                if (s.source == "voices" || s.source == "aux")
                    continue;
                const auto it = laneIndex.find(s.source);
                if (it == laneIndex.end())
                    err(path + ".input", "unknown source \"" + s.source + "\"");
                else
                    deps.emplace_back(it->second, i);
            }
            for (const auto& c : l.chain)
            {
                if (inst.find(c) == inst.end())
                    err(path + ".chain", "unknown instance \"" + c + "\"");
                else if (!placed.insert(c).second)
                    err(path + ".chain", "instance \"" + c + "\" is in more than one lane");
            }
        }
        for (size_t i = 0; i < p.fx.sends.size(); ++i)
        {
            const auto& s = p.fx.sends[i];
            const std::string path = "fx.sends[" + std::to_string(i) + "]";
            const auto a = laneIndex.find(s.from);
            const auto b = laneIndex.find(s.to);
            if (a == laneIndex.end() || b == laneIndex.end())
                err(path, "sends connect lanes by id");
            else
                deps.emplace_back(a->second, b->second);
        }
        for (const auto& d : p.fx.instances)
            if (placed.find(d.id) == placed.end())
                err("fx.instances", "instance \"" + d.id + "\" is in no lane");
        std::vector<size_t> sorted;
        if (!topoSort(p.fx.lanes.size(), deps, sorted))
            err("fx.lanes", "lanes feed each other in a cycle");
    }
    else if (p.fx.instances.size() + p.fx.lanes.size() + p.fx.sends.size() != 0)
        err("fx", "inconsistent definition");

    std::set<std::string> pinned;
    for (const auto& [gen, to] : p.idMap)
        if (!validPinned(to) || !pinned.insert(to).second)
            err("idMap." + gen, "\"" + to + "\" must be a unique id of letters, digits and underscores");
    return res;
}

// ---------------------------------------------------------------------------------------------------------------
std::vector<float> CompiledProduct::defaultValues() const
{
    const auto& model = registry->parameters();
    auto v = model.defaultValues();
    for (const auto& [id, value] : def.defaults)
    {
        const int i = model.indexOf(id);
        if (i >= 0)
        {
            const auto& p = model.params()[static_cast<size_t>(i)];
            v[static_cast<size_t>(i)] = std::clamp(static_cast<float>(value), p.min, p.max);
        }
    }
    return v;
}

std::vector<std::string> CompiledProduct::parameterIds() const
{
    std::vector<std::string> ids;
    for (const auto& p : registry->parameters().params())
        ids.push_back(p.id);
    return ids;
}

bool CompiledProduct::sameParameters(const CompiledProduct& other) const
{
    return parameterIds() == other.parameterIds();
}

std::string CompiledProduct::nodeParameterId(const std::string& node, const std::string& local) const
{
    const std::string key = node + "/" + local;
    for (const auto& [k, id] : nodeParamIds)
        if (k == key)
            return id;
    return {};
}

bool CompiledProduct::buildFxPlan(const ModuleRack& rack, FxPlan& out) const
{
    if (fx.classic)
    {
        out = FxPlan::classic(rack.size());
        return true;
    }
    out = FxPlan{};
    out.masterLevel = fx.masterLevel;
    for (const auto& spec : fx.lanes)
    {
        FxLane lane;
        lane.inputs = spec.inputs;
        if (lane.inputs.empty())
            lane.inputs.push_back({FxSourceKind::Voices, -1, 1.0f});
        lane.byPosition = spec.byPosition;
        lane.toMaster = spec.toMaster;
        lane.level = spec.level;
        for (const auto& m : spec.modules)
        {
            const int slot = rack.slotOf(m);
            if (slot < 0)
                return false;
            lane.slots.push_back(slot);
        }
        out.lanes.push_back(std::move(lane));
    }
    return rack.validPlan(out);
}

namespace
{
ModuleDescriptor cloneEffect(const ModuleDescriptor& proto, int k, const std::string& label,
                             const std::function<std::string(const std::string&)>& pin)
{
    ModuleDescriptor d = proto;
    const std::string suffix = std::to_string(k);
    d.id = proto.id + "." + suffix;
    d.name = proto.name + " " + suffix;
    d.instanceOf = proto.id;
    d.instanceLabel = label;
    std::unordered_map<std::string, std::string> renamed;
    std::unordered_map<std::string, std::string> pages;
    for (auto& p : d.params)
    {
        const std::string from = p.id;
        p.id = pin(fxGeneratedId(from, k));
        renamed[from] = p.id;
        const std::string page = p.pageId;
        if (pages.find(page) == pages.end())
            pages[page] = fxGeneratedId(page, k);
        p.pageId = pages[page];
        p.since = 1;
    }
    for (auto& mp : d.pages)
    {
        mp.id = fxGeneratedId(mp.id, k);
        mp.name += " " + suffix;
        mp.title += " " + suffix;
    }
    if (renamed.count(d.onParam) != 0)
        d.onParam = renamed[d.onParam];
    if (renamed.count(d.positionParam) != 0)
        d.positionParam = renamed[d.positionParam];
    d.lateVoiceMod.clear();
    d.migrations.clear();
    d.version = 1;
    return d;
}

// The first instance of an effect module the SDK added: the module as it is, with its parameter ids pinned.
ModuleDescriptor firstExternalEffect(const ModuleDescriptor& proto,
                                     const std::function<std::string(const std::string&)>& pin)
{
    ModuleDescriptor d = proto;
    std::unordered_map<std::string, std::string> renamed;
    for (auto& p : d.params)
    {
        const std::string from = p.id;
        p.id = pin(from);
        renamed[from] = p.id;
    }
    if (renamed.count(d.onParam) != 0)
        d.onParam = renamed[d.onParam];
    if (renamed.count(d.positionParam) != 0)
        d.positionParam = renamed[d.positionParam];
    return d;
}

ModuleDescriptor instantiateNode(const NodeType& type, const NodeDef& def,
                                 const std::function<std::string(const std::string&)>& pin,
                                 std::vector<std::pair<std::string, std::string>>& ids)
{
    ModuleDescriptor d;
    d.id = def.id;
    d.name = def.id;
    d.kind = type.kind;
    d.category = type.category;
    d.version = type.version;
    d.description = type.description;
    d.sectionHint = type.kind == ModuleKind::Oscillator ? "osc1" : (type.kind == ModuleKind::Filter ? "mod" : "env");
    d.instanceOf = type.type;
    d.instanceLabel = def.id;
    d.inputs = type.inputs;
    d.outputs = type.outputs;
    int n = 0;
    for (auto p : type.params)
    {
        const std::string local = p.id;
        p.id = pin(def.id + "_" + local);
        p.name = def.id + " " + p.name;
        p.pageId = def.id + (n / 6 > 0 ? "_" + std::to_string(n / 6 + 1) : std::string());
        for (const auto& [k, v] : def.params)
            if (k == local)
                p.defaultValue = static_cast<float>(v);
        ids.emplace_back(def.id + "/" + local, p.id);
        d.params.push_back(std::move(p));
        ++n;
    }
    return d;
}

std::shared_ptr<const CompiledProduct> compileImpl(const ProductDef& product, std::string& error, const NodeRegistry& nodes,
                                                   const ProductLimits& limits, const CompiledProduct* base)
{
    const auto v = validate(product, nodes, limits);
    if (!v.ok())
    {
        error = v.text();
        return nullptr;
    }

    auto out = std::make_shared<CompiledProduct>();
    out->def = product;
    out->nodeTypes = nodes;

    // ---- Layout of the registry.
    ProductLayout layout = ModuleRegistry::neutralLayout();
    layout.id = product.id;
    if (product.oscillators > 0)
        layout.voice.oscillators = product.oscillators;
    if (product.filters > 0)
        layout.voice.filters = product.filters;
    if (product.macros > 0)
        layout.macros = product.macros;
    const auto groups = groupsOf(product);
    for (const auto& g : groups)
        for (const auto& n : *g.nodes)
            if (const NodeType* t = nodes.find(n.type); t != nullptr && t->usesSlot)
                layout.voice.oscillators = std::max(layout.voice.oscillators, n.slot);
    if (product.fx.present)
    {
        std::set<std::string> used;
        for (const auto& i : product.fx.instances)
            used.insert(i.type);
        for (const auto* d : referenceRegistry().ofKind(ModuleKind::Effect))
            if (used.count(d->id) == 0)
                layout.omit.push_back(d->id);
    }

    if (product.surface.present)
    {
        const auto& surface = product.surface;
        layout.exposeListed = surface.listed;
        // The surface's sections and pages come first, in declared order. The neutral (and module) pages and sections follow
        // so that a parameter the surface does not list still names a page that exists; the parameter model drops the ones
        // that end up holding nothing.
        const std::vector<SectionDef> neutralSections = layout.sections;
        const std::vector<PageDef> neutralPages = layout.pages;
        if (!surface.sections.empty() || !surface.pages.empty())
        {
            layout.sections.clear();
            for (const auto& s : surface.sections)
                layout.sections.push_back({s.id, s.name});
            if (!surface.sections.empty())
                for (const auto& s : neutralSections)
                    if (std::none_of(layout.sections.begin(), layout.sections.end(),
                                     [&](const SectionDef& d) { return d.id == s.id; }))
                        layout.sections.push_back(s);
            const auto sectionIndex = [&](const std::string& id)
            {
                for (size_t i = 0; i < layout.sections.size(); ++i)
                    if (layout.sections[i].id == id)
                        return static_cast<int>(i);
                return 0;
            };
            if (surface.sections.empty())
                layout.sections = neutralSections;
            layout.pages.clear();
            for (const auto& page : surface.pages)
            {
                PageDef d;
                d.id = page.id;
                d.name = page.name;
                d.layoutTitle = page.name;
                d.layoutDescription = page.name;
                if (!page.section.empty())
                    d.section = sectionIndex(page.section);
                layout.pages.push_back(std::move(d));
                for (const auto& id : page.params)
                {
                    layout.pageOf.emplace_back(id, page.id);
                    layout.surface.push_back(id);
                }
            }
            for (auto d : neutralPages)
            {
                if (std::any_of(layout.pages.begin(), layout.pages.end(), [&](const PageDef& e) { return e.id == d.id; }))
                    continue;
                d.section = d.section >= 0 && static_cast<size_t>(d.section) < neutralSections.size()
                                ? sectionIndex(neutralSections[static_cast<size_t>(d.section)].id)
                                : 0;
                layout.pages.push_back(std::move(d));
            }
            layout.dropEmptyPages = true;
            layout.keepPages = surface.pages.size();
            layout.keepSections = surface.sections.size();
        }
    }

    auto builder = ModuleRegistry::builder(layout);

    // ---- Parameter ids: collisions, pins.
    std::unordered_set<std::string> taken;
    for (const auto& m : builder->modules())
        for (const auto& p : m.params)
            taken.insert(p.id);
    std::map<std::string, std::string> pins(product.idMap.begin(), product.idMap.end());
    std::set<std::string> pinUsed;
    std::string problem;
    const std::function<std::string(const std::string&)> pin = [&](const std::string& generated)
    {
        std::string id = generated;
        const auto it = pins.find(generated);
        if (it != pins.end())
        {
            id = it->second;
            pinUsed.insert(generated);
        }
        if (!taken.insert(id).second && problem.empty())
            problem = "parameter id \"" + id + "\" already exists; rename the instance or pin the id";
        return id;
    };

    // Registers a generated module unless a problem is already recorded; the registry also refuses duplicate parameter
    // ids, and that becomes the product's error instead of an exception.
    const auto registerGenerated = [&](ModuleDescriptor d)
    {
        if (!problem.empty())
            return;
        try
        {
            sdk::registerModule(*builder, std::move(d));
        }
        catch (const std::logic_error& e)
        {
            problem = e.what();
        }
    };

    // ---- Further instances of effects.
    std::map<std::string, int> ordinal;
    for (const auto& inst : product.fx.instances)
    {
        const int k = ++ordinal[inst.type];
        FxInstanceInfo info;
        info.id = inst.id;
        info.type = inst.type;
        info.ordinal = k;
        const ModuleDescriptor* external =
            referenceRegistry().find(inst.type) == nullptr ? nodes.findEffect(inst.type) : nullptr;
        if (k == 1 && external == nullptr)
            info.module = inst.type;
        else if (k == 1)
        {
            info.module = inst.type;
            registerGenerated(firstExternalEffect(*external, pin));
        }
        else
        {
            const auto* proto = external != nullptr ? external : referenceRegistry().find(inst.type);
            ModuleDescriptor d = cloneEffect(*proto, k, inst.id, pin);
            info.module = d.id;
            registerGenerated(std::move(d));
        }
        out->fxInstances.push_back(std::move(info));
    }

    // ---- Graph nodes.
    for (const auto& g : groups)
        for (const auto& n : *g.nodes)
        {
            const NodeType* t = nodes.find(n.type);
            if (t->params.empty())
                continue; // oscillator wrappers use the slot's own parameters
            registerGenerated(instantiateNode(*t, n, pin, out->nodeParamIds));
        }
    if (!problem.empty())
    {
        error = problem;
        return nullptr;
    }
    for (const auto& [gen, to] : pins)
        if (pinUsed.count(gen) == 0)
        {
            error = "idMap: \"" + gen + "\" matches no generated parameter id";
            return nullptr;
        }
    builder->finalize();
    std::shared_ptr<const ModuleRegistry> registry = builder;

    if (base != nullptr)
    {
        std::vector<std::string> ids;
        for (const auto& p : registry->parameters().params())
            ids.push_back(p.id);
        if (ids != base->parameterIds())
        {
            error =
                "the new definition has different parameters from the product that is playing; build a new engine for it";
            return nullptr;
        }
        registry = base->registry;
    }
    out->registry = registry;
    const ParameterModel& model = registry->parameters();

    {
        std::unordered_map<std::uint32_t, const std::string*> hostIds;
        hostIds.reserve(model.params().size());
        for (const auto& p : model.params())
        {
            const auto [it, fresh] = hostIds.emplace(hostIdOf(p.id), &p.id);
            if (!fresh)
            {
                error = "parameter ids \"" + *it->second + "\" and \"" + p.id +
                        "\" have the same host id; rename one (idMap pins a generated id to another)";
                return nullptr;
            }
        }
    }

    {
        std::unordered_map<std::string, std::string> listedAt;
        for (size_t i = 0; i < product.surface.pages.size(); ++i)
            for (size_t k = 0; k < product.surface.pages[i].params.size(); ++k)
            {
                const std::string& id = product.surface.pages[i].params[k];
                const std::string here = "surface.pages[" + std::to_string(i) + "].params[" + std::to_string(k) + "]";
                if (model.indexOf(id) < 0)
                {
                    error = here + ": unknown parameter id '" + id + "'";
                    return nullptr;
                }
                const auto [it, fresh] = listedAt.emplace(id, here);
                if (!fresh)
                {
                    error = here + ": parameter '" + id + "' is already listed at " + it->second;
                    return nullptr;
                }
            }
    }

    for (const auto& [id, value] : product.defaults)
        if (model.indexOf(id) < 0)
        {
            error = "defaults: \"" + id + "\" is not a parameter of the product";
            return nullptr;
        }

    // ---- Voice schedules.
    std::map<std::string, int> chokeIds, rrIds;
    const auto layerPlan = [&](const Group& g, LayerPlan& plan) -> bool
    {
        const auto& defs = *g.nodes;
        std::unordered_map<std::string, size_t> index;
        for (size_t i = 0; i < defs.size(); ++i)
            index[defs[i].id] = i;
        std::vector<std::pair<size_t, size_t>> deps;
        for (const auto& e : *g.edges)
        {
            PortRef a, b;
            splitPort(e.from, a);
            splitPort(e.to, b);
            deps.emplace_back(index[a.node], index[b.node]);
        }
        std::vector<size_t> order;
        topoSort(defs.size(), deps, order);
        std::vector<size_t> position(defs.size());
        for (size_t i = 0; i < order.size(); ++i)
            position[order[i]] = i;
        plan.nodes.resize(defs.size());
        for (size_t i = 0; i < order.size(); ++i)
        {
            const NodeDef& d = defs[order[i]];
            const NodeType* t = nodes.find(d.type);
            NodePlan& np = plan.nodes[i];
            np.id = d.id;
            np.type = out->nodeTypes.find(d.type);
            np.slot = t->usesSlot ? d.slot - 1 : -1;
            if (t->usesSlot)
                np.oscillator = registry->find(d.type);
            for (const auto& p : t->params)
                np.params.push_back(model.indexOf(out->nodeParameterId(d.id, p.id)));
            for (const auto& port : t->inputs)
            {
                if (isAudio(port.type))
                    np.audioIn.emplace_back();
                else if (isRate(port.type))
                    np.rateIn.emplace_back();
                else
                    np.modIn.emplace_back();
            }
            for (const auto& port : t->outputs)
                (isAudio(port.type) ? np.numAudioOut : np.numModOut) += 1;
            if (d.type == "mix.out")
                plan.sink = static_cast<int>(i);
        }
        const auto compact = [](const std::vector<PortDef>& ports, int portIdx)
        {
            int c = 0;
            const int cls = portClass(ports[static_cast<size_t>(portIdx)].type);
            for (int k = 0; k < portIdx; ++k)
                if (portClass(ports[static_cast<size_t>(k)].type) == cls)
                    ++c;
            return c;
        };
        for (const auto& e : *g.edges)
        {
            PortRef a, b;
            splitPort(e.from, a);
            splitPort(e.to, b);
            const NodeType* ft = nodes.find(defs[index[a.node]].type);
            const NodeType* tt = nodes.find(defs[index[b.node]].type);
            const int fp = portIndex(ft->outputs, a.port);
            const int tp = portIndex(tt->inputs, b.port);
            Source s;
            s.node = static_cast<int>(position[index[a.node]]);
            s.port = compact(ft->outputs, fp);
            s.amount = e.amount;
            NodePlan& target = plan.nodes[position[index[b.node]]];
            const PortType toType = tt->inputs[static_cast<size_t>(tp)].type;
            auto& list = isAudio(toType) ? target.audioIn : (isRate(toType) ? target.rateIn : target.modIn);
            list[static_cast<size_t>(compact(tt->inputs, tp))].push_back(s);
        }
        return true;
    };
    if (product.voice.kind == VoiceKind::Graph)
    {
        for (size_t i = 0; i < groups.size(); ++i)
        {
            LayerPlan plan;
            if (product.voice.layers.empty())
            {
                plan.id = "voice";
            }
            else
            {
                const LayerDef& l = product.voice.layers[i];
                plan.id = l.id;
                plan.noteLo = l.noteLo;
                plan.noteHi = l.noteHi;
                plan.velLo = l.velLo;
                plan.velHi = l.velHi;
                plan.voices = l.voices;
                plan.oneShot = l.oneShot;
                if (!l.chokeGroup.empty())
                {
                    const int next = static_cast<int>(chokeIds.size()) + 1;
                    plan.chokeGroup = chokeIds.emplace(l.chokeGroup, next).first->second;
                }
                if (!l.roundRobin.empty())
                {
                    const int next = static_cast<int>(rrIds.size()) + 1;
                    plan.roundRobin = rrIds.emplace(l.roundRobin, next).first->second;
                }
            }
            layerPlan(groups[i], plan);
            out->layers.push_back(std::move(plan));
        }
    }

    // ---- FX lanes.
    out->fx.classic = !product.fx.present;
    out->fx.masterLevel = product.fx.masterLevel;
    if (product.fx.present)
    {
        const auto& lanes = product.fx.lanes;
        std::unordered_map<std::string, size_t> laneIndex;
        for (size_t i = 0; i < lanes.size(); ++i)
            laneIndex[lanes[i].id] = i;
        std::vector<std::pair<size_t, size_t>> deps;
        for (size_t i = 0; i < lanes.size(); ++i)
            for (const auto& s : lanes[i].inputs)
                if (s.source != "voices" && s.source != "aux")
                    deps.emplace_back(laneIndex[s.source], i);
        for (const auto& s : product.fx.sends)
            deps.emplace_back(laneIndex[s.from], laneIndex[s.to]);
        std::vector<size_t> order;
        topoSort(lanes.size(), deps, order);
        std::vector<size_t> position(lanes.size());
        for (size_t i = 0; i < order.size(); ++i)
            position[order[i]] = i;
        std::unordered_map<std::string, std::string> moduleOf;
        for (const auto& i : out->fxInstances)
            moduleOf[i.id] = i.module;
        out->fx.lanes.resize(lanes.size());
        for (size_t i = 0; i < order.size(); ++i)
        {
            const FxLaneDef& l = lanes[order[i]];
            FxLaneSpec& spec = out->fx.lanes[i];
            spec.id = l.id;
            spec.byPosition = l.byPosition;
            spec.toMaster = l.toMaster;
            spec.level = l.level;
            for (const auto& c : l.chain)
                spec.modules.push_back(moduleOf[c]);
            const auto source = [&](const std::string& s, float gain)
            {
                if (s == "voices")
                    return FxSource{FxSourceKind::Voices, -1, gain};
                if (s == "aux")
                    return FxSource{FxSourceKind::Aux, -1, gain};
                return FxSource{FxSourceKind::Lane, static_cast<int>(position[laneIndex[s]]), gain};
            };
            for (const auto& s : l.inputs)
                spec.inputs.push_back(source(s.source, s.gain));
            for (const auto& s : product.fx.sends)
                if (s.to == l.id)
                    spec.inputs.push_back(source(s.from, s.gain));
        }
    }

    return out;
}
} // namespace

std::shared_ptr<const CompiledProduct> compile(const ProductDef& product, std::string& error, const NodeRegistry& nodes,
                                               const ProductLimits& limits)
{
    return compileImpl(product, error, nodes, limits, nullptr);
}

std::shared_ptr<const CompiledProduct> compileReconfiguration(const CompiledProduct& base, const ProductDef& product,
                                                              std::string& error)
{
    return compileImpl(product, error, base.nodeTypes, ProductLimits{}, &base);
}

ProductDef defaultProductDef()
{
    ProductDef p;
    p.id = "crucible.default";
    p.name = "Crucible Default";
    p.description = "The engine's neutral product: the classic voice (3 oscillators, 2 filters, 6 macros) and every "
                    "built-in effect in one position-ordered chain, with the flat parameter ids of every release.";
    p.oscillators = 3;
    p.filters = 2;
    p.macros = 6;
    p.voice.kind = VoiceKind::Classic;
    p.fx.present = true;
    FxLaneDef lane;
    lane.id = "main";
    lane.byPosition = true;
    lane.inputs.push_back({"voices", 1.0f});
    for (const auto* d : referenceRegistry().ofKind(ModuleKind::Effect))
    {
        if (!d->createEffect)
            continue;
        FxInstanceDef inst;
        inst.id = d->id.substr(d->id.find('.') + 1);
        inst.type = d->id;
        p.fx.instances.push_back(inst);
        lane.chain.push_back(inst.id);
    }
    p.fx.lanes.push_back(std::move(lane));
    return p;
}

} // namespace crucible::graph
