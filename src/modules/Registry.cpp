// SPDX-License-Identifier: MIT
#include <crucible/module/Module.h>
#include <crucible/module/ParamBuilder.h>

#include "Modules.h"
#include "fx/FxModuleSupport.h"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace crucible
{

const char* moduleKindName(ModuleKind kind)
{
    switch (kind)
    {
    case ModuleKind::Voice:
        return "voice";
    case ModuleKind::Oscillator:
        return "oscillator";
    case ModuleKind::Filter:
        return "filter";
    case ModuleKind::Effect:
        return "effect";
    case ModuleKind::Modulator:
        return "modulator";
    case ModuleKind::Performance:
        return "performance";
    case ModuleKind::Sequencer:
        return "sequencer";
    }
    return "unknown";
}

const char* moduleStatusName(ModuleStatus status)
{
    return status == ModuleStatus::Legacy ? "legacy" : "current";
}

int EffectInputs::rounded(int local) const
{
    return static_cast<int>(std::lround(value(local)));
}

const char* portTypeName(PortType type)
{
    switch (type)
    {
    case PortType::AudioMono:
        return "audio.mono";
    case PortType::AudioStereo:
        return "audio.stereo";
    case PortType::Modulation:
        return "mod";
    case PortType::Trigger:
        return "trigger";
    case PortType::AudioRate:
        return "audio.rate";
    }
    return "audio.stereo";
}

bool portTypeFromName(const std::string& name, PortType& out)
{
    for (const auto t :
         {PortType::AudioMono, PortType::AudioStereo, PortType::Modulation, PortType::Trigger, PortType::AudioRate})
        if (name == portTypeName(t))
        {
            out = t;
            return true;
        }
    return false;
}

namespace
{
// State of the deprecated process-wide default product (the singleton shim). Engines never read it except to learn
// the layout of the default product (ModuleRegistry::processLayout()).
std::mutex& shapeMutex()
{
    static std::mutex m;
    return m;
}
std::optional<VoiceGraphShape>& shapeOverride()
{
    static std::optional<VoiceGraphShape> o;
    return o;
}
std::optional<ProductLayout>& productOverride()
{
    static std::optional<ProductLayout> o;
    return o;
}
bool& registryBuilt()
{
    static bool b = false;
    return b;
}

ProductLayout clampLayout(ProductLayout layout)
{
    layout.voice.oscillators = std::clamp(layout.voice.oscillators, 1, kMaxOscSlots);
    layout.voice.filters = std::clamp(layout.voice.filters, 1, kMaxFilterSlots);
    layout.macros = std::clamp(layout.macros, 1, kNumMacros);
    return layout;
}

void defaultPorts(ModuleDescriptor& d)
{
    if (!d.inputs.empty() || !d.outputs.empty())
        return;
    switch (d.kind)
    {
    case ModuleKind::Voice:
        d.inputs = {{"note", PortType::Trigger}};
        d.outputs = {{"out", PortType::AudioStereo}};
        break;
    case ModuleKind::Oscillator:
        d.inputs = {{"note", PortType::Trigger}, {"pitch", PortType::Modulation}};
        d.outputs = {{"out", PortType::AudioStereo}};
        break;
    case ModuleKind::Filter:
        d.inputs = {{"in", PortType::AudioMono}, {"cutoff", PortType::Modulation}};
        d.outputs = {{"out", PortType::AudioMono}};
        break;
    case ModuleKind::Effect:
        d.inputs = {{"in", PortType::AudioStereo}};
        d.outputs = {{"out", PortType::AudioStereo}};
        break;
    case ModuleKind::Modulator:
    case ModuleKind::Performance:
        d.outputs = {{"out", PortType::Modulation}};
        break;
    case ModuleKind::Sequencer:
        d.outputs = {{"gate", PortType::Trigger}};
        break;
    }
}
} // namespace

bool ModuleRegistry::configureVoiceGraph(VoiceGraphShape shape)
{
    std::lock_guard<std::mutex> lock(shapeMutex());
    if (registryBuilt())
        return false;
    shape.oscillators = std::clamp(shape.oscillators, 1, kMaxOscSlots);
    shape.filters = std::clamp(shape.filters, 1, kMaxFilterSlots);
    shapeOverride() = shape;
    return true;
}

bool ModuleRegistry::configureProduct(ProductLayout layout)
{
    std::lock_guard<std::mutex> lock(shapeMutex());
    if (registryBuilt())
        return false;
    layout = clampLayout(std::move(layout));
    shapeOverride() = layout.voice;
    productOverride() = std::move(layout);
    return true;
}

std::optional<ProductLayout> ModuleRegistry::configuredProduct()
{
    std::lock_guard<std::mutex> lock(shapeMutex());
    return productOverride();
}

int ModuleRegistry::resolveMacros(int productDefault)
{
    std::lock_guard<std::mutex> lock(shapeMutex());
    registryBuilt() = true;
    return productOverride() ? productOverride()->macros : productDefault;
}

VoiceGraphShape ModuleRegistry::resolveVoiceGraph(VoiceGraphShape productDefault)
{
    std::lock_guard<std::mutex> lock(shapeMutex());
    return shapeOverride().value_or(productDefault);
}

ProductLayout ModuleRegistry::processLayout()
{
    std::lock_guard<std::mutex> lock(shapeMutex());
    registryBuilt() = true;
    auto layout = productOverride().value_or(neutralLayout());
    layout.voice = shapeOverride().value_or(layout.voice);
    return layout;
}

const ModuleRegistry& ModuleRegistry::instance()
{
    static const std::shared_ptr<const ModuleRegistry> registry = build(processLayout());
    return *registry;
}

ModuleRegistry::ModuleRegistry() : productLayout(clampLayout(neutralLayout())) {}

ModuleRegistry::ModuleRegistry(ProductLayout layout) : productLayout(clampLayout(std::move(layout))) {}

ModuleRegistry::~ModuleRegistry() = default;

ProductLayout ModuleRegistry::neutralLayout()
{
    return modules::defaultLayout();
}

std::shared_ptr<ModuleRegistry> ModuleRegistry::builder(ProductLayout layout)
{
    auto r = std::make_shared<ModuleRegistry>(std::move(layout));
    registerBuiltInModules(*r);
    return r;
}

std::shared_ptr<const ModuleRegistry> ModuleRegistry::build(ProductLayout layout)
{
    auto r = builder(std::move(layout));
    r->finalize();
    return r;
}

ModuleRegistry& ModuleRegistry::finalize()
{
    if (model == nullptr)
        model = std::make_unique<ParameterModel>(*this);
    return *this;
}

const ParameterModel& ModuleRegistry::parameters() const
{
    if (model == nullptr)
        throw std::logic_error("ModuleRegistry::parameters() before finalize()");
    return *model;
}

void ModuleRegistry::add(ModuleDescriptor d)
{
    if (model != nullptr)
        throw std::logic_error("module registered after the registry was finalized: " + d.id);
    if (d.id.empty() || find(d.id) != nullptr)
        throw std::logic_error("module id missing or registered twice: " + d.id);
    {
        std::unordered_set<std::string> mine;
        for (const auto& p : d.params)
            if (!mine.insert(p.id).second || paramIds.count(p.id) != 0)
                throw std::logic_error("module " + d.id + ": parameter id registered twice: " + p.id);
        paramIds.insert(mine.begin(), mine.end());
    }
    for (auto& p : d.params)
        p.module = d.id;
    defaultPorts(d);
    list.push_back(std::move(d));
}

const ModuleDescriptor* ModuleRegistry::find(const std::string& id) const
{
    for (const auto& d : list)
        if (d.id == id)
            return &d;
    return nullptr;
}

std::vector<const ModuleDescriptor*> ModuleRegistry::ofKind(ModuleKind kind) const
{
    std::vector<const ModuleDescriptor*> out;
    for (const auto& d : list)
        if (d.kind == kind)
            out.push_back(&d);
    return out;
}

std::vector<const ModuleDescriptor*> ModuleRegistry::oscillatorTypes() const
{
    std::vector<const ModuleDescriptor*> out;
    for (const char* id : modules::builtInOscillatorIds())
        out.push_back(find(id));
    for (const auto* d : ofKind(ModuleKind::Oscillator))
        if (std::find(out.begin(), out.end(), d) == out.end())
            out.push_back(d);
    return out;
}

void ModuleRegistry::migrate(const std::map<std::string, int>& savedVersions, PatchValues& values) const
{
    for (const auto& module : list)
    {
        const auto saved = savedVersions.find(module.id);
        int version = saved == savedVersions.end() ? 1 : saved->second;
        while (version < module.version)
        {
            const ModuleMigration* step = nullptr;
            for (const auto& m : module.migrations)
                if (m.fromVersion == version && m.toVersion > version && (step == nullptr || m.toVersion > step->toVersion))
                    step = &m;
            if (step == nullptr)
            {
                // No step for this version: parameters were only added, which needs no migration. Continue at the
                // next version that has one.
                int next = module.version;
                for (const auto& m : module.migrations)
                    if (m.fromVersion > version && m.fromVersion < next)
                        next = m.fromVersion;
                version = next;
                continue;
            }
            for (const auto& [from, to] : step->renames)
                for (auto& [id, v] : values)
                    if (id == from)
                        id = to;
            for (const auto& [id, fn] : step->remaps)
                for (auto& [pid, v] : values)
                    if (pid == id && fn)
                        v = fn(v);
            if (step->custom)
                step->custom(values);
            version = step->toVersion;
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
namespace params
{
ParamDef floating(std::string id, std::string name, std::string pageId, float min, float max, float def, std::string unit,
                  float skew, Smoothing smoothing)
{
    ParamDef p;
    p.id = std::move(id);
    p.name = std::move(name);
    p.pageId = std::move(pageId);
    p.type = ParamType::Float;
    p.min = min;
    p.max = max;
    p.defaultValue = def;
    p.skew = skew;
    p.unit = std::move(unit);
    p.smoothing = smoothing;
    return p;
}

ParamDef frequency(std::string id, std::string name, std::string pageId, float min, float max, float def, float skew)
{
    return floating(std::move(id), std::move(name), std::move(pageId), min, max, def, "Hz", skew, Smoothing::Log);
}

ParamDef integer(std::string id, std::string name, std::string pageId, int min, int max, int def, std::string unit)
{
    ParamDef p;
    p.id = std::move(id);
    p.name = std::move(name);
    p.pageId = std::move(pageId);
    p.type = ParamType::Int;
    p.min = static_cast<float>(min);
    p.max = static_cast<float>(max);
    p.defaultValue = static_cast<float>(def);
    p.interval = 1.0f;
    p.unit = std::move(unit);
    p.smoothing = Smoothing::None;
    return p;
}

ParamDef choice(std::string id, std::string name, std::string pageId, std::vector<std::string> choices, int def)
{
    ParamDef p;
    p.id = std::move(id);
    p.name = std::move(name);
    p.pageId = std::move(pageId);
    p.type = ParamType::Choice;
    p.min = 0.0f;
    p.max = static_cast<float>(choices.size() - 1);
    p.defaultValue = static_cast<float>(def);
    p.interval = 1.0f;
    p.choices = std::move(choices);
    p.smoothing = Smoothing::None;
    return p;
}

ParamDef toggle(std::string id, std::string name, std::string pageId, bool def)
{
    ParamDef p;
    p.id = std::move(id);
    p.name = std::move(name);
    p.pageId = std::move(pageId);
    p.type = ParamType::Bool;
    p.min = 0.0f;
    p.max = 1.0f;
    p.defaultValue = def ? 1.0f : 0.0f;
    p.interval = 1.0f;
    p.smoothing = Smoothing::None;
    return p;
}

const std::vector<std::string>& tempoSyncNames()
{
    static const std::vector<std::string> names = {"Off",  "4 Bar", "2 Bar", "1 Bar", "1/2",  "1/2T",  "1/4",
                                                   "1/4T", "1/8.",  "1/8",   "1/8T",  "1/16", "1/16T", "1/32"};
    return names;
}

const std::vector<std::string>& lfoShapeNames()
{
    static const std::vector<std::string> names = {"Sine",          "Triangle",  "Saw",           "Square",
                                                   "Sample & Hold", "Saw Down",  "Smooth Random", "Pulse 25%",
                                                   "Stairs",        "Rect Sine", "Exp Decay",     "Custom"};
    return names;
}
} // namespace params

namespace modules::fx
{
int localIndex(const ModuleDescriptor& d, const std::string& id)
{
    for (size_t i = 0; i < d.params.size(); ++i)
        if (d.params[i].id == id)
            return static_cast<int>(i);
    throw std::logic_error("module " + d.id + " has no parameter " + id);
}

ModuleDescriptor effectDescriptor(std::string id, std::string name, std::string category, CpuClass cpu,
                                  std::string description)
{
    ModuleDescriptor d;
    d.id = std::move(id);
    d.name = std::move(name);
    d.kind = ModuleKind::Effect;
    d.category = std::move(category);
    d.cpu = cpu;
    d.description = std::move(description);
    d.sectionHint = "fx";
    return d;
}
} // namespace modules::fx

} // namespace crucible
