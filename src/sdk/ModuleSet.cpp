// SPDX-License-Identifier: MIT
#include <crucible/sdk/Host.h>

#include "SharedLibrary.h"

#include <crucible/Version.h>
#include <crucible/module/ParamBuilder.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <set>
#include <stdexcept>

// Host side of the module ABI: validation of descriptors, the owned copy the host keeps of a module, and the adapters
// that make an ABI module an EffectModule (FX graph) or a graph::Node (voice graph). The adapters build the C structs on
// the stack and call the module's function pointers; nothing here allocates once an instance is prepared.

namespace crucible::sdk
{
namespace
{
// Layout of version 1: the end of each struct's last version 1 field. A struct this small is missing fields the host
// needs. These are fixed numbers of the version 1 layout, not sizeof of the structs as the header has them today: a
// later ABI version appends fields (growing sizeof) and adds its own constant here, and must keep accepting version 1
// modules, whose structs are shorter. Fields added later are read only when the struct's `size` covers them.
constexpr size_t kDescriptorSizeV1 = offsetof(crucible_module_descriptor, holds_voice) + sizeof(void*);
constexpr size_t kParamSizeV1 = offsetof(crucible_param_spec, choices) + sizeof(void*);
constexpr size_t kPortSizeV1 = offsetof(crucible_port_spec, type) + sizeof(std::uint32_t);
static_assert(CRUCIBLE_MODULE_ABI_VERSION != 1 || kDescriptorSizeV1 == sizeof(crucible_module_descriptor),
              "version 1 descriptor layout changed: append-only means the floor stays put");
static_assert(CRUCIBLE_MODULE_ABI_VERSION != 1 || kParamSizeV1 == sizeof(crucible_param_spec),
              "version 1 parameter layout changed: append-only means the floor stays put");
static_assert(kDescriptorSizeV1 <= sizeof(crucible_module_descriptor) && kParamSizeV1 <= sizeof(crucible_param_spec) &&
                  kPortSizeV1 <= sizeof(crucible_port_spec),
              "the version 1 floors cannot exceed the structs");

constexpr int kMaxAudioPorts = CRUCIBLE_MODULE_MAX_PORTS;

bool validSegment(const std::string& s)
{
    if (s.empty() || !(s[0] >= 'a' && s[0] <= 'z'))
        return false;
    return std::all_of(s.begin(), s.end(),
                       [](char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'; });
}

bool reservedVendor(const std::string& s)
{
    static const char* const names[] = {"crucible", "fx", "osc", "voice", "filter", "env", "lfo", "amp", "mix", "classic"};
    return std::any_of(std::begin(names), std::end(names), [&](const char* n) { return s == n; });
}

// "acme.tremolo" -> "acme_tremolo": the prefix of the parameter ids of an effect.
std::string flatId(const std::string& id)
{
    std::string out = id;
    std::replace(out.begin(), out.end(), '.', '_');
    return out;
}

std::string str(const char* s)
{
    return s != nullptr ? std::string(s) : std::string();
}

int fail(std::string& error, int code, std::string message)
{
    error = std::move(message);
    return code;
}

bool versionOk(std::uint32_t v)
{
    return v >= CRUCIBLE_MODULE_ABI_MIN && v <= CRUCIBLE_MODULE_ABI_VERSION;
}

std::string abiText(std::uint32_t v)
{
    return "ABI version " + std::to_string(v) + " (this engine speaks " + std::to_string(CRUCIBLE_MODULE_ABI_MIN) + ".." +
           std::to_string(CRUCIBLE_MODULE_ABI_VERSION) + ")";
}

bool isAudioPort(std::uint32_t t)
{
    return t == CRUCIBLE_PORT_AUDIO_MONO || t == CRUCIBLE_PORT_AUDIO_STEREO;
}

int validatePorts(const std::string& who, const char* what, std::uint32_t count, const crucible_port_spec* const* ports,
                  std::string& error)
{
    if (count > CRUCIBLE_MODULE_MAX_PORTS)
        return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                    who + ": more than " + std::to_string(CRUCIBLE_MODULE_MAX_PORTS) + " " + what);
    if (count != 0 && ports == nullptr)
        return fail(error, CRUCIBLE_MODULE_ERR_INVALID, who + ": " + what + " table is null");
    std::set<std::string> names;
    for (std::uint32_t i = 0; i < count; ++i)
    {
        const crucible_port_spec* p = ports[i];
        if (p == nullptr)
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID, who + ": " + what + "[" + std::to_string(i) + "] is null");
        if (!versionOk(p->abi_version))
            return fail(error, CRUCIBLE_MODULE_ERR_ABI_VERSION,
                        who + ": " + what + "[" + std::to_string(i) + "] uses " + abiText(p->abi_version));
        if (p->size < kPortSizeV1)
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                        who + ": " + what + "[" + std::to_string(i) + "] is smaller than the version 1 layout");
        if (!validSegment(str(p->name)) || !names.insert(str(p->name)).second)
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                        who + ": " + what + "[" + std::to_string(i) +
                            "] needs a unique name of lower case letters, digits and underscores");
        if (p->type < CRUCIBLE_PORT_AUDIO_MONO || p->type > CRUCIBLE_PORT_TRIGGER)
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID, who + ": port \"" + str(p->name) + "\" has an unknown type");
    }
    return CRUCIBLE_MODULE_OK;
}

// Parameter definition of the ABI parameter (the id is the local one; callers rewrite it).
ParamDef toParamDef(const crucible_param_spec& p, bool effect)
{
    ParamDef d;
    d.id = str(p.id);
    d.name = str(p.name);
    d.shortName = str(p.short_name);
    d.unit = str(p.unit);
    d.automatable = (p.flags & CRUCIBLE_PARAM_NOT_AUTOMATABLE) == 0;
    d.modulatable = effect && (p.flags & CRUCIBLE_PARAM_MODULATABLE) != 0;
    switch (p.type)
    {
    case CRUCIBLE_PARAM_INT:
        d.type = ParamType::Int;
        d.min = std::round(p.min);
        d.max = std::round(p.max);
        d.defaultValue = std::clamp(std::round(p.default_value), d.min, d.max);
        d.interval = 1.0f;
        d.smoothing = Smoothing::None;
        break;
    case CRUCIBLE_PARAM_CHOICE:
        d.type = ParamType::Choice;
        for (std::uint32_t i = 0; i < p.num_choices; ++i)
            d.choices.push_back(str(p.choices[i]));
        d.min = 0.0f;
        d.max = static_cast<float>(d.choices.size() - 1);
        d.defaultValue = std::clamp(std::round(p.default_value), d.min, d.max);
        d.interval = 1.0f;
        d.smoothing = Smoothing::None;
        break;
    case CRUCIBLE_PARAM_BOOL:
        d.type = ParamType::Bool;
        d.min = 0.0f;
        d.max = 1.0f;
        d.defaultValue = p.default_value > 0.5f ? 1.0f : 0.0f;
        d.interval = 1.0f;
        d.smoothing = Smoothing::None;
        break;
    default:
        d.type = ParamType::Float;
        d.min = p.min;
        d.max = p.max;
        d.defaultValue = std::clamp(p.default_value, p.min, p.max);
        d.interval = p.interval;
        d.skew = p.skew > 0.0f ? p.skew : 1.0f;
        d.smoothing = (p.flags & CRUCIBLE_PARAM_NO_SMOOTHING) != 0
                          ? Smoothing::None
                          : ((p.flags & CRUCIBLE_PARAM_LOG_SMOOTHING) != 0 ? Smoothing::Log : Smoothing::Linear);
        break;
    }
    return d;
}

// What the host keeps of a registered module. Immutable once built, shared by every adapter and instance made from it.
struct Module
{
    crucible_module_descriptor desc{}; // the host's copy; fields beyond the module's `size` are zero
    crucible_module_host host{};
    std::string id, name, category, description;
    ModuleKind kind = ModuleKind::Effect;
    std::vector<ParamDef> params; // local ids
    std::vector<PortDef> inputs, outputs;
    std::vector<char> audioInMono, audioOutMono; // per audio port (compacted): declared mono
    std::shared_ptr<SharedLibrary> library;      // keeps the code loaded

    bool isEffect() const { return desc.kind == CRUCIBLE_MODULE_KIND_EFFECT; }
};

void hostLog(const crucible_module_host*, int32_t, const char*) {}

// Owner of a module instance.
class Instance
{
public:
    explicit Instance(std::shared_ptr<const Module> module) : m(std::move(module))
    {
        handle = m->desc.create(&m->desc, &m->host);
        if (handle == nullptr)
            throw std::runtime_error("module " + m->id + ": create() returned NULL");
    }
    ~Instance()
    {
        if (handle != nullptr)
            m->desc.destroy(handle);
    }
    Instance(const Instance&) = delete;
    Instance& operator=(const Instance&) = delete;

    const std::shared_ptr<const Module> m;
    void* handle = nullptr;
};

// ---- Effect: the module processes the stereo bus in place.
class SdkEffect final : public EffectModule, public ViewSource
{
public:
    explicit SdkEffect(std::shared_ptr<const Module> module) : inst(std::move(module)), values(inst.m->params.size(), 0.0f)
    {
    }

    void prepare(const ModuleContext& c) override
    {
        const auto& d = inst.m->desc;
        usable = true;
        if (d.prepare == nullptr)
            return;
        crucible_module_prepare info{};
        info.size = sizeof(info);
        info.abi_version = CRUCIBLE_MODULE_ABI_VERSION;
        info.sample_rate = c.sampleRate;
        info.max_block_size = c.maxBlockSize;
        info.quality = static_cast<std::uint32_t>(c.quality);
        info.seed = c.seed;
        usable = d.prepare(inst.handle, &info) != 0;
    }

    void reset() override
    {
        if (inst.m->desc.reset != nullptr)
            inst.m->desc.reset(inst.handle);
    }

    void process(float* left, float* right, int n, const EffectInputs& in) override
    {
        if (!usable)
            return; // the module refused the configuration: the bus passes through
        const int count = static_cast<int>(values.size());
        for (int i = 0; i < count; ++i)
            values[static_cast<size_t>(i)] = in.value(i);
        crucible_module_audio buffer{left, right};
        crucible_module_process data{};
        data.size = sizeof(data);
        data.abi_version = CRUCIBLE_MODULE_ABI_VERSION;
        data.num_samples = n;
        data.audio_in = &buffer;
        data.num_audio_in = 1;
        data.audio_out = &buffer;
        data.num_audio_out = 1;
        data.params = values.data();
        data.num_params = static_cast<std::uint32_t>(count);
        data.tempo = in.bpm();
        data.aux_left = in.auxLeft();
        data.aux_right = in.auxRight();
        inst.m->desc.process(inst.handle, &data);
    }

    const char* view(const char* viewId) override
    {
        return inst.m->desc.get_view != nullptr ? inst.m->desc.get_view(inst.handle, viewId) : nullptr;
    }

private:
    Instance inst;
    std::vector<float> values;
    bool usable = false;
};

// ---- Node: one instance per voice.
class SdkNode final : public graph::Node
{
public:
    explicit SdkNode(std::shared_ptr<const Module> module) : inst(std::move(module)) {}

    void prepare(const graph::NodePrepare& p) override
    {
        const auto& d = inst.m->desc;
        usable = true;
        if (d.prepare == nullptr)
            return;
        crucible_module_prepare info{};
        info.size = sizeof(info);
        info.abi_version = CRUCIBLE_MODULE_ABI_VERSION;
        info.sample_rate = p.sampleRate;
        info.max_block_size = p.maxBlock;
        info.quality = static_cast<std::uint32_t>(p.quality);
        info.seed = p.seed;
        usable = d.prepare(inst.handle, &info) != 0;
    }

    void reset() override
    {
        if (inst.m->desc.reset != nullptr)
            inst.m->desc.reset(inst.handle);
    }

    void noteOn(const graph::NodeNote& n) override
    {
        if (inst.m->desc.note_on == nullptr)
            return;
        crucible_module_note info{};
        info.size = sizeof(info);
        info.abi_version = CRUCIBLE_MODULE_ABI_VERSION;
        info.note = n.note;
        info.velocity = n.velocity;
        info.seed = n.seed;
        info.flags = n.oneShot ? CRUCIBLE_NOTE_ONE_SHOT : 0U;
        inst.m->desc.note_on(inst.handle, &info);
    }

    void noteOff() override
    {
        if (inst.m->desc.note_off != nullptr)
            inst.m->desc.note_off(inst.handle);
    }

    bool holdsVoice() const override
    {
        return inst.m->desc.holds_voice != nullptr && inst.m->desc.holds_voice(inst.handle) != 0;
    }

    int latency() const override
    {
        return inst.m->desc.latency != nullptr ? static_cast<int>(inst.m->desc.latency(inst.handle)) : 0;
    }

    void process(graph::NodeIO& io, int n) override
    {
        Frame f;
        fill(f, io, n);
        if (usable)
            inst.m->desc.process(inst.handle, &f.data);
        else
            silence(io, n);
        finish(f, io, n);
    }

    bool processQuad(graph::Node* const peers[4], graph::NodeIO* const io[4], int n) override
    {
        const auto& d = inst.m->desc;
        if (d.process_quad == nullptr || !usable)
            return false;
        std::array<Frame, 4> frames;
        void* handles[4];
        const crucible_module_process* data[4];
        for (int k = 0; k < 4; ++k)
        {
            const auto* node = static_cast<SdkNode*>(peers[k]);
            if (node->inst.m.get() != inst.m.get() || !node->usable)
                return false;
            handles[k] = node->inst.handle;
            fill(frames[static_cast<size_t>(k)], *io[k], n);
            data[k] = &frames[static_cast<size_t>(k)].data;
        }
        if (d.process_quad(handles, data) == 0)
            return false;
        for (int k = 0; k < 4; ++k)
            finish(frames[static_cast<size_t>(k)], *io[k], n);
        return true;
    }

private:
    struct Frame
    {
        std::array<crucible_module_audio, kMaxAudioPorts> in{}, out{};
        std::array<float, CRUCIBLE_MODULE_MAX_PARAMS> values{};
        crucible_module_process data{};
    };

    void fill(Frame& f, graph::NodeIO& io, int n) const
    {
        const Module& m = *inst.m;
        const int numIn = std::min(io.numIn, kMaxAudioPorts);
        const int numOut = std::min(io.numOut, kMaxAudioPorts);
        for (int i = 0; i < numIn; ++i)
        {
            const bool mono = m.audioInMono[static_cast<size_t>(i)] != 0;
            f.in[static_cast<size_t>(i)] = {io.in[i].left, mono ? nullptr : io.in[i].right};
        }
        for (int i = 0; i < numOut; ++i)
        {
            const bool mono = m.audioOutMono[static_cast<size_t>(i)] != 0;
            f.out[static_cast<size_t>(i)] = {io.out[i].left, mono ? nullptr : io.out[i].right};
        }
        const int count = std::min(static_cast<int>(m.params.size()), CRUCIBLE_MODULE_MAX_PARAMS);
        for (int i = 0; i < count; ++i)
            f.values[static_cast<size_t>(i)] = io.param(i);
        auto& d = f.data;
        d.size = sizeof(d);
        d.abi_version = CRUCIBLE_MODULE_ABI_VERSION;
        d.num_samples = n;
        d.audio_in = f.in.data();
        d.num_audio_in = static_cast<std::uint32_t>(numIn);
        d.audio_out = f.out.data();
        d.num_audio_out = static_cast<std::uint32_t>(numOut);
        d.mod_in = io.modIn;
        d.num_mod_in = static_cast<std::uint32_t>(io.numModIn);
        d.mod_connected = io.modUsed;
        d.mod_out = io.modOut;
        d.num_mod_out = static_cast<std::uint32_t>(io.numModOut);
        d.params = f.values.data();
        d.num_params = static_cast<std::uint32_t>(count);
        d.note = io.note;
        d.velocity = io.velocity;
        d.tempo = 120.0;
    }

    // Mono outputs are mirrored to the right channel so that stereo consumers read a valid buffer.
    void finish(const Frame&, graph::NodeIO& io, int n) const
    {
        const int numOut = std::min(io.numOut, kMaxAudioPorts);
        for (int i = 0; i < numOut; ++i)
            if (inst.m->audioOutMono[static_cast<size_t>(i)] != 0)
                std::memcpy(io.out[i].right, io.out[i].left, static_cast<size_t>(n) * sizeof(float));
    }

    static void silence(graph::NodeIO& io, int n)
    {
        for (int i = 0; i < io.numOut; ++i)
        {
            std::fill(io.out[i].left, io.out[i].left + n, 0.0f);
            std::fill(io.out[i].right, io.out[i].right + n, 0.0f);
        }
    }

    Instance inst;
    bool usable = false;
};

graph::NodeType nodeTypeOf(const std::shared_ptr<const Module>& m)
{
    graph::NodeType t;
    t.type = m->id;
    t.name = m->name;
    t.kind = m->kind;
    t.category = m->category;
    t.description = m->description;
    t.version = static_cast<int>(m->desc.version);
    t.params = m->params;
    t.inputs = m->inputs;
    t.outputs = m->outputs;
    t.create = [m](const graph::NodeCreate&) { return std::make_unique<SdkNode>(m); };
    return t;
}

ModuleDescriptor effectOf(const std::shared_ptr<const Module>& m)
{
    ModuleDescriptor d;
    d.id = m->id;
    d.name = m->name;
    d.kind = ModuleKind::Effect;
    d.category = m->category;
    d.version = static_cast<int>(m->desc.version);
    d.cpu = static_cast<CpuClass>(std::min<std::uint32_t>(m->desc.cpu_class, 2U));
    d.description = m->description;
    d.sectionHint = "fx";
    const std::string prefix = flatId(m->id);
    int n = 0;
    const auto page = [&](int k) { return k / 6 > 0 ? prefix + "_" + std::to_string(k / 6 + 1) : prefix; };
    for (auto p : m->params)
    {
        p.id = prefix + "_" + p.id;
        p.pageId = page(n++);
        d.params.push_back(std::move(p));
    }
    // The host adds the bypass switch; the module never sees it.
    ParamDef on = params::toggle(prefix + "_on", m->name + " On", page(n), true);
    on.automatable = true;
    d.params.push_back(std::move(on));
    d.onParam = prefix + "_on";
    d.createEffect = [m]() -> std::unique_ptr<EffectModule> { return std::make_unique<SdkEffect>(m); };
    return d;
}
} // namespace

// ---------------------------------------------------------------------------------------------------------------
int validateDescriptor(const crucible_module_descriptor* d, std::string& error)
{
    if (d == nullptr)
        return fail(error, CRUCIBLE_MODULE_ERR_ARGUMENT, "the module descriptor is null");
    const std::string id = str(d->id);
    const std::string who = "module \"" + id + "\"";
    // The version first: a newer module may have a layout this host cannot read at all.
    if (!versionOk(d->abi_version))
        return fail(error, CRUCIBLE_MODULE_ERR_ABI_VERSION, who + " was built for " + abiText(d->abi_version));
    if (d->size < kDescriptorSizeV1)
        return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                    who + ": the descriptor (" + std::to_string(d->size) + " bytes) is smaller than the version 1 layout");

    // Identity.
    {
        std::vector<std::string> segments;
        size_t from = 0;
        for (;;)
        {
            const size_t dot = id.find('.', from);
            segments.push_back(id.substr(from, dot == std::string::npos ? std::string::npos : dot - from));
            if (dot == std::string::npos)
                break;
            from = dot + 1;
        }
        if (segments.size() < 2 || !std::all_of(segments.begin(), segments.end(), validSegment))
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                        who + ": the id must be dotted lower case segments, vendor first (\"acme.tremolo\")");
        if (reservedVendor(segments[0]))
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                        who + ": \"" + segments[0] +
                            "\" is reserved for the engine; start the id with your own vendor name");
    }
    if (str(d->name).empty())
        return fail(error, CRUCIBLE_MODULE_ERR_INVALID, who + ": a module needs a name");
    if (d->kind < CRUCIBLE_MODULE_KIND_EFFECT || d->kind > CRUCIBLE_MODULE_KIND_MODULATOR)
        return fail(error, CRUCIBLE_MODULE_ERR_INVALID, who + ": unknown kind " + std::to_string(d->kind));
    if (d->version < 1)
        return fail(error, CRUCIBLE_MODULE_ERR_INVALID, who + ": the version starts at 1");
    if (d->cpu_class > CRUCIBLE_MODULE_CPU_HEAVY)
        return fail(error, CRUCIBLE_MODULE_ERR_INVALID, who + ": unknown cpu class");
    if (d->create == nullptr || d->destroy == nullptr || d->process == nullptr)
        return fail(error, CRUCIBLE_MODULE_ERR_INVALID, who + ": create, destroy and process are required");

    // Parameters.
    if (d->num_params > CRUCIBLE_MODULE_MAX_PARAMS)
        return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                    who + ": more than " + std::to_string(CRUCIBLE_MODULE_MAX_PARAMS) + " parameters");
    if (d->num_params != 0 && d->params == nullptr)
        return fail(error, CRUCIBLE_MODULE_ERR_INVALID, who + ": the parameter table is null");
    std::set<std::string> ids;
    for (std::uint32_t i = 0; i < d->num_params; ++i)
    {
        const crucible_param_spec* p = d->params[i];
        const std::string at = who + ": parameter " + std::to_string(i);
        if (p == nullptr)
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID, at + " is null");
        if (!versionOk(p->abi_version))
            return fail(error, CRUCIBLE_MODULE_ERR_ABI_VERSION, at + " uses " + abiText(p->abi_version));
        if (p->size < kParamSizeV1)
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID, at + " is smaller than the version 1 layout");
        if (!validSegment(str(p->id)) || !ids.insert(str(p->id)).second)
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                        at + " needs a unique id of lower case letters, digits and underscores");
        if (str(p->name).empty())
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID, at + " (\"" + str(p->id) + "\") needs a name");
        if (p->type < CRUCIBLE_PARAM_FLOAT || p->type > CRUCIBLE_PARAM_BOOL)
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID, at + " (\"" + str(p->id) + "\") has an unknown type");
        if (p->type == CRUCIBLE_PARAM_CHOICE)
        {
            if (p->num_choices < 2 || p->num_choices > CRUCIBLE_MODULE_MAX_CHOICES || p->choices == nullptr)
                return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                            at + " (\"" + str(p->id) + "\") needs 2.." + std::to_string(CRUCIBLE_MODULE_MAX_CHOICES) +
                                " choices");
            for (std::uint32_t k = 0; k < p->num_choices; ++k)
                if (p->choices[k] == nullptr)
                    return fail(error, CRUCIBLE_MODULE_ERR_INVALID, at + " (\"" + str(p->id) + "\") has a null choice");
        }
        else if (p->type != CRUCIBLE_PARAM_BOOL)
        {
            if (!std::isfinite(p->min) || !std::isfinite(p->max) || !(p->min < p->max))
                return fail(error, CRUCIBLE_MODULE_ERR_INVALID, at + " (\"" + str(p->id) + "\") needs min < max");
            if (!(p->default_value >= p->min && p->default_value <= p->max))
                return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                            at + " (\"" + str(p->id) + "\") has a default outside its range");
            if ((p->flags & CRUCIBLE_PARAM_LOG_SMOOTHING) != 0 && !(p->min > 0.0f))
                return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                            at + " (\"" + str(p->id) + "\"): log smoothing needs min > 0");
        }
    }

    // Ports.
    if (d->kind == CRUCIBLE_MODULE_KIND_EFFECT)
    {
        if (d->num_inputs != 0 || d->num_outputs != 0)
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID,
                        who + ": an effect has the implicit stereo ports and declares none");
    }
    else
    {
        if (const int r = validatePorts(who, "input", d->num_inputs, d->inputs, error); r != CRUCIBLE_MODULE_OK)
            return r;
        if (const int r = validatePorts(who, "output", d->num_outputs, d->outputs, error); r != CRUCIBLE_MODULE_OK)
            return r;
        if (d->num_outputs == 0)
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID, who + ": a node needs at least one output port");
        const auto countAudio = [](std::uint32_t n, const crucible_port_spec* const* ports)
        {
            int c = 0;
            for (std::uint32_t i = 0; i < n; ++i)
                c += isAudioPort(ports[i]->type) ? 1 : 0;
            return c;
        };
        if (countAudio(d->num_inputs, d->inputs) > kMaxAudioPorts || countAudio(d->num_outputs, d->outputs) > kMaxAudioPorts)
            return fail(error, CRUCIBLE_MODULE_ERR_INVALID, who + ": too many audio ports");
    }
    error.clear();
    return CRUCIBLE_MODULE_OK;
}

void registerModule(ModuleRegistry& registry, ModuleDescriptor descriptor)
{
    registry.add(std::move(descriptor));
}

// ---------------------------------------------------------------------------------------------------------------
struct ModuleSet::Impl
{
    std::vector<std::shared_ptr<const Module>> modules;
    std::string lastError;
    bool allow = false;

    // Builds the host's copy of a validated descriptor.
    static std::shared_ptr<Module> make(const crucible_module_descriptor* d, std::shared_ptr<SharedLibrary> library)
    {
        auto m = std::make_shared<Module>();
        std::memcpy(&m->desc, d, std::min<size_t>(d->size, sizeof(m->desc)));
        m->desc.size = sizeof(m->desc);
        m->desc.abi_version = std::min<std::uint32_t>(d->abi_version, CRUCIBLE_MODULE_ABI_VERSION);
        m->host.size = sizeof(m->host);
        m->host.abi_version = CRUCIBLE_MODULE_ABI_VERSION;
        m->host.name = "Crucible Engine";
        m->host.version = CRUCIBLE_VERSION_STRING;
        m->host.log = &hostLog;
        m->id = str(d->id);
        m->name = str(d->name);
        m->category = str(d->category);
        m->description = str(d->description);
        switch (d->kind)
        {
        case CRUCIBLE_MODULE_KIND_GENERATOR:
            m->kind = ModuleKind::Oscillator;
            break;
        case CRUCIBLE_MODULE_KIND_FILTER:
            m->kind = ModuleKind::Filter;
            break;
        case CRUCIBLE_MODULE_KIND_MODULATOR:
            m->kind = ModuleKind::Modulator;
            break;
        default:
            m->kind = ModuleKind::Effect;
            break;
        }
        const bool effect = d->kind == CRUCIBLE_MODULE_KIND_EFFECT;
        for (std::uint32_t i = 0; i < d->num_params; ++i)
            m->params.push_back(toParamDef(*d->params[i], effect));
        const auto ports =
            [&](std::uint32_t n, const crucible_port_spec* const* in, std::vector<PortDef>& out, std::vector<char>& mono)
        {
            for (std::uint32_t i = 0; i < n; ++i)
            {
                PortDef p;
                p.name = str(in[i]->name);
                p.type = in[i]->type == CRUCIBLE_PORT_AUDIO_MONO
                             ? PortType::AudioMono
                             : (in[i]->type == CRUCIBLE_PORT_AUDIO_STEREO
                                    ? PortType::AudioStereo
                                    : (in[i]->type == CRUCIBLE_PORT_MODULATION ? PortType::Modulation : PortType::Trigger));
                out.push_back(p);
                if (isAudioPort(in[i]->type))
                    mono.push_back(in[i]->type == CRUCIBLE_PORT_AUDIO_MONO ? 1 : 0);
            }
        };
        ports(d->num_inputs, d->inputs, m->inputs, m->audioInMono);
        ports(d->num_outputs, d->outputs, m->outputs, m->audioOutMono);
        m->library = std::move(library);
        return m;
    }

    // Parameter ids of effects are global ("acme_tremolo_depth", plus the host's "acme_tremolo_on"), so a module's
    // parameter may not collide with the bypass switch, with another of its own, or with an effect registered
    // earlier (underscores allow "a.b" + "c_d" and "a.b_c" + "d" to flatten to the same id).
    std::set<std::string> effectParamIds;

    // `claimed` holds the ids of modules checked earlier in the same load, which are not in `modules` yet.
    int check(const crucible_module_descriptor* d, std::set<std::string>& claimed)
    {
        const int r = validateDescriptor(d, lastError);
        if (r != CRUCIBLE_MODULE_OK)
            return r;
        for (const auto& m : modules)
            if (m->id == str(d->id))
                return fail(lastError, CRUCIBLE_MODULE_ERR_DUPLICATE, "module \"" + m->id + "\" is already registered");
        if (d->kind == CRUCIBLE_MODULE_KIND_EFFECT)
        {
            const std::string prefix = flatId(str(d->id));
            std::vector<std::string> ids;
            for (std::uint32_t i = 0; i < d->num_params; ++i)
                ids.push_back(prefix + "_" + str(d->params[i]->id));
            ids.push_back(prefix + "_on");
            std::set<std::string> mine;
            for (const auto& id : ids)
                if (!mine.insert(id).second || effectParamIds.count(id) != 0 || claimed.count(id) != 0)
                    return fail(lastError, CRUCIBLE_MODULE_ERR_DUPLICATE,
                                "module \"" + str(d->id) + "\": the parameter id \"" + id + "\" is already in use");
            claimed.insert(mine.begin(), mine.end());
        }
        return CRUCIBLE_MODULE_OK;
    }
};

ModuleSet::ModuleSet() : impl(std::make_unique<Impl>()) {}
ModuleSet::~ModuleSet() = default;

void ModuleSet::allowLibraries(bool allow)
{
    impl->allow = allow;
}

bool ModuleSet::librariesAllowed() const
{
    return impl->allow;
}

const std::string& ModuleSet::error() const
{
    return impl->lastError;
}

int ModuleSet::count() const
{
    return static_cast<int>(impl->modules.size());
}

std::vector<std::string> ModuleSet::ids() const
{
    std::vector<std::string> out;
    for (const auto& m : impl->modules)
        out.push_back(m->id);
    return out;
}

const std::string& ModuleSet::idOf(int k) const
{
    return impl->modules[static_cast<size_t>(k)]->id;
}

int ModuleSet::add(const crucible_module_descriptor* descriptor)
{
    std::set<std::string> claimed;
    if (const int r = impl->check(descriptor, claimed); r != CRUCIBLE_MODULE_OK)
        return r;
    impl->effectParamIds.insert(claimed.begin(), claimed.end());
    impl->modules.push_back(Impl::make(descriptor, nullptr));
    impl->lastError.clear();
    return CRUCIBLE_MODULE_OK;
}

int ModuleSet::loadLibrary(const std::string& path)
{
    if (!impl->allow)
        return fail(impl->lastError, CRUCIBLE_MODULE_ERR_DISABLED,
                    "loading shared libraries is not enabled for this registry (the host must allow it)");
    auto lib = SharedLibrary::open(path, impl->lastError);
    if (!lib)
        return CRUCIBLE_MODULE_ERR_LOAD;
    auto* entry = reinterpret_cast<crucible_get_modules_fn>(lib->symbol("crucible_get_modules"));
    if (entry == nullptr)
        return fail(impl->lastError, CRUCIBLE_MODULE_ERR_SYMBOL, path + ": no crucible_get_modules entry point");
    const crucible_module_library* list = entry();
    if (list == nullptr)
        return fail(impl->lastError, CRUCIBLE_MODULE_ERR_SYMBOL, path + ": crucible_get_modules returned no library");
    if (!versionOk(list->abi_version))
        return fail(impl->lastError, CRUCIBLE_MODULE_ERR_ABI_VERSION,
                    path + ": the library was built for " + abiText(list->abi_version));
    if (list->size < sizeof(crucible_module_library) || (list->num_modules != 0 && list->modules == nullptr))
        return fail(impl->lastError, CRUCIBLE_MODULE_ERR_INVALID, path + ": malformed module library");
    // All or nothing: check every module (and the ids among themselves) before keeping any.
    std::set<std::string> seen;
    std::set<std::string> claimed;
    for (std::uint32_t i = 0; i < list->num_modules; ++i)
    {
        const crucible_module_descriptor* d = list->modules[i];
        if (const int r = impl->check(d, claimed); r != CRUCIBLE_MODULE_OK)
        {
            impl->lastError = path + ": " + impl->lastError;
            return r;
        }
        if (!seen.insert(str(d->id)).second)
            return fail(impl->lastError, CRUCIBLE_MODULE_ERR_DUPLICATE,
                        path + ": module \"" + str(d->id) + "\" is listed twice");
    }
    impl->effectParamIds.insert(claimed.begin(), claimed.end());
    for (std::uint32_t i = 0; i < list->num_modules; ++i)
        impl->modules.push_back(Impl::make(list->modules[i], lib));
    impl->lastError.clear();
    return CRUCIBLE_MODULE_OK;
}

void ModuleSet::addTo(graph::NodeRegistry& nodes) const
{
    for (const auto& m : impl->modules)
    {
        if (m->isEffect())
            nodes.addEffect(effectOf(m));
        else
            nodes.add(nodeTypeOf(m));
    }
}

} // namespace crucible::sdk
