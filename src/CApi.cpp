// SPDX-License-Identifier: MIT
#include <crucible/CApiInterop.h>
#include <crucible/crucible.h>

#include <crucible/Engine.h>
#include <crucible/Introspection.h>
#include <crucible/Patch.h>
#include <crucible/Version.h>
#include <crucible/crucible_module.h>
#include <crucible/graph/Compiler.h>

#include "sdk/CModuleRegistry.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <exception>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

struct CrucibleEngine
{
    explicit CrucibleEngine(const crucible::EngineConfig& c) : engine(c) {}
    crucible::Engine engine;
    std::vector<crucible::MidiInput> midi;
    std::set<std::string> routeTargets; // interned target ids handed out by crucible_route_get (node-stable)
};

struct CrucibleKeyBus
{
    std::shared_ptr<crucible::KeyBus> bus;
};

namespace
{
void copyError(const std::string& e, char* buffer, size_t size)
{
    if (buffer == nullptr || size == 0)
        return;
    const size_t n = std::min(size - 1, e.size());
    std::memcpy(buffer, e.data(), n);
    buffer[n] = '\0';
}

// No exception crosses the C boundary: a throwing call returns its documented failure value, and the text goes to the
// caller's error buffer when the function has one.
template <class R, class F> R guarded(R fallback, F&& fn) noexcept
{
    try
    {
        return fn();
    }
    catch (...)
    {
        return fallback;
    }
}

template <class R, class F> R guardedText(R fallback, char* buffer, size_t size, F&& fn) noexcept
{
    try
    {
        return fn();
    }
    catch (const std::exception& e)
    {
        copyError(e.what(), buffer, size);
        return fallback;
    }
    catch (...)
    {
        copyError("unknown error", buffer, size);
        return fallback;
    }
}

template <class F> void guardedVoid(F&& fn) noexcept
{
    try
    {
        fn();
    }
    catch (...)
    {
    }
}

// snprintf-style: writes at most cap bytes (NUL terminated) and returns the length of the full text.
int copyOut(const std::string& text, char* buffer, size_t cap)
{
    copyError(text, buffer, cap);
    return static_cast<int>(text.size());
}

// A field of a struct that starts with struct_size is present when the caller's struct reaches past it.
#define CRUCIBLE_HAS(ptr, field)                                                                                            \
    ((ptr)->struct_size >= offsetof(std::remove_pointer_t<decltype(ptr)>, field) + sizeof((ptr)->field))

// Copies the part of a fully filled struct that the caller's struct_size covers; struct_size itself stays the caller's.
// A struct_size of 0 (or one that does not reach past struct_size) writes nothing and returns false.
template <typename T> bool copyStruct(T* out, const T& full)
{
    const size_t size = out->struct_size;
    if (size <= sizeof(size_t))
        return false;
    std::memcpy(out, &full, std::min(size, sizeof(T)));
    out->struct_size = size;
    return true;
}

void applyConfig(const CrucibleConfig* config, crucible::EngineConfig& c)
{
    if (config == nullptr)
        return;
    if (CRUCIBLE_HAS(config, max_voices) && config->max_voices > 0)
        c.maxVoices = config->max_voices;
    if (CRUCIBLE_HAS(config, max_block_size) && config->max_block_size > 0)
        c.maxBlockSize = config->max_block_size;
    if (CRUCIBLE_HAS(config, quality))
        c.quality = static_cast<crucible::Quality>(std::clamp(config->quality, 0, 2));
    if (CRUCIBLE_HAS(config, seed))
        c.seed = config->seed;
    if (CRUCIBLE_HAS(config, preload_wavetables))
        c.preloadWavetables = config->preload_wavetables != 0;
    if (CRUCIBLE_HAS(config, oversampling_filter))
        c.oversampling = config->oversampling_filter == 1 ? crucible::OversamplingFilter::Polyphase
                                                          : crucible::OversamplingFilter::HalfBand;
}

// Converts without allocating once the scratch vector has grown to the largest block (set up in prepare()).
const crucible::MidiInput* convert(CrucibleEngine* e, const CrucibleMidiEvent* events, int count)
{
    if (events == nullptr || count <= 0)
        return nullptr;
    const auto n = std::min(static_cast<size_t>(count), e->midi.size());
    for (size_t i = 0; i < n; ++i)
        e->midi[i] = {events[i].position, events[i].status, events[i].data1, events[i].data2};
    return e->midi.data();
}
} // namespace

namespace
{
// Host tempo and position as the sequencer takes them: an infinite or NaN tempo is unknown, a tempo above kMaxHostBpm is
// clamped, and a position that is not finite or lies beyond kMaxHostPpq is unknown.
constexpr double kMaxHostBpm = 1000.0;
constexpr double kMaxHostPpq = 1.0e12;

void setHostBpm(crucible::SeqHostTime& host, double bpm)
{
    host.hasBpm = std::isfinite(bpm) && bpm > 0.0;
    host.bpm = host.hasBpm ? std::min(bpm, kMaxHostBpm) : 0.0;
}

void setHostPpq(crucible::SeqHostTime& host, double ppq)
{
    host.hasPpq = std::isfinite(ppq) && ppq >= 0.0 && ppq <= kMaxHostPpq;
    host.ppq = host.hasPpq ? ppq : 0.0;
}

void setHostTempo(crucible::SeqHostTime& host, double bpm, double ppq)
{
    setHostBpm(host, bpm);
    setHostPpq(host, ppq);
}
} // namespace

extern "C"
{

    int crucible_api_version(void)
    {
        return CRUCIBLE_API_VERSION;
    }

    const char* crucible_version(void)
    {
        return CRUCIBLE_VERSION_STRING;
    }

    void crucible_config_init(CrucibleConfig* config)
    {
        if (config == nullptr)
            return;
        const crucible::EngineConfig defaults;
        std::memset(config, 0, sizeof(*config));
        config->struct_size = sizeof(*config);
        config->max_voices = defaults.maxVoices;
        config->max_block_size = defaults.maxBlockSize;
        config->quality = static_cast<int>(defaults.quality);
    }

    CrucibleEngine* crucible_create(const CrucibleConfig* config)
    {
        try
        {
            crucible::EngineConfig c;
            applyConfig(config, c);
            auto* e = new CrucibleEngine(c);
            e->midi.resize(CRUCIBLE_MAX_EVENTS_PER_BLOCK);
            return e;
        }
        catch (...)
        {
            return nullptr;
        }
    }

    CrucibleEngine* crucible_create_from_product(const CrucibleConfig* config, const char* product_json,
                                                 const crucible_module_registry* modules, char* error_buffer,
                                                 size_t error_size)
    {
        try
        {
            if (product_json == nullptr)
            {
                copyError("no product definition", error_buffer, error_size);
                return nullptr;
            }
            crucible::EngineConfig c;
            applyConfig(config, c);
            crucible::graph::ProductDef def;
            std::string error;
            if (!crucible::graph::productFromText(product_json, def, error))
            {
                copyError(error, error_buffer, error_size);
                return nullptr;
            }
            crucible::graph::NodeRegistry nodes = crucible::graph::NodeRegistry::builtIn();
            if (modules != nullptr)
                modules->set.addTo(nodes);
            c.compiled = crucible::graph::compile(def, error, nodes);
            if (!c.compiled)
            {
                copyError(error, error_buffer, error_size);
                return nullptr;
            }
            auto* e = new CrucibleEngine(c);
            e->midi.resize(CRUCIBLE_MAX_EVENTS_PER_BLOCK);
            return e;
        }
        catch (const std::exception& ex)
        {
            copyError(ex.what(), error_buffer, error_size);
            return nullptr;
        }
        catch (...)
        {
            copyError("unknown error", error_buffer, error_size);
            return nullptr;
        }
    }

    void crucible_destroy(CrucibleEngine* engine)
    {
        delete engine;
    }

    void crucible_prepare(CrucibleEngine* engine, double sampleRate, int maxBlockSize)
    {
        guardedVoid(
            [&]
            {
                if (engine != nullptr)
                    engine->engine.prepare(sampleRate, maxBlockSize);
            });
    }

    int crucible_param_count(const CrucibleEngine* engine)
    {
        return engine != nullptr ? engine->engine.model().size() : 0;
    }

    int crucible_param_index(const CrucibleEngine* engine, const char* id)
    {
        return engine != nullptr && id != nullptr ? engine->engine.model().indexOf(id) : -1;
    }

    int crucible_param_meta(const CrucibleEngine* engine, int index, CrucibleParamMeta* out)
    {
        if (engine == nullptr || out == nullptr || index < 0 || index >= engine->engine.model().size())
            return 0;
        const auto& p = engine->engine.model().params()[static_cast<size_t>(index)];
        CrucibleParamMeta full{};
        full.struct_size = sizeof(full);
        CrucibleParamMeta* const o = &full;
        o->id = p.id.c_str();
        o->name = p.name.c_str();
        o->short_name = p.shortName.c_str();
        o->unit = p.unit.c_str();
        o->module = p.module.c_str();
        o->group = p.group.c_str();
        o->page_id = p.pageId.c_str();
        o->type = static_cast<int>(p.type);
        o->min = p.min;
        o->max = p.max;
        o->default_value = p.defaultValue;
        o->interval = p.interval;
        o->skew = p.skew;
        o->steps = p.steps;
        o->num_choices = static_cast<int>(p.choices.size());
        o->page = p.page;
        o->smoothing = static_cast<int>(p.smoothing);
        o->polarity = static_cast<int>(p.polarity);
        o->ui_hint = static_cast<int>(p.uiHint);
        o->text_format = static_cast<int>(p.textFormat);
        o->modulatable = p.modulatable ? 1 : 0;
        o->voice_mod = p.voiceMod ? 1 : 0;
        o->automatable = p.automatable ? 1 : 0;
        o->scene = p.scene ? 1 : 0;
        o->routable = p.routable ? 1 : 0;
        o->since = p.since;
        o->host_id = crucible::hostIdOf(p.id);
        o->exposed = p.exposed ? 1 : 0;
        return copyStruct(out, full) ? 1 : 0;
    }

    int crucible_surface_count(const CrucibleEngine* engine)
    {
        return engine != nullptr ? static_cast<int>(engine->engine.model().surface().size()) : 0;
    }

    int crucible_surface_param(const CrucibleEngine* engine, int k)
    {
        if (engine == nullptr || k < 0)
            return -1;
        const auto& s = engine->engine.model().surface();
        return static_cast<size_t>(k) < s.size() ? s[static_cast<size_t>(k)] : -1;
    }

    const char* crucible_param_choice(const CrucibleEngine* engine, int index, int choice)
    {
        if (engine == nullptr || index < 0 || index >= engine->engine.model().size())
            return nullptr;
        const auto& p = engine->engine.model().params()[static_cast<size_t>(index)];
        if (choice < 0 || choice >= static_cast<int>(p.choices.size()))
            return nullptr;
        return p.choices[static_cast<size_t>(choice)].c_str();
    }

    int crucible_value_to_text(const CrucibleEngine* engine, int index, float value, char* buffer, size_t buffer_size)
    {
        return guarded<int>(-1,
                            [&]() -> int
                            {
                                if (engine == nullptr || index < 0 || index >= engine->engine.model().size())
                                    return -1;
                                const std::string text = engine->engine.model().valueToText(index, value);
                                copyError(text, buffer, buffer_size);
                                return static_cast<int>(text.size());
                            });
    }

    int crucible_text_to_value(const CrucibleEngine* engine, int index, const char* text, float* value)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                float v = 0.0f;
                                if (engine == nullptr || text == nullptr ||
                                    !engine->engine.model().textToValue(index, text, v))
                                    return 0;
                                if (value != nullptr)
                                    *value = v;
                                return 1;
                            });
    }

    int crucible_choice_available(const CrucibleEngine* engine, int param_index, int choice)
    {
        return engine != nullptr && engine->engine.choiceAvailable(param_index, choice) ? 1 : 0;
    }

    int crucible_module_count(const CrucibleEngine* engine)
    {
        return engine != nullptr ? static_cast<int>(engine->engine.registry().modules().size()) : 0;
    }

    namespace
    {
    const crucible::ModuleDescriptor* moduleAt(const CrucibleEngine* engine, int module)
    {
        if (engine == nullptr || module < 0 || module >= static_cast<int>(engine->engine.registry().modules().size()))
            return nullptr;
        return &engine->engine.registry().modules()[static_cast<size_t>(module)];
    }
    } // namespace

    int crucible_module_meta(const CrucibleEngine* engine, int module, CrucibleModuleMeta* out)
    {
        const auto* m = moduleAt(engine, module);
        if (m == nullptr || out == nullptr)
            return 0;
        CrucibleModuleMeta full{};
        full.struct_size = sizeof(full);
        CrucibleModuleMeta* const o = &full;
        o->id = m->id.c_str();
        o->name = m->name.c_str();
        o->kind = crucible::moduleKindName(m->kind);
        o->category = m->category.c_str();
        o->description = m->description.c_str();
        o->version = m->version;
        o->cpu = static_cast<int>(m->cpu);
        o->num_params = static_cast<int>(m->params.size());
        o->num_inputs = static_cast<int>(m->inputs.size());
        o->num_outputs = static_cast<int>(m->outputs.size());
        o->legacy = m->status == crucible::ModuleStatus::Legacy ? 1 : 0;
        o->superseded_by = m->supersededBy.c_str();
        return copyStruct(out, full) ? 1 : 0;
    }

    int crucible_module_param(const CrucibleEngine* engine, int module, int k)
    {
        const auto* m = moduleAt(engine, module);
        if (m == nullptr || k < 0 || k >= static_cast<int>(m->params.size()))
            return -1;
        return engine->engine.model().indexOf(m->params[static_cast<size_t>(k)].id);
    }

    int crucible_module_input(const CrucibleEngine* engine, int module, int k, CruciblePortMeta* out)
    {
        const auto* m = moduleAt(engine, module);
        if (m == nullptr || out == nullptr || k < 0 || k >= static_cast<int>(m->inputs.size()))
            return 0;
        out->name = m->inputs[static_cast<size_t>(k)].name.c_str();
        out->type = crucible::portTypeName(m->inputs[static_cast<size_t>(k)].type);
        return 1;
    }

    int crucible_module_output(const CrucibleEngine* engine, int module, int k, CruciblePortMeta* out)
    {
        const auto* m = moduleAt(engine, module);
        if (m == nullptr || out == nullptr || k < 0 || k >= static_cast<int>(m->outputs.size()))
            return 0;
        out->name = m->outputs[static_cast<size_t>(k)].name.c_str();
        out->type = crucible::portTypeName(m->outputs[static_cast<size_t>(k)].type);
        return 1;
    }

    int crucible_describe_json(const CrucibleEngine* engine, char* buffer, size_t capacity)
    {
        if (engine == nullptr)
            return -1;
        try
        {
            return copyOut(engine->engine.describeJSON(), buffer, capacity);
        }
        catch (...)
        {
            return -1;
        }
    }

    int crucible_set_param(CrucibleEngine* engine, int index, float value)
    {
        return guarded<int>(0, [&]() -> int { return engine != nullptr && engine->engine.setParam(index, value) ? 1 : 0; });
    }

    int crucible_set_param_id(CrucibleEngine* engine, const char* id, float value)
    {
        return guarded<int>(
            0, [&]() -> int
            { return engine != nullptr && id != nullptr && engine->engine.setParam(std::string(id), value) ? 1 : 0; });
    }

    int crucible_set_param_text(CrucibleEngine* engine, const char* id, const char* text)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                return engine != nullptr && id != nullptr && text != nullptr &&
                                               engine->engine.setParamText(std::string(id), std::string(text))
                                           ? 1
                                           : 0;
                            });
    }

    int crucible_param_index_by_host_id(const CrucibleEngine* engine, uint32_t host_id)
    {
        return engine != nullptr ? engine->engine.paramIndexByHostId(host_id) : -1;
    }

    int crucible_set_param_normalised(CrucibleEngine* engine, int index, float value)
    {
        return guarded<int>(0, [&]() -> int
                            { return engine != nullptr && engine->engine.setParamNormalised(index, value) ? 1 : 0; });
    }

    float crucible_get_param(const CrucibleEngine* engine, int index)
    {
        return engine != nullptr ? engine->engine.getParam(index) : 0.0f;
    }

    float crucible_modulated_value(const CrucibleEngine* engine, int index)
    {
        return engine != nullptr ? const_cast<CrucibleEngine*>(engine)->engine.coreEngine().engine().modulatedValue(index)
                                 : 0.0f;
    }

    int crucible_note_on(CrucibleEngine* engine, int note, float velocity)
    {
        return engine != nullptr && engine->engine.noteOn(note, velocity) ? 1 : 0;
    }

    int crucible_note_off(CrucibleEngine* engine, int note)
    {
        return engine != nullptr && engine->engine.noteOff(note) ? 1 : 0;
    }

    int crucible_pitch_bend(CrucibleEngine* engine, float normalised)
    {
        return engine != nullptr && engine->engine.pitchBend(normalised) ? 1 : 0;
    }

    int crucible_mod_wheel(CrucibleEngine* engine, float value)
    {
        return engine != nullptr && engine->engine.modWheel(value) ? 1 : 0;
    }

    int crucible_sustain(CrucibleEngine* engine, int down)
    {
        return engine != nullptr && engine->engine.sustain(down != 0) ? 1 : 0;
    }

    int crucible_all_notes_off(CrucibleEngine* engine)
    {
        return engine != nullptr && engine->engine.allNotesOff() ? 1 : 0;
    }

    int crucible_kill_all_voices(CrucibleEngine* engine)
    {
        return engine != nullptr && engine->engine.killAllVoices() ? 1 : 0;
    }

    int crucible_midi(CrucibleEngine* engine, uint8_t status, uint8_t data1, uint8_t data2)
    {
        return engine != nullptr && engine->engine.midi(status, data1, data2) ? 1 : 0;
    }

    void crucible_map_cc(CrucibleEngine* engine, int cc, int paramIndex)
    {
        if (engine != nullptr)
            engine->engine.mapCC(cc, paramIndex);
    }

    void crucible_process(CrucibleEngine* engine, float* left, float* right, int numSamples, const CrucibleMidiEvent* events,
                          int numEvents)
    {
        if (engine == nullptr || left == nullptr || right == nullptr)
            return;
        const auto* midi = convert(engine, events, numEvents);
        engine->engine.process(left, right, numSamples, midi,
                               midi != nullptr ? std::min(numEvents, CRUCIBLE_MAX_EVENTS_PER_BLOCK) : 0);
    }

    void crucible_process_host(CrucibleEngine* engine, float* left, float* right, int numSamples,
                               const CrucibleMidiEvent* events, int numEvents, double bpm, double ppq, int playing)
    {
        if (engine == nullptr || left == nullptr || right == nullptr)
            return;
        crucible::SeqHostTime host;
        setHostTempo(host, bpm, ppq);
        host.playing = playing != 0;
        const auto* midi = convert(engine, events, numEvents);
        engine->engine.process(left, right, numSamples, midi,
                               midi != nullptr ? std::min(numEvents, CRUCIBLE_MAX_EVENTS_PER_BLOCK) : 0, &host);
    }

    void crucible_process_events(CrucibleEngine* engine, float* left, float* right, int numSamples,
                                 const CrucibleMidiEvent* events, int numEvents, const CrucibleParamEvent* params,
                                 int numParams, const CrucibleTransport* transport)
    {
        crucible_process_events_aux(engine, left, right, nullptr, nullptr, numSamples, events, numEvents, params,
                                    numParams, transport);
    }

    void crucible_process_events_aux(CrucibleEngine* engine, float* left, float* right, const float* auxLeft,
                                     const float* auxRight, int numSamples, const CrucibleMidiEvent* events,
                                     int numEvents, const CrucibleParamEvent* params, int numParams,
                                     const CrucibleTransport* transport)
    {
        if (engine == nullptr || left == nullptr || right == nullptr)
            return;
        if (params != nullptr)
            for (int i = 0; i < numParams; ++i)
                engine->engine.setParamRt(params[i].index, params[i].value);
        const auto* midi = convert(engine, events, numEvents);
        const int numMidi = midi != nullptr ? std::min(numEvents, CRUCIBLE_MAX_EVENTS_PER_BLOCK) : 0;
        if (transport == nullptr || transport->struct_size == 0) // size 0: no transport, not an error
        {
            engine->engine.process(left, right, numSamples, midi, numMidi, nullptr, auxLeft, auxRight);
            return;
        }
        crucible::SeqHostTime host;
        if (CRUCIBLE_HAS(transport, bpm))
            setHostBpm(host, transport->bpm);
        if (CRUCIBLE_HAS(transport, ppq))
            setHostPpq(host, transport->ppq);
        if (CRUCIBLE_HAS(transport, playing))
            host.playing = transport->playing != 0;
        engine->engine.process(left, right, numSamples, midi, numMidi, &host, auxLeft, auxRight);
    }

    void crucible_service(CrucibleEngine* engine)
    {
        guardedVoid(
            [&]
            {
                if (engine != nullptr)
                    engine->engine.service();
            });
    }

    int crucible_service_pending(const CrucibleEngine* engine)
    {
        return engine != nullptr && engine->engine.servicePending() ? 1 : 0;
    }

    int crucible_param_changes(CrucibleEngine* engine, int* indices, int* sources, int cap)
    {
        return engine != nullptr ? engine->engine.paramChanges(indices, sources, cap) : 0;
    }

    int crucible_latency_samples(const CrucibleEngine* engine)
    {
        return engine != nullptr ? engine->engine.latencySamples() : 0;
    }

    double crucible_tail_seconds(const CrucibleEngine* engine)
    {
        return engine != nullptr ? engine->engine.tailSeconds() : 0.0;
    }

    void crucible_reset(CrucibleEngine* engine)
    {
        guardedVoid(
            [&]
            {
                if (engine != nullptr)
                    engine->engine.reset();
            });
    }

    void crucible_set_render_offline(CrucibleEngine* engine, int offline)
    {
        if (engine != nullptr)
            engine->engine.setRenderOffline(offline != 0);
    }

    int crucible_render_offline(const CrucibleEngine* engine)
    {
        return engine != nullptr && engine->engine.isRenderOffline() ? 1 : 0;
    }

    void crucible_process_aux(CrucibleEngine* engine, float* left, float* right, const float* auxLeft, const float* auxRight,
                              int numSamples, const CrucibleMidiEvent* events, int numEvents, double bpm, double ppq,
                              int playing)
    {
        if (engine == nullptr || left == nullptr || right == nullptr)
            return;
        crucible::SeqHostTime host;
        setHostTempo(host, bpm, ppq);
        host.playing = playing != 0;
        const auto* midi = convert(engine, events, numEvents);
        engine->engine.process(left, right, numSamples, midi,
                               midi != nullptr ? std::min(numEvents, CRUCIBLE_MAX_EVENTS_PER_BLOCK) : 0, &host, auxLeft,
                               auxRight);
    }

    CrucibleMeter crucible_meter(const CrucibleEngine* engine)
    {
        CrucibleMeter m{-120.0f, -120.0f, 0};
        if (engine != nullptr)
        {
            const auto s = engine->engine.meter();
            m.peak_left_db = s.peakLeftDb;
            m.peak_right_db = s.peakRightDb;
            m.clip = s.clip ? 1 : 0;
        }
        return m;
    }

    int crucible_view_count(const CrucibleEngine* engine)
    {
        return engine != nullptr ? engine->engine.views().count() : 0;
    }

    int crucible_view_info(const CrucibleEngine* engine, int view, CrucibleViewInfo* out)
    {
        if (engine == nullptr || out == nullptr || view < 0 || view >= engine->engine.views().count())
            return 0;
        const auto& i = engine->engine.views().info(view);
        const CrucibleViewInfo full{sizeof(CrucibleViewInfo), i.instance.c_str(), i.id.c_str(), i.description.c_str(),
                                    static_cast<int>(i.kind), i.maxElements,      i.stride,     i.live ? 1 : 0};
        return copyStruct(out, full) ? 1 : 0;
    }

    int crucible_view_size(const CrucibleEngine* engine, int view)
    {
        if (engine == nullptr || view < 0 || view >= engine->engine.views().count())
            return -1;
        return engine->engine.views().info(view).maxElements;
    }

    int crucible_view_find(CrucibleEngine* engine, const char* instance, const char* id)
    {
        if (engine == nullptr || instance == nullptr || id == nullptr)
            return -1;
        try
        {
            return engine->engine.views().find(instance, id);
        }
        catch (...)
        {
            return -1;
        }
    }

    int crucible_view_subscribe(CrucibleEngine* engine, int view, float maxHz)
    {
        if (engine == nullptr)
            return 0;
        try
        {
            return engine->engine.views().subscribe(view, maxHz);
        }
        catch (...)
        {
            return 0;
        }
    }

    void crucible_view_unsubscribe(CrucibleEngine* engine, int subscription)
    {
        if (engine != nullptr)
            engine->engine.views().unsubscribe(subscription);
    }

    int crucible_view_read(const CrucibleEngine* engine, int view, float* buffer, int capacity, uint64_t* sequence)
    {
        if (sequence != nullptr)
            *sequence = 0;
        if (engine == nullptr || (buffer == nullptr && capacity > 0))
            return -1;
        return engine->engine.views().read(view, buffer, capacity, sequence);
    }

    int crucible_view_read_mask(const CrucibleEngine* engine, int view, uint32_t* words, int capacity, uint64_t* sequence)
    {
        static_assert(sizeof(float) == sizeof(uint32_t), "view elements are 32 bits");
        return crucible_view_read(engine, view, reinterpret_cast<float*>(words), capacity, sequence);
    }

    CrucibleLoudness crucible_loudness(const CrucibleEngine* engine)
    {
        CrucibleLoudness out{-120.0f, -120.0f, -120.0f, -120.0f, 0.0f};
        if (engine != nullptr)
        {
            const auto l = engine->engine.loudness();
            out = {l.momentaryLufs, l.shortTermLufs, l.truePeakDb, l.maxTruePeakDb, l.correlation};
        }
        return out;
    }

    void crucible_reset_clip(CrucibleEngine* engine)
    {
        if (engine != nullptr)
        {
            engine->engine.resetClip();
            engine->engine.analyzer().resetMax();
        }
    }

    void crucible_set_analyzer_enabled(CrucibleEngine* engine, int enabled)
    {
        if (engine != nullptr)
            engine->engine.analyzer().setEnabled(enabled != 0);
    }

    int crucible_scope(const CrucibleEngine* engine, float* left, float* right, int count)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || left == nullptr || right == nullptr || count <= 0)
                                    return 0;
                                std::vector<float> l, r;
                                const_cast<CrucibleEngine*>(engine)->engine.analyzer().scope(l, r, count);
                                std::copy(l.begin(), l.end(), left);
                                std::copy(r.begin(), r.end(), right);
                                return static_cast<int>(l.size());
                            });
    }

    int crucible_spectrum(const CrucibleEngine* engine, int fftSize, float* bins, int capacity)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || bins == nullptr)
                                    return 0;
                                const auto s = const_cast<CrucibleEngine*>(engine)->engine.analyzer().spectrum(fftSize);
                                if (static_cast<int>(s.size()) > capacity)
                                    return 0;
                                std::copy(s.begin(), s.end(), bins);
                                return static_cast<int>(s.size());
                            });
    }

    int crucible_load_patch(CrucibleEngine* engine, const char* json, char* errorBuffer, size_t errorSize)
    {
        if (engine == nullptr || json == nullptr)
            return 0;
        try
        {
            crucible::Patch p;
            std::string error;
            if (!crucible::patchFromJson(json, p, error))
            {
                copyError(error, errorBuffer, errorSize);
                return 0;
            }
            crucible::applyPatch(engine->engine, p);
            return 1;
        }
        catch (const std::exception& e)
        {
            copyError(e.what(), errorBuffer, errorSize);
            return 0;
        }
        catch (...)
        {
            copyError("unknown error", errorBuffer, errorSize);
            return 0;
        }
    }

    int crucible_save_patch(CrucibleEngine* engine, const char* name, char* buffer, size_t capacity)
    {
        if (engine == nullptr)
            return -1;
        try
        {
            crucible::PatchMeta meta;
            if (name != nullptr)
                meta.name = name;
            return copyOut(crucible::patchToJson(crucible::capturePatch(engine->engine, meta)), buffer, capacity);
        }
        catch (...)
        {
            return -1;
        }
    }

    int crucible_load_patch_file(CrucibleEngine* engine, const char* path, char* errorBuffer, size_t errorSize)
    {
        if (engine == nullptr || path == nullptr)
            return 0;
        try
        {
            crucible::Patch p;
            std::string error;
            if (!crucible::loadPatchFile(path, p, error))
            {
                copyError(error, errorBuffer, errorSize);
                return 0;
            }
            crucible::applyPatch(engine->engine, p);
            return 1;
        }
        catch (const std::exception& e)
        {
            copyError(e.what(), errorBuffer, errorSize);
            return 0;
        }
        catch (...)
        {
            copyError("unknown error", errorBuffer, errorSize);
            return 0;
        }
    }

    namespace
    {
    void fillWavetableInfo(CrucibleEngine* engine, int slot, int frameSize, CrucibleWavetableInfo* info)
    {
        if (info == nullptr)
            return;
        const auto* table = engine->engine.userWavetables().table(slot);
        const int in = table != nullptr ? table->sourceFrameCount() : 0;
        if (CRUCIBLE_HAS(info, frames_in))
            info->frames_in = in;
        if (CRUCIBLE_HAS(info, frames_used))
            info->frames_used = std::min(in, CRUCIBLE_WAVETABLE_FRAMES);
        if (CRUCIBLE_HAS(info, decimated))
            info->decimated = in > CRUCIBLE_WAVETABLE_FRAMES ? 1 : 0;
        if (CRUCIBLE_HAS(info, frame_size))
            info->frame_size = frameSize;
    }

    void clearWavetableInfo(CrucibleWavetableInfo* info)
    {
        if (info == nullptr)
            return;
        if (CRUCIBLE_HAS(info, frames_in))
            info->frames_in = 0;
        if (CRUCIBLE_HAS(info, frames_used))
            info->frames_used = 0;
        if (CRUCIBLE_HAS(info, decimated))
            info->decimated = 0;
        if (CRUCIBLE_HAS(info, frame_size))
            info->frame_size = 0;
    }
    } // namespace

    int crucible_load_wavetable_file(CrucibleEngine* engine, int slot, const char* path, CrucibleWavetableInfo* info,
                                     char* error_buffer, size_t error_size)
    {
        return guardedText<int>(0, error_buffer, error_size,
                                [&]() -> int
                                {
                                    clearWavetableInfo(info);
                                    if (engine == nullptr || path == nullptr)
                                        return 0;
                                    if (slot < 0 || slot >= CRUCIBLE_USER_WAVETABLES)
                                    {
                                        copyError("slot out of range (0..7)", error_buffer, error_size);
                                        return 0;
                                    }
                                    std::string error;
                                    crucible::AudioData audio;
                                    if (!crucible::readWavFile(path, audio, error))
                                    {
                                        copyError(error, error_buffer, error_size);
                                        return 0;
                                    }
                                    const crucible::WavetableImport options{};
                                    const int frameSize = crucible::detectWavetableFrames(audio, options).frameSize;
                                    std::string name = path;
                                    const auto cut = name.find_last_of("/\\");
                                    if (cut != std::string::npos)
                                        name = name.substr(cut + 1);
                                    if (!engine->engine.loadWavetable(slot, audio, options, name, error))
                                    {
                                        copyError(error, error_buffer, error_size);
                                        return 0;
                                    }
                                    fillWavetableInfo(engine, slot, frameSize, info);
                                    return 1;
                                });
    }

    int crucible_load_wavetable(CrucibleEngine* engine, int slot, const float* frames, int frame_count, int frame_size,
                                const char* name, CrucibleWavetableInfo* info, char* error_buffer, size_t error_size)
    {
        return guardedText<int>(
            0, error_buffer, error_size,
            [&]() -> int
            {
                clearWavetableInfo(info);
                if (engine == nullptr)
                    return 0;
                if (slot < 0 || slot >= CRUCIBLE_USER_WAVETABLES)
                {
                    copyError("slot out of range (0..7)", error_buffer, error_size);
                    return 0;
                }
                if (frames == nullptr || frame_count < 1 || frame_size < 4 || frame_size > 65536 ||
                    static_cast<long long>(frame_count) * frame_size > (1LL << 26))
                {
                    copyError("need frames, frame_count >= 1 and frame_size 4..65536 (at most 64M samples in all)",
                              error_buffer, error_size);
                    return 0;
                }
                crucible::AudioData audio;
                audio.channels.emplace_back(frames,
                                            frames + static_cast<size_t>(frame_count) * static_cast<size_t>(frame_size));
                for (float& x : audio.channels[0])
                    if (!std::isfinite(x))
                        x = 0.0f;
                crucible::WavetableImport options;
                options.frameSize = frame_size;
                std::string error;
                if (!engine->engine.loadWavetable(slot, audio, options, name != nullptr ? name : "Table", error))
                {
                    copyError(error, error_buffer, error_size);
                    return 0;
                }
                fillWavetableInfo(engine, slot, frame_size, info);
                return 1;
            });
    }

    int crucible_load_wavetable_zones(CrucibleEngine* engine, int slot, const CrucibleWavetableZone* zones, int zone_count,
                                      const char* name, CrucibleWavetableInfo* info, char* error_buffer, size_t error_size)
    {
        return guardedText<int>(
            0, error_buffer, error_size,
            [&]() -> int
            {
                clearWavetableInfo(info);
                if (engine == nullptr)
                    return 0;
                if (slot < 0 || slot >= CRUCIBLE_USER_WAVETABLES)
                {
                    copyError("slot out of range (0..7)", error_buffer, error_size);
                    return 0;
                }
                if (zones == nullptr || zone_count < 1 || zone_count > crucible::kMaxWavetableZones)
                {
                    copyError("need 1..16 zones", error_buffer, error_size);
                    return 0;
                }
                std::vector<crucible::WavetableZone> list;
                long long total = 0;
                int firstFrameSize = 0;
                double previous = 0.0;
                for (int z = 0; z < zone_count; ++z)
                {
                    // The caller's zones are struct_size apart (the first one's), not our sizeof: a caller built against a
                    // later, larger struct still reads right.
                    const size_t stride = zones[0].struct_size;
                    if (stride < offsetof(CrucibleWavetableZone, max_hz) + sizeof(double))
                    {
                        copyError("zone struct_size too small", error_buffer, error_size);
                        return 0;
                    }
                    const CrucibleWavetableZone& zone =
                        *reinterpret_cast<const CrucibleWavetableZone*>(reinterpret_cast<const char*>(zones) + static_cast<size_t>(z) * stride);
                    if (zone.struct_size != stride)
                    {
                        copyError("every zone needs the same struct_size", error_buffer, error_size);
                        return 0;
                    }
                    if (zone.struct_size < offsetof(CrucibleWavetableZone, max_hz) + sizeof(double))
                    {
                        copyError("zone struct_size too small", error_buffer, error_size);
                        return 0;
                    }
                    if (zone.frames == nullptr || zone.frame_count < 1 || zone.frame_size < 4 || zone.frame_size > 65536)
                    {
                        copyError("each zone needs frames, frame_count >= 1 and frame_size 4..65536", error_buffer,
                                  error_size);
                        return 0;
                    }
                    total += static_cast<long long>(zone.frame_count) * zone.frame_size;
                    if (total > (1LL << 26))
                    {
                        copyError("too much data (at most 64M samples in all)", error_buffer, error_size);
                        return 0;
                    }
                    if (!std::isfinite(zone.max_hz) || zone.max_hz <= 0.0 || (z > 0 && zone.max_hz <= previous))
                    {
                        copyError("max_hz must be finite, positive and strictly ascending", error_buffer, error_size);
                        return 0;
                    }
                    previous = zone.max_hz;
                    if (z == 0)
                        firstFrameSize = zone.frame_size;
                    crucible::WavetableZone out;
                    out.maxHz = zone.max_hz;
                    out.cycles.reserve(static_cast<size_t>(zone.frame_count));
                    for (int f = 0; f < zone.frame_count; ++f)
                    {
                        const float* src = zone.frames + static_cast<size_t>(f) * static_cast<size_t>(zone.frame_size);
                        std::vector<float> cycle(src, src + zone.frame_size);
                        for (float& x : cycle)
                            if (!std::isfinite(x))
                                x = 0.0f;
                        out.cycles.push_back(std::move(cycle));
                    }
                    list.push_back(std::move(out));
                }
                if (!engine->engine.userWavetables().loadZones(slot, list, crucible::WavetableImport::Normalise::Table,
                                                               name != nullptr ? name : "Table"))
                {
                    copyError("zones refused", error_buffer, error_size);
                    return 0;
                }
                fillWavetableInfo(engine, slot, firstFrameSize, info);
                return 1;
            });
    }

    int crucible_set_formant_table(CrucibleEngine* engine, int filter_slot, const CrucibleFormant* formants,
                                   int vowel_count, int formant_count, float output_gain, char* error_buffer,
                                   size_t error_size)
    {
        return guarded<int>(
            0,
            [&]() -> int
            {
                if (engine == nullptr)
                    return 0;
                std::string error;
                static_assert(sizeof(CrucibleFormant) == sizeof(crucible::FormantBand), "layout");
                const auto* bands = reinterpret_cast<const crucible::FormantBand*>(formants);
                if (!engine->engine.setFormantTable(filter_slot, bands, vowel_count, formant_count, output_gain, error))
                {
                    copyError(error.c_str(), error_buffer, error_size);
                    return 0;
                }
                return 1;
            });
    }

    int crucible_set_formant_saturation(CrucibleEngine* engine, int filter_slot, int on, char* error_buffer,
                                        size_t error_size)
    {
        return guarded<int>(
            0,
            [&]() -> int
            {
                if (engine == nullptr)
                    return 0;
                std::string error;
                if (!engine->engine.setFormantSaturation(filter_slot, on != 0, error))
                {
                    copyError(error.c_str(), error_buffer, error_size);
                    return 0;
                }
                return 1;
            });
    }

    int crucible_load_sample(CrucibleEngine* engine, int slot, const float* interleaved, int frames, int channels,
                             double sample_rate, const char* name)
    {
        return guarded<int>(
            0,
            [&]() -> int
            {
                if (engine == nullptr || interleaved == nullptr || frames < 2 || channels < 1 || channels > 2)
                    return 0;
                crucible::AudioData audio;
                audio.sampleRate = sample_rate;
                audio.channels.assign(static_cast<size_t>(channels), std::vector<float>(static_cast<size_t>(frames)));
                for (int i = 0; i < frames; ++i)
                    for (int c = 0; c < channels; ++c)
                        audio.channels[static_cast<size_t>(c)][static_cast<size_t>(i)] = interleaved[i * channels + c];
                std::string error;
                return engine->engine.loadSample(slot, audio, name != nullptr ? name : "Sample", error) ? 1 : 0;
            });
    }

    int crucible_load_sample_file(CrucibleEngine* engine, int slot, const char* path)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || path == nullptr)
                                    return 0;
                                std::string error;
                                return engine->engine.loadSampleFile(slot, path, error) ? 1 : 0;
                            });
    }

    void crucible_clear_sample(CrucibleEngine* engine, int slot)
    {
        guardedVoid(
            [&]
            {
                if (engine != nullptr)
                    engine->engine.clearSample(slot);
            });
    }

    int crucible_set_sample_slices(CrucibleEngine* engine, int slot, const float* markers, int count)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || (markers == nullptr && count > 0) || count < 0)
                                    return 0;
                                return engine->engine.setSampleSlices(slot, std::vector<float>(markers, markers + count))
                                           ? 1
                                           : 0;
                            });
    }

    int crucible_get_sample_slices(CrucibleEngine* engine, int slot, float* markers, int capacity)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr)
                                    return 0;
                                const auto list = engine->engine.sampleSlices(slot);
                                for (int i = 0; i < capacity && markers != nullptr && i < static_cast<int>(list.size()); ++i)
                                    markers[i] = list[static_cast<size_t>(i)];
                                return static_cast<int>(list.size());
                            });
    }

    int crucible_slice_sample_equally(CrucibleEngine* engine, int slot, int slices)
    {
        return guarded<int>(0, [&]() -> int
                            { return engine != nullptr ? engine->engine.sliceSampleEqually(slot, slices) : 0; });
    }

    int crucible_slice_sample_transients(CrucibleEngine* engine, int slot, int maxSlices, float sensitivity)
    {
        return guarded<int>(
            0, [&]() -> int
            { return engine != nullptr ? engine->engine.sliceSampleByTransients(slot, maxSlices, sensitivity) : 0; });
    }

    int crucible_add_zone_sample(CrucibleEngine* engine, int slot, const float* interleaved, int frames, int channels,
                                 double sample_rate, const char* name)
    {
        return guarded<int>(
            0,
            [&]() -> int
            {
                if (engine == nullptr || interleaved == nullptr || frames < 2 || channels < 1 || channels > 2)
                    return 0;
                crucible::AudioData audio;
                audio.sampleRate = sample_rate;
                audio.channels.assign(static_cast<size_t>(channels), std::vector<float>(static_cast<size_t>(frames)));
                for (int i = 0; i < frames; ++i)
                    for (int c = 0; c < channels; ++c)
                        audio.channels[static_cast<size_t>(c)][static_cast<size_t>(i)] = interleaved[i * channels + c];
                std::string error;
                return engine->engine.addZoneSample(slot, audio, name != nullptr ? name : "Sample", error);
            });
    }

    int crucible_set_zones(CrucibleEngine* engine, int slot, const CrucibleZone* zones, int count)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || (zones == nullptr && count > 0) || count < 0)
                                    return 0;
                                std::vector<crucible::Zone> list;
                                for (int i = 0; i < count && i < crucible::kMaxZones; ++i)
                                {
                                    const CrucibleZone& c = zones[i];
                                    crucible::Zone z;
                                    const auto u8 = [](int v) { return static_cast<std::uint8_t>(std::clamp(v, 0, 255)); };
                                    z.sampleId = c.sample_id;
                                    z.keyLo = u8(c.key_lo);
                                    z.keyHi = u8(c.key_hi);
                                    z.velLo = u8(c.vel_lo);
                                    z.velHi = u8(c.vel_hi);
                                    z.root = c.root;
                                    z.gainDb = c.gain_db;
                                    z.tuneCents = c.tune_cents;
                                    z.loopStart = c.loop_start;
                                    z.loopEnd = c.loop_end;
                                    z.loopMode = c.loop_mode;
                                    z.group = u8(c.group);
                                    list.push_back(z);
                                }
                                return engine->engine.setZones(slot, std::move(list)) ? 1 : 0;
                            });
    }

    int crucible_get_zones(CrucibleEngine* engine, int slot, CrucibleZone* zones, int capacity)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr)
                                    return 0;
                                const auto list = engine->engine.zoneList(slot);
                                for (int i = 0; i < capacity && zones != nullptr && i < static_cast<int>(list.size()); ++i)
                                {
                                    const crucible::Zone& z = list[static_cast<size_t>(i)];
                                    zones[i] =
                                        CrucibleZone{z.sampleId, z.keyLo,     z.keyHi,     z.velLo,   z.velHi,    z.root,
                                                     z.gainDb,   z.tuneCents, z.loopStart, z.loopEnd, z.loopMode, z.group};
                                }
                                return static_cast<int>(list.size());
                            });
    }

    int crucible_zone_sample_count(CrucibleEngine* engine, int slot)
    {
        return engine != nullptr ? engine->engine.zoneSampleCount(slot) : 0;
    }

    void crucible_clear_zones(CrucibleEngine* engine, int slot)
    {
        guardedVoid(
            [&]
            {
                if (engine != nullptr)
                    engine->engine.clearZones(slot);
            });
    }

    int crucible_import_zone_folder(CrucibleEngine* engine, int slot, const char* directory)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || directory == nullptr)
                                    return 0;
                                std::string error;
                                return engine->engine.importZoneFolder(slot, directory, error);
                            });
    }

    int crucible_import_zone_sfz(CrucibleEngine* engine, int slot, const char* sfz_path)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || sfz_path == nullptr)
                                    return 0;
                                std::string error;
                                return engine->engine.importZoneSfz(slot, sfz_path, error);
                            });
    }

    int crucible_set_cloud(CrucibleEngine* engine, int slot, const char* cloud_text)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || cloud_text == nullptr)
                                    return 0;
                                crucible::CloudModel model;
                                if (!crucible::CloudModel::deserialise(cloud_text, model))
                                    return 0;
                                engine->engine.setCloud(slot, model);
                                return 1;
                            });
    }

    int crucible_load_ir(CrucibleEngine* engine, const char* instance_id, const char* path, char* error, size_t error_size)
    {
        return guardedText<int>(0, error, error_size,
                                [&]() -> int
                                {
                                    std::string message;
                                    if (engine == nullptr || instance_id == nullptr || path == nullptr)
                                        message = "missing argument";
                                    else if (engine->engine.loadIr(instance_id, path, message))
                                        return 1;
                                    copyError(message, error, error_size);
                                    return 0;
                                });
    }

    int crucible_clear_ir(CrucibleEngine* engine, const char* instance_id)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                std::string message;
                                if (engine == nullptr || instance_id == nullptr)
                                    return 0;
                                return engine->engine.clearIr(instance_id, message) ? 1 : 0;
                            });
    }

    int crucible_set_shaper_curve(CrucibleEngine* engine, const char* instance_id, const CrucibleShaperPoint* points,
                                  int count, int symmetric, char* error, size_t error_size)
    {
        return guardedText<int>(0, error, error_size,
                                [&]() -> int
                                {
                                    std::string message;
                                    if (engine == nullptr || instance_id == nullptr)
                                        message = "missing argument";
                                    else
                                    {
                                        static_assert(sizeof(CrucibleShaperPoint) == sizeof(crucible::ShaperPoint), "layout");
                                        const auto* p = reinterpret_cast<const crucible::ShaperPoint*>(points);
                                        if (engine->engine.setShaperCurve(instance_id, p, count, symmetric != 0, message))
                                            return 1;
                                    }
                                    copyError(message, error, error_size);
                                    return 0;
                                });
    }

    int crucible_get_shaper_curve(CrucibleEngine* engine, const char* instance_id, CrucibleShaperPoint* points,
                                  int max_points, int* symmetric)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || instance_id == nullptr)
                                    return 0;
                                std::vector<crucible::ShaperPoint> curve;
                                bool sym = false;
                                if (!engine->engine.shaperCurve(instance_id, curve, sym))
                                    return 0;
                                if (symmetric != nullptr)
                                    *symmetric = sym ? 1 : 0;
                                for (size_t i = 0; points != nullptr && i < curve.size() && static_cast<int>(i) < max_points; ++i)
                                    points[i] = {curve[i].x, curve[i].y, curve[i].curve};
                                return static_cast<int>(curve.size());
                            });
    }

    int crucible_add_route(CrucibleEngine* engine, int source, const char* target, float amount)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || target == nullptr || !std::isfinite(amount))
                                    return 0;
                                return engine->engine.modRoutes().add({source, target, amount}) ? 1 : 0;
                            });
    }

    int crucible_add_route_ex(CrucibleEngine* engine, const CrucibleRoute* route)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || route == nullptr || route->target_id == nullptr ||
                                    !CRUCIBLE_HAS(route, amount))
                                    return 0;
                                crucible::ModRoute r;
                                r.source = route->source;
                                r.target = route->target_id;
                                r.amount = route->amount;
                                if (CRUCIBLE_HAS(route, polarity))
                                    r.polarity = route->polarity;
                                if (CRUCIBLE_HAS(route, curve))
                                    r.curve = route->curve;
                                if (CRUCIBLE_HAS(route, smoothing_ms))
                                    r.smoothingMs = route->smoothing_ms;
                                if (!std::isfinite(r.amount) || !std::isfinite(r.curve) || !std::isfinite(r.smoothingMs))
                                    return 0;
                                return engine->engine.modRoutes().add(r) ? 1 : 0;
                            });
    }

    int crucible_route_count(CrucibleEngine* engine)
    {
        return guarded<int>(0, [&]() -> int { return engine != nullptr ? engine->engine.modRoutes().routeCount() : 0; });
    }

    int crucible_route_get(CrucibleEngine* engine, int k, CrucibleRoute* out)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                crucible::ModRoute r;
                                if (engine == nullptr || out == nullptr || !engine->engine.modRoutes().routeAt(k, r))
                                    return 0;
                                CrucibleRoute full{};
                                full.struct_size = sizeof(full);
                                full.source = r.source;
                                full.target_id = engine->routeTargets.insert(r.target).first->c_str();
                                full.amount = r.amount;
                                full.polarity = r.polarity;
                                full.curve = r.curve;
                                full.smoothing_ms = r.smoothingMs;
                                return copyStruct(out, full) ? 1 : 0;
                            });
    }

    int crucible_remove_route(CrucibleEngine* engine, int source, const char* target)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || target == nullptr)
                                    return 0;
                                return engine->engine.modRoutes().remove(source, target) ? 1 : 0;
                            });
    }

    CrucibleKeyBus* crucible_keybus_open_local(void)
    {
        return guarded<CrucibleKeyBus*>(nullptr,
                                        [&]() -> CrucibleKeyBus* { return new CrucibleKeyBus{crucible::KeyBus::local()}; });
    }

    void crucible_keybus_close(CrucibleKeyBus* bus)
    {
        delete bus;
    }

    uint32_t crucible_keybus_publish(CrucibleKeyBus* bus, int root, int mask, int tie_up)
    {
        return bus == nullptr ? 0 : bus->bus->publish(root, static_cast<std::uint16_t>(mask & 0x0fff), tie_up != 0);
    }

    int crucible_keybus_read(const CrucibleKeyBus* bus, int* root, int* mask, int* tie_up, uint32_t* generation)
    {
        if (bus == nullptr)
            return 0;
        const auto s = bus->bus->read();
        if (root != nullptr)
            *root = s.root;
        if (mask != nullptr)
            *mask = s.mask;
        if (tie_up != nullptr)
            *tie_up = s.tieUp ? 1 : 0;
        if (generation != nullptr)
            *generation = s.generation;
        return 1;
    }

    int crucible_attach_keybus(CrucibleEngine* engine, CrucibleKeyBus* bus)
    {
        return guarded<int>(0,
                            [&]() -> int
                            {
                                if (engine == nullptr || bus == nullptr)
                                    return 0;
                                engine->engine.attachKeyBus(bus->bus);
                                return 1;
                            });
    }

    int crucible_key_scale(const CrucibleEngine* engine, int* mode, int* root, int in_key[12], int* following)
    {
        if (engine == nullptr)
            return 0;
        const auto v = const_cast<CrucibleEngine*>(engine)->engine.keyScale();
        if (mode != nullptr)
            *mode = static_cast<int>(v.mode);
        if (root != nullptr)
            *root = v.root;
        if (in_key != nullptr)
            for (int i = 0; i < 12; ++i)
                in_key[i] = v.inKey[static_cast<size_t>(i)] ? 1 : 0;
        if (following != nullptr)
            *following = v.following ? 1 : 0;
        return 1;
    }

    int crucible_key_detect(CrucibleEngine* engine, int* root, int* minor, float* confidence)
    {
        if (engine == nullptr)
            return 0;
        const auto p = engine->engine.keyDetector().propose();
        if (p.evidence <= 0.0f)
            return 0;
        if (root != nullptr)
            *root = p.root;
        if (minor != nullptr)
            *minor = p.minor ? 1 : 0;
        if (confidence != nullptr)
            *confidence = p.confidence;
        return 1;
    }

    void crucible_key_detect_reset(CrucibleEngine* engine)
    {
        if (engine != nullptr)
            engine->engine.keyDetector().reset();
    }

} // extern "C"

CrucibleKeyBus* crucible_keybus_wrap(std::shared_ptr<crucible::KeyBus> bus)
{
    return bus ? new CrucibleKeyBus{std::move(bus)} : nullptr;
}
