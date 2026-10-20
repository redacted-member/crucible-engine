// SPDX-License-Identifier: MIT
#pragma once

// C++ layer of the module SDK: write a module as a class and let crucible::sdk::Module<T> generate the C descriptor of
// crucible_module.h. The header only needs the C header and the standard library, so a module links against nothing of
// the engine; build it as a shared library that exports crucible_get_modules (CRUCIBLE_SDK_EXPORT_MODULES) and the host
// loads it at run time.
//
//   class Gain
//   {
//   public:
//       static crucible::sdk::Info info()
//       {
//           crucible::sdk::Info i;
//           i.id = "acme.gain";
//           i.name = "Gain";
//           i.kind = crucible::sdk::Kind::Effect;
//           i.params = {crucible::sdk::Param::floating("level", "Level", 0.0f, 2.0f, 1.0f)};
//           return i;
//       }
//       void process(crucible::sdk::Process& p)
//       {
//           for (int i = 0; i < p.frames(); ++i) { p.left()[i] *= p.param(0); p.right()[i] *= p.param(0); }
//       }
//   };
//   CRUCIBLE_SDK_EXPORT_MODULES(Gain)
//
// The class T provides (the others are optional and found by their signatures):
//   static Info info();                                  what the module is: id, kind, parameters, ports
//   void process(Process&);                              audio thread, required
//   bool prepare(const Prepare&) / void prepare(...)     control thread: allocate here; false refuses the configuration
//   void reset();   void noteOn(const Note&);   void noteOff();
//   bool holdsVoice() const;   int latency() const;
//   static bool processQuad(T* const (&instances)[4], Process (&data)[4]);   four voices at once; false = not handled
//   const char* view(const char* viewId);                 control thread: JSON of a view
// T is default constructible, or constructible from `const crucible_module_host&`.

#include <crucible/crucible_module.h>

#include <array>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace crucible::sdk
{

enum class Kind : std::uint32_t
{
    Effect = CRUCIBLE_MODULE_KIND_EFFECT,
    Generator = CRUCIBLE_MODULE_KIND_GENERATOR,
    Filter = CRUCIBLE_MODULE_KIND_FILTER,
    Modulator = CRUCIBLE_MODULE_KIND_MODULATOR
};

enum class Cpu : std::uint32_t
{
    Light = CRUCIBLE_MODULE_CPU_LIGHT,
    Medium = CRUCIBLE_MODULE_CPU_MEDIUM,
    Heavy = CRUCIBLE_MODULE_CPU_HEAVY
};

enum class PortKind : std::uint32_t
{
    AudioMono = CRUCIBLE_PORT_AUDIO_MONO,
    AudioStereo = CRUCIBLE_PORT_AUDIO_STEREO,
    Modulation = CRUCIBLE_PORT_MODULATION,
    Trigger = CRUCIBLE_PORT_TRIGGER
};

struct Port
{
    std::string name;
    PortKind type = PortKind::AudioStereo;
};

// One parameter. Build with the factory that matches the type, then refine:
//   Param::floating("cutoff", "Cutoff", 20.0f, 20000.0f, 1000.0f).unit("Hz").logSmoothing().skew(0.3f)
class Param
{
public:
    static Param floating(std::string id, std::string name, float min, float max, float def)
    {
        return Param(CRUCIBLE_PARAM_FLOAT, std::move(id), std::move(name), min, max, def);
    }
    static Param integer(std::string id, std::string name, int min, int max, int def)
    {
        Param p(CRUCIBLE_PARAM_INT, std::move(id), std::move(name), static_cast<float>(min), static_cast<float>(max),
                static_cast<float>(def));
        p.noSmoothing();
        return p;
    }
    static Param choice(std::string id, std::string name, std::vector<std::string> choices, int def)
    {
        Param p(CRUCIBLE_PARAM_CHOICE, std::move(id), std::move(name), 0.0f, static_cast<float>(choices.size() - 1),
                static_cast<float>(def));
        p.names = std::move(choices);
        p.noSmoothing();
        return p;
    }
    static Param toggle(std::string id, std::string name, bool def)
    {
        Param p(CRUCIBLE_PARAM_BOOL, std::move(id), std::move(name), 0.0f, 1.0f, def ? 1.0f : 0.0f);
        p.noSmoothing();
        return p;
    }

    Param& unit(std::string u)
    {
        unitText = std::move(u);
        return *this;
    }
    Param& shortName(std::string s)
    {
        shortText = std::move(s);
        return *this;
    }
    Param& skew(float s)
    {
        skewValue = s;
        return *this;
    }
    Param& interval(float i)
    {
        step = i;
        return *this;
    }
    Param& modulatable()
    {
        flagBits |= CRUCIBLE_PARAM_MODULATABLE;
        return *this;
    }
    Param& logSmoothing()
    {
        flagBits |= CRUCIBLE_PARAM_LOG_SMOOTHING;
        return *this;
    }
    Param& noSmoothing()
    {
        flagBits |= CRUCIBLE_PARAM_NO_SMOOTHING;
        return *this;
    }
    Param& notAutomatable()
    {
        flagBits |= CRUCIBLE_PARAM_NOT_AUTOMATABLE;
        return *this;
    }

private:
    template <typename T> friend class Module;
    Param(std::uint32_t t, std::string i, std::string n, float lo, float hi, float d)
        : type(t), id(std::move(i)), name(std::move(n)), min(lo), max(hi), def(d)
    {
    }
    std::uint32_t type;
    std::string id, name, unitText, shortText;
    float min, max, def;
    float skewValue = 1.0f;
    float step = 0.0f;
    std::uint32_t flagBits = 0;
    std::vector<std::string> names;
};

// What a module is. Everything the host shows in menus and generated UIs comes from here.
struct Info
{
    std::string id; // "vendor.module"
    std::string name;
    std::string vendor;
    std::string category;
    std::string description;
    Kind kind = Kind::Effect;
    std::uint32_t version = 1;
    Cpu cpu = Cpu::Light;
    std::vector<Param> params;
    // Nodes only (an effect has one stereo input and one stereo output).
    std::vector<Port> inputs, outputs;
};

struct Prepare
{
    double sampleRate = 48000.0;
    int maxBlockSize = 512;
    std::uint32_t quality = 1; // 0 draft, 1 standard, 2 high
    std::uint32_t seed = 1;
};

struct Note
{
    float note = 60.0f;
    float velocity = 1.0f;
    std::uint32_t seed = 1;
    bool oneShot = false;
};

// One call of process(): the buffers, parameters and modulation of a chunk.
class Process
{
public:
    Process() = default;
    explicit Process(const crucible_module_process& data) : d(&data) {}

    int frames() const { return d->num_samples; }
    float param(int i) const { return d->params[i]; }
    // Output buffer of audio port `port`; right() equals left() on a mono port. For an effect these are the bus itself
    // (input and output are the same buffers).
    float* left(int port = 0) const { return d->audio_out[port].left; }
    float* right(int port = 0) const
    {
        return d->audio_out[port].right != nullptr ? d->audio_out[port].right : d->audio_out[port].left;
    }
    const float* inLeft(int port = 0) const { return d->audio_in[port].left; }
    const float* inRight(int port = 0) const
    {
        return d->audio_in[port].right != nullptr ? d->audio_in[port].right : d->audio_in[port].left;
    }
    float mod(int port) const { return static_cast<std::uint32_t>(port) < d->num_mod_in ? d->mod_in[port] : 0.0f; }
    bool modConnected(int port) const { return ((d->mod_connected >> port) & 1U) != 0U; }
    void setMod(int port, float value) const
    {
        if (static_cast<std::uint32_t>(port) < d->num_mod_out)
            d->mod_out[port] = value;
    }
    float note() const { return d->note; }
    float velocity() const { return d->velocity; }
    double tempo() const { return d->tempo; }
    // Effects: the sidechain input of the host, or null.
    const float* auxLeft() const { return d->aux_left; }
    const float* auxRight() const { return d->aux_right; }
    const crucible_module_process& raw() const { return *d; }

private:
    const crucible_module_process* d = nullptr;
};

template <typename T> class Module
{
public:
    // The C descriptor of T, in static storage; valid for the life of the program.
    static const crucible_module_descriptor* descriptor() { return &holder().desc; }

private:
    struct Holder
    {
        Info info;
        std::vector<crucible_param_spec> params;
        std::vector<std::vector<const char*>> choices;
        std::vector<const crucible_param_spec*> paramTable;
        std::vector<crucible_port_spec> ports;
        std::vector<const crucible_port_spec*> inputTable, outputTable;
        crucible_module_descriptor desc{};

        Holder() : info(T::info())
        {
            const auto n = info.params.size();
            params.resize(n);
            choices.resize(n);
            paramTable.resize(n);
            for (size_t i = 0; i < n; ++i)
            {
                const Param& p = info.params[i];
                crucible_param_spec& c = params[i];
                c.size = sizeof(c);
                c.abi_version = CRUCIBLE_MODULE_ABI_VERSION;
                c.id = p.id.c_str();
                c.name = p.name.c_str();
                c.short_name = p.shortText.empty() ? nullptr : p.shortText.c_str();
                c.unit = p.unitText.empty() ? nullptr : p.unitText.c_str();
                c.type = p.type;
                c.flags = p.flagBits;
                c.min = p.min;
                c.max = p.max;
                c.default_value = p.def;
                c.interval = p.step;
                c.skew = p.skewValue;
                for (const auto& name : p.names)
                    choices[i].push_back(name.c_str());
                c.num_choices = static_cast<std::uint32_t>(choices[i].size());
                c.choices = choices[i].empty() ? nullptr : choices[i].data();
                paramTable[i] = &c;
            }
            ports.resize(info.inputs.size() + info.outputs.size());
            size_t k = 0;
            for (const auto& list : {&info.inputs, &info.outputs})
                for (const Port& p : *list)
                {
                    crucible_port_spec& c = ports[k++];
                    c.size = sizeof(c);
                    c.abi_version = CRUCIBLE_MODULE_ABI_VERSION;
                    c.name = p.name.c_str();
                    c.type = static_cast<std::uint32_t>(p.type);
                }
            for (size_t i = 0; i < info.inputs.size(); ++i)
                inputTable.push_back(&ports[i]);
            for (size_t i = 0; i < info.outputs.size(); ++i)
                outputTable.push_back(&ports[info.inputs.size() + i]);

            desc.size = sizeof(desc);
            desc.abi_version = CRUCIBLE_MODULE_ABI_VERSION;
            desc.id = info.id.c_str();
            desc.name = info.name.c_str();
            desc.vendor = info.vendor.empty() ? nullptr : info.vendor.c_str();
            desc.category = info.category.c_str();
            desc.description = info.description.c_str();
            desc.kind = static_cast<std::uint32_t>(info.kind);
            desc.version = info.version;
            desc.cpu_class = static_cast<std::uint32_t>(info.cpu);
            desc.num_params = static_cast<std::uint32_t>(n);
            desc.params = paramTable.empty() ? nullptr : paramTable.data();
            desc.num_inputs = static_cast<std::uint32_t>(inputTable.size());
            desc.inputs = inputTable.empty() ? nullptr : inputTable.data();
            desc.num_outputs = static_cast<std::uint32_t>(outputTable.size());
            desc.outputs = outputTable.empty() ? nullptr : outputTable.data();
            desc.create = &create;
            desc.destroy = &destroy;
            desc.process = &process;
            if constexpr (requires(T& t, const Prepare& p) { t.prepare(p); })
                desc.prepare = &prepare;
            if constexpr (requires(T& t) { t.reset(); })
                desc.reset = &reset;
            if constexpr (requires(T& t, const Note& n) { t.noteOn(n); })
                desc.note_on = &noteOn;
            if constexpr (requires(T& t) { t.noteOff(); })
                desc.note_off = &noteOff;
            if constexpr (requires(const T& t) { t.holdsVoice(); })
                desc.holds_voice = &holdsVoice;
            if constexpr (requires(const T& t) { t.latency(); })
                desc.latency = &latency;
            if constexpr (requires(T& t, const char* v) { t.view(v); })
                desc.get_view = &view;
            if constexpr (requires(T* const(&t)[4], Process(&p)[4]) { T::processQuad(t, p); })
                desc.process_quad = &processQuad;
        }
    };

    static Holder& holder()
    {
        static Holder h;
        return h;
    }

    static void* create(const crucible_module_descriptor*, const crucible_module_host* host)
    {
        try
        {
            if constexpr (requires(const crucible_module_host& h) { T(h); })
                return new T(*host);
            else
                return new T();
        }
        catch (...)
        {
            return nullptr;
        }
    }
    static void destroy(void* instance) { delete static_cast<T*>(instance); }
    static std::int32_t prepare(void* instance, const crucible_module_prepare* info)
    {
        Prepare p;
        p.sampleRate = info->sample_rate;
        p.maxBlockSize = info->max_block_size;
        p.quality = info->quality;
        p.seed = info->seed;
        try
        {
            if constexpr (std::is_same_v<decltype(static_cast<T*>(instance)->prepare(p)), bool>)
                return static_cast<T*>(instance)->prepare(p) ? 1 : 0;
            else
            {
                static_cast<T*>(instance)->prepare(p);
                return 1;
            }
        }
        catch (...)
        {
            return 0;
        }
    }
    static void reset(void* instance) { static_cast<T*>(instance)->reset(); }
    static void process(void* instance, const crucible_module_process* data)
    {
        Process p(*data);
        static_cast<T*>(instance)->process(p);
    }
    static void noteOn(void* instance, const crucible_module_note* info)
    {
        Note n;
        n.note = info->note;
        n.velocity = info->velocity;
        n.seed = info->seed;
        n.oneShot = (info->flags & CRUCIBLE_NOTE_ONE_SHOT) != 0U;
        static_cast<T*>(instance)->noteOn(n);
    }
    static void noteOff(void* instance) { static_cast<T*>(instance)->noteOff(); }
    static std::int32_t holdsVoice(const void* instance) { return static_cast<const T*>(instance)->holdsVoice() ? 1 : 0; }
    static std::int32_t latency(const void* instance)
    {
        return static_cast<std::int32_t>(static_cast<const T*>(instance)->latency());
    }
    static const char* view(void* instance, const char* viewId) { return static_cast<T*>(instance)->view(viewId); }
    static std::int32_t processQuad(void* const instances[4], const crucible_module_process* const data[4])
    {
        T* t[4] = {static_cast<T*>(instances[0]), static_cast<T*>(instances[1]), static_cast<T*>(instances[2]),
                   static_cast<T*>(instances[3])};
        Process p[4] = {Process(*data[0]), Process(*data[1]), Process(*data[2]), Process(*data[3])};
        return T::processQuad(t, p) ? 1 : 0;
    }
};

// The library entry: lists the modules T... (see CRUCIBLE_SDK_EXPORT_MODULES).
template <typename... Ts> const crucible_module_library* library()
{
    static const std::array<const crucible_module_descriptor*, sizeof...(Ts)> list = {Module<Ts>::descriptor()...};
    static const crucible_module_library lib = {sizeof(crucible_module_library), CRUCIBLE_MODULE_ABI_VERSION,
                                                static_cast<std::uint32_t>(sizeof...(Ts)), list.data()};
    return &lib;
}

} // namespace crucible::sdk

// Exports crucible_get_modules for the listed module classes. Use once, at namespace scope, in the library's source file.
#define CRUCIBLE_SDK_EXPORT_MODULES(...)                                                                                    \
    extern "C" CRUCIBLE_MODULE_EXPORT const crucible_module_library* crucible_get_modules(void)                             \
    {                                                                                                                       \
        return ::crucible::sdk::library<__VA_ARGS__>();                                                                     \
    }
