/* SPDX-License-Identifier: MIT */
/*
 * Crucible module ABI: a plain-C interface for writing synth modules (effects, oscillators, filters, modulators) that
 * the engine loads at run time, from any language that can export C symbols. The conventions follow CLAP's: structs
 * are versioned and append-only, every struct starts with `size` and `abi_version`, strings are UTF-8, the host hands
 * the module a table of callbacks, and a shared library exports one entry point that lists its modules.
 *
 * Versioning. CRUCIBLE_MODULE_ABI_VERSION counts layouts. Fields are only ever appended to a struct and a field is
 * never moved or changed, so a host accepts every module whose abi_version lies in
 * [CRUCIBLE_MODULE_ABI_MIN, CRUCIBLE_MODULE_ABI_VERSION] and reads a struct only up to its own `size`. A module with a
 * newer abi_version than the host is refused with CRUCIBLE_MODULE_ERR_ABI_VERSION; it is never partly loaded.
 *
 * Real-time contract (tested by tools/crucible-module-check). process, process_quad, note_on and note_off run on the
 * audio thread: they never allocate, lock, wait, log or call into the operating system. The output is finite and
 * bounded for every parameter value inside its range, the module does not rely on flush-to-zero, and its output is a
 * function of its seed, its parameters and its input only. create, destroy, prepare and get_view run on a control
 * thread and may allocate.
 *
 * Lifetime. The descriptor, the parameter and port structs and every string they point to must stay valid for as long as
 * the module can be created (static storage). The host copies what it needs when a module is registered.
 */
#ifndef CRUCIBLE_MODULE_H
#define CRUCIBLE_MODULE_H

#include "crucible_export.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define CRUCIBLE_MODULE_ABI_VERSION 1u
#define CRUCIBLE_MODULE_ABI_MIN 1u

#if defined(_WIN32)
#define CRUCIBLE_MODULE_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#define CRUCIBLE_MODULE_EXPORT __attribute__((visibility("default")))
#else
#define CRUCIBLE_MODULE_EXPORT
#endif

/* Limits a module must stay within. */
#define CRUCIBLE_MODULE_MAX_PARAMS 64
#define CRUCIBLE_MODULE_MAX_PORTS 8
#define CRUCIBLE_MODULE_MAX_CHOICES 128

/* ---- Result codes of the host functions (0 is success, errors are negative). */
#define CRUCIBLE_MODULE_OK 0
#define CRUCIBLE_MODULE_ERR_ARGUMENT (-1)    /* a null or malformed argument */
#define CRUCIBLE_MODULE_ERR_ABI_VERSION (-2) /* the module was built for an ABI version this host does not speak */
#define CRUCIBLE_MODULE_ERR_INVALID (-3)     /* the descriptor breaks a rule; the error text says which */
#define CRUCIBLE_MODULE_ERR_DUPLICATE (-4)   /* a module with this id is already registered */
#define CRUCIBLE_MODULE_ERR_LOAD (-5)        /* the shared library cannot be opened */
#define CRUCIBLE_MODULE_ERR_SYMBOL (-6)      /* the library has no usable crucible_get_modules */
#define CRUCIBLE_MODULE_ERR_DISABLED (-7)    /* the host has not enabled loading shared libraries */

/* ---- Kinds. A module of kind EFFECT is a stereo bus processor in a product's FX graph (no ports: one stereo input and
 * one stereo output, processed in place). The other kinds are nodes of the voice graph with the ports they declare. */
#define CRUCIBLE_MODULE_KIND_EFFECT 1u
#define CRUCIBLE_MODULE_KIND_GENERATOR 2u /* oscillators and other sound sources */
#define CRUCIBLE_MODULE_KIND_FILTER 3u
#define CRUCIBLE_MODULE_KIND_MODULATOR 4u /* envelopes, LFOs, ...: modulation outputs */

#define CRUCIBLE_MODULE_CPU_LIGHT 0u
#define CRUCIBLE_MODULE_CPU_MEDIUM 1u
#define CRUCIBLE_MODULE_CPU_HEAVY 2u

/* ---- Ports (nodes only). */
#define CRUCIBLE_PORT_AUDIO_MONO 1u
#define CRUCIBLE_PORT_AUDIO_STEREO 2u
#define CRUCIBLE_PORT_MODULATION 3u /* one value per chunk */
#define CRUCIBLE_PORT_TRIGGER 4u

    typedef struct crucible_port_spec
    {
        uint32_t size;        /* sizeof(crucible_port_spec) */
        uint32_t abi_version; /* CRUCIBLE_MODULE_ABI_VERSION */
        const char* name;     /* [a-z][a-z0-9_]*, unique among the inputs (or the outputs) of the module */
        uint32_t type;        /* CRUCIBLE_PORT_* */
    } crucible_port_spec;

/* ---- Parameters. The host hands them to process() in declaration order, as natural values, already smoothed. */
#define CRUCIBLE_PARAM_FLOAT 1u
#define CRUCIBLE_PARAM_INT 2u
#define CRUCIBLE_PARAM_CHOICE 3u /* values 0 .. num_choices - 1; min and max are ignored */
#define CRUCIBLE_PARAM_BOOL 4u   /* 0 or 1 */

#define CRUCIBLE_PARAM_MODULATABLE 0x1u   /* effects: accepts the generic modulation routes */
#define CRUCIBLE_PARAM_LOG_SMOOTHING 0x2u /* smooth in the log domain (frequencies); needs min > 0 */
#define CRUCIBLE_PARAM_NO_SMOOTHING 0x4u  /* the value jumps (switches, choices, times) */
#define CRUCIBLE_PARAM_NOT_AUTOMATABLE 0x8u

    typedef struct crucible_param_spec
    {
        uint32_t size;
        uint32_t abi_version;
        const char* id;         /* local id, [a-z][a-z0-9_]*, unique in the module and stable across releases */
        const char* name;       /* display name */
        const char* short_name; /* at most 6 ASCII characters, or NULL */
        const char* unit;       /* "Hz", "ms", "dB", ... or NULL */
        uint32_t type;          /* CRUCIBLE_PARAM_* */
        uint32_t flags;         /* CRUCIBLE_PARAM_* flags */
        float min, max, default_value;
        float interval; /* snapping step of a float, 0 = continuous */
        float skew;     /* 1 = linear; < 1 gives the lower part of the range more of the knob; 0 means 1 */
        uint32_t num_choices;
        const char* const* choices; /* CRUCIBLE_PARAM_CHOICE: num_choices display names */
    } crucible_param_spec;

    /* ---- What the host tells the module. */
    typedef struct crucible_module_host
    {
        uint32_t size;
        uint32_t abi_version;
        const char* name;    /* "Crucible Engine" */
        const char* version; /* the engine version, "major.minor.patch" */
        /* Control thread only. severity: 0 info, 1 warning, 2 error; message is UTF-8. May be NULL in a host that does
         * not log. */
        void (*log)(const struct crucible_module_host* host, int32_t severity, const char* message);
    } crucible_module_host;

    typedef struct crucible_module_prepare
    {
        uint32_t size;
        uint32_t abi_version;
        double sample_rate;
        int32_t max_block_size; /* the longest block process() is given */
        uint32_t quality;       /* 0 draft, 1 standard, 2 high */
        uint32_t seed;          /* seeds every random choice of the instance (noise, drift, tolerances) */
    } crucible_module_prepare;

    typedef struct crucible_module_note
    {
        uint32_t size;
        uint32_t abi_version;
        float note;     /* MIDI note units, fractional */
        float velocity; /* 0..1 */
        uint32_t seed;  /* differs per note and voice; re-seeds the random choices of the note */
        uint32_t flags; /* CRUCIBLE_NOTE_ONE_SHOT */
    } crucible_module_note;
#define CRUCIBLE_NOTE_ONE_SHOT 0x1u /* note-off will not come: finish by yourself */

    typedef struct crucible_module_audio
    {
        float* left;
        float* right; /* NULL on a mono port */
    } crucible_module_audio;

    typedef struct crucible_module_process
    {
        uint32_t size;
        uint32_t abi_version;
        int32_t num_samples; /* at most max_block_size */
        /* Audio, one entry per declared audio port in declaration order (an effect has one stereo input and one stereo
         * output, and audio_in[0] and audio_out[0] are the same buffers: process in place). A node's input buffers are
         * read-only and are never the output buffers. */
        const crucible_module_audio* audio_in;
        uint32_t num_audio_in;
        crucible_module_audio* audio_out;
        uint32_t num_audio_out;
        /* Modulation, one value per declared modulation port. mod_in holds the sum of the connected edges (0 when none);
         * bit p of mod_connected is set when input port p has an edge. The module writes mod_out. */
        const float* mod_in;
        uint32_t num_mod_in;
        uint32_t mod_connected;
        float* mod_out;
        uint32_t num_mod_out;
        /* Parameters in declaration order. */
        const float* params;
        uint32_t num_params;
        float note;     /* nodes: pitch of the voice in MIDI note units, bend included */
        float velocity; /* nodes: 0..1 */
        double tempo;   /* beats per minute, 120 when the host does not know */
        /* Effects: the sidechain input of the host for this block (NULL when there is none). A mono input has the same
         * pointer twice. */
        const float* aux_left;
        const float* aux_right;
    } crucible_module_process;

    /* ---- The module. */
    typedef struct crucible_module_descriptor
    {
        uint32_t size;        /* sizeof(crucible_module_descriptor) */
        uint32_t abi_version; /* CRUCIBLE_MODULE_ABI_VERSION the module was built against */
        /* Identity. id: dotted lower case segments, [a-z][a-z0-9_]*, at least two ("acme.tremolo"); the first segment names
         * the vendor. The engine reserves "crucible", "fx", "osc", "voice", "filter", "env", "lfo", "amp", "mix" and
         * "classic". The id never changes once released. */
        const char* id;
        const char* name;
        const char* vendor; /* or NULL */
        const char* category;
        const char* description;
        uint32_t kind;      /* CRUCIBLE_MODULE_KIND_* */
        uint32_t version;   /* >= 1; bumped when parameters are added or change meaning */
        uint32_t cpu_class; /* CRUCIBLE_MODULE_CPU_* */
        uint32_t flags;     /* reserved, 0 */
        void* user_data;    /* the module's own, handed back to create() through the descriptor */
        /* Parameters and ports: arrays of pointers, so a struct can grow without moving its neighbours. */
        uint32_t num_params;
        const crucible_param_spec* const* params;
        uint32_t num_inputs;
        const crucible_port_spec* const* inputs;
        uint32_t num_outputs;
        const crucible_port_spec* const* outputs;

        /* Control thread. Returns the instance, or NULL on failure. */
        void* (*create)(const struct crucible_module_descriptor* descriptor, const crucible_module_host* host);
        void (*destroy)(void* instance);
        /* Allocate everything the instance needs. Returns 1 on success, 0 to refuse the configuration. May be NULL. */
        int32_t (*prepare)(void* instance, const crucible_module_prepare* info);
        /* Return to the state right after prepare() (clear delay lines, filters, envelopes). Audio thread safe. May be NULL.
         */
        void (*reset)(void* instance);
        /* Audio thread. Required. */
        void (*process)(void* instance, const crucible_module_process* data);
        /* Audio thread, nodes only; both optional. */
        void (*note_on)(void* instance, const crucible_module_note* info);
        void (*note_off)(void* instance);
        /* Optional, nodes only: the same node of four voices in one call (SIMD). instances[k] and data[k] belong to voice k.
         * Return nonzero when handled; zero makes the host call process() for each voice. */
        int32_t (*process_quad)(void* const instances[4], const crucible_module_process* const data[4]);
        /* Optional. Control thread. A UTF-8 JSON document for the view with this id (see the view API); valid until the next
         * call on this instance. Reserved: the host does not call it yet. */
        const char* (*get_view)(void* instance, const char* view_id);
        /* Optional. Samples of delay the instance adds to its audio path. */
        int32_t (*latency)(const void* instance);
        /* Optional, nodes only. Nonzero while the instance keeps the voice alive (an envelope that has not finished). */
        int32_t (*holds_voice)(const void* instance);
    } crucible_module_descriptor;

    /* A library lists its modules here. */
    typedef struct crucible_module_library
    {
        uint32_t size;
        uint32_t abi_version;
        uint32_t num_modules;
        const crucible_module_descriptor* const* modules;
    } crucible_module_library;

    /* The one symbol a module library exports. Called once per load on the control thread; the result lives in static
     * storage. */
    typedef const crucible_module_library* (*crucible_get_modules_fn)(void);
    CRUCIBLE_MODULE_EXPORT const crucible_module_library* crucible_get_modules(void);

    /* ---- Host side: what a program that embeds the engine calls. ---------------------------------------------------- */

    /* The modules a host has registered. Engines are built from a registry (crucible_create_from_product); the registry
     * keeps every library it loaded open for as long as a module of it exists. */
    typedef struct crucible_module_registry crucible_module_registry;

    CRUCIBLE_API crucible_module_registry* crucible_module_registry_create(void);
    /* Safe while engines built from the registry still run: modules stay alive as long as they are used. */
    CRUCIBLE_API void crucible_module_registry_destroy(crucible_module_registry* registry);

    /* Shared libraries are not loaded unless the host asks for it (0 by default). The engine never scans directories: the
     * host decides which paths to trust. */
    CRUCIBLE_API void crucible_module_registry_allow_libraries(crucible_module_registry* registry, int allow);

    /* Registers one module. Returns CRUCIBLE_MODULE_OK or a CRUCIBLE_MODULE_ERR_* code; the text of the last failure is
     * crucible_module_registry_error(). A module whose abi_version is newer than CRUCIBLE_MODULE_ABI_VERSION fails with
     * CRUCIBLE_MODULE_ERR_ABI_VERSION and nothing is registered. */
    CRUCIBLE_API int crucible_register_module(crucible_module_registry* registry,
                                              const crucible_module_descriptor* descriptor);

    /* Opens a shared library (UTF-8 path), looks up crucible_get_modules and registers every module it lists. All or
     * nothing: when one module is refused none is kept and the library is closed again. */
    CRUCIBLE_API int crucible_load_module_library(crucible_module_registry* registry, const char* path);

    /* The reason of the last failed call on this registry ("" when none): UTF-8 written into a caller buffer, NUL terminated
     * when buffer_size > 0. Returns the length of the full text (snprintf style): pass NULL / 0 to ask first. */
    CRUCIBLE_API int crucible_module_registry_error(const crucible_module_registry* registry, char* buffer,
                                                    size_t buffer_size);

    CRUCIBLE_API int crucible_module_registry_count(const crucible_module_registry* registry);
    /* Id of the k-th module, or NULL. */
    CRUCIBLE_API const char* crucible_module_registry_id(const crucible_module_registry* registry, int k);

#ifdef __cplusplus
}
#endif

#endif
