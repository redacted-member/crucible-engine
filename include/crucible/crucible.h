// SPDX-License-Identifier: MIT
/*
 * CrucibleEngine C API: a stable, plain-C interface for embedding the engine from Rust, Swift, Python (ctypes / cffi),
 * C# or any language with a C FFI. Opaque handles, no C++ types, no exceptions across the boundary.
 *
 * ABI rules: functions are only ever added. Structs that start with a size_t struct_size (CrucibleConfig,
 * CrucibleTransport, CrucibleRoute, CrucibleWavetableInfo, CrucibleParamMeta, CrucibleModuleMeta, CrucibleViewInfo) may
 * grow by appending fields: set struct_size to sizeof of the struct you compiled against. Structs you pass in (Config,
 * Transport, Route) give fields beyond it their defaults; structs the engine fills (WavetableInfo, ParamMeta,
 * ModuleMeta, ViewInfo) get only the fields that fit, and a struct_size of 0 (or not past the field itself) fills
 * nothing and fails. A Transport struct_size of 0 means "no transport", without error. All other struct layouts are
 * frozen per CRUCIBLE_API_VERSION (a new layout gets a new struct name). Strings returned by the engine stay valid for the
 * lifetime of the engine handle unless noted.
 *
 * Threads. Every function below carries one of these tags, in CLAP's vocabulary:
 *
 *   [audio]  the audio thread; never allocates or locks. Only the crucible_process* calls.
 *   [main]   the host's main thread (or a timer / GUI thread you serialise with it). May allocate, lock, read files.
 *            Calls tagged [main] never run on the audio thread, but they are safe while it processes unless the tag
 *            says "not concurrent with process".
 *   [any]    any thread, including audio: lock-free, no allocation. Immutable data (parameter metadata) and atomics.
 *
 *   Thread           What it may call
 *   audio            crucible_process, _process_host, _process_aux, _process_events, _process_events_aux; and every
 *                    [any] call
 *   main             every [main] call and every [any] call (one main thread at a time)
 *   note input       crucible_note_on / _note_off / _pitch_bend / _mod_wheel / _sustain / _all_notes_off / _kill_all_voices
 * [any]: a single-producer queue, so one thread at a time. crucible_midi shares that queue but is [main] any reads of values
 * and meters, crucible_view_read of pushed views, the keybus read / publish, crucible_service_pending, crucible_tail_seconds
 *   "[main, not concurrent with process]": crucible_prepare and crucible_destroy only.
 *   crucible_reset is [audio]: call it from the audio thread between process calls (activate, transport jump, bypass),
 *   or from the control thread while the audio thread is not inside a process call.
 *
 * Parameter changes made through crucible_process_events that need setup work (oscillator type, wavetable, sequencer
 * pattern) take effect after the next crucible_service call; call it regularly (a timer is fine), or after a block
 * when crucible_service_pending says there is work (an offline bounce has no timer).
 *
 * Buffers: functions that return text or arrays write into a caller-owned buffer and return the size they need
 * (snprintf style: pass NULL / 0 to ask first). The engine hands out no "valid until the next call" pointers; the only
 * returned pointers are strings inside a Meta struct and crucible_version, valid for the life of the engine / process.
 *
 * MIDI: the CrucibleMidiEvent array passed to a process call must be sorted by position (ascending). At most
 * CRUCIBLE_MAX_EVENTS_PER_BLOCK events per call are used; the rest are dropped, so split very long blocks.
 */
#ifndef CRUCIBLE_ENGINE_C_API_H
#define CRUCIBLE_ENGINE_C_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Symbol export: CRUCIBLE_API (crucible_export.h) marks every function below. Linking the static library: define
 * CRUCIBLE_STATIC (the CMake target crucible_engine does it for you). Building a shared library from the engine sources:
 * define CRUCIBLE_BUILD_SHARED. Using a shared library: define nothing (dllimport on Windows). */
#include "crucible_export.h"

#define CRUCIBLE_API_VERSION 1
#define CRUCIBLE_MAX_EVENTS_PER_BLOCK 1024 /* MIDI events used per process call; further events are dropped */

    typedef struct CrucibleEngine CrucibleEngine;

    /* Set struct_size to sizeof(CrucibleConfig). A struct_size of 0, or one that ends before a field, gives that field its
     * default; new fields are only ever appended. */
    typedef struct CrucibleConfig
    {
        size_t struct_size;
        int max_voices;          /* 1..64, 0 = default (16) */
        int max_block_size;      /* 0 = default (512) */
        int quality;             /* 0 draft, 1 standard, 2 high */
        uint32_t seed;           /* 0 = default */
        int preload_wavetables;  /* 1: generate all factory wavetables (about 130 MB) at creation; default 0 */
        int oversampling_filter; /* 0 linear-phase half-band FIR (default), 1 all-pass polyphase IIR */
    } CrucibleConfig;

    /* [any] Fills config with every default and sets struct_size, so only the fields you change need touching. */
    CRUCIBLE_API void crucible_config_init(CrucibleConfig* config);

    /* Self-description of one parameter of one engine instance (strings stay valid for the life of the engine). */
    typedef struct CrucibleParamMeta
    {
        size_t struct_size; /* set to sizeof(CrucibleParamMeta) before the call; the engine fills the fields that fit */
        const char* id;
        const char* name;
        const char* short_name; /* at most 6 ASCII characters */
        const char* unit;
        const char* module;
        const char* group; /* "section/page" */
        const char* page_id;
        int type; /* 0 float, 1 int, 2 choice, 3 bool */
        float min, max, default_value, interval, skew;
        int steps; /* distinct values, 0 = continuous */
        int num_choices;
        int page;
        int smoothing;   /* 0 none, 1 linear, 2 log */
        int polarity;    /* 1 unipolar, 2 bipolar */
        int ui_hint;     /* 1 knob, 2 slider, 3 switch, 4 menu, 5 xy, 6 drawbar */
        int text_format; /* TextFormat of ParameterMap.h */
        int modulatable, voice_mod, automatable, scene;
        int routable;     /* accepts assignable modulation routes (generic or fixed voice destination) */
        int since;        /* module version that introduced it */
        uint32_t host_id; /* stable number for plug-in hosts (CLAP / VST3 / AU parameter id): 32-bit FNV-1a of id with
                             the top bit cleared (31 bits: never in VST3's reserved range, never 0xFFFFFFFF, rarely 0).
                             Independent of the index and identical in every product that has the parameter; the
                             compiler refuses a product whose ids collide after the masking. */
        int exposed;      /* 1 when a host lists it: on the product's surface (see crucible_surface_count) */
    } CrucibleParamMeta;

    typedef struct CrucibleModuleMeta
    {
        size_t struct_size; /* set to sizeof(CrucibleModuleMeta) before the call; the engine fills the fields that fit */
        const char* id;
        const char* name;
        const char* kind; /* voice, oscillator, filter, effect, modulator, performance, sequencer */
        const char* category;
        const char* description;
        int version;
        int cpu; /* 0 light, 1 medium, 2 heavy */
        int num_params;
        int num_inputs, num_outputs;
        int legacy;                /* 1 when a newer module replaces this one (kept for existing patches) */
        const char* superseded_by; /* id of that module, "" when not legacy */
    } CrucibleModuleMeta;

    typedef struct CruciblePortMeta
    {
        const char* name;
        const char* type; /* audio.mono, audio.stereo, mod, trigger */
    } CruciblePortMeta;

    typedef struct CrucibleMidiEvent
    {
        int position; /* sample offset in the block */
        uint8_t status, data1, data2;
    } CrucibleMidiEvent;

    /* A parameter change delivered with a block (natural units, see crucible_process_events). The events of one call must be
     * sorted by position (ascending); a position is clamped to [0, num_samples - 1]. */
    typedef struct CrucibleParamEvent
    {
        int position; /* sample offset in the block; currently ignored: changes apply at the start of the block */
        int index;
        float value;
    } CrucibleParamEvent;

    /* Host tempo and transport. Set struct_size to sizeof(CrucibleTransport); fields beyond it take their defaults (unknown
     * tempo, no position, stopped). A struct_size of 0 means "no transport" (the same as passing NULL), without error. */
    typedef struct CrucibleTransport
    {
        size_t struct_size;
        double bpm; /* <= 0: unknown */
        double ppq; /* position in quarter notes, < 0: unknown */
        int playing;
    } CrucibleTransport;

    typedef struct CrucibleMeter
    {
        float peak_left_db, peak_right_db;
        int clip;
    } CrucibleMeter;

    /* [any] */
    CRUCIBLE_API int crucible_api_version(void);
    /* [any] The library version, "major.minor.patch" (the CMake project version). Not the ABI: see crucible_api_version().
     */
    CRUCIBLE_API const char* crucible_version(void);

    /* [main] config may be NULL for defaults. Returns NULL on failure. The engine plays the process default product. */
    CRUCIBLE_API CrucibleEngine* crucible_create(const CrucibleConfig* config);
    /* [main] A new engine that plays a product definition: JSON of format "crucible-product" (see crucible/graph/Product.h
     * and docs/products/template.json), the one way to build an instrument from data. Every engine owns its product, so
     * engines with different products can coexist in one process. config may be NULL for defaults. modules is the registry
     * of third-party modules the product may use (crucible_module.h), or NULL for the built-in ones. Returns NULL on failure
     * and writes the reason (the validation errors, one per line) to error_buffer (may be NULL). */
    struct crucible_module_registry;
    CRUCIBLE_API CrucibleEngine* crucible_create_from_product(const CrucibleConfig* config, const char* product_json,
                                                              const struct crucible_module_registry* modules,
                                                              char* error_buffer, size_t error_size);
    /* [main, not concurrent with process] */
    CRUCIBLE_API void crucible_destroy(CrucibleEngine* engine);
    /* [main, not concurrent with process] Call before the first process call and whenever the sample rate changes. */
    CRUCIBLE_API void crucible_prepare(CrucibleEngine* engine, double sample_rate, int max_block_size);

    /* Parameters (natural units). Index-based calls are the fast path; ids are stable across versions. These answer for
     * the engine's own product. The metadata calls below are [any]: the parameter list never changes after creation. */
    /* [any] */
    CRUCIBLE_API int crucible_param_count(const CrucibleEngine* engine);
    /* [any] -1 when unknown or id is NULL. A binary search over the immutable id table: no allocation, no lock. Resolving
     * ids once and using indices is still faster. */
    CRUCIBLE_API int crucible_param_index(const CrucibleEngine* engine, const char* id);
    /* [any] Index of the parameter whose CrucibleParamMeta.host_id is host_id, or -1. A table lookup, safe to call per host
     * event. */
    CRUCIBLE_API int crucible_param_index_by_host_id(const CrucibleEngine* engine, uint32_t host_id);
    /* [any] 1 on success; 0 for a bad index or a struct_size of 0. */
    CRUCIBLE_API int crucible_param_meta(const CrucibleEngine* engine, int index, CrucibleParamMeta* out);
    /* [any] The name of choice `choice`, or NULL. */
    CRUCIBLE_API const char* crucible_param_choice(const CrucibleEngine* engine, int index, int choice);
    /* [any] The parameters a plug-in host lists, in the product's declared order (its "surface"; a product without one lists
     * every automatable parameter in index order). crucible_surface_param returns the parameter index for position k, or -1.
     * Hidden parameters still load from patches and stay modulation targets. */
    CRUCIBLE_API int crucible_surface_count(const CrucibleEngine* engine);
    CRUCIBLE_API int crucible_surface_param(const CrucibleEngine* engine, int k);
    /* [any] Choice numbers are the same in every product: a choice a product leaves out (an oscillator type it does not
     * include) keeps its number and stays in the list, and the numbers after it do not move. crucible_choice_available
     * returns 1 when choice k of a choice parameter can be used in this product, 0 when the product omits it, k is outside
     * the list, the parameter is not a choice, or the index is bad. Setting an unavailable choice is accepted and stored
     * (patches that name it still load and save), but it does nothing: an oscillator slot set to an omitted type is silent.
     * Use crucible_param_meta (choice count) and crucible_value_to_text for the choice names. */
    CRUCIBLE_API int crucible_choice_available(const CrucibleEngine* engine, int param_index, int choice);
    /* [main] Display text of a natural value. Writes at most buffer_size bytes (NUL terminated) and returns the length of
     * the full text, or -1 for a bad index. */
    CRUCIBLE_API int crucible_value_to_text(const CrucibleEngine* engine, int index, float value, char* buffer,
                                            size_t buffer_size);
    /* [main] Value of a text ("440 Hz", "1.5k", "-6dB", "C#4", "on"); 1 on success. */
    CRUCIBLE_API int crucible_text_to_value(const CrucibleEngine* engine, int index, const char* text, float* value);
    /* [any] The module calls (count, meta, param, input, output) describe the product and never change after creation. */
    CRUCIBLE_API int crucible_module_count(const CrucibleEngine* engine);
    CRUCIBLE_API int
    crucible_module_meta(const CrucibleEngine* engine, int module,
                         CrucibleModuleMeta* out); /* 1 on success; 0 for a bad index or a struct_size of 0 */
    /* Index (into the engine's parameters) of the k-th parameter of a module, or -1. */
    CRUCIBLE_API int crucible_module_param(const CrucibleEngine* engine, int module, int k);
    CRUCIBLE_API int crucible_module_input(const CrucibleEngine* engine, int module, int k, CruciblePortMeta* out);
    CRUCIBLE_API int crucible_module_output(const CrucibleEngine* engine, int module, int k, CruciblePortMeta* out);
    /* [main] The whole product as JSON (see crucible/Introspection.h), written to buffer (NUL terminated, at most capacity
     * bytes; buffer may be NULL). Returns the length of the full text, or -1 on failure: when it exceeds capacity - 1, call
     * again with a larger buffer. */
    CRUCIBLE_API int crucible_describe_json(const CrucibleEngine* engine, char* buffer, size_t capacity);
    /* [main] Set a parameter (natural units, clamped). The setters may build lazily generated data (a wavetable, an
     * oscillator), which is why they are not [any]; on the audio thread use CrucibleParamEvent with crucible_process_events.
     * 1 on success, 0 for a bad index / id. */
    CRUCIBLE_API int crucible_set_param(CrucibleEngine* engine, int index, float value);
    CRUCIBLE_API int crucible_set_param_id(CrucibleEngine* engine, const char* id, float value);
    /* [main] Sets a parameter from display text, as crucible_text_to_value reads it; 1 on success, 0 for an unknown id or
     * text that does not parse (nothing is changed). Choice text may be the exact name, the name in any letter case, or the
     * name without its trailing "(...)" ("analog" for "Analog (VA)"); a short form that fits more than one choice is
     * refused. */
    CRUCIBLE_API int crucible_set_param_text(CrucibleEngine* engine, const char* id, const char* text);
    /* [main] */
    CRUCIBLE_API int crucible_set_param_normalised(CrucibleEngine* engine, int index, float value01);
    /* [any] The stored value (what the last setter or process event wrote). */
    CRUCIBLE_API float crucible_get_param(const CrucibleEngine* engine, int index);
    /* [any] The value after global modulation, as of the last process call. */
    CRUCIBLE_API float crucible_modulated_value(const CrucibleEngine* engine, int index);

    /* Performance input. The six note / bend calls are [any]: they push into a lock-free single-producer queue that the
     * next process call drains, so call them from one thread at a time (and not alongside crucible_midi). 1 on success,
     * 0 when rejected (bad note, non-finite value, queue full). For sample-accurate input pass events to a process call. */
    CRUCIBLE_API int crucible_note_on(CrucibleEngine* engine, int note, float velocity01);
    CRUCIBLE_API int crucible_note_off(CrucibleEngine* engine, int note);
    CRUCIBLE_API int crucible_pitch_bend(CrucibleEngine* engine, float normalised);
    CRUCIBLE_API int crucible_mod_wheel(CrucibleEngine* engine, float value01);
    CRUCIBLE_API int crucible_sustain(CrucibleEngine* engine, int down);
    CRUCIBLE_API int crucible_all_notes_off(CrucibleEngine* engine);
    /* [any] Panic: every voice fades out within about 5 ms (no release tail), and the held-note stack, the sustain pedal,
     * the arpeggiator's held keys and the sequencer's sounding notes are cleared. Same queue as crucible_note_on, so it
     * is ordered with the notes sent before it; takes effect at the start of the next process call. Effect tails keep
     * ringing. Returns 1 when queued, 0 when the queue is full. */
    CRUCIBLE_API int crucible_kill_all_voices(CrucibleEngine* engine);
    /* [main] A raw MIDI channel-voice message. Unlike the calls above it may apply a mapped CC parameter directly (and so
     * build wavetables), hence [main]; it feeds the same single-producer queue. */
    CRUCIBLE_API int crucible_midi(CrucibleEngine* engine, uint8_t status, uint8_t data1, uint8_t data2);
    /* [main] Maps a MIDI CC to a parameter; param_index -1 removes the mapping. */
    CRUCIBLE_API void crucible_map_cc(CrucibleEngine* engine, int cc, int param_index);

    /* [audio] left / right are overwritten. events may be NULL; if given they must be sorted by position and at most
     * CRUCIBLE_MAX_EVENTS_PER_BLOCK are used. */
    CRUCIBLE_API void crucible_process(CrucibleEngine* engine, float* left, float* right, int num_samples,
                                       const CrucibleMidiEvent* events, int num_events);
    /* [audio] Same with host tempo / transport (ppq < 0 means no position). */
    CRUCIBLE_API void crucible_process_host(CrucibleEngine* engine, float* left, float* right, int num_samples,
                                            const CrucibleMidiEvent* events, int num_events, double bpm, double ppq,
                                            int playing);

    /* [audio] The plug-in entry point: MIDI events, parameter changes and the host transport in one call. params may be
     * NULL; transport may be NULL or have struct_size 0 (no tempo, no position, stopped). The parameter events must be
     * sorted by position (ascending) and are applied in order at the start of the block through the engine's real-time
     * parameter path: no allocation, no locks. Their position is clamped to [0, num_samples - 1] and otherwise reserved:
     * sample accurate application will come through this same signature, so fill it in correctly (0 is fine today). Int,
     * choice and bool values are rounded to the nearest whole number, exactly as crucible_set_param does. Changes to
     * oscillator type, wavetable or sequencer pattern are armed by the next crucible_service call. MIDI events must be
     * sorted by position; events beyond CRUCIBLE_MAX_EVENTS_PER_BLOCK are dropped. */
    CRUCIBLE_API void crucible_process_events(CrucibleEngine* engine, float* left, float* right, int num_samples,
                                              const CrucibleMidiEvent* events, int num_events,
                                              const CrucibleParamEvent* params, int num_params,
                                              const CrucibleTransport* transport);
    /* [main] Performs the setup work that real-time parameter changes deferred (cheap when there is none). */
    CRUCIBLE_API void crucible_service(CrucibleEngine* engine);
    /* [any] 1 when crucible_process_events, a mapped CC or another real-time parameter change left work for crucible_service
     * (so a wrapper can ask its host for a main-thread callback, or an offline render can call it between blocks); 0 when
     * nothing waits. One atomic load. */
    CRUCIBLE_API int crucible_service_pending(const CrucibleEngine* engine);

/* Engine-initiated changes and plug-in essentials. */
#define CRUCIBLE_CHANGE_LATENCY (-1) /* pseudo index in crucible_param_changes: latency changed */
#define CRUCIBLE_CHANGE_TAIL (-2)    /* pseudo index in crucible_param_changes: the tail changed */
#define CRUCIBLE_CHANGE_ROUTES (-3)  /* pseudo index in crucible_param_changes: the modulation route list changed */
#define CRUCIBLE_SOURCE_HOST 0       /* crucible_set_param*, crucible_process_events parameter events, set-all */
#define CRUCIBLE_SOURCE_MIDI 1       /* a mapped MIDI CC or CC learn (crucible_midi, or a CC in a process call) */
#define CRUCIBLE_SOURCE_INTERNAL                                                                                            \
    2 /* patch load, scene recall / morph, reset to defaults, sequencer length / pattern coupling,                          \
         latency and tail */
    /* [main] Drains the parameters that changed since the previous call into indices[] and sources[] (cap entries each;
     * sources may be NULL), in index order, and returns how many were written. The set is lock-free and allocation-free; the
     * audio thread (process_events, mapped CCs) and [main] setters mark it. sources[k] says who made the latest change of
     * indices[k] (CRUCIBLE_SOURCE_*): a wrapper skips CRUCIBLE_SOURCE_HOST entries, which the host already knows, and
     * reports the rest (CLAP_EVENT_PARAM_VALUE out of process, VST3 performEdit). A parameter changed twice between polls
     * appears once with the latest source. Only exposed parameters are reported (crucible_param_meta().exposed): an internal
     * change to a hidden parameter is dropped, so every index returned has a host id the host has seen.
     * Entries beyond cap stay for the next call. If crucible_latency_samples changed
     * since the previous call, the first entry is index CRUCIBLE_CHANGE_LATENCY with source CRUCIBLE_SOURCE_INTERNAL: ask
     * for the new value and tell the host (CLAP latency changed, VST3 restartComponent). If crucible_tail_seconds changed,
     * an entry with index CRUCIBLE_CHANGE_TAIL and the same source follows it (CLAP tail changed, VST3 getTailSamples). If
     * the modulation route list changed (crucible_add_route*, crucible_remove_route, a patch load or scene recall that
     * leaves a different list), an entry with index CRUCIBLE_CHANGE_ROUTES and source CRUCIBLE_SOURCE_INTERNAL follows:
     * re-read crucible_route_count / crucible_route_get. It is reported whoever made the change, including the caller's own
     * add / remove; a call that leaves the list as it was (the same values again, a patch with the same routes) does not
     * report it. Call
     * it from a timer or after each process call; it costs one atomic load when nothing changed. */
    CRUCIBLE_API int crucible_param_changes(CrucibleEngine* engine, int* indices, int* sources, int cap);
    /* [main] Latency of the whole engine in samples: the largest sum of the latencies that the enabled effect modules on a
     * path to the master report (parallel lanes take the larger one; voices add none). Changes with module settings (a
     * lookahead or linear-phase mode, a module switched on or off). */
    CRUCIBLE_API int crucible_latency_samples(const CrucibleEngine* engine);
    /* [any] Longest tail in seconds among the enabled effect modules that report one (reverb decay, delay feedback,
     * convolution length); HUGE_VAL when one never decays (freeze). Voice release time is not included. For CLAP tail / VST3
     * getTailSamples. A cached value, published like the latency (at prepare, after parameter changes and after each block):
     * one atomic load. Changes are reported through crucible_param_changes as CRUCIBLE_CHANGE_TAIL. */
    CRUCIBLE_API double crucible_tail_seconds(const CrucibleEngine* engine);
    /* [audio] Clears voices, held notes, queued notes and every effect's state (delay lines, reverb tanks, convolution
     * history, filters) so the next block starts silent. Allocates nothing and takes no lock, so it may run on the audio
     * thread between process calls; never concurrently with a process call. Does not re-prepare. Call it on activate, on a
     * transport jump and on bypass. Parameters, patches, scenes and loaded content are kept. */
    CRUCIBLE_API void crucible_reset(CrucibleEngine* engine);
    /* [any] 1 while the host renders offline (CLAP render mode offline, VST3 kOffline, a bounce or a test render), 0 for
     * real time again (the default). fx.convolution and fx.cabinet build their kernels on a background thread; in real
     * time the audio thread never waits for it, so the block at which a new impulse response (crucible_load_ir) or cabinet
     * takes effect depends on thread timing. While offline is set they wait on the audio thread for every build, at any
     * quality, and a render is the same on every run and machine (quality 2, high, always waits). Never set it for live
     * playback: a block may then block for the length of a build. Takes effect from the next process call. */
    CRUCIBLE_API void crucible_set_render_offline(CrucibleEngine* engine, int offline);
    CRUCIBLE_API int crucible_render_offline(const CrucibleEngine* engine);

    /* [audio] Same with a sidechain input (the vocoder's modulator): aux_left / aux_right hold num_samples samples each,
     * aux_right may be NULL for a mono input, aux_left NULL means no input. bpm <= 0 and ppq < 0 mean unknown. */
    CRUCIBLE_API void crucible_process_aux(CrucibleEngine* engine, float* left, float* right, const float* aux_left,
                                           const float* aux_right, int num_samples, const CrucibleMidiEvent* events,
                                           int num_events, double bpm, double ppq, int playing);
    /* [audio] crucible_process_events with a sidechain input: MIDI events, parameter events, the host transport and the
     * aux buffers in one call, for a plug-in with a sidechain bus. Everything crucible_process_events says about events,
     * params and transport holds; aux_left / aux_right are as in crucible_process_aux (aux_left NULL: no input, aux_right
     * NULL: mono). With aux_left NULL it is exactly crucible_process_events. */
    CRUCIBLE_API void crucible_process_events_aux(CrucibleEngine* engine, float* left, float* right,
                                                  const float* aux_left, const float* aux_right, int num_samples,
                                                  const CrucibleMidiEvent* events, int num_events,
                                                  const CrucibleParamEvent* params, int num_params,
                                                  const CrucibleTransport* transport);

    /* [any] */
    CRUCIBLE_API CrucibleMeter crucible_meter(const CrucibleEngine* engine);

    /* Views: GUI-agnostic read-back (see crucible/Views.h and docs/ARCHITECTURE.md, "Views"). A view is {instance, id} with
     * a typed payload of 32-bit elements. Subscribe on [main] (the audio thread then publishes it at most once per
     * crucible_process call and no faster than max_hz); read from [any] thread. A view nobody subscribes to costs nothing.
     */
    enum
    {
        CRUCIBLE_VIEW_SCALAR = 0, /* 1 float */
        CRUCIBLE_VIEW_VECTOR = 1, /* up to max_elements floats */
        CRUCIBLE_VIEW_POINTS = 2, /* up to max_elements floats, tuples of `stride` */
        CRUCIBLE_VIEW_MASK = 3    /* up to max_elements 32-bit words; read with crucible_view_read_mask */
    };

    typedef struct CrucibleViewInfo
    {
        size_t struct_size;   /* set to sizeof(CrucibleViewInfo) before the call; the engine fills the fields that fit */
        const char* instance; /* "master", "params", "osc1"... (valid for the life of the engine) */
        const char* id;
        const char* description;
        int kind;         /* CRUCIBLE_VIEW_* */
        int max_elements; /* payload capacity in 32-bit elements (4 bytes each): the capacity crucible_view_read needs */
        int stride;       /* floats per point (points), else 1 */
        int live;         /* 1: published by the audio thread; 0: computed by the reader */
    } CrucibleViewInfo;

    /* [main] Number of registered views, and the description of the k-th (1 on success; 0 for a bad index or a struct_size
     * of 0). */
    CRUCIBLE_API int crucible_view_count(const CrucibleEngine* engine);
    CRUCIBLE_API int crucible_view_info(const CrucibleEngine* engine, int view, CrucibleViewInfo* out);
    /* [main] Payload capacity of a view in 32-bit elements (the same as max_elements of crucible_view_info; 4 * this many
     * bytes): size the buffer for crucible_view_read / crucible_view_read_mask with it. -1 for an unknown view. */
    CRUCIBLE_API int crucible_view_size(const CrucibleEngine* engine, int view);
    /* Index of {instance, id} or -1. "params" / "<parameter id>.voice@<n>" (the value under voice n) is created on demand.
     * [main] */
    CRUCIBLE_API int crucible_view_find(CrucibleEngine* engine, const char* instance, const char* id);
    /* Starts publishing a view at most max_hz times per second (<= 0: every process call). Returns a subscription id (> 0)
     * or 0 on failure. [main] */
    CRUCIBLE_API int crucible_view_subscribe(CrucibleEngine* engine, int view, float max_hz);
    /* [main] */
    CRUCIBLE_API void crucible_view_unsubscribe(CrucibleEngine* engine, int subscription);
    /* Copies the newest consistent snapshot into buffer. Returns the number of elements (0 before the first publication or
     * when nobody subscribed), -1 for an unknown view, -2 when capacity is too small. *sequence (may be NULL) receives the
     * publication counter: it changes whenever the payload may have changed. [any] for a pushed view (live == 1 in
     * crucible_view_info: the audio thread publishes it); [main] for a pulled view (live == 0: the reader computes it and
     * locks briefly; never read one from the audio thread). The pulled views are "master/spectrum_<size>",
     * "osc<n>/sample_info", "sample_peaks", "sample_slices", "sample_zones", "cloud_points", and the "response" and
     * "response_mod" views of the filters. */
    CRUCIBLE_API int crucible_view_read(const CrucibleEngine* engine, int view, float* buffer, int capacity,
                                        uint64_t* sequence);
    /* [any] The same for a mask view (bits 32 * k .. 32 * k + 31 in word k). */
    CRUCIBLE_API int crucible_view_read_mask(const CrucibleEngine* engine, int view, uint32_t* words, int capacity,
                                             uint64_t* sequence);

    typedef struct CrucibleLoudness
    {
        float momentary_lufs, short_term_lufs, true_peak_db, max_true_peak_db, correlation;
    } CrucibleLoudness;

    /* [any] */
    CRUCIBLE_API CrucibleLoudness crucible_loudness(const CrucibleEngine* engine);
    /* [any] */
    CRUCIBLE_API void crucible_reset_clip(CrucibleEngine* engine);
    /* [any] */
    CRUCIBLE_API void crucible_set_analyzer_enabled(CrucibleEngine* engine, int enabled);
    /* [main] Most recent count samples per channel (count <= 4096); returns the number written. */
    CRUCIBLE_API int crucible_scope(const CrucibleEngine* engine, float* left, float* right, int count);
    /* [main] Magnitude spectrum in dB of the most recent fft_size samples (power of two, 256..4096); writes fft_size / 2 + 1
     * bins and returns that count, or 0 when bins_capacity is too small. */
    CRUCIBLE_API int crucible_spectrum(const CrucibleEngine* engine, int fft_size, float* bins, int bins_capacity);

    /* Patches (JSON text, see crucible/Patch.h). All [main]; loading is safe while the audio thread
     * processes (parameters are atomics, content is swapped in), but the sound changes mid-block, so load between notes when
     * you can. error_buffer may be NULL.
     * crucible_save_patch writes the patch to buffer (NUL terminated, at most capacity bytes; buffer may be NULL) and
     * returns the length of the full text, or -1 on failure; when it exceeds capacity - 1 call again with a larger buffer.
     */
    CRUCIBLE_API int crucible_load_patch(CrucibleEngine* engine, const char* json_text, char* error_buffer,
                                         size_t error_size);
    CRUCIBLE_API int crucible_save_patch(CrucibleEngine* engine, const char* name, char* buffer, size_t capacity);
    CRUCIBLE_API int crucible_load_patch_file(CrucibleEngine* engine, const char* path, char* error_buffer,
                                              size_t error_size);

/* User content. All [main] (they decode files and allocate); safe while the audio thread processes. */
/* What a wavetable load did. Set struct_size to sizeof(CrucibleWavetableInfo) (fields beyond it are left alone). A user
 * table is always CRUCIBLE_WAVETABLE_FRAMES frames of 2048 samples: fewer source frames are crossfaded to fill it, more are
 * decimated (frames_used of the frames_in contribute, evenly spaced, the rest are dropped). There are
 * CRUCIBLE_USER_WAVETABLES slots. */
#define CRUCIBLE_WAVETABLE_FRAMES 64
#define CRUCIBLE_USER_WAVETABLES 8
    typedef struct CrucibleWavetableInfo
    {
        size_t struct_size;
        int frames_in;   /* complete source frames found in the data */
        int frames_used; /* source frames that shape the table: min(frames_in, CRUCIBLE_WAVETABLE_FRAMES) */
        int decimated;   /* 1 when frames_in exceeded CRUCIBLE_WAVETABLE_FRAMES and frames were dropped */
        int frame_size;  /* samples per source frame that were used */
    } CrucibleWavetableInfo;
    /* [main] A user wavetable from a WAV file into slot 0..CRUCIBLE_USER_WAVETABLES-1 (selected with osc*_table = 14 +
     * slot). The frame size comes from the file ('clm ' marker) or is detected. Returns 1 on success; 0 on a bad slot or
     * file, with the reason in error (may be NULL). info (may be NULL) reports the frame count and any decimation. */
    CRUCIBLE_API int crucible_load_wavetable_file(CrucibleEngine* engine, int slot, const char* wav_path,
                                                  CrucibleWavetableInfo* info, char* error, size_t error_size);
    /* [main] The same from memory: frame_count frames of frame_size mono samples each, one after another (frame_size >= 4,
     * one single-cycle waveform per frame, resampled spectrally to 2048). name may be NULL. Normalised as one table. Returns
     * 1 on success; 0 on a bad slot, frame_size or empty data, with the reason in error. */
    CRUCIBLE_API int crucible_load_wavetable(CrucibleEngine* engine, int slot, const float* frames, int frame_count,
                                             int frame_size, const char* name, CrucibleWavetableInfo* info, char* error,
                                             size_t error_size);
    /* One pitch range of a zoned user wavetable: a full set of source frames (frame_count frames of frame_size mono samples,
     * the rules of crucible_load_wavetable) and max_hz, the highest fundamental (Hz) the zone is meant for. Set struct_size to
     * sizeof(CrucibleWavetableZone). */
    typedef struct CrucibleWavetableZone
    {
        size_t struct_size;
        const float* frames;
        int frame_count;
        int frame_size;
        double max_hz;
    } CrucibleWavetableZone;
    /* [main] A user wavetable whose source changes with pitch, so formants fixed in Hz survive across the keyboard: 1..16
     * zones, max_hz finite, positive and strictly ascending (the last zone also covers everything above its max_hz). Each
     * of the 27 mip levels is band-limited from the zone that contains the level's representative fundamental (the
     * geometric centre of the notes that read it) at the engine's sample rate; the table is rebuilt on the control thread
     * when the engine is prepared at another rate. One gain normalises the whole table (zones keep their relative level).
     * One zone equals crucible_load_wavetable. Returns 1 on success; 0 on a bad slot, zone or order, with the reason in error
     * (may be NULL). info (may be NULL) describes the lowest zone. Saved in patches (zone sources and max_hz). */
    CRUCIBLE_API int crucible_load_wavetable_zones(CrucibleEngine* engine, int slot, const CrucibleWavetableZone* zones,
                                                   int zone_count, const char* name, CrucibleWavetableInfo* info,
                                                   char* error, size_t error_size);
    /* One formant of a vowel: centre frequency (Hz), -3 dB bandwidth (Hz, 0 = derive the damping from the filter's
     * resonance as the built-in vowels do) and level in dB. */
    typedef struct CrucibleFormant
    {
        float freq_hz;
        float bandwidth_hz;
        float gain_db;
    } CrucibleFormant;
    /* [main] A user vowel table for the Formant filter model of filter_slot (0-based; every voice), so any instrument can
     * define its own vowels. formants holds vowel_count (2..8) x formant_count (1..4) entries, row-major per vowel.
     * freq_hz must be finite within 20..20000, bandwidth_hz finite and >= 0, gain_db finite and <= +24; output_gain is the
     * linear gain of the summed formants (finite, > 0; the built-in table uses 1.4). The filter cutoff moves along the
     * vowels (position m = (vowel_count - 1) * log(clamp(cutoff, 20, 20000) / 20) / log(1000), blending the two
     * neighbours: frequency and bandwidth geometrically, gain linearly in dB). Each formant is a unity-peak two-pole
     * band-pass summed in parallel and scaled by 10^(gain_db / 20); with bandwidth_hz > 0 its Q is
     * max(freq_hz / bandwidth_hz * 0.5 * 16^resonance, 0.3) (resonance 0..1), with 0 the damping the built-in vowels use
     * (1 - 0.93 resonance + 0.07). NULL formants restores the built-in vowels (bit-identical to an engine never given a
     * table). The filter response view (filterN/response) follows the table. Saved in patches. Returns 1 on success; 0 on a
     * bad slot or table, with the reason in error (may be NULL). */
    CRUCIBLE_API int crucible_set_formant_table(CrucibleEngine* engine, int filter_slot, const CrucibleFormant* formants,
                                                int vowel_count, int formant_count, float output_gain, char* error,
                                                size_t error_size);
    /* [main] Switches the Hardware-style state saturation of the user formant table of filter_slot on (non-zero) or off. On, every
     * formant's band-pass integrator state is soft-clipped (softClip(x) = clamp(x, -3, 3) * (27 + x^2) / (27 + 9 x^2)) and
     * its low-pass state clamped to +-8, so loud input compresses and thickens like the legacy vowel filter; the output is
     * still the unity-peak band-pass sum, so quiet signals are unchanged. Off (the default) is the linear filter, bit for
     * bit. The built-in vowels are never saturated (this fails without a user table), and calling
     * crucible_set_formant_table again turns it off. The filterN/response view stays the small-signal (linear) response.
     * Saved in patches. Returns 1 on success; 0 on a bad slot or a slot without a user table, with the reason in error (may
     * be NULL). */
    CRUCIBLE_API int crucible_set_formant_saturation(CrucibleEngine* engine, int filter_slot, int on, char* error,
                                                     size_t error_size);
    /* Loads an impulse response file into one module instance, named by its module id from crucible_module_meta:
     * "fx.convolution" (WAV or AIFF, 1, 2 or 4 (true-stereo LL, LR, RL, RR) channels, up to 30 s, any sample rate),
     * "fx.convolution.2" for a second instance in a graph product. Returns 1
     * on success; 0 for an unknown instance, a module that takes no impulse response or an invalid file, with the reason in
     * error (may be NULL). */
    CRUCIBLE_API int crucible_load_ir(CrucibleEngine* engine, const char* instance_id, const char* path, char* error,
                                      size_t error_size);
    /* Removes the impulse response of one module instance (same instance ids as crucible_load_ir).
     * Returns 1 on success; 0 for an unknown instance or a module without a clearable impulse response. */
    CRUCIBLE_API int crucible_clear_ir(CrucibleEngine* engine, const char* instance_id);
    /* One point of a waveshaper curve: x and y within -1..1, curve (-1..1) bends the segment from this point to the next
     * (0 = straight; the last point's curve is unused). */
    typedef struct CrucibleShaperPoint
    {
        float x, y, curve;
    } CrucibleShaperPoint;
    /* [main] The transfer curve of one waveshaper instance (fx.shaper; "fx.shaper.2" for a second instance in a graph
     * product): count (2..64) points with x strictly ascending, y and curve within -1..1, all finite. A segment from
     * (x0, y0) to (x1, y1) with curvature c is y = y0 + (y1 - y0) * t^(4^c), t = (x - x0) / (x1 - x0): 0 is the straight
     * line, positive bows towards the start value, negative towards the end value. Outside the first and last point the
     * curve holds the end value (it clips there). With symmetric non-zero the points draw the positive half (x within
     * 0..1, y = 0 at x = 0; a first point right of 0 gets an implicit (0, 0) before it) and the curve is mirrored, odd:
     * f(-x) = -f(x). The default curve (also after points = NULL with count 0) is the identity y = x, unbounded, and with 0 dB
     * drive the module then passes the input through bit for bit. The curve is published to the audio thread as an immutable
     * table and cross-faded in over 10 ms; the "curve" view of the instance ("fx.shaper/curve", 257 values at x = -1..1)
     * returns it for drawing. Saved in patches. Returns 1 on success; 0 for an unknown instance, a module without a curve or
     * an invalid curve (the curve stays as it was), with the reason in error (may be NULL). */
    CRUCIBLE_API int crucible_set_shaper_curve(CrucibleEngine* engine, const char* instance_id,
                                               const CrucibleShaperPoint* points, int count, int symmetric, char* error,
                                               size_t error_size);
    /* The curve last set on the instance, as given: copies up to max_points points to points (may be NULL to ask for the
     * count only) and stores the symmetric flag (may be NULL). Returns the number of points of the curve, or 0 for an unknown
     * instance or while the curve is the default identity. */
    CRUCIBLE_API int crucible_get_shaper_curve(CrucibleEngine* engine, const char* instance_id, CrucibleShaperPoint* points,
                                               int max_points, int* symmetric);
    /* Sample / granular oscillator audio for an oscillator slot (0-based): interleaved float frames, or a WAV file.
     * crucible_set_cloud takes the text form of a cloud (CloudModel::serialise()). All return 1 on success. Sample values are
     * clamped to -16..16 on import (non-finite values become 0), also for crucible_add_zone_sample; ordinary audio is within
     * -1..1 and unchanged. A sample_rate of 0 or less is read as 48000. */
    CRUCIBLE_API int crucible_load_sample(CrucibleEngine* engine, int slot, const float* interleaved, int frames,
                                          int channels, double sample_rate, const char* name);
    CRUCIBLE_API int crucible_load_sample_file(CrucibleEngine* engine, int slot, const char* wav_path);
    CRUCIBLE_API void crucible_clear_sample(CrucibleEngine* engine, int slot);
    /* Slice markers (positions 0..1 along the sample, any order) for the oscillator's slice modes. Loading a sample removes
     * its markers. set returns 1 on success; get writes up to capacity markers and returns how many there are; the two
     * slicing helpers return the number of slices (0 when the slot holds no sample). */
    CRUCIBLE_API int crucible_set_sample_slices(CrucibleEngine* engine, int slot, const float* markers, int count);
    CRUCIBLE_API int crucible_get_sample_slices(CrucibleEngine* engine, int slot, float* markers, int capacity);
    CRUCIBLE_API int crucible_slice_sample_equally(CrucibleEngine* engine, int slot, int slices);
    CRUCIBLE_API int crucible_slice_sample_transients(CrucibleEngine* engine, int slot, int max_slices, float sensitivity);
    /* Multisample zones of a slot (osc*_smp_zone_mode = Zones). Zone sample id 0 is the slot's own sample, ids 1.. are added
     * with crucible_add_zone_sample (returns the id, 0 on failure). set replaces the zone list (at most 64 zones; fields are
     * clamped); get writes up to capacity zones and returns how many there are. The importers return the number of zones
     * (0 on failure). */
    typedef struct CrucibleZone
    {
        int sample_id;
        int key_lo, key_hi, vel_lo, vel_hi;
        float root, gain_db, tune_cents, loop_start, loop_end;
        int loop_mode; /* -1 slot loop parameters, 0 off, 1 forward, 2 ping-pong */
        int group;
    } CrucibleZone;
    CRUCIBLE_API int crucible_add_zone_sample(CrucibleEngine* engine, int slot, const float* interleaved, int frames,
                                              int channels, double sample_rate, const char* name);
    CRUCIBLE_API int crucible_set_zones(CrucibleEngine* engine, int slot, const CrucibleZone* zones, int count);
    CRUCIBLE_API int crucible_get_zones(CrucibleEngine* engine, int slot, CrucibleZone* zones, int capacity);
    CRUCIBLE_API int crucible_zone_sample_count(CrucibleEngine* engine, int slot);
    CRUCIBLE_API void crucible_clear_zones(CrucibleEngine* engine, int slot);
    CRUCIBLE_API int crucible_import_zone_folder(CrucibleEngine* engine, int slot, const char* directory);
    CRUCIBLE_API int crucible_import_zone_sfz(CrucibleEngine* engine, int slot, const char* sfz_path);
    CRUCIBLE_API int crucible_set_cloud(CrucibleEngine* engine, int slot, const char* cloud_text);

    /* [main] Assignable modulation: source is a ModSource value or 1000 + MIDI CC number. amount is clamped to -1..1. This
     * call adds a route with the default polarity, no curve and no smoothing; crucible_add_route_ex sets them. A route with
     * the same source and target is replaced. 1 on success, 0 when refused (unknown source or target, a cycle, too many
     * routes), or when amount is not finite. */
    CRUCIBLE_API int crucible_add_route(CrucibleEngine* engine, int source, const char* target_id, float amount);
    typedef struct CrucibleRoute
    {
        size_t struct_size; /* sizeof(CrucibleRoute); fields beyond it take the defaults below */
        int source;
        const char* target_id;
        float amount;       /* -1..1 */
        int polarity;       /* 0 source native (default), 1 unipolar, 2 bipolar; clamped */
        float curve;        /* -1..1, 0 linear (default); clamped */
        float smoothing_ms; /* one-pole smoothing of the route output, 0..2000, 0 none (default); clamped */
    } CrucibleRoute;
    /* [main] crucible_add_route with polarity, curve and smoothing. */
    CRUCIBLE_API int crucible_add_route_ex(CrucibleEngine* engine, const CrucibleRoute* route);
    CRUCIBLE_API int crucible_remove_route(CrucibleEngine* engine, int source, const char* target_id);
    /* [main] Route read-back. crucible_route_count is the number of routes. crucible_route_get(k) fills *out with route k,
     * 0 <= k < count, and returns 1; 0 for a k out of range, NULL arguments or an out->struct_size of 0 (set it to
     * sizeof(CrucibleRoute): like the other structs the engine fills, only the fields that fit are written). target_id
     * stays valid for the lifetime of the engine handle.
     * Identity: k is the position in creation order. crucible_add_route* appends a new source/target pair at the end and
     * replaces an existing pair in place, so a route keeps its k while it exists and while routes after it come and go.
     * crucible_remove_route(source, target) removes the route whose k crucible_route_get reports for that pair, and every
     * later route moves down by one; so does the "route:<k>:amount" target of a route that modulates another route's
     * amount. A patch load replaces the list. The pair (source, target_id) is the stable key; re-read the list when
     * crucible_param_changes reports CRUCIBLE_CHANGE_ROUTES. */
    CRUCIBLE_API int crucible_route_count(CrucibleEngine* engine);
    CRUCIBLE_API int crucible_route_get(CrucibleEngine* engine, int k, CrucibleRoute* out);

    /* Key lock, [main] unless tagged (scale quantiser; parameters key_mode, key_root, key_scale, key_mask, key_tie,
     * key_sync). Key Sync lets engines follow one key through a KeyBus. The core provides the in-process bus; a host that
     * shares the key between processes supplies its own transport (see KeyBus.h and CApiInterop.h). */
    typedef struct CrucibleKeyBus CrucibleKeyBus;
    /* A private in-process bus (instances in one process can share it). */
    CRUCIBLE_API CrucibleKeyBus* crucible_keybus_open_local(void);
    CRUCIBLE_API void crucible_keybus_close(CrucibleKeyBus* bus);
    /* [any] mask: 12 bits, bit 0 = root. Returns the new generation. Last writer wins. */
    CRUCIBLE_API uint32_t crucible_keybus_publish(CrucibleKeyBus* bus, int root, int mask, int tie_up);
    /* [any] generation 0 = nothing published yet. Returns 1 on success. */
    CRUCIBLE_API int crucible_keybus_read(const CrucibleKeyBus* bus, int* root, int* mask, int* tie_up,
                                          uint32_t* generation);
    /* The engine keeps its own reference; the handle can be closed afterwards. */
    CRUCIBLE_API int crucible_attach_keybus(CrucibleEngine* engine, CrucibleKeyBus* bus);
    /* Read-only scale view: mode 0 off / 1 snap / 2 block / 3 remap; in_key[12] receives 0/1 per pitch class (C = 0). */
    CRUCIBLE_API int crucible_key_scale(const CrucibleEngine* engine, int* mode, int* root, int in_key[12], int* following);
    /* Key detection from the notes played so far. Returns 1 when there is a proposal. */
    CRUCIBLE_API int crucible_key_detect(CrucibleEngine* engine, int* root, int* minor, float* confidence);
    CRUCIBLE_API void crucible_key_detect_reset(CrucibleEngine* engine);

#ifdef __cplusplus
}
#endif

#endif
