/* SPDX-License-Identifier: MIT */

/* Instrument skeleton: the whole plug-in lifecycle on the Crucible C API, with a fake host in place of CLAP / VST3.
 *
 *   instrument_skeleton skeleton_product.json
 *
 * Copy this file and the product JSON to start a real instrument. The functions named after host callbacks
 * (on_activate, process, main_thread_tick, gui_timer, save_state, load_state) are the part that moves into your
 * wrapper; main() and the fake_host_* functions stand in for the host and are thrown away. Each function says which
 * thread the host calls it on. Exits 0 when every check passes. */

#include <crucible/crucible.h>

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    kSampleRate = 48000,
    kBlockSize = 256,
    kMaxHostParams = 64,
    kStateCapacity = 1 << 20
};

/* One entry of the parameter list the host shows. Built once from the product's surface. */
typedef struct HostParam
{
    uint32_t host_id; /* the number a CLAP / VST3 host stores in sessions; stable across versions */
    char name[64];
    char page[32];
    float min, max, default_value;
    int engine_index; /* the fast-path index for crucible_set_param and parameter events */
} HostParam;

typedef struct Plugin
{
    CrucibleEngine* engine;
    HostParam params[kMaxHostParams];
    int num_params;
    int meter_view, meter_subscription;
    int cutoff_mod_view, cutoff_mod_subscription;
    int cutoff_index;
    int response_view, response_size; /* the filter's frequency response curve, drawn by a GUI */
    int latency;                      /* the latency the host was last told */
    int notified_params;              /* parameter changes the wrapper would send to the host */
    int notified_latency;
    int notified_tail;
} Plugin;

static int g_failures = 0;

static void check(int ok, const char* what)
{
    printf("  [%s] %s\n", ok ? "ok" : "FAIL", what);
    if (!ok)
        ++g_failures;
}

/* ---- Plug-in side: what you copy ------------------------------------------------------------------------------- */

/* Host main thread, at instantiation. Builds the engine and the host parameter list. */
static int plugin_create(Plugin* plugin, const char* product_json)
{
    CrucibleConfig config;
    crucible_config_init(&config);
    config.max_voices = 8;

    char error[1024] = "";
    plugin->engine = crucible_create_from_product(&config, product_json, NULL, error, sizeof error);
    if (plugin->engine == NULL)
    {
        fprintf(stderr, "product rejected:\n%s\n", error);
        return 0;
    }

    /* The surface is the list of parameters the product wants a host to show, in page order. */
    plugin->num_params = 0;
    for (int k = 0; k < crucible_surface_count(plugin->engine) && k < kMaxHostParams; ++k)
    {
        const int index = crucible_surface_param(plugin->engine, k);
        CrucibleParamMeta meta;
        meta.struct_size = sizeof meta; /* the engine fills the fields that fit this size */
        if (!crucible_param_meta(plugin->engine, index, &meta))
            continue;
        HostParam* hp = &plugin->params[plugin->num_params++];
        hp->host_id = meta.host_id;
        snprintf(hp->name, sizeof hp->name, "%s", meta.name);
        snprintf(hp->page, sizeof hp->page, "%s", meta.page_id);
        hp->min = meta.min;
        hp->max = meta.max;
        hp->default_value = meta.default_value;
        hp->engine_index = index;
    }
    plugin->cutoff_index = crucible_param_index(plugin->engine, "filter_cutoff");
    return 1;
}

/* Host main thread, when the host activates the plug-in (sample rate and maximum block size are known). */
static void on_activate(Plugin* plugin, double sample_rate, int max_block_size)
{
    crucible_prepare(plugin->engine, sample_rate, max_block_size);
    crucible_reset(plugin->engine); /* silent start: no voices, no effect tails from a previous run */

    /* Tell the host what it needs to compensate and to keep the tail alive. The latency changes with module settings:
     * main_thread_tick sees that as a change entry and reports it again. */
    plugin->latency = crucible_latency_samples(plugin->engine);
    printf("  latency %d samples, tail %.2f s\n", plugin->latency, crucible_tail_seconds(plugin->engine));

    /* Views are read-back for a GUI. Subscribe once; the audio thread then publishes at most 30 times a second. */
    plugin->meter_view = crucible_view_find(plugin->engine, "master", "meter");
    plugin->meter_subscription = crucible_view_subscribe(plugin->engine, plugin->meter_view, 30.0f);
    /* The value of a parameter after modulation: what a GUI knob shows moving with the envelope or an LFO. */
    plugin->cutoff_mod_view = crucible_view_find(plugin->engine, "params", "filter_cutoff.mod");
    plugin->cutoff_mod_subscription = crucible_view_subscribe(plugin->engine, plugin->cutoff_mod_view, 30.0f);
    /* The filter response is computed by the reader when asked, so it needs no subscription. crucible_view_size is the
     * buffer size to read it with. */
    plugin->response_view = crucible_view_find(plugin->engine, "filter", "response");
    plugin->response_size = crucible_view_size(plugin->engine, plugin->response_view);
}

/* Host audio thread, once per block. Never allocates or locks. The host's note and parameter events have already
 * been turned into CrucibleMidiEvent and CrucibleParamEvent (index is the engine index, see HostParam). */
static void process(Plugin* plugin, float* left, float* right, int num_samples, const CrucibleMidiEvent* midi, int num_midi,
                    const CrucibleParamEvent* params, int num_params, const CrucibleTransport* transport)
{
    crucible_process_events(plugin->engine, left, right, num_samples, midi, num_midi, params, num_params, transport);
}

/* Host main thread, from the host's idle callback or a timer. Finishes work the audio thread asked for: a changed
 * oscillator type, wavetable or sequencer pattern takes effect after this call. Then reports the changes the engine
 * made itself (patch load, MIDI CC, scene) to the host. Cheap when there is nothing to do. */
static void main_thread_tick(Plugin* plugin)
{
    crucible_service(plugin->engine);

    int indices[kMaxHostParams], sources[kMaxHostParams];
    int count;
    while ((count = crucible_param_changes(plugin->engine, indices, sources, kMaxHostParams)) > 0)
    {
        for (int k = 0; k < count; ++k)
        {
            if (indices[k] == CRUCIBLE_CHANGE_LATENCY)
            {
                /* CLAP: host->latency->changed; VST3: restartComponent(kLatencyChanged). */
                plugin->latency = crucible_latency_samples(plugin->engine);
                ++plugin->notified_latency;
            }
            else if (indices[k] == CRUCIBLE_CHANGE_TAIL)
            {
                /* CLAP: the tail extension's changed callback; VST3: the host asks getTailSamples again.
                 * crucible_tail_seconds is a cached read, safe from any thread. */
                ++plugin->notified_tail;
            }
            else if (sources[k] != CRUCIBLE_SOURCE_HOST)
            {
                /* The host made the host-sourced changes itself and already knows them. For the others send a parameter
                 * value event out of process (CLAP) or performEdit (VST3), by indices[k] -> host_id. */
                ++plugin->notified_params;
            }
        }
    }
}

/* Host GUI timer. Reading values and pushed (live) views is safe from any thread; the pulled filter response view is
 * computed by the reader and may only be read from the main thread, which a GUI timer is in this skeleton. Returns the
 * number of points of the filter curve it drew. */
static int gui_timer(const Plugin* plugin)
{
    float meter[3] = {0.0f, 0.0f, 0.0f}; /* peak left dB, peak right dB, clip */
    uint64_t sequence = 0;
    const int meter_count = crucible_view_read(plugin->engine, plugin->meter_view, meter, 3, &sequence);

    float cutoff_modulated[1] = {0.0f};
    crucible_view_read(plugin->engine, plugin->cutoff_mod_view, cutoff_modulated, 1, &sequence);

    /* The filter curve: 256 values in dB across the audible range, drawn as the response graph. */
    float response[256];
    float response_peak = -1000.0f;
    int response_count = 0;
    if (plugin->response_size > 0 && plugin->response_size <= 256)
    {
        response_count = crucible_view_read(plugin->engine, plugin->response_view, response, 256, &sequence);
        for (int i = 0; i < response_count; ++i)
            response_peak = fmaxf(response_peak, response[i]);
    }

    char text[64];
    crucible_value_to_text(plugin->engine, plugin->cutoff_index, cutoff_modulated[0], text, sizeof text);
    printf("  gui: meter %s%.1f / %.1f dB, cutoff now %s, filter curve %d points peaking %.1f dB\n",
           meter_count == 3 ? "" : "(none yet) ", meter[0], meter[1], text, response_count, response_peak);
    return response_count;
    return response_count;
}

/* Host main thread, when the host asks for the plug-in state. Writes the patch JSON into the host's buffer.
 * Returns the number of bytes written (without the terminator), or 0 when it does not fit. */
static size_t save_state(Plugin* plugin, char* buffer, size_t capacity)
{
    const int length = crucible_save_patch(plugin->engine, "Skeleton state", buffer, capacity);
    if (length < 0 || (size_t)length + 1 > capacity)
        return 0;
    return (size_t)length;
}

/* Host main thread, when the host restores a saved state. Returns 1 on success. */
static int load_state(Plugin* plugin, const char* json)
{
    char error[512] = "";
    const int ok = crucible_load_patch(plugin->engine, json, error, sizeof error);
    if (!ok)
        fprintf(stderr, "patch rejected: %s\n", error);
    return ok;
}

/* Host main thread, at destruction. */
static void plugin_destroy(Plugin* plugin)
{
    crucible_view_unsubscribe(plugin->engine, plugin->meter_subscription);
    crucible_view_unsubscribe(plugin->engine, plugin->cutoff_mod_subscription);
    crucible_destroy(plugin->engine);
    plugin->engine = NULL;
}

/* ---- Fake host: stands in for CLAP / VST3 and is thrown away ---------------------------------------------------- */

static char* read_text_file(const char* path)
{
    FILE* f = fopen(path, "rb");
    if (f == NULL)
        return NULL;
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* text = (char*)calloc((size_t)size + 1, 1);
    if (text != NULL && fread(text, 1, (size_t)size, f) != (size_t)size)
    {
        free(text);
        text = NULL;
    }
    fclose(f);
    return text;
}

/* A tiny mono 16-bit WAV: a decaying burst of noise, long enough to be a convolution impulse response. */
static int write_test_ir(const char* path)
{
    enum
    {
        kFrames = 2400
    };
    FILE* f = fopen(path, "wb");
    if (f == NULL)
        return 0;
    const uint32_t data_bytes = kFrames * 2;
    const uint32_t header[] = {36 + data_bytes, 16};
    const uint16_t format[] = {1, 1}; /* PCM, mono */
    const uint32_t rates[] = {kSampleRate, kSampleRate * 2};
    const uint16_t block[] = {2, 16};
    fwrite("RIFF", 1, 4, f);
    fwrite(&header[0], 4, 1, f);
    fwrite("WAVEfmt ", 1, 8, f);
    fwrite(&header[1], 4, 1, f);
    fwrite(format, 2, 2, f);
    fwrite(rates, 4, 2, f);
    fwrite(block, 2, 2, f);
    fwrite("data", 1, 4, f);
    fwrite(&data_bytes, 4, 1, f);
    uint32_t seed = 12345;
    for (int i = 0; i < kFrames; ++i)
    {
        seed = seed * 1664525u + 1013904223u;
        const float noise = ((float)(seed >> 8) / 8388608.0f) - 1.0f;
        const int16_t sample = (int16_t)(noise * expf(-6.0f * (float)i / kFrames) * 20000.0f);
        fwrite(&sample, 2, 1, f);
    }
    return fclose(f) == 0;
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        fprintf(stderr, "usage: instrument_skeleton skeleton_product.json\n");
        return 2;
    }
    char* product_json = read_text_file(argv[1]);
    if (product_json == NULL)
    {
        fprintf(stderr, "cannot read %s\n", argv[1]);
        return 2;
    }
    printf("Crucible Engine %s, instrument skeleton\n", crucible_version());

    Plugin plugin;
    memset(&plugin, 0, sizeof plugin);

    printf("create and activate\n");
    check(plugin_create(&plugin, product_json), "engine built from the product JSON");
    free(product_json);
    if (plugin.engine == NULL)
        return 1;
    on_activate(&plugin, kSampleRate, kBlockSize);

    check(plugin.num_params == 15, "the surface lists 15 parameters");
    int ids_nonzero = 1;
    for (int i = 0; i < plugin.num_params; ++i)
    {
        ids_nonzero = ids_nonzero && plugin.params[i].host_id != 0;
        printf("  param %2d: id %08x  %-18s page %-7s %g .. %g (default %g)\n", i, (unsigned)plugin.params[i].host_id,
               plugin.params[i].name, plugin.params[i].page, plugin.params[i].min, plugin.params[i].max,
               plugin.params[i].default_value);
    }
    check(ids_nonzero, "every host id is non-zero");

    /* Parameter changes from the host, resolved once to engine indexes. Text goes through the engine's own parser, so
     * "1.2k" and "Analog (VA)" mean the same in every product. */
    const int osc1_type = crucible_param_index(plugin.engine, "osc1_type");
    CrucibleParamMeta type_meta;
    type_meta.struct_size = sizeof type_meta;
    crucible_param_meta(plugin.engine, osc1_type, &type_meta);
    for (int k = 0; k < type_meta.num_choices; ++k)
        printf("  osc1_type choice %d: %s%s\n", k, crucible_param_choice(plugin.engine, osc1_type, k),
               crucible_choice_available(plugin.engine, osc1_type, k) ? "" : " (not in this product)");
    float cutoff_value = 0.0f;
    float type_value = 0.0f;
    check(crucible_text_to_value(plugin.engine, plugin.cutoff_index, "1.2k", &cutoff_value), "text \"1.2k\" parsed");
    check(crucible_text_to_value(plugin.engine, osc1_type, "Analog (VA)", &type_value), "text \"Analog (VA)\" parsed");
    check(crucible_choice_available(plugin.engine, osc1_type, (int)type_value) == 1,
          "the Analog oscillator is in this product");
    check(crucible_set_param_text(plugin.engine, "osc1_type", "analog") == 1 &&
              crucible_get_param(plugin.engine, osc1_type) == type_value,
          "short text \"analog\" sets the same choice");

    CrucibleParamMeta cutoff_meta;
    cutoff_meta.struct_size = sizeof cutoff_meta;
    crucible_param_meta(plugin.engine, plugin.cutoff_index, &cutoff_meta);
    check(crucible_param_index_by_host_id(plugin.engine, cutoff_meta.host_id) == plugin.cutoff_index,
          "host id maps back to the engine index");
    const CrucibleParamEvent param_events[] = {
        {0, plugin.cutoff_index, cutoff_value},
        {0, osc1_type, type_value},
    };

    printf("play (audio thread blocks, with the main-thread tick between them)\n");
    float left[kBlockSize], right[kBlockSize];
    float loudest = 0.0f;
    int curve_points = 0;
    for (int block = 0; block < 40; ++block)
    {
        CrucibleMidiEvent midi[2];
        int num_midi = 0;
        if (block == 0)
        {
            midi[num_midi++] = (CrucibleMidiEvent){10, 0x90, 60, 100}; /* note on, middle C */
            midi[num_midi++] = (CrucibleMidiEvent){10, 0x90, 67, 90};
        }
        if (block == 24)
            midi[num_midi++] = (CrucibleMidiEvent){0, 0x80, 60, 0}; /* note off */

        const CrucibleParamEvent* params = (block == 4) ? param_events : NULL;
        const int num_params = (block == 4) ? 2 : 0;

        CrucibleTransport transport;
        memset(&transport, 0, sizeof transport);
        transport.struct_size = sizeof transport;
        transport.bpm = 120.0;
        transport.ppq = (double)block * kBlockSize / kSampleRate * 2.0;
        transport.playing = 1;

        process(&plugin, left, right, kBlockSize, midi, num_midi, params, num_params, &transport);
        for (int i = 0; i < kBlockSize; ++i)
            loudest = fmaxf(loudest, fabsf(left[i]));

        if (block % 8 == 7)
        {
            main_thread_tick(&plugin); /* a real host calls this from its main thread, not from the audio callback */
            curve_points = gui_timer(&plugin);
        }
    }
    check(loudest > 0.01f, "the notes made sound");
    check(fabsf(crucible_get_param(plugin.engine, plugin.cutoff_index) - cutoff_value) < 1e-3f,
          "the cutoff event was applied");
    check(curve_points == plugin.response_size && curve_points == 256, "the GUI read a 256-point filter response curve");
    check(plugin.notified_params == 0, "the host's own parameter events were not echoed back");

    printf("changes the engine makes itself\n");
    /* A mapped MIDI CC moves a parameter: it arrives from the control surface, so the host has to be told. */
    crucible_map_cc(plugin.engine, 74, plugin.cutoff_index);
    crucible_midi(plugin.engine, 0xB0, 74, 20);
    main_thread_tick(&plugin);
    check(plugin.notified_params == 1, "a mapped MIDI CC was reported for the host");
    /* A modulation route with a curve and smoothing, added through the extended call. */
    CrucibleRoute route;
    memset(&route, 0, sizeof route);
    route.struct_size = sizeof route;
    route.source = 1000 + 1; /* mod wheel */
    route.target_id = "filter_res";
    route.amount = 0.3f;
    route.polarity = 1;
    route.curve = 0.4f;
    route.smoothing_ms = 25.0f;
    check(crucible_add_route_ex(plugin.engine, &route) == 1, "a mod wheel route with curve and smoothing added");

    /* The host jumped in time (loop, locate): flush voices and effect tails; the notes must not ring on. crucible_reset
     * allocates and locks nothing, so the audio thread may call it between process calls. */
    crucible_reset(plugin.engine);
    float silence = 0.0f;
    for (int block = 0; block < 4; ++block)
    {
        process(&plugin, left, right, kBlockSize, NULL, 0, NULL, 0, NULL);
        for (int i = 0; i < kBlockSize; ++i)
            silence = fmaxf(silence, fabsf(left[i]) + fabsf(right[i]));
    }
    check(silence == 0.0f, "after crucible_reset the next blocks are silent");

    printf("impulse response\n");
    check(write_test_ir("skeleton_ir.wav"), "test impulse response written");
    char ir_error[256] = "";
    check(crucible_load_ir(plugin.engine, "fx.convolution", "skeleton_ir.wav", ir_error, sizeof ir_error),
          "impulse response loaded into fx.convolution");
    if (ir_error[0] != '\0')
        printf("  reason: %s\n", ir_error);
    remove("skeleton_ir.wav");

    printf("state\n");
    char* state = (char*)malloc(kStateCapacity);
    const size_t state_size = state != NULL ? save_state(&plugin, state, kStateCapacity) : 0;
    check(state_size > 0, "patch saved");
    check(state_size > 0 && strstr(state, "convolutionIrs") != NULL,
          "the patch carries the impulse response of its convolution instance");

    Plugin second;
    memset(&second, 0, sizeof second);
    char* product_again = read_text_file(argv[1]);
    check(product_again != NULL && plugin_create(&second, product_again), "second engine built");
    free(product_again);
    if (second.engine != NULL && state_size > 0)
    {
        on_activate(&second, kSampleRate, kBlockSize);
        check(load_state(&second, state), "patch loaded into the second engine");
        main_thread_tick(&second);
        check(second.notified_params > 0, "the patch load was reported as engine-made changes");
        int same = crucible_param_count(plugin.engine) == crucible_param_count(second.engine);
        for (int i = 0; same && i < crucible_param_count(plugin.engine); ++i)
            same = crucible_get_param(plugin.engine, i) == crucible_get_param(second.engine, i);
        check(same, "every parameter value matches after the round trip");
        plugin_destroy(&second);
    }

    free(state);
    plugin_destroy(&plugin);
    printf(g_failures == 0 ? "all checks passed\n" : "%d check(s) failed\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
