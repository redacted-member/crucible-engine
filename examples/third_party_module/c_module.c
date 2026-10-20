/* SPDX-License-Identifier: MIT */
/*
 * A module written in plain C against crucible_module.h: a tanh saturator for the FX graph. It links against nothing of
 * the engine; the host finds it through crucible_get_modules after loading the shared library.
 *
 * Build it as a shared library and check it:
 *   crucible-module-check libcrucible_example_c_module.so
 */
#include <crucible/crucible_module.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* The ABI version this file declares. The build overrides it (-DEXAMPLE_ABI_VERSION=...) to make a library the host must
 * refuse; leave it alone in a real module. */
#ifndef EXAMPLE_ABI_VERSION
#define EXAMPLE_ABI_VERSION CRUCIBLE_MODULE_ABI_VERSION
#endif

enum
{
    PARAM_DRIVE,
    PARAM_MIX,
    PARAM_LEVEL,
    PARAM_COUNT
};

static const crucible_param_spec drive_param = {sizeof(crucible_param_spec),
                                                EXAMPLE_ABI_VERSION,
                                                "drive",
                                                "Drive",
                                                "Drive",
                                                "dB",
                                                CRUCIBLE_PARAM_FLOAT,
                                                0,
                                                0.0f,
                                                36.0f,
                                                12.0f,
                                                0.0f,
                                                1.0f,
                                                0,
                                                NULL};
static const crucible_param_spec mix_param = {sizeof(crucible_param_spec),
                                              EXAMPLE_ABI_VERSION,
                                              "mix",
                                              "Mix",
                                              "Mix",
                                              NULL,
                                              CRUCIBLE_PARAM_FLOAT,
                                              CRUCIBLE_PARAM_MODULATABLE,
                                              0.0f,
                                              1.0f,
                                              1.0f,
                                              0.0f,
                                              1.0f,
                                              0,
                                              NULL};
static const crucible_param_spec level_param = {sizeof(crucible_param_spec),
                                                EXAMPLE_ABI_VERSION,
                                                "level",
                                                "Level",
                                                "Level",
                                                "dB",
                                                CRUCIBLE_PARAM_FLOAT,
                                                0,
                                                -24.0f,
                                                12.0f,
                                                0.0f,
                                                0.0f,
                                                1.0f,
                                                0,
                                                NULL};

static const crucible_param_spec* const param_table[PARAM_COUNT] = {&drive_param, &mix_param, &level_param};

/* One instance: a DC blocker per channel (a saturator that is not symmetric adds offset). */
typedef struct
{
    float x1[2], y1[2];
} saturator;

static void* sat_create(const crucible_module_descriptor* descriptor, const crucible_module_host* host)
{
    (void)descriptor;
    (void)host;
    return calloc(1, sizeof(saturator));
}

static void sat_destroy(void* instance)
{
    free(instance);
}

static void sat_reset(void* instance)
{
    memset(instance, 0, sizeof(saturator));
}

static int32_t sat_prepare(void* instance, const crucible_module_prepare* info)
{
    (void)info;
    sat_reset(instance);
    return 1;
}

static void sat_process(void* instance, const crucible_module_process* data)
{
    saturator* s = (saturator*)instance;
    const float gain = powf(10.0f, data->params[PARAM_DRIVE] * 0.05f);
    const float mix = data->params[PARAM_MIX];
    const float level = powf(10.0f, data->params[PARAM_LEVEL] * 0.05f);
    const float norm = 1.0f / tanhf(gain * 0.5f);
    float* channel[2] = {data->audio_out[0].left, data->audio_out[0].right};
    for (int c = 0; c < 2; ++c)
    {
        float* x = channel[c];
        float x1 = s->x1[c], y1 = s->y1[c];
        for (int i = 0; i < data->num_samples; ++i)
        {
            const float dry = x[i];
            const float wet = tanhf(gain * dry) * norm * 0.5f;
            const float blocked = wet - x1 + 0.995f * y1;
            x1 = wet;
            y1 = blocked;
            x[i] = (dry + (blocked - dry) * mix) * level;
        }
        s->x1[c] = x1;
        s->y1[c] = y1;
    }
}

static const crucible_module_descriptor saturator_descriptor = {sizeof(crucible_module_descriptor),
                                                                EXAMPLE_ABI_VERSION,
                                                                "acme.c_saturator",
                                                                "C Saturator",
                                                                "Acme",
                                                                "Distortion",
                                                                "A tanh saturator with a DC blocker, written in C.",
                                                                CRUCIBLE_MODULE_KIND_EFFECT,
                                                                1,
                                                                CRUCIBLE_MODULE_CPU_LIGHT,
                                                                0,
                                                                NULL,
                                                                PARAM_COUNT,
                                                                param_table,
                                                                0,
                                                                NULL,
                                                                0,
                                                                NULL,
                                                                sat_create,
                                                                sat_destroy,
                                                                sat_prepare,
                                                                sat_reset,
                                                                sat_process,
                                                                NULL,
                                                                NULL,
                                                                NULL,
                                                                NULL,
                                                                NULL,
                                                                NULL};

static const crucible_module_descriptor* const modules[] = {&saturator_descriptor};

static const crucible_module_library library = {sizeof(crucible_module_library), EXAMPLE_ABI_VERSION, 1, modules};

CRUCIBLE_MODULE_EXPORT const crucible_module_library* crucible_get_modules(void)
{
    return &library;
}
