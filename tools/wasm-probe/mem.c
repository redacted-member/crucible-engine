/* SPDX-License-Identifier: MIT */

/* WASM spike: heap use (malloc in use and total wasm memory) after creation, prepare, and after each factory wavetable is
 * touched, with lazy wavetables (default) or preload (arg "preload"). Prints per-table generation time.
 *
 *   mem [preload]
 */

#include <crucible/crucible.h>

#include <emscripten.h>
#include <emscripten/heap.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static void report(const char* what)
{
    struct mallinfo mi = mallinfo();
    fprintf(stderr, "%-28s in-use %7.1f MB   wasm memory %7.1f MB\n", what, mi.uordblks / 1048576.0,
            emscripten_get_heap_size() / 1048576.0);
}

int main(int argc, char** argv)
{
    CrucibleConfig config;
    crucible_config_init(&config);
    config.max_block_size = 128;
    config.preload_wavetables = argc > 1 && strcmp(argv[1], "preload") == 0;
    report("start");
    double t = now_ms();
    CrucibleEngine* engine = crucible_create(&config);
    fprintf(stderr, "create: %.0f ms\n", now_ms() - t);
    report("after create");
    t = now_ms();
    crucible_prepare(engine, 48000.0, 128);
    crucible_service(engine);
    fprintf(stderr, "prepare+service: %.0f ms\n", now_ms() - t);
    report("after prepare");
    const int index = crucible_param_index(engine, "osc1_table");
    CrucibleParamMeta meta;
    meta.struct_size = sizeof(meta);
    if (index < 0 || !crucible_param_meta(engine, index, &meta))
        return 3;
    const int tables = (int)meta.max + 1;
    fprintf(stderr, "osc1_table choices: %d\n", tables);
    double worst = 0, sum = 0;
    for (int i = 0; i < tables; ++i)
    {
        t = now_ms();
        crucible_set_param(engine, index, (float)i);
        crucible_service(engine);
        t = now_ms() - t;
        sum += t;
        if (t > worst)
            worst = t;
        char label[64];
        snprintf(label, sizeof label, "table %d (%.1f ms)", i, t);
        report(label);
    }
    fprintf(stderr, "tables touched one by one: mean %.1f ms, worst %.1f ms, total %.0f ms\n", sum / tables, worst, sum);
    report("after all tables");
    crucible_destroy(engine);
    return 0;
}
