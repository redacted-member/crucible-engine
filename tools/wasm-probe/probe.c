/* SPDX-License-Identifier: MIT */

/* Feasibility probe for the WASM spike: renders 2 s at 48 kHz in 128-sample blocks through the public C API, writes the
 * interleaved-free raw float32 output (left block then right block per block is NOT used: all left samples, then all
 * right samples) and prints the per-block time.
 *
 *   probe <product.json|default> <out.raw> [voices]     (voices: notes held, default 1)
 */

#include <crucible/crucible.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum
{
    kRate = 48000,
    kBlock = 128,
    kBlocks = 2 * kRate / kBlock + 1,
    kFrames = kBlocks * kBlock
};

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static int cmp_double(const void* a, const void* b)
{
    const double x = *(const double*)a, y = *(const double*)b;
    return (x > y) - (x < y);
}

int main(int argc, char** argv)
{
    if (argc < 3)
        return 2;
    const int voices = argc > 3 ? atoi(argv[3]) : 1;
    CrucibleConfig config;
    crucible_config_init(&config);
    config.max_block_size = kBlock;
    CrucibleEngine* engine = NULL;
    char error[2048] = {0};
    if (strcmp(argv[1], "default") == 0)
        engine = crucible_create(&config);
    else
    {
        FILE* f = fopen(argv[1], "rb");
        if (f == NULL)
            return 3;
        fseek(f, 0, SEEK_END);
        const long size = ftell(f);
        fseek(f, 0, SEEK_SET);
        char* text = (char*)calloc((size_t)size + 1, 1);
        if (fread(text, 1, (size_t)size, f) != (size_t)size)
            return 3;
        fclose(f);
        engine = crucible_create_from_product(&config, text, NULL, error, sizeof error);
        free(text);
    }
    if (engine == NULL)
    {
        fprintf(stderr, "create failed: %s\n", error);
        return 4;
    }
    const double t0 = now_ms();
    crucible_prepare(engine, kRate, kBlock);
    crucible_service(engine);
    fprintf(stderr, "prepare+service: %.1f ms\n", now_ms() - t0);

    float* left = (float*)calloc(kFrames, sizeof(float));
    float* right = (float*)calloc(kFrames, sizeof(float));
    double* times = (double*)calloc(kBlocks, sizeof(double));
    for (int b = 0; b < kBlocks; ++b)
    {
        CrucibleMidiEvent events[64];
        int n = 0;
        if (b == 0)
            for (int v = 0; v < voices && v < 32; ++v)
            {
                events[n].position = 0;
                events[n].status = 0x90;
                events[n].data1 = (uint8_t)(48 + v * 3);
                events[n].data2 = 100;
                ++n;
            }
        if (b == (3 * kRate / 2) / kBlock)
            for (int v = 0; v < voices && v < 32; ++v)
            {
                events[n].position = 0;
                events[n].status = 0x80;
                events[n].data1 = (uint8_t)(48 + v * 3);
                events[n].data2 = 0;
                ++n;
            }
        const double s = now_ms();
        crucible_process_events(engine, left + b * kBlock, right + b * kBlock, kBlock, events, n, NULL, 0, NULL);
        times[b] = now_ms() - s;
        if ((b & 15) == 0)
            crucible_service(engine);
    }
    double sum = 0, peak = 0;
    for (int i = 0; i < kBlocks; ++i)
    {
        sum += times[i];
        if (times[i] > peak)
            peak = times[i];
    }
    qsort(times, kBlocks, sizeof(double), cmp_double);
    const double budget = kBlock * 1000.0 / kRate;
    fprintf(stderr,
            "voices=%d blocks=%d mean=%.4f ms median=%.4f p99=%.4f max=%.4f ms (budget %.3f ms) mean/budget=%.1f%%\n",
            voices, kBlocks, sum / kBlocks, times[kBlocks / 2], times[kBlocks * 99 / 100], peak, budget,
            100.0 * sum / kBlocks / budget);
    FILE* out = fopen(argv[2], "wb");
    if (out == NULL)
        return 5;
    fwrite(left, sizeof(float), kFrames, out);
    fwrite(right, sizeof(float), kFrames, out);
    fclose(out);
    crucible_destroy(engine);
    free(left);
    free(right);
    free(times);
    return 0;
}
