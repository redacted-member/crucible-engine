/* SPDX-License-Identifier: MIT */

/* WASM spike: linear-phase EQ and convolution on the control thread. Enables both in the default product, loads a generated
 * impulse response, plays a note and times crucible_service() (where the kernels are built when there are no threads) and
 * crucible_process_events(). Prints a few output energies so that runs can be compared.
 *
 *   threads <ir.wav>
 */

#include <crucible/crucible.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum
{
    kRate = 48000,
    kBlock = 128,
    kBlocks = 600
};

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static void le16(FILE* f, int v)
{
    fputc(v & 255, f);
    fputc((v >> 8) & 255, f);
}
static void le32(FILE* f, int v)
{
    le16(f, v);
    le16(f, v >> 16);
}

static void writeIr(const char* path, double seconds)
{
    const int frames = (int)(seconds * kRate);
    FILE* f = fopen(path, "wb");
    fwrite("RIFF", 1, 4, f);
    le32(f, 36 + frames * 4);
    fwrite("WAVEfmt ", 1, 8, f);
    le32(f, 16);
    le16(f, 1);
    le16(f, 2);
    le32(f, kRate);
    le32(f, kRate * 4);
    le16(f, 4);
    le16(f, 16);
    fwrite("data", 1, 4, f);
    le32(f, frames * 4);
    unsigned seed = 12345;
    for (int i = 0; i < frames; ++i)
        for (int c = 0; c < 2; ++c)
        {
            seed = seed * 1664525u + 1013904223u;
            const double noise = ((seed >> 8) / 8388608.0) - 1.0;
            le16(f, (int)(noise * exp(-4.0 * i / frames) * 12000.0));
        }
    fclose(f);
}

int main(int argc, char** argv)
{
    const char* irPath = argc > 1 ? argv[1] : "ir.wav";
    const double irSeconds = argc > 2 ? atof(argv[2]) : 1.0;
    writeIr(irPath, irSeconds);
    CrucibleConfig config;
    crucible_config_init(&config);
    config.max_block_size = kBlock;
    CrucibleEngine* engine = crucible_create(&config);
    if (engine == NULL)
        return 4;
    crucible_prepare(engine, kRate, kBlock);
    crucible_service(engine);
    char error[512] = {0};
    const int loaded = crucible_load_ir(engine, "fx.convolution", irPath, error, sizeof error);
    fprintf(stderr, "load_ir=%d %s\n", loaded, error);
    crucible_set_param_id(engine, "fx_ep_on", 1.0f);
    crucible_set_param_id(engine, "fx_ep_phase", 2.0f);
    crucible_set_param_id(engine, "fx_ep_b2_on", 1.0f);
    crucible_set_param_id(engine, "fx_ep_b2_gain", 6.0f);
    crucible_set_param_id(engine, "fx_cx_on", 1.0f);
    crucible_set_param_id(engine, "fx_cx_mix", 1.0f);
    float left[kBlock], right[kBlock];
    double svcMax = 0, svcSum = 0, procMax = 0, procSum = 0, energy[4] = {0, 0, 0, 0};
    int svcCalls = 0;
    for (int b = 0; b < kBlocks; ++b)
    {
        CrucibleMidiEvent ev[1];
        int n = 0;
        if (b == 0 || b == 300)
        {
            ev[0].position = 0;
            ev[0].status = b == 0 ? 0x90 : 0x80;
            ev[0].data1 = 48;
            ev[0].data2 = 100;
            n = 1;
        }
        double s = now_ms();
        crucible_service(engine);
        double t = now_ms() - s;
        svcSum += t;
        ++svcCalls;
        if (t > 0.5)
            fprintf(stderr, "  service call at block %d took %.2f ms\n", b, t);
        if (t > svcMax)
            svcMax = t;
        memset(left, 0, sizeof left);
        memset(right, 0, sizeof right);
        s = now_ms();
        crucible_process_events(engine, left, right, kBlock, ev, n, NULL, 0, NULL);
        t = now_ms() - s;
        procSum += t;
        if (b > 20 && t > procMax)
            procMax = t;
        for (int i = 0; i < kBlock; ++i)
            energy[b * 4 / kBlocks] += left[i] * left[i];
    }
    fprintf(stderr,
            "service: mean %.3f max %.3f ms (%d calls)\nprocess: mean %.4f max(after warmup) %.4f ms (budget 2.667)\n",
            svcSum / svcCalls, svcMax, svcCalls, procSum / kBlocks, procMax);
    fprintf(stderr, "energy per quarter: %.4f %.4f %.4f %.4f\n", energy[0], energy[1], energy[2], energy[3]);
    crucible_destroy(engine);
    return 0;
}
