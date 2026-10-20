/* SPDX-License-Identifier: MIT */

/* Renders one note through the C API and writes a 16-bit stereo WAV file.
 *
 *   render_c [output.wav] [midi-note]      (default: crucible_c_demo.wav, note 57)
 *
 * Exits with a non-zero status when the file cannot be written or the render is silent. */

#include <crucible/crucible.h>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

enum
{
    kRate = 48000,
    kBlock = 256
};

static void put16(FILE* f, uint16_t v)
{
    fputc(v & 0xff, f);
    fputc(v >> 8, f);
}

static void put32(FILE* f, uint32_t v)
{
    put16(f, (uint16_t)(v & 0xffff));
    put16(f, (uint16_t)(v >> 16));
}

static int write_wav(const char* path, const float* left, const float* right, uint32_t frames)
{
    FILE* f = fopen(path, "wb");
    if (f == NULL)
        return 0;
    const uint32_t data_bytes = frames * 4;
    fwrite("RIFF", 1, 4, f);
    put32(f, 36 + data_bytes);
    fwrite("WAVEfmt ", 1, 8, f);
    put32(f, 16);
    put16(f, 1); /* PCM */
    put16(f, 2); /* channels */
    put32(f, kRate);
    put32(f, kRate * 4);
    put16(f, 4);
    put16(f, 16);
    fwrite("data", 1, 4, f);
    put32(f, data_bytes);
    for (uint32_t i = 0; i < frames; ++i)
    {
        const float l = fmaxf(-1.0f, fminf(1.0f, left[i]));
        const float r = fmaxf(-1.0f, fminf(1.0f, right[i]));
        put16(f, (uint16_t)(int16_t)lrintf(l * 32767.0f));
        put16(f, (uint16_t)(int16_t)lrintf(r * 32767.0f));
    }
    return fclose(f) == 0;
}

int main(int argc, char** argv)
{
    const char* path = argc > 1 ? argv[1] : "crucible_c_demo.wav";
    const int note = argc > 2 ? atoi(argv[2]) : 57;
    const int seconds = 2;
    const uint32_t frames = (uint32_t)(kRate * seconds);

    if (crucible_api_version() != CRUCIBLE_API_VERSION)
    {
        fprintf(stderr, "header and library disagree on the API version\n");
        return 2;
    }
    printf("Crucible Engine %s\n", crucible_version());

    CrucibleEngine* engine = crucible_create(NULL);
    float* left = (float*)calloc(frames, sizeof(float));
    float* right = (float*)calloc(frames, sizeof(float));
    if (engine == NULL || left == NULL || right == NULL)
    {
        fprintf(stderr, "allocation failed\n");
        return 2;
    }
    crucible_prepare(engine, kRate, kBlock);
    crucible_set_param_id(engine, "filter_cutoff", 2400.0f);
    crucible_set_param_id(engine, "osc1_level", 0.8f);

    crucible_note_on(engine, note, 0.8f);
    for (uint32_t pos = 0; pos < frames; pos += kBlock)
    {
        const int n = (int)(frames - pos < kBlock ? frames - pos : kBlock);
        if (pos >= (uint32_t)(kRate * 3 / 2))
            crucible_note_off(engine, note);
        crucible_process(engine, left + pos, right + pos, n, NULL, 0);
    }

    float peak = 0.0f;
    for (uint32_t i = 0; i < frames; ++i)
        peak = fmaxf(peak, fmaxf(fabsf(left[i]), fabsf(right[i])));
    printf("peak %.4f\n", (double)peak);

    int status = 0;
    if (!write_wav(path, left, right, frames))
    {
        fprintf(stderr, "cannot write %s\n", path);
        status = 1;
    }
    else if (peak < 1.0e-3f)
    {
        fprintf(stderr, "the render is silent\n");
        status = 1;
    }
    else
    {
        printf("wrote %s\n", path);
    }
    crucible_destroy(engine);
    free(left);
    free(right);
    return status;
}
