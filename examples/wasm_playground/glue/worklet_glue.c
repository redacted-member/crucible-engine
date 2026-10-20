/* SPDX-License-Identifier: MIT */

/* Thin wrapper for the AudioWorklet: one engine, 128-sample blocks, fixed output buffers in wasm memory. */

#include <crucible/crucible.h>

#include <emscripten.h>
#include <string.h>

enum
{
    kBlock = 128
};

static CrucibleEngine* engine;
static float left[kBlock];
static float right[kBlock];
static CrucibleMidiEvent pending[32];
static int pendingCount;

EMSCRIPTEN_KEEPALIVE int cw_init(int sampleRate)
{
    CrucibleConfig config;
    crucible_config_init(&config);
    config.max_block_size = kBlock;
    engine = crucible_create(&config);
    if (engine == NULL)
        return 0;
    crucible_prepare(engine, (double)sampleRate, kBlock);
    crucible_service(engine);
    return 1;
}

EMSCRIPTEN_KEEPALIVE void cw_midi(int status, int data1, int data2)
{
    if (pendingCount >= 32)
        return;
    CrucibleMidiEvent* e = &pending[pendingCount++];
    e->position = 0;
    e->status = (unsigned char)status;
    e->data1 = (unsigned char)data1;
    e->data2 = (unsigned char)data2;
}

/* Renders one block into cw_left() / cw_right(). The service call runs every 16th block (control-thread work such as
 * wavetable and kernel builds happens there when there are no background threads). */
EMSCRIPTEN_KEEPALIVE void cw_process(void)
{
    static int counter;
    memset(left, 0, sizeof left);
    memset(right, 0, sizeof right);
    crucible_process_events(engine, left, right, kBlock, pending, pendingCount, NULL, 0, NULL);
    pendingCount = 0;
    if ((++counter & 15) == 0)
        crucible_service(engine);
}

EMSCRIPTEN_KEEPALIVE float* cw_left(void)
{
    return left;
}
EMSCRIPTEN_KEEPALIVE float* cw_right(void)
{
    return right;
}
