// SPDX-License-Identifier: MIT
// AudioWorkletProcessor that runs the Crucible wasm engine. The main thread posts the wasm bytes, then note messages.
// Plain JS on purpose: the worklet file is loaded by addModule() and needs no tooling.

const WARM_UP_BLOCKS = 32; // absorbs the first-block spikes (memory growth, JIT tier-up) before audio starts

class CrucibleProcessor extends AudioWorkletProcessor {
    constructor() {
        super();
        this.api = null;
        this.blocks = 0;
        this.peak = 0;
        this.port.onmessage = (event) => this.onMessage(event.data);
    }

    onMessage(message) {
        if (message.type === 'wasm') {
            this.start(message.bytes);
        } else if (message.type === 'midi' && this.api) {
            this.api.cw_midi(message.status, message.data1, message.data2);
        }
    }

    start(bytes) {
        const t0 = currentTime;
        const instance = new WebAssembly.Instance(new WebAssembly.Module(bytes), {
            env: { emscripten_notify_memory_growth: () => {} },
            wasi_snapshot_preview1: {
                clock_time_get: (id, precision, out) => {
                    const view = new DataView(instance.exports.memory.buffer);
                    view.setBigUint64(out, BigInt(Math.round(currentTime * 1e9)), true);
                    return 0;
                },
            },
        });
        this.api = instance.exports;
        this.api._initialize();
        const ok = this.api.cw_init(sampleRate);
        for (let i = 0; i < WARM_UP_BLOCKS; i++) this.api.cw_process();
        this.port.postMessage({ type: ok ? 'ready' : 'failed', sampleRate, wasmBytes: bytes.byteLength, startTime: currentTime - t0 });
    }

    process(inputs, outputs) {
        const out = outputs[0];
        if (!this.api) return true;
        this.api.cw_process();
        const memory = this.api.memory.buffer; // re-read: memory growth detaches the old buffer
        const left = new Float32Array(memory, this.api.cw_left(), 128);
        const right = new Float32Array(memory, this.api.cw_right(), 128);
        out[0].set(left);
        if (out[1]) out[1].set(right);
        for (let i = 0; i < 128; i++) {
            const a = Math.abs(left[i]);
            if (a > this.peak) this.peak = a;
        }
        if (++this.blocks % 8 === 0) {
            this.port.postMessage({ type: 'meter', peak: this.peak, blocks: this.blocks });
            this.peak = 0;
        }
        return true;
    }
}

registerProcessor('crucible-processor', CrucibleProcessor);
