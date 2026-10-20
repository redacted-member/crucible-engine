<!-- SPDX-License-Identifier: MIT -->
# wasm playground

The engine in a browser AudioWorklet: 128-sample blocks, computer-keyboard notes (keys `A` to `;`), a level meter.
A browser playground: the engine compiled to WebAssembly and driven from an AudioWorklet.

Setup and run (needs the Emscripten SDK and Node 22.6 or later):

    source ~/emsdk/emsdk_env.sh
    ./build-wasm.sh          # builds the engine for wasm if needed, writes dist/crucible.wasm
    node build.mjs           # strips types from src/main.ts, copies the page into dist/
    python3 -m http.server 8765 --directory dist

Open http://localhost:8765/ and press Start (browsers need a click before audio). Add `?mute` to the URL to run the worklet
with the output silenced. Any static host works: no special headers are needed (no threads, no SharedArrayBuffer).

Files: `glue/worklet_glue.c` (C wrapper, fixed buffers), `src/engine-worklet.js` (the AudioWorkletProcessor, plain JS),
`src/main.ts` (page code), `src/index.html`, `src/style.css`.
