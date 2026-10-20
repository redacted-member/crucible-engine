#!/bin/sh
# SPDX-License-Identifier: MIT
# Builds dist/crucible.wasm. Needs the Emscripten SDK on PATH (source emsdk_env.sh first). Run from this directory.
set -e
ROOT=../..
BUILD=${BUILD:-$ROOT/build-wasm}
if [ ! -f "$BUILD/libcrucible_engine.a" ]; then
    emcmake cmake -S "$ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCRUCIBLE_FORCE_SCALAR=ON \
        -DCRUCIBLE_ENGINE_MODULE_LOADING=OFF -DCRUCIBLE_ENGINE_BUILD_TESTS=OFF -DCRUCIBLE_ENGINE_BUILD_EXAMPLES=OFF \
        -DCRUCIBLE_ENGINE_BUILD_TOOLS=OFF -DCRUCIBLE_ENGINE_INSTALL=OFF -DCRUCIBLE_ENGINE_FFT_BACKEND=PORTABLE
    cmake --build "$BUILD" -j4
fi
mkdir -p dist
emcc -O3 -c -I"$ROOT/include" -I"$BUILD/generated" glue/worklet_glue.c -o dist/glue.o
em++ -O3 dist/glue.o "$BUILD/libcrucible_engine.a" -o dist/crucible.wasm -sSTANDALONE_WASM --no-entry \
    -sALLOW_MEMORY_GROWTH -sSTACK_SIZE=1048576 \
    -sEXPORTED_FUNCTIONS=_cw_init,_cw_midi,_cw_process,_cw_left,_cw_right
rm dist/glue.o
