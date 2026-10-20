#!/bin/sh
# SPDX-License-Identifier: MIT
# Builds and tests CrucibleEngine in the configurations CI runs: Release, AddressSanitizer + UndefinedBehaviorSanitizer,
# ThreadSanitizer, the scalar fallback (CRUCIBLE_FORCE_SCALAR) and, where an x86_64 toolchain runs (Rosetta), x86_64
# builds with SSE and with AVX2. Usage: engine/tools/ci.sh [release|asan|tsan|scalar|x86|all]   (default: all)
# Every requested configuration runs even when an earlier one fails; a summary at the end lists each result, and the
# exit status is non-zero if any configuration failed.
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
mode=${1:-all}
jobs=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)

passed=""
failed=""
record() {
    if [ "$2" -eq 0 ]; then passed="$passed $1"; else failed="$failed $1"; fi
}

run() {
    name=$1
    shift
    echo "== $name"
    # errexit is ignored inside a subshell that is part of an || list, so switch it off around the subshell instead.
    set +e
    (
        set -e
        cmake -S "$here" -B "$here/build-$name" "$@" > /dev/null
        cmake --build "$here/build-$name" -j "$jobs"
        ctest --test-dir "$here/build-$name" --output-on-failure
    )
    status=$?
    set -e
    record "$name" "$status"
}

case $mode in
    release|all) run release -DCMAKE_BUILD_TYPE=Release ;;
esac
case $mode in
    asan|all) run asan -DCMAKE_BUILD_TYPE=RelWithDebInfo "-DCRUCIBLE_ENGINE_SANITIZE=address;undefined" ;;
esac
case $mode in
    tsan|all) run tsan -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCRUCIBLE_ENGINE_SANITIZE=thread ;;
esac
case $mode in
    scalar|all) run scalar -DCMAKE_BUILD_TYPE=Release -DCRUCIBLE_FORCE_SCALAR=ON ;;
esac
case $mode in
    x86|all)
        # x86_64 under Rosetta (macOS) or natively: SSE baseline, then AVX2. Skipped when it cannot run.
        if [ "$(uname -m)" = x86_64 ]; then
            runner=""
        elif [ "$(uname -s)" = Darwin ] && arch -x86_64 /usr/bin/true 2> /dev/null; then
            runner="arch -x86_64"
        else
            echo "== x86: no x86_64 runner available, skipped"
            runner=skip
        fi
        if [ "$runner" != skip ]; then
            for variant in sse avx2; do
                name=x86-$variant
                echo "== $name"
                flags="-DCMAKE_BUILD_TYPE=Release"
                [ "$(uname -s)" = Darwin ] && flags="$flags -DCMAKE_OSX_ARCHITECTURES=x86_64"
                [ $variant = avx2 ] && flags="$flags -DCRUCIBLE_X86_AVX2=ON"
                set +e
                (
                    set -e
                    # shellcheck disable=SC2086
                    cmake -S "$here" -B "$here/build-$name" $flags > /dev/null
                    cmake --build "$here/build-$name" -j "$jobs"
                    # shellcheck disable=SC2086
                    # ctest itself is a native binary, so the test executables run under the emulator directly.
                    for t in crucible_engine_tests crucible_capi_test crucible_engine_graph_tests; do
                        $runner "$here/build-$name/$t" > "$here/build-$name/$t.log" 2>&1 || { tail -20 "$here/build-$name/$t.log"; exit 1; }
                    done
                )
                status=$?
                set -e
                record "$name" "$status"
            done
        fi
        ;;
esac
echo "== summary"
for n in $passed; do echo "   passed: $n"; done
for n in $failed; do echo "   FAILED: $n"; done
if [ -n "$failed" ]; then
    echo "== FAILED:$failed"
    exit 1
fi
echo "== all requested configurations passed"
