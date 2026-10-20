#!/bin/sh
# SPDX-License-Identifier: MIT
# Runs crucible-fuzz over a seed range, one process per seed, in parallel, and keeps the log of every failing seed.
# Usage: tools/fuzz/campaign.sh <crucible-fuzz> <first> <last> <steps> <outdir> [jobs] [timeout-seconds] [wall-seconds]
# A seed that runs longer than the timeout is recorded as a hang. Seeds not started within wall-seconds (0: no limit)
# are skipped. Extra crucible-fuzz arguments come from $FUZZ_ARGS (for example --avoid-known). Summary lines go to
# <outdir>/summary.txt.
set -eu
bin=$1 first=$2 last=$3 steps=$4 out=$5 jobs=${6:-4} limit=${7:-900} wall=${8:-0}
deadline=0
[ "$wall" -gt 0 ] && deadline=$(($(date +%s) + wall))
args=${FUZZ_ARGS:-}
root=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$out/fail"
export ASAN_OPTIONS=${ASAN_OPTIONS:-abort_on_error=0:detect_leaks=1:allocator_may_return_null=0}
export UBSAN_OPTIONS=${UBSAN_OPTIONS:-print_stacktrace=1:halt_on_error=1}
seq "$first" "$last" | xargs -P "$jobs" -I{} sh -c '
    s={}
    [ '"$deadline"' -gt 0 ] && [ "$(date +%s)" -ge '"$deadline"' ] && exit 0
    log="'"$out"'/fail/seed-$s.log"
    start=$(date +%s)
    if timeout '"$limit"' "'"$bin"'" --root "'"$root"'" --seed "$s" --steps '"$steps"' --stats '"$args"' > "$log.tmp" 2>&1; then
        echo "ok $s $(($(date +%s) - start))s $(grep "^seed" "$log.tmp" | head -1)" >> "'"$out"'/summary.txt"
        grep "^FUZZ-NOTE" "$log.tmp" >> "'"$out"'/notes.txt" || true
        rm -f "$log.tmp"
    else
        code=$?
        mv "$log.tmp" "$log"
        kind=$(grep -m1 -oE "FUZZ-FAIL [a-z]+|ERROR: AddressSanitizer: [a-z-]+|runtime error: .*|ERROR: LeakSanitizer|AddressSanitizer:DEADLYSIGNAL" "$log" | head -1)
        [ "$code" -eq 124 ] && kind="hang (>'"$limit"' s)"
        echo "FAIL $s exit=$code $(($(date +%s) - start))s $kind" >> "'"$out"'/summary.txt"
    fi
'
echo "campaign $first..$last done: $(grep -c "^ok" "$out/summary.txt" || true) ok, $(grep -c "^FAIL" "$out/summary.txt" || true) failed"
