#!/bin/sh
# SPDX-License-Identifier: MIT
# Scores the engine against the reference targets (tools/reference-bench): renders the bench plan with the engine's
# job renderer (crucible-measure), analyses the WAVs and compares them with the recorded numbers of two commercial
# synthesizers (tools/reference-bench/data, summary numbers only). Prints the failing targets (all targets with
# --all). Needs python3 with numpy.
#   tools/refbench.sh [engine-build-dir] [--all]
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
build=${1:-$here/build}
case $build in --all) build=$here/build ;; esac
bench="$here/tools/reference-bench/analysis"
data="$here/tools/reference-bench/data"
work=${REFBENCH_WORK:-$(mktemp -d)}
mkdir -p "$work/renders"
(cd "$bench" && python3 refbench.py job --profile profiles/crucible.json --out "$work/job.json" >/dev/null &&
    python3 refbench.py plan --out "$work/plan.json" >/dev/null)
"$build/crucible-measure" --job "$work/job.json" --out-dir "$work/renders"
(cd "$bench" && python3 refbench.py analyze --renders "$work/renders" --label "Crucible Engine" --out "$work/ours.json" --plan "$work/plan.json" >/dev/null &&
    python3 refbench.py compare --result "$data/serum2.json" --result "$data/pigments.json" \
        --candidate "$work/ours.json" --out-dir "$work/report" >/dev/null)
echo "report: $work/report/REPORT.md"
grep -E "^\*\*Candidate" "$work/report/REPORT.md" || true
if [ "${2:-}" = "--all" ] || [ "${1:-}" = "--all" ]; then
    grep -E "\| (PASS|FAIL) \|$" "$work/report/REPORT.md"
else
    grep -E "\| FAIL \|$" "$work/report/REPORT.md" | cut -d'|' -f2,4,7,8 || true
fi
