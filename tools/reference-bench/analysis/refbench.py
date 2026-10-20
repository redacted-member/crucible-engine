#!/usr/bin/env python3
"""Reference bench measurement suite.

  refbench.py plan     --out plan.json                      plugin-agnostic test plan
  refbench.py job      --profile P.json --out job.json      reference-bench job for one plugin
  refbench.py run      --profile P.json --work DIR          render with reference-bench, then analyse
  refbench.py analyze  --renders DIR --label NAME --out R.json [--profile P.json]
  refbench.py compare  --result A.json --result B.json [--candidate C.json] --out-dir DIR
  refbench.py selftest                                      synthetic checks of the analog-behaviour analysers

Needs Python 3.10+ and numpy; matplotlib is optional (used for PNGs when present).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from refbench_analysis import analyze, report, suite  # noqa: E402

DEFAULT_BENCH = HERE.parent / "build" / "reference-bench_artefacts" / "Release" / "reference-bench"


def selected_tests(args) -> list:
    groups = args.groups.split(",") if getattr(args, "groups", None) else None
    names = args.tests.split(",") if getattr(args, "tests", None) else None
    return suite.filter_tests(suite.build_tests(getattr(args, "include_pending", False)), groups, names)


def cmd_plan(args):
    plan = suite.plan(selected_tests(args))
    Path(args.out).write_text(json.dumps(plan, indent=1))
    print(f"{len(plan['tests'])} tests -> {args.out}")


def cmd_job(args):
    profile = suite.load_profile(args.profile)
    job, skipped = suite.make_job(profile, selected_tests(args))
    Path(args.out).write_text(json.dumps(job, indent=1))
    print(f"{len(job['renders'])} renders, {len(skipped)} skipped -> {args.out}")
    for k, v in skipped.items():
        print(f"  skip {k}: {v}")


def cmd_run(args):
    profile = suite.load_profile(args.profile)
    if not args.analyze_only and (not profile["plugin"] or not Path(profile["plugin"]).exists()):
        sys.exit(f"plugin not found: '{profile['plugin']}' (check the profile or its environment variable)")
    tests = selected_tests(args)
    work = Path(args.work)
    renders = work / "renders"
    renders.mkdir(parents=True, exist_ok=True)
    job, skipped = suite.make_job(profile, tests)
    (work / "job.json").write_text(json.dumps(job, indent=1))
    plan = suite.plan(tests)
    (work / "plan.json").write_text(json.dumps(plan, indent=1))
    bench = Path(args.bench)
    if not args.analyze_only:
        if not bench.exists():
            sys.exit(f"plugin host not found: {bench} (the host is not part of this repository; render with tools/refbench.sh or pass --bench)")
        cmd = [str(bench), "render", profile["plugin"], "--job", str(work / "job.json"), "--out-dir", str(renders)]
        print("+", " ".join(cmd))
        t0 = time.time()
        proc = subprocess.run(cmd, timeout=args.timeout)
        print(f"render finished in {time.time() - t0:.1f} s (exit {proc.returncode})")
        if proc.returncode not in (0, 3):
            sys.exit(f"reference-bench failed with exit code {proc.returncode}")
    out, plots = analyze.analyze_dir(plan, renders, args.label or profile["name"], profile, skipped)
    analyze.save(out, plots, work / "results.json")
    print(f"{len(out['tests'])} analysed, {len(out['skipped'])} skipped, {len(out['errors'])} errors -> "
          f"{work / 'results.json'}")


def cmd_analyze(args):
    plan = json.loads(Path(args.plan).read_text()) if args.plan else suite.plan(selected_tests(args))
    profile = suite.load_profile(args.profile) if args.profile else None
    out, plots = analyze.analyze_dir(plan, Path(args.renders), args.label, profile)
    analyze.save(out, plots, Path(args.out))
    print(f"{len(out['tests'])} analysed, {len(out['skipped'])} skipped -> {args.out}")


def cmd_compare(args):
    targets = json.loads(Path(args.targets).read_text())
    report.compare(args.result, args.candidate, targets, Path(args.out_dir))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def selection(sp):
        sp.add_argument("--groups", help="comma-separated test groups (oscillator, unison, wavetable, filter, "
                                         "envelope, pitch, noise, lfo, fx, cpu)")
        sp.add_argument("--tests", help="comma-separated test names")
        sp.add_argument("--include-pending", action="store_true",
                        help="also list the analog-behaviour tests whose targets are pending (suite_analog.py)")

    sp = sub.add_parser("plan")
    sp.add_argument("--out", required=True)
    selection(sp)
    sp.set_defaults(fn=cmd_plan)

    sp = sub.add_parser("job")
    sp.add_argument("--profile", required=True)
    sp.add_argument("--out", required=True)
    selection(sp)
    sp.set_defaults(fn=cmd_job)

    sp = sub.add_parser("run")
    sp.add_argument("--profile", required=True)
    sp.add_argument("--work", required=True, help="output folder (renders, job, results)")
    sp.add_argument("--bench", default=os.environ.get("REFBENCH_BIN", str(DEFAULT_BENCH)))
    sp.add_argument("--label")
    sp.add_argument("--timeout", type=float, default=3600)
    sp.add_argument("--analyze-only", action="store_true", help="reuse existing renders")
    selection(sp)
    sp.set_defaults(fn=cmd_run)

    sp = sub.add_parser("analyze")
    sp.add_argument("--renders", required=True)
    sp.add_argument("--label", required=True)
    sp.add_argument("--out", required=True)
    sp.add_argument("--plan")
    sp.add_argument("--profile")
    selection(sp)
    sp.set_defaults(fn=cmd_analyze)

    sp = sub.add_parser("selftest", help="check the analog-behaviour analysers against synthetic renders")
    sp.set_defaults(fn=lambda args: sys.exit(__import__("selftest").main()))

    sp = sub.add_parser("compare")
    sp.add_argument("--result", action="append", required=True, help="reference results (repeatable)")
    sp.add_argument("--candidate", help="results of our engine")
    sp.add_argument("--targets", default=str(HERE / "targets.json"))
    sp.add_argument("--out-dir", required=True)
    sp.set_defaults(fn=cmd_compare)

    args = p.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
