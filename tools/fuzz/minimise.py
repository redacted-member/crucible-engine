# SPDX-License-Identifier: MIT
"""Shrinks a failing crucible-fuzz seed to the fewest steps that still fail the same way (delta debugging over --keep).

Every crucible-fuzz step draws from its own generator, so a kept step does exactly what it did in the full run; only
the engine state it meets differs. A candidate counts as failing when its output matches the signature regex (or, for
a hang, when it runs past the timeout).

usage: python3 tools/fuzz/minimise.py <crucible-fuzz> <seed> <fail-step> <signature-regex | hang> [--timeout S]
       [--root DIR] [--extra=--strict-frames]
prints the minimal --keep list and the command line that reproduces it.
"""

import argparse
import os
import re
import subprocess
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("binary")
    ap.add_argument("seed", type=int)
    ap.add_argument("fail_step", type=int)
    ap.add_argument("signature")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--extra", default="", help="extra crucible-fuzz arguments, space separated")
    ap.add_argument("--root", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    args = ap.parse_args()
    hang = args.signature == "hang"
    sig = None if hang else re.compile(args.signature)
    env = dict(os.environ)
    env.setdefault("ASAN_OPTIONS", "detect_leaks=1")
    env.setdefault("UBSAN_OPTIONS", "print_stacktrace=1:halt_on_error=1")
    runs = [0]

    def fails(keep):
        runs[0] += 1
        cmd = [args.binary, "--root", args.root, "--seed", str(args.seed), "--steps", str(args.fail_step + 1),
               "--no-alloc-stacks"] + args.extra.split() + ["--keep", ",".join(map(str, keep)) if keep else "-1"]
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, errors="replace", timeout=args.timeout, env=env)
        except subprocess.TimeoutExpired:
            return hang
        if hang:
            return False
        return p.returncode != 0 and sig.search(p.stdout + p.stderr) is not None

    steps = list(range(args.fail_step + 1))
    if not fails(steps):
        print("the full run does not reproduce the signature", file=sys.stderr)
        return 1
    # ddmin
    n = 2
    while len(steps) >= 2:
        chunk = max(1, len(steps) // n)
        subsets = [steps[i:i + chunk] for i in range(0, len(steps), chunk)]
        reduced = False
        for i in range(len(subsets)):
            complement = [s for j, sub in enumerate(subsets) if j != i for s in sub]
            if complement and fails(complement):
                steps = complement
                n = max(n - 1, 2)
                reduced = True
                break
        if not reduced:
            if n >= len(steps):
                break
            n = min(len(steps), n * 2)
    # one-at-a-time removal pass
    for s in list(steps):
        trial = [x for x in steps if x != s]
        if trial and fails(trial):
            steps = trial
    keep = ",".join(map(str, steps))
    print(f"minimal keep ({len(steps)} steps, {runs[0]} runs): {keep}")
    print(f"repro: {args.binary} --root . --seed {args.seed} --steps {args.fail_step + 1} --keep {keep} {args.extra} --trace".rstrip())
    return 0


if __name__ == "__main__":
    sys.exit(main())
