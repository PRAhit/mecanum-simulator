#!/usr/bin/env python3
"""Run the scenario suite and emit a pass/fail acceptance report.

Exits non-zero if any gate fails, so this is usable directly as a CI step:
a control regression breaks the build the same way a failing unit test does.

    python3 tools/run_scenarios.py                 # all scenarios
    python3 tools/run_scenarios.py --only estop    # one scenario
    python3 tools/run_scenarios.py --seeds 5       # repeat over random seeds
    python3 tools/run_scenarios.py --json out.json
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

import numpy as np  # noqa: E402
from odc_sim import scenarios  # noqa: E402

GREEN, RED, YELLOW, DIM, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", default=None, help="scenario names to run")
    ap.add_argument("--seeds", type=int, default=1, help="repeat each scenario N times")
    ap.add_argument("--json", type=Path, default=None, help="write results as JSON")
    ap.add_argument("--no-color", action="store_true")
    args = ap.parse_args()

    if args.no_color or not sys.stdout.isatty():
        g = r = y = d = e = ""
    else:
        g, r, y, d, e = GREEN, RED, YELLOW, DIM, RESET

    suite = scenarios.all_scenarios()
    if args.only:
        suite = [s for s in suite if s.name in args.only]
        if not suite:
            print(f"no scenario matched {args.only}", file=sys.stderr)
            return 2

    report: dict = {"scenarios": [], "passed": 0, "failed": 0}
    t_start = time.time()

    for sc in suite:
        print(f"\n{sc.name}")
        print(f"  {d}{sc.description}{e}")

        gate_values: dict[str, list[float]] = {gt.name: [] for gt in sc.gates}
        sim_time = 0.0
        for seed in range(args.seeds):
            t0 = time.time()
            res = sc.execute(seed=seed)
            sim_time += time.time() - t0
            for gt in sc.gates:
                _, v = gt.evaluate(res)
                gate_values[gt.name].append(v)

        entry = {"name": sc.name, "description": sc.description,
                 "seeds": args.seeds, "gates": []}
        scenario_ok = True

        for gt in sc.gates:
            vals = np.array(gate_values[gt.name])
            worst = float(vals.max() if gt.op == "<=" else vals.min())
            ok = worst <= gt.limit if gt.op == "<=" else worst >= gt.limit
            scenario_ok &= ok

            mark = f"{g}PASS{e}" if ok else f"{r}FAIL{e}"
            margin = ""
            if ok and gt.limit not in (0.0,):
                used = abs(worst) / abs(gt.limit) if gt.limit else 0.0
                if used > 0.85:
                    margin = f" {y}(margin {100 * (1 - used):.0f}%){e}"
            print(f"    [{mark}] {gt.name:<32} {worst:>10.4g} {gt.op} "
                  f"{gt.limit:<10.4g} {gt.units}{margin}")

            entry["gates"].append({
                "name": gt.name, "value": worst, "limit": gt.limit,
                "op": gt.op, "units": gt.units, "passed": bool(ok),
                "all_seeds": [float(v) for v in vals],
            })

        entry["passed"] = bool(scenario_ok)
        entry["sim_seconds"] = round(sim_time, 2)
        report["scenarios"].append(entry)
        report["passed" if scenario_ok else "failed"] += 1

    total = report["passed"] + report["failed"]
    print(f"\n{'-' * 64}")
    verdict = (f"{g}ALL PASS{e}" if report["failed"] == 0
               else f"{r}{report['failed']} FAILED{e}")
    print(f"{report['passed']}/{total} scenarios passed  [{verdict}]  "
          f"{time.time() - t_start:.1f}s wall")

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2))
        print(f"wrote {args.json}")

    return 1 if report["failed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
