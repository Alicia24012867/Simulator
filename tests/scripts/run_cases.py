#!/usr/bin/env python3
"""Execute one analysis suite and report wall-clock time for every netlist."""

import argparse
import subprocess
import sys
import time
from pathlib import Path


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Run SPICE regression cases with per-case timing."
    )
    parser.add_argument("--analysis", choices=("op", "tran"), required=True)
    parser.add_argument("--simulator", type=Path, required=True)
    parser.add_argument("--case-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    simulator = arguments.simulator.resolve()
    cases = sorted(arguments.case_dir.glob("*.cir"))
    if not cases:
        print(f"No {arguments.analysis} cases found in {arguments.case_dir}", file=sys.stderr)
        return 2

    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    failures = 0
    suite_start = time.perf_counter()

    for case in cases:
        stem = case.stem
        listing = arguments.output_dir / f"{stem}.out"
        raw = arguments.output_dir / f"{stem}.raw"
        error = arguments.output_dir / f"{stem}.err"

        start = time.perf_counter()
        with error.open("w", encoding="utf-8") as error_stream:
            result = subprocess.run(
                [simulator, "-b", "-o", listing, "-r", raw, case],
                stdout=subprocess.DEVNULL,
                stderr=error_stream,
                check=False,
            )
        elapsed_ms = (time.perf_counter() - start) * 1_000.0
        outcome = "PASS" if result.returncode == 0 else f"FAIL ({result.returncode})"
        print(
            f"TIME {arguments.analysis:<4} {stem:<48} "
            f"{elapsed_ms:10.3f} ms  {outcome}"
        )
        failures += result.returncode != 0

    suite_elapsed_ms = (time.perf_counter() - suite_start) * 1_000.0
    print(
        f"TIME {arguments.analysis:<4} suite ({len(cases)} cases) "
        f"{suite_elapsed_ms:.3f} ms"
    )
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
