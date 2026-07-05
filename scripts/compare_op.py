#!/usr/bin/env python3
import argparse
import re
import sys
from pathlib import Path


NUMBER_RE = re.compile(
    r"^[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?$"
)
ACTUAL_RE = re.compile(
    r"^\s*([vi])\(([^)]+)\)\s+"
    r"([+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?)\s*$"
)


def normalize_key(token):
    token = token.strip().strip("\f").lower()
    if token.startswith("v(") and token.endswith(")"):
        return "v(" + token[2:-1].strip().lower() + ")"
    if token.startswith("i(") and token.endswith(")"):
        return "i(" + token[2:-1].strip().lower() + ")"
    if token.endswith("#branch"):
        return "i(" + token[:-7].strip().lower() + ")"
    return None


def parse_number(token):
    token = token.strip()
    if not NUMBER_RE.match(token):
        raise ValueError(f"not a number: {token}")
    return float(token)


def parse_standard(path):
    values = {}
    header = None

    for raw_line in path.read_text(errors="replace").splitlines():
        line = raw_line.strip().strip("\f")
        if not line:
            continue

        parts = line.split()
        if not parts:
            continue

        if parts[0] == "Index":
            header = parts[1:]
            continue

        if header is None:
            continue

        if parts[0].isdigit():
            data = parts[1:]
            for name, value in zip(header, data):
                key = normalize_key(name)
                if key is not None:
                    values[key] = parse_number(value)
            header = None

    return values


def parse_actual(path):
    values = {}
    converged = None

    for raw_line in path.read_text(errors="replace").splitlines():
        line = raw_line.strip()
        if line.startswith("converged "):
            converged = line.split(None, 1)[1].strip().lower() == "yes"
            continue

        match = ACTUAL_RE.match(line)
        if match:
            kind, name, value = match.groups()
            values[f"{kind.lower()}({name.strip().lower()})"] = float(value)

    return values, converged


def close_enough(actual, expected, atol, rtol):
    diff = abs(actual - expected)
    limit = atol + rtol * abs(expected)
    return diff <= limit, diff, limit


def format_range(expected, limit):
    return f"[{expected - limit:.10e}, {expected + limit:.10e}]"


def format_record(record):
    return (
        f"{record['key']}: expected {record['expected']:.10e}, "
        f"actual {record['actual']:.10e}, diff {record['diff']:.3e}, "
        f"allowed +/-{record['limit']:.3e}, "
        f"range {format_range(record['expected'], record['limit'])}"
    )


def worst_record(records):
    if not records:
        return None
    return max(records, key=lambda r: r["diff"] / r["limit"] if r["limit"] > 0.0 else 0.0)


def compare_case(standard_file, actual_file, err_file, atol, rtol):
    failures = []
    records = []
    checked = 0

    if not actual_file.exists():
        return 0, [f"missing actual output: {actual_file}"], records

    if err_file.exists() and err_file.stat().st_size != 0:
        failures.append(f"stderr is not empty: {err_file}")

    try:
        expected = parse_standard(standard_file)
    except Exception as exc:
        return checked, [f"failed to parse standard output: {exc}"], records

    try:
        actual, converged = parse_actual(actual_file)
    except Exception as exc:
        return checked, [f"failed to parse actual output: {exc}"], records

    if converged is False:
        failures.append("solver did not report convergence")
    elif converged is None:
        failures.append("actual output has no convergence line")

    if not expected:
        failures.append("standard output has no parseable OP table")

    for key in sorted(expected):
        checked += 1
        if key not in actual:
            failures.append(f"{key}: missing in actual output")
            continue

        expected_value = expected[key]
        actual_value = actual[key]
        ok, diff, limit = close_enough(actual_value, expected_value, atol, rtol)
        record = {
            "key": key,
            "expected": expected_value,
            "actual": actual_value,
            "diff": diff,
            "limit": limit,
        }
        records.append(record)
        if not ok:
            failures.append(format_record(record))

    return checked, failures, records


def main():
    parser = argparse.ArgumentParser(
        description="Compare DC operating point values against ngspice-style standard output."
    )
    parser.add_argument("--standard", default="standard", help="directory with reference .out files")
    parser.add_argument("--actual", default="actual", help="directory with generated .out/.err files")
    parser.add_argument("--atol", type=float, default=1.0e-3, help="absolute tolerance")
    parser.add_argument("--rtol", type=float, default=2.0e-2, help="relative tolerance")
    parser.add_argument("--verbose", action="store_true", help="print every compared value and range")
    args = parser.parse_args()

    standard_dir = Path(args.standard)
    actual_dir = Path(args.actual)

    if not standard_dir.is_dir():
        print(f"error: standard directory not found: {standard_dir}", file=sys.stderr)
        return 2
    if not actual_dir.is_dir():
        print(f"error: actual directory not found: {actual_dir}", file=sys.stderr)
        return 2

    standard_files = sorted(standard_dir.glob("*.out"))
    if not standard_files:
        print(f"error: no .out files in {standard_dir}", file=sys.stderr)
        return 2

    failed_cases = 0
    total_checked = 0

    print("Comparing OP results")
    print(f"Tolerance: atol={args.atol:g}, rtol={args.rtol:g}")
    print("Rule: pass when |actual - expected| <= atol + rtol * |expected|")
    for standard_file in standard_files:
        actual_file = actual_dir / standard_file.name
        err_file = actual_file.with_suffix(".err")
        checked, failures, records = compare_case(
            standard_file, actual_file, err_file, args.atol, args.rtol
        )
        total_checked += checked

        if failures:
            failed_cases += 1
            print(f"FAIL {standard_file.stem}")
            for failure in failures:
                print(f"  {failure}")
        else:
            worst = worst_record(records)
            if worst is None:
                print(f"PASS {standard_file.stem} ({checked} values)")
            else:
                ratio = worst["diff"] / worst["limit"] if worst["limit"] > 0.0 else 0.0
                print(
                    f"PASS {standard_file.stem} ({checked} values, "
                    f"worst {worst['key']} uses {ratio:.1%} of tolerance, "
                    f"allowed range {format_range(worst['expected'], worst['limit'])})"
                )

        if args.verbose:
            for record in records:
                print(f"  {format_record(record)}")

    passed_cases = len(standard_files) - failed_cases
    print(
        f"Summary: {passed_cases}/{len(standard_files)} cases passed, "
        f"{total_checked} values checked"
    )

    return 1 if failed_cases else 0


if __name__ == "__main__":
    raise SystemExit(main())
