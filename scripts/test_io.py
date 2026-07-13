#!/usr/bin/env python3
import os
import subprocess
import sys
import tempfile
from pathlib import Path


def run(simulator, *arguments):
    return subprocess.run(
        [str(simulator), *map(str, arguments)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        timeout=10,
    )


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <simulator>", file=sys.stderr)
        return 2

    simulator = Path(sys.argv[1]).resolve()
    compare_script = Path(__file__).with_name("compare_spice.py").resolve()
    validate_raw_script = Path(__file__).with_name("validate_raw.py").resolve()
    failures = []

    with tempfile.TemporaryDirectory(prefix="simulator-io-") as directory:
        root = Path(directory)

        try:
            valid = root / "valid.cir"
            valid.write_text(
                "Original title\n"
                ".title Mixed-case # node and continuation\n"
                ".model UNUSED D IS =1e-14 N= 1\n"
                "V1 IN 0 DC 5 ; source comment\n"
                "R1 in OUT#sense 1k // slash comment\n"
                "C1 out#SENSE 0 1u $ dollar comment\n"
                ".op\n"
                ".print op V(in) V(in,gnd) V(gnd) I(V1)\n"
                ".tran 100u 200u UIC\n"
                ".print tran time V(in) V(out#sense,in)\n"
                "+ I(V1)\n"
                ".end\n"
            )
            listing = root / "valid.out"
            raw = root / "valid.raw"
            listing.write_text("old listing\n")
            raw.write_text("old rawfile\n")
            result = run(
                simulator,
                "-b",
                "-o",
                listing,
                "-r",
                raw,
                valid,
            )
            require(result.returncode == 0, f"valid netlist failed: {result.stderr}")
            require(not result.stderr, f"valid netlist wrote stderr: {result.stderr}")
            listing_text = listing.read_text()
            require("v(out#sense,in)" in listing_text, "differential voltage missing")
            require("v1#branch" in listing_text, "branch current column missing")
            headers = [
                line.split()
                for line in listing_text.splitlines()
                if line.strip().startswith("Index")
            ]
            require(len(headers) == 2, "expected OP and TRAN listing headers")
            require(
                headers[0] == ["Index", "v(in)", "v(0)", "v1#branch"],
                f"unexpected OP .print order or duplicate: {headers[0]}",
            )
            require(
                headers[1]
                == ["Index", "time", "v(in)", "v(out#sense,in)", "v1#branch"],
                f"unexpected TRAN .print order: {headers[1]}",
            )
            raw_text = raw.read_text()
            require("Date:" in raw_text, "raw date header missing")
            require("Plotname: Transient Analysis" in raw_text, "raw plot missing")
            require(raw_text.count("Plotname:") == 2, "raw multi-plot output missing")
            require("\t0\ttime\ttime" in raw_text, "raw time variable missing")
            require(
                "\tv(out#sense)\tvoltage" in raw_text,
                "raw output was incorrectly filtered by .print",
            )

            standard_dir = root / "mixed-standard"
            actual_dir = root / "mixed-actual"
            standard_dir.mkdir()
            actual_dir.mkdir()
            (standard_dir / "mixed.out").write_text(listing_text)
            (actual_dir / "mixed.out").write_text(listing_text)
            mixed_raw = actual_dir / "mixed.raw"
            mixed_raw.write_text(raw_text)
            for analysis in ("op", "tran"):
                comparison = subprocess.run(
                    [
                        sys.executable,
                        str(compare_script),
                        "--analysis",
                        analysis,
                        "--standard",
                        str(standard_dir),
                        "--actual",
                        str(actual_dir),
                        "--atol",
                        "0",
                        "--rtol",
                        "0",
                    ],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                )
                require(
                    comparison.returncode == 0,
                    f"mixed-analysis {analysis} comparison failed: "
                    f"{comparison.stdout}{comparison.stderr}",
                )
                raw_validation = subprocess.run(
                    [
                        sys.executable,
                        str(validate_raw_script),
                        "--analysis",
                        analysis,
                        "--listing-dir",
                        str(actual_dir),
                        str(mixed_raw),
                    ],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                )
                require(
                    raw_validation.returncode == 0,
                    f"mixed-analysis {analysis} raw/listing check failed: "
                    f"{raw_validation.stdout}{raw_validation.stderr}",
                )
            print("PASS valid SPICE syntax, comments, continuation, case folding")
        except Exception as exc:
            failures.append(str(exc))

        try:
            same_path = root / "same-path.cir"
            original = "Same path protection\nR1 1 0 1k\n.op\n.end\n"
            same_path.write_text(original)
            result = run(simulator, same_path, same_path)
            require(result.returncode != 0, "same input/output path was accepted")
            require(same_path.read_text() == original, "input file was truncated")

            hard_link = root / "same-path-hard-link.out"
            os.link(same_path, hard_link)
            result = run(simulator, same_path, hard_link)
            require(result.returncode != 0, "input hard-link alias was accepted")
            require(same_path.read_text() == original, "hard-link input was truncated")

            case_probe = root / "case-probe"
            case_probe.write_text("probe\n")
            case_insensitive = (root / "CASE-PROBE").exists()
            case_probe.unlink()
            upper_output = root / "Case-Alias.OUT"
            lower_output = root / "case-alias.out"
            result = run(
                simulator,
                "-o",
                upper_output,
                "-r",
                lower_output,
                valid,
            )
            if case_insensitive:
                require(
                    result.returncode != 0,
                    "case-only listing/raw aliases were accepted",
                )
                require(
                    not upper_output.exists() and not lower_output.exists(),
                    "case-alias rollback left a partial output",
                )
            else:
                require(
                    result.returncode == 0,
                    f"distinct case-sensitive output paths failed: {result.stderr}",
                )
            print("PASS input/output same-path protection")
        except Exception as exc:
            failures.append(str(exc))

        try:
            invalid = root / "missing-end.cir"
            invalid.write_text("Missing end\nV1 in 0 1\nR1 in 0 1k\n.op\n")
            protected_output = root / "protected.out"
            protected_output.write_text("keep me\n")
            result = run(simulator, "-o", protected_output, invalid)
            require(result.returncode != 0, "missing .end was accepted")
            require(
                protected_output.read_text() == "keep me\n",
                "failed parse truncated an existing output",
            )

            protected_output.write_text("keep after staged write failure\n")
            invalid_raw = root / "missing-directory" / "result.raw"
            result = run(
                simulator,
                "-o",
                protected_output,
                "-r",
                invalid_raw,
                valid,
            )
            require(result.returncode != 0, "invalid raw destination succeeded")
            require(
                protected_output.read_text() == "keep after staged write failure\n",
                "raw write failure replaced the existing listing",
            )
            print("PASS parse/write failure preserves existing output")
        except Exception as exc:
            failures.append(str(exc))

        try:
            unsolved = root / "unsolved.cir"
            unsolved.write_text(
                "Conflicting ideal sources\n"
                "V1 out 0 1\n"
                "V2 out 0 2\n"
                ".op\n"
                ".end\n"
            )
            protected_output = root / "solve-protected.out"
            protected_output.write_text("keep after solve failure\n")
            result = run(simulator, "-o", protected_output, unsolved)
            require(result.returncode != 0, "singular circuit unexpectedly solved")
            require(
                protected_output.read_text() == "keep after solve failure\n",
                "failed solve truncated an existing output",
            )
            print("PASS solve failure preserves existing output")
        except Exception as exc:
            failures.append(str(exc))

        try:
            trailing = root / "trailing-after-end.cir"
            trailing.write_text(
                "Trailing statement\nR1 in 0 1k\n.op\n.end\nR2 in 0 2k\n"
            )
            result = run(simulator, trailing)
            require(result.returncode != 0, "statement after .end was accepted")
            print("PASS .end last-statement validation")
        except Exception as exc:
            failures.append(str(exc))

        try:
            unsupported = root / "unsupported.cir"
            unsupported.write_text(
                "Unsupported directive\nV1 in 0 1\nR1 in 0 1k\n.ac dec 10 1 1meg\n.end\n"
            )
            result = run(simulator, unsupported)
            require(result.returncode != 0, "unsupported directive was ignored")
            require("Unsupported control directive" in result.stderr, "missing diagnostic")

            malformed_cases = [
                (
                    "element-assignment",
                    "Bad resistor value\nR1 in 0 nonsense=1k\n.op\n.end\n",
                    "Invalid SPICE number",
                ),
                (
                    "source-assignment",
                    "Bad source value\nV1 in 0 nonsense=2\nR1 in 0 1k\n.op\n.end\n",
                    "Invalid SPICE number",
                ),
                (
                    "malformed-number",
                    "Bad numeric token\nR1 in 0 1..2\n.op\n.end\n",
                    "Invalid SPICE number",
                ),
                (
                    "print-without-tran",
                    "Missing tran\nR1 in 0 1k\n.print tran time\n.end\n",
                    ".print tran requires a .tran analysis",
                ),
                (
                    "malformed-model",
                    "Bad model\n.model DMOD D IS\nD1 in 0 DMOD\nR1 in 0 1k\n.op\n.end\n",
                    "Malformed model parameter",
                ),
                (
                    "unsupported-model-parameter",
                    "Unknown model parameter\n.model DMOD D TYPO=1\n"
                    "D1 in 0 DMOD\nR1 in 0 1k\n.op\n.end\n",
                    "Unsupported model parameter",
                ),
                (
                    "invalid-model-level",
                    "Unsupported model level\n.model NMOD NMOS LEVEL=2\n"
                    "M1 out out 0 0 NMOD W=1u L=1u\nR1 out 0 1k\n"
                    ".op\n.end\n",
                    "Only MOSFET LEVEL=1",
                ),
                (
                    "invalid-model-domain",
                    "Bad model domain\n.model DMOD D N=-1\n"
                    "D1 in 0 DMOD\nR1 in 0 1k\n.op\n.end\n",
                    "must be positive",
                ),
                (
                    "invalid-area",
                    "Bad area\n.model DMOD D IS=1e-14\nD1 in 0 DMOD 0\n"
                    "R1 in 0 1k\n.op\n.end\n",
                    "area must be positive",
                ),
                (
                    "empty-circuit",
                    "No elements\n.op\n.end\n",
                    "requires at least one element",
                ),
                (
                    "ground-only-circuit",
                    "No MNA unknowns\nR1 0 gnd 1k\n.op\n.end\n",
                    "requires at least one non-ground node",
                ),
            ]
            for name, netlist, diagnostic in malformed_cases:
                malformed = root / f"{name}.cir"
                malformed.write_text(netlist)
                result = run(simulator, malformed)
                require(result.returncode != 0, f"{name} netlist was accepted")
                require(
                    diagnostic in result.stderr,
                    f"{name} diagnostic missing: {result.stderr}",
                )
            print("PASS unsupported and malformed input diagnostics")
        except Exception as exc:
            failures.append(str(exc))

        try:
            result = run(simulator, "--help")
            require(result.returncode == 0, "--help returned failure")
            require("Usage:" in result.stdout, "--help output is missing usage")
            result = run(simulator, "-o", "-r", valid)
            require(
                result.returncode != 0,
                "-o incorrectly accepted another option as its filename",
            )
            print("PASS command-line help")
        except Exception as exc:
            failures.append(str(exc))

    if failures:
        for failure in failures:
            print(f"FAIL {failure}")
        print(f"I/O summary: {7 - len(failures)}/7 checks passed")
        return 1

    print("I/O summary: 7/7 checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
