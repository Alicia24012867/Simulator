#!/usr/bin/env python3
import subprocess
import sys
import tempfile
from pathlib import Path


def run(simulator, *arguments, cwd=None):
    return subprocess.run(
        [str(simulator), *map(str, arguments)],
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        timeout=10,
    )


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def write_netlist(path):
    path.write_text(
        "Configuration CLI test\n"
        "V1 in 0 1\n"
        "R1 in 0 1k\n"
        ".op\n"
        ".end\n"
    )


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <simulator>", file=sys.stderr)
        return 2

    simulator = Path(sys.argv[1]).resolve()
    failures = []

    with tempfile.TemporaryDirectory(prefix="simulator-config-cli-") as directory:
        root = Path(directory)
        deck = root / "input.cir"
        write_netlist(deck)

        try:
            explicit = root / "explicit.json"
            explicit.write_text('{"schema_version": 1}\n')
            result = run(
                simulator,
                "--config",
                explicit,
                "--print-config-path",
                "--parse-only",
                deck,
            )
            require(
                result.returncode == 0,
                f"explicit configuration failed: {result.stderr}",
            )
            require(
                f"Configuration file: <{explicit}>" in result.stderr,
                "explicit configuration path was not reported",
            )
            require(not result.stdout, "parse-only configuration check wrote stdout")
            print("PASS explicit configuration path")
        except Exception as error:
            failures.append(str(error))

        try:
            project = root / "project"
            nested = project / "nested"
            nested.mkdir(parents=True)
            discovered = project / "config.json"
            discovered.write_text('{"schema_version": 1}\n')
            result = run(
                simulator,
                "--print-config-path",
                "--parse-only",
                deck,
                cwd=nested,
            )
            require(
                result.returncode == 0,
                f"automatic configuration discovery failed: {result.stderr}",
            )
            require(
                f"Configuration file: <{discovered.resolve()}>" in result.stderr,
                "nearest parent configuration was not reported",
            )

            result = run(
                simulator,
                "--config-search-depth",
                "0",
                "--print-config-path",
                "--parse-only",
                deck,
                cwd=nested,
            )
            require(
                result.returncode == 0,
                f"limited configuration search failed: {result.stderr}",
            )
            require(
                "Configuration file: none (using built-in defaults)"
                in result.stderr,
                "zero search depth did not retain built-in defaults",
            )
            print("PASS automatic configuration discovery and search depth")
        except Exception as error:
            failures.append(str(error))

        try:
            missing = root / "missing.json"
            result = run(
                simulator,
                "--config",
                missing,
                "--parse-only",
                deck,
            )
            require(
                result.returncode == 2,
                "missing explicit configuration did not return a usage error",
            )
            require(
                "Configuration file not found" in result.stderr,
                "missing explicit configuration diagnostic is absent",
            )

            invalid = root / "invalid.json"
            invalid.write_text("{\n")
            result = run(
                simulator,
                "--config",
                invalid,
                "--parse-only",
                deck,
            )
            require(
                result.returncode == 2,
                "invalid json did not return a usage error",
            )
            require(
                "Invalid json in configuration file" in result.stderr,
                "invalid json diagnostic is absent",
            )

            unsupported = root / "unsupported.json"
            unsupported.write_text('{"schema_version": 1, "source": true}\n')
            result = run(
                simulator,
                "--config",
                unsupported,
                "--parse-only",
                deck,
            )
            require(
                result.returncode == 2,
                "unsupported configuration field did not return a usage error",
            )
            require(
                "$.source is not supported" in result.stderr,
                "unsupported configuration field diagnostic is absent",
            )

            analysis_config = root / "analysis.json"
            analysis_config.write_text(
                "{\n"
                '  "schema_version": 1,\n'
                '  "op": {"newton": {"maximum_iterations": 5}},\n'
                '  "tran": {"output_interval": "1n", "stop_time": "2n"}\n'
                "}\n"
            )
            result = run(
                simulator,
                "--config",
                analysis_config,
                "--parse-only",
                deck,
            )
            require(
                result.returncode == 0,
                f"valid analysis configuration failed: {result.stderr}",
            )

            invalid_range = root / "invalid-range.json"
            invalid_range.write_text(
                '{"schema_version": 1, '
                '"op": {"newton": {"tolerance": 0}}}\n'
            )
            result = run(
                simulator,
                "--config",
                invalid_range,
                "--parse-only",
                deck,
            )
            require(
                result.returncode == 2,
                "invalid analysis configuration range was accepted",
            )
            require(
                "Invalid analysis configuration" in result.stderr,
                "invalid analysis configuration diagnostic is absent",
            )

            result = run(
                simulator,
                "--config-search-depth",
                "-1",
                "--parse-only",
                deck,
            )
            require(
                result.returncode == 2,
                "negative configuration search depth was accepted",
            )
            print("PASS configuration diagnostics and argument validation")
        except Exception as error:
            failures.append(str(error))

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1

    print("Configuration CLI summary: 3/3 checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
