"""
A simple test runner.
It discovers tests/test*/test.toml, builds, runs, and validates each cases.
"""

import sys
import subprocess
from pathlib import Path
from typing import Optional, Literal, cast
from dataclasses import dataclass, field
import tomllib

# Paths
TESTS_DIR = Path(__file__).resolve().parent


# Data models
@dataclass
class BuildConfig:
    command: str
    build_path: Optional[Path] = None  # Directory to run the build command in


@dataclass
class RunConfig:
    command: str
    stdin: Optional[str] = None


@dataclass
class ValidateConfig:
    kind: Literal["txt", "jsonlines"]
    expected: Path
    tolerance: dict[str, float] | None = (
        None  # Tolerance for numeric comparisons; if None, exact match is required
    )


@dataclass
class CaseConfig:
    name: str
    run: RunConfig
    validate: ValidateConfig


@dataclass
class TestConfig:
    name: str
    cases: list[CaseConfig] = field(default_factory=list)
    description: Optional[str] = None
    build: Optional[BuildConfig] = None


# Load Config
def load_validate_config(raw: dict) -> ValidateConfig:
    kind: Literal["txt", "jsonlines"] = raw["kind"]
    if kind not in ["txt", "jsonlines"]:
        print(f"Warning: Unknown validation kind: {kind}. Defaulting to 'txt'.")
        kind = "txt"

    expected = Path(raw["expected"])

    tolerance: dict[str, float] | float | int | None = raw.get("tolerance", None)
    if tolerance is not None and kind != "jsonlines":
        print(
            f"Warning: 'tolerance' specified for validation kind '{kind}', which does not support tolerance. Ignoring 'tolerance'."
        )
        tolerance = None
    if isinstance(tolerance, (int, float)):
        tolerance = {
            "__all__": float(tolerance)
        }  # Apply same tolerance to all numeric comparisons
    elif isinstance(tolerance, dict):
        cleaned = {}
        for k, v in tolerance.items():
            if not isinstance(v, (int, float)):
                print(
                    f"Warning: Tolerance for field '{k}' is not a number. Ignoring this tolerance."
                )
                continue
            cleaned[k] = float(v)
        tolerance = cleaned

    return ValidateConfig(kind=kind, expected=expected, tolerance=tolerance)


def load_test_config(tomal_config_path: Path) -> TestConfig:
    with open(tomal_config_path, "rb") as f:
        raw = tomllib.load(f)

    name = raw.get("name", tomal_config_path.parent.name)
    description = raw.get("description", None)
    build_config = BuildConfig(**raw["build"]) if "build" in raw else None

    cases = []
    if "case" in raw:
        for i, case in enumerate(raw["case"]):
            case_name = name + "-" + case.get("name", f"case{i}")
            run = RunConfig(**case["run"])
            validate = load_validate_config(case["validate"])
            cases.append(CaseConfig(name=case_name, run=run, validate=validate))
    else:
        case_name = name
        run = RunConfig(**raw["run"])
        validate = load_validate_config(raw["validate"])
        cases.append(CaseConfig(name=case_name, run=run, validate=validate))

    return TestConfig(
        cases=cases,
        name=name,
        description=description,
        build=build_config,
    )


def build_target(path: Path, build_command: str) -> bool:
    try:
        result = subprocess.run(
            build_command, shell=True, cwd=path, capture_output=True, text=True
        )
        if result.returncode == 0:
            return True
        else:
            print(
                f"Failed to build target at {path}, exited with {result.returncode}: {result.stderr}"
            )
            return False
    except Exception as e:
        print(f"Failed to build target at {path}: {e}")
        return False


def run_case(
    path: Path, case: CaseConfig
) -> tuple[Literal[True], str] | tuple[Literal[False], None]:
    try:
        stdin_data = None
        if case.run.stdin is not None:
            stdin_path = path / case.run.stdin
            if not stdin_path.exists():
                print(f"Input file {stdin_path} does not exist.")
                return False, None

            with open(stdin_path, "r") as f:
                stdin_data = f.read()

        result = subprocess.run(
            case.run.command,
            shell=True,
            cwd=path,
            input=stdin_data,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            print("Run succeeded.")
            with open(path / "output.log", "w") as f:
                f.write(result.stdout)
            return True, result.stdout
        else:
            print(
                f"Failed to run case {case.name}, exited with {result.returncode}: {result.stderr}"
            )
    except Exception as e:
        print(f"Failed to run case {case.name}: {e}")

    return False, None


# Maybe I will add more validation methods in the future.
# I will keep this simple function for now so that I can easily change it in the future.
def values_match(output: float, expected: float, tolerance: float = 0.0) -> bool:
    return abs(output - expected) <= tolerance


def validate_case(path: Path, case: CaseConfig, output: str) -> Optional[bool]:
    expected_path = path / case.validate.expected
    if not expected_path.exists():
        print(f"Expected file {expected_path} does not exist.")
        return None

    try:
        with open(expected_path, "r") as f:
            expected_content = f.read()

        match case.validate.kind:
            case "txt":
                if output.strip() == expected_content.strip():
                    print("Validation succeeded.")
                    return True
                else:
                    print("Validation failed: output does not match expected.")
                    return False
            case "jsonlines":
                import json

                output_lines: list[dict] = [
                    json.loads(line)
                    for line in output.strip().splitlines()
                    if line.strip()
                ]
                expected_lines: list[dict] = [
                    json.loads(line)
                    for line in expected_content.strip().splitlines()
                    if line.strip()
                ]

                if len(expected_lines) > len(output_lines):
                    print(
                        f"Validation failed: output ended early, "
                        f"missing {len(expected_lines) - len(output_lines)} line(s)"
                    )
                    return False

                tol = case.validate.tolerance or {}

                for i, (out_line, exp_line) in enumerate(
                    zip(output_lines, expected_lines)
                ):
                    for field in exp_line:
                        if field not in out_line:
                            print(
                                f"Validation failed at line {i}, field '{field}': "
                                f"missing in output"
                            )
                            return False
                        out_val = out_line[field]
                        exp_val = exp_line[field]

                        if isinstance(out_val, (int, float)) and isinstance(
                            exp_val, (int, float)
                        ):
                            field_tol = tol.get(field)
                            if field_tol is None:
                                field_tol = tol.get("__all__", 0.0)
                            if not values_match(
                                float(out_val), float(exp_val), float(field_tol)
                            ):
                                print(
                                    f"Validation failed at line {i}, field '{field}': "
                                    f"output={out_val}, expected={exp_val}, tolerance={field_tol}"
                                )
                                return False
                        else:
                            if out_val != exp_val:
                                print(
                                    f"Validation failed at line {i}, field '{field}': "
                                    f"output={out_val!r}, expected={exp_val!r}"
                                )
                                return False

                print("Validation succeeded.")
                return True

            case _:
                print(f"Unknown validation kind: {case.validate.kind}")
                return None

    except Exception as e:
        print(f"Failed to validate case {case.name}: {e}")
        return False


def main():
    test_target = sys.argv[1] if len(sys.argv) > 1 else None

    toml_config_paths = list(TESTS_DIR.glob("test*/test.toml"))
    if not toml_config_paths:
        print("No test targets found.")
        return

    results: dict[str, str] = {}

    for toml_config_path in toml_config_paths:
        config = load_test_config(toml_config_path)
        print(f"\nRunning test: {config.name}")
        if config.description:
            print(f"Description: {config.description}")

        for case in config.cases:
            if test_target is not None and case.name not in test_target:
                continue

            case_path = toml_config_path.parent

            if config.build is not None:
                print(f"\nBuilding target: {case.name} ...")
                build_path = config.build.build_path or case_path
                build_success = build_target(build_path, config.build.command)
                if build_success:
                    print("Build succeeded.")
                    build_command = None  # Only build once for all cases
                else:
                    print("Skip all cases.")
                    for c in config.cases:
                        if test_target is None or c.name in test_target:
                            results[c.name] = "BUILD FAIL"
                    break

            print(f"\nRunning case: {case.name} ...")
            run_success, output = run_case(toml_config_path.parent, case)
            if not run_success:
                print("Skip validation due to run failure.")
                results[case.name] = "RUN FAIL"
                continue
            output = cast(str, output)
            # output is guaranteed to be str if run_success is True

            print(f"Validating case: {case.name} ...")
            result = validate_case(toml_config_path.parent, case, output)
            if result is True:
                results[case.name] = "PASS"
            elif result is False:
                results[case.name] = "FAIL"
            else:
                results[case.name] = "NO EXPECTED"

    # Summary
    sep = "─" * 55
    print(f"\n{sep}")
    print(f"  {'STATUS':12s} NAME")
    print(f"{sep}")
    passed = sum(1 for r in results.values() if r == "PASS")
    failed = sum(1 for r in results.values() if r == "FAIL")
    skipped = sum(1 for r in results.values() if r != "PASS" and r != "FAIL")
    for name, status in results.items():
        print(f"  {status:12s} {name}")
    print(sep)
    print(
        f"  Total: {len(results)}  |  Pass: {passed}  |  Fail: {failed}  |  Other: {skipped}"
    )
    print(
        "\nOutput logs saved to tests/*/output.log.\n"
        "To accept output as expected, copy it to the expected file "
        "configured in test.toml."
    )

    if failed > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
