#!/usr/bin/env python3
"""One-click test runner: build → run → compare against expected.csv"""

import csv
import os
import subprocess
import sys

BUILD_DIR = os.path.join(os.path.dirname(__file__), "build")
TESTS_DIR = os.path.dirname(__file__)

def build():
    os.makedirs(BUILD_DIR, exist_ok=True)
    subprocess.run(["cmake", TESTS_DIR], cwd=BUILD_DIR, check=True)
    subprocess.run(["cmake", "--build", "."], cwd=BUILD_DIR, check=True)

def run_test(name, exe_path, expected_path):
    print(f"  Running {name}...", end=" ")
    result = subprocess.run([exe_path], capture_output=True, text=True)
    if result.returncode != 0:
        print(f"❌ CRASHED")
        print(result.stderr)
        return False

    if not os.path.exists(expected_path):
        # First run: save output as expected
        with open(expected_path, "w") as f:
            f.write(result.stdout)
        print(f"✅ (baseline saved)")
        return True

    # Compare
    with open(expected_path) as f:
        expected = f.read()

    expected_lines = expected.strip().split("\n")
    actual_lines = result.stdout.strip().split("\n")

    if len(expected_lines) != len(actual_lines):
        print(f"❌ line count mismatch: expected {len(expected_lines)}, got {len(actual_lines)}")
        return False

    all_ok = True
    for i, (exp_line, act_line) in enumerate(zip(expected_lines, actual_lines)):
        exp_vals = exp_line.split(",")
        act_vals = act_line.split(",")
        if len(exp_vals) != len(act_vals):
            print(f"❌ col count mismatch at line {i+1}")
            all_ok = False
            continue
        for j, (e, a) in enumerate(zip(exp_vals, act_vals)):
            try:
                ev = float(e); av = float(a)
                if abs(ev - av) > 1e-6:
                    print(f"❌ mismatch at line {i+1}, col {j+1}: expected {e}, got {a}")
                    all_ok = False
            except ValueError:
                if e.strip() != a.strip():
                    print(f"❌ mismatch at line {i+1}: expected '{e}', got '{a}'")
                    all_ok = False

    if all_ok:
        print("✅ PASS")
    return all_ok


def main():
    print("🔨 Building tests...")
    build()
    print("✅ Build complete\n")

    passed = 0
    failed = 0

    for entry in sorted(os.listdir(TESTS_DIR)):
        test_dir = os.path.join(TESTS_DIR, entry)
        if not os.path.isdir(test_dir) or entry.startswith(".") or entry == "build":
            continue
        target = entry  # CMake target name = directory name
        exe = os.path.join(BUILD_DIR, target)
        expected = os.path.join(test_dir, "expected.csv")
        if run_test(target, exe, expected):
            passed += 1
        else:
            failed += 1

    print(f"\n{'='*40}")
    print(f"Results: {passed} passed, {failed} failed")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())