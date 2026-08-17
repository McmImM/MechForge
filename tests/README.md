# MechForge Tests

## Requirements

- Python 3.11+ (uses `tomllib` from the standard library)

## Quick Start

```bash
# Run all tests
python3 run_tests.py

# Run a specific test
python3 run_tests.py test0_crank_slider
```

## How It Works

Each `tests/testX_xxx/` directory contains a `test.toml` that describes:

- **Build**: how to compile the test (CMake target or direct command)
- **Run**: the command to execute, optional stdin file
- **Validate**: how to check the output (compare against expected values with tolerance)

`run_tests.py` discovers all `test.toml` files, runs them, and prints a pass/fail summary.

## Output Logs

Every case's stdout is saved to its **own** log file, so a later case never
overwrites an earlier one. The default name is `output_<case>.log` (spaces in
the name become underscores); override it with the `log` key:

```toml
[[case]]
name = "server_basics"
run = { command = "python3 run_cases.py server_basics" }
validate = { kind = "txt", expected = "expected_server_basics.txt" }
log = "custom_name.log"          # optional; default is output_<case>.log
```

For a single-case test (no `[[case]]`), the default is `output_<testname>.log`,
overridable with a top-level `log` key next to `[run]` / `[validate]`.

## Expected Files

The expected file format should **match the test output format** to avoid unnecessary conversions. Supported formats include:

| Format | Expected file | Description |
|---|---|---|
| Plain text | `expected.txt` | Exact line-by-line text comparison |
| JSON Lines | `expected.jsonlines` | One JSON object per line, with per-field tolerance |

More formats can be added in `run_tests.py` as needed.

## `test.toml` Reference

### Single-case (simple)

Most tests only need `[run]` + `[validate]` — no `[[case]]` needed:

```toml
# ─── Optional ──────────────────────────────────
name = "Short test name"         # human label; defaults to directory name
description = "..."              # explanation of what this test does
log = "custom_name.log"          # log filename; default output_<testname>.log

[build]                          # only if the test needs compilation
command = "cmake --build build --target my_target"
build_path = "tests"             # (optional) directory to run build in; defaults to test directory

# ─── Required ──────────────────────────────────
[run]
command = "./some/executable"    # command to run
stdin = "input.json"             # (optional) stdin input file

[validate]
kind = "txt" | "jsonlines"       # output format
expected = "expected.txt"       # file to compare against
# tolerance applies to jsonlines only; txt ignores it:
tolerance = 0.01                 # global tolerance for all numeric fields
```

Without `[[case]]`

### Tolerance

`tolerance` can be a single number (applies to all numeric fields) or a per-field dict:

```toml
# Global tolerance — every numeric field allows ±0.01
tolerance = 0.01

# Per-field tolerance — different tolerance per field
tolerance = { x_slider = 0.01, y_slider = 0.001 }

# Mixed — global default, but override specific fields
tolerance = { __all__ = 0.01, x_slider = 0.0 }
```

`__all__` sets a fallback for any field not explicitly listed. A field set to `0.0` forces exact match (no tolerance)., `run_tests.py` treats the whole config as one implicit case.

### Multi-case (same binary, different inputs)

When the same binary should be tested with several inputs, use `[[case]]`:

```toml
name = "Crank-slider tests"
description = "Same solver, multiple input configurations"

[build]
command = "cmake --build build --target mf_engine"

[[case]]
name = "Default config"
run = { command = "../../build/debug/bin/mf_engine", stdin = "input_default.json" }
validate = { kind = "jsonlines", expected = "expected_default.jsonlines", tolerance = 0.01 }

[[case]]
name = "High speed"
run = { command = "../../build/debug/bin/mf_engine", stdin = "input_highspeed.json" }
validate = { kind = "jsonlines", expected = "expected_highspeed.jsonlines", tolerance = 0.05 }
```

`run_tests.py` iterates over all `[[case]]` entries and runs/validates each one.
Each `[[case]]` may also set `log = "..."` for its own output log filename
(default `output_<case>.log`; see [Output Logs](#output-logs)).

## Adding a New Test

```
tests/testX_xxx/
├── test.toml       # test configuration
├── expected.*      # expected output (format matches the test's output)
└── ...             # other files (e.g., input, source code)
```

See `test0_crank_slider/test.toml` for a complete example.

