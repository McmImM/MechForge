"""Test cases for the GUI command-line model (mfgui_commandLine).

Each case prints a small, deterministic transcript to stdout; run_tests.py
compares it against the matching expected_*.txt file. Run a single case with:

    python3 run_cases.py <case>

The model is pure Python (no Qt), so these tests exercise the token state
machine directly: typing, confirming, flag values, positional prompts,
skipping optionals, and command execution via a fake client.
"""

import sys
from pathlib import Path

# The mf_*.py modules are flattened into build/debug/lib/ by CMake.
BUILD_LIB = Path(__file__).resolve().parent.parent.parent / "build" / "debug" / "lib"
sys.path.insert(0, str(BUILD_LIB))

import mfgui_commandLine as cl  # noqa: E402  # type: ignore[import-not-found]
from mf_commands import PromptKind  # noqa: E402  # type: ignore[import-not-found]


# --- fake client -------------------------------------------------------------


class FakeClient:
    """Records send_command calls; never talks to a real server."""

    def __init__(self):
        self.commands: list[str] = []

    def send_command(self, line: str):
        self.commands.append(line)
        return None


# --- helpers -----------------------------------------------------------------


def reset():
    """Reset the module-level state to a clean slate."""
    cl.cmd = []
    cl.cur_token = ""
    cl.cursor = -1
    cl.all_valid_choices = cl.build_prompts([])
    cl.choices = cl.all_valid_choices.copy()
    cl.choice = -1
    cl.active_prompt = None


def type_text(text: str):
    """Feed printable characters through add_letter."""
    for ch in text:
        cl.add_letter(ch)


def press_space(client) -> tuple[bool, str]:
    """Press space (confirm). Returns (ok, message)."""
    return cl.confirm(client)


def state() -> str:
    """Render the current model state as a compact string."""
    parts = []
    parts.append(f"cmd={cl.cmd!r}")
    parts.append(f"cur={cl.cur_token!r}")
    parts.append(f"choices={[c[0] for c in cl.choices]!r}")
    if cl.active_prompt is not None:
        parts.append(f"prompt={cl.active_prompt[0]!r}")
    return " | ".join(parts)


def kind_of(label: str) -> str:
    """Find the PromptKind of a label in all_valid_choices."""
    for l, _, k in cl.all_valid_choices:
        if l == label:
            return k.value
    return "?"


# --- cases -------------------------------------------------------------------


def case_completion():
    """Command-name completion and CHOICE selection."""
    reset()
    print("== command-name completion ==")
    type_text("j")
    print(state())
    print(f"choices: {[c[0] for c in cl.choices]}")

    print("\n== confirm jointAdd, then TYPE choices ==")
    ok, msg = press_space(FakeClient())
    print(f"space -> ok={ok} msg={msg!r}")
    print(state())
    print(f"choices: {[c[0] for c in cl.choices]}")
    print(f"choice={cl.choice} (auto-selected first CHOICE)")


def case_positional():
    """Positional args are prompted one at a time (active_prompt)."""
    reset()
    client = FakeClient()

    print("== jointAdd revolute: positional X, Y ==")
    type_text("jointAdd")
    press_space(client)
    print(state())

    # TYPE is a CHOICE; auto-selected first (ground). Select revolute instead.
    cl.set_choice(3)  # ground=0 fixed=1 revolute=2 prismatic=3 free=4
    press_space(client)
    print(state())
    print(f"prompt={cl.active_prompt[0]!r} (should be X)")

    type_text("1")
    press_space(client)
    print(state())
    print(f"prompt={cl.active_prompt[0]!r} (should be Y)")

    type_text("2")
    press_space(client)
    print(state())
    print(f"choices: {[c[0] for c in cl.choices]} (should be flags only)")


def case_flag_value():
    """A value-taking flag (--vel, nargs=2) prompts for its values."""
    reset()
    client = FakeClient()

    print("== jointAdd revolute 1 2 --vel: two values ==")
    type_text("jointAdd")
    press_space(client)
    cl.set_choice(2)  # revolute
    press_space(client)
    type_text("1")
    press_space(client)
    type_text("2")
    press_space(client)
    print(state())

    type_text("--vel")
    press_space(client)
    print(state())
    print(f"prompt={cl.active_prompt[0]!r} (should be value for --vel (2 left))")

    type_text("1")
    press_space(client)
    print(state())
    print(f"prompt={cl.active_prompt[0]!r} (should be value for --vel (1 left))")

    type_text("0")
    press_space(client)
    print(state())
    print(f"choices: {[c[0] for c in cl.choices]} (should be --acc only)")


def case_required_flag():
    """REQUIRED flags (drivingAdd position --posX) are auto-selected."""
    reset()
    client = FakeClient()

    print("== drivingAdd position: REQUIRED flags ==")
    type_text("drivingAdd")
    press_space(client)
    print(state())

    cl.set_choice(0)  # position
    press_space(client)
    print(state())
    print(f"prompt={cl.active_prompt[0]!r} (should be JOINTA)")

    type_text("1")
    press_space(client)
    print(state())
    print(f"prompt={cl.active_prompt[0]!r} (should be JOINTB)")

    type_text("2")
    press_space(client)
    print(state())
    print(f"choices: {[c[0] for c in cl.choices]} (should be [--posX], auto-selected)")

    # GUI auto-confirms the single REQUIRED flag; simulate that here.
    ok, msg = press_space(client)
    print(f"space -> ok={ok} msg={msg!r}")
    print(state())
    print(f"prompt={cl.active_prompt[0]!r} (should be value for --posX (1 left))")


def case_skip_optional():
    """Space with only OPTIONAL flags left executes the command."""
    reset()
    client = FakeClient()

    print("== linkAdd 1 2 10: skip --name, execute ==")
    type_text("linkAdd")
    press_space(client)
    print(state())
    print(f"prompt={cl.active_prompt[0]!r} (should be JOINTA)")

    type_text("1")
    press_space(client)
    print(f"prompt={cl.active_prompt[0]!r} (should be JOINTB)")
    type_text("2")
    press_space(client)
    print(f"prompt={cl.active_prompt[0]!r} (should be LENGTH)")
    type_text("10")
    press_space(client)
    print(state())
    print(f"choices: {[c[0] for c in cl.choices]} (should be [--name], NOT auto-selected)")

    ok, msg = press_space(client)
    print(f"space -> ok={ok} msg={msg!r}")
    print(f"executed: {client.commands!r}")
    print(state())


def case_negative():
    """Negative numbers and negative expressions are valid free text."""
    reset()
    client = FakeClient()

    print("== jointAdd revolute -1 -2: negative positionals ==")
    type_text("jointAdd")
    press_space(client)
    cl.set_choice(2)  # revolute
    press_space(client)
    type_text("-1")
    print(f"is_token_valid('-1') = {cl.is_token_valid('-1')}")
    press_space(client)
    type_text("-2")
    print(f"is_token_valid('-2') = {cl.is_token_valid('-2')}")
    press_space(client)
    print(state())

    print("\n== drivingAdd position 1 2 --posX -t*2: negative expression ==")
    reset()
    type_text("drivingAdd")
    press_space(client)
    cl.set_choice(0)  # position
    press_space(client)
    type_text("1")
    press_space(client)
    type_text("2")
    press_space(client)
    press_space(client)  # auto-confirm --posX
    type_text("-t*2")
    print(f"is_token_valid('-t*2') = {cl.is_token_valid('-t*2')}")
    press_space(client)
    print(state())


def case_reject_flag_name():
    """Bug 2 regression: flag names rejected while a value is pending."""
    reset()
    client = FakeClient()

    print("== --posX value pending: '--v' must be rejected ==")
    type_text("drivingAdd")
    press_space(client)
    cl.set_choice(0)  # position
    press_space(client)
    type_text("1")
    press_space(client)
    type_text("2")
    press_space(client)
    press_space(client)  # auto-confirm --posX
    print(state())
    print(f"prompt={cl.active_prompt[0]!r} (value for --posX pending)")

    type_text("--v")
    print(f"is_token_valid('--v') = {cl.is_token_valid('--v')} (should be False)")
    ok, msg = press_space(client)
    print(f"space -> ok={ok} msg={msg!r} (should be False, Invalid token)")
    print(state())


def case_backspace():
    """Backspace pops the last token back into cur_token."""
    reset()
    client = FakeClient()

    print("== pop_token: jointAdd revolute 1 -> back to 1 ==")
    type_text("jointAdd")
    press_space(client)
    cl.set_choice(2)  # revolute
    press_space(client)
    type_text("1")
    press_space(client)
    print(state())

    cl.pop_token()
    print(state())
    print(f"cur={cl.cur_token!r} (should be '1')")
    print(f"prompt={cl.active_prompt[0]!r} (should be X again)")


def case_full_flow():
    """End-to-end: jointAdd revolute 1 2 --vel 1 0, then execute."""
    reset()
    client = FakeClient()

    print("== full flow: jointAdd revolute 1 2 --vel 1 0 ==")
    type_text("jointAdd")
    press_space(client)
    cl.set_choice(2)  # revolute
    press_space(client)
    type_text("1")
    press_space(client)
    type_text("2")
    press_space(client)
    type_text("--vel")
    press_space(client)
    type_text("1")
    press_space(client)
    type_text("0")
    press_space(client)
    print(state())
    print(f"choices: {[c[0] for c in cl.choices]} (should be [--acc], optional)")

    ok, msg = press_space(client)
    print(f"space -> ok={ok} msg={msg!r}")
    print(f"executed: {client.commands!r}")
    print(state())


CASES = {
    "completion": case_completion,
    "positional": case_positional,
    "flag_value": case_flag_value,
    "required_flag": case_required_flag,
    "skip_optional": case_skip_optional,
    "negative": case_negative,
    "reject_flag_name": case_reject_flag_name,
    "backspace": case_backspace,
    "full_flow": case_full_flow,
}


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else None
    if target is None:
        for name, fn in CASES.items():
            print(f"\n########## {name} ##########")
            fn()
    else:
        fn = CASES.get(target)
        if fn is None:
            print(f"unknown case: {target}")
            sys.exit(1)
        fn()


if __name__ == "__main__":
    main()