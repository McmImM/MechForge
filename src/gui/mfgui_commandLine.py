from typing import Literal

from mf_commands import (
    COMMANDS,
    NORMAL_COMMANDS,
    PromptKind,
    build_prompts,
    flag_value_pending,
    is_flag,
)
from mf_remote import RemoteCore

cmd: list[str] = []  # confirmed tokens
cur_token: str = ""  # current token being typed
cursor: int = -1  # cursor position in cur_token.
# In other words, the letter index of cur_token that the cursor is on.
# -1 means cur_token is empty and the cursor is at the beginning of the token.

# All valid token choices for now.
all_valid_choices: list[tuple[str, str, PromptKind]] = build_prompts([])

# possible completions for cur_token (if cur_token is not complete);
# possible completions for the next token (if cur_token is complete).
choices: list[tuple[str, str, PromptKind]] = all_valid_choices.copy()

choice: int = -1  # index of the currently selected choice in choices.
# -1 means no choice is selected.

# the prompt for position args or flag values;
# None means choices are for the next token.
# Otherwise, the prompt is for the current token.
active_prompt: tuple[str, str, PromptKind] | None = None


def is_token_valid(token: str) -> bool:
    if not token:
        return False

    if active_prompt is not None:
        # We cannot allow the user to enter a flag's name;
        # This will cause the flag name to be silently accepted as the value
        # of the previous flag (e.g. "--posX --v"), producing a malformed
        # command that only fails at execution time with a confusing error.
        if is_flag(token, cmd):
            return False
        if token.startswith("--"):
            return False
        return True

    return any(token == label for label, _, _ in all_valid_choices)


def is_cmd_valid() -> bool:
    # We assume that every token in cmd is valid,
    # since we only add valid tokens to cmd.
    # So we only need to check if the command is complete.
    if not cmd:
        return False

    if flag_value_pending(cmd) is not None:
        # If a flag value is pending, the command is not complete and therefore not valid.
        return False

    for _, _, kind in all_valid_choices:
        # If any of the valid choices for the next token is a required token or a choice token,
        # then the command is not complete and therefore not valid.
        if kind in (PromptKind.REQUIRED, PromptKind.CHOICE):
            return False

    return True


def update_all_valid_choices():
    global all_valid_choices
    all_valid_choices = build_prompts(cmd)


def update_choices():
    global choices, choice, active_prompt

    # Reset the selected choice index whenever choices are updated.
    choice = -1

    # flag values
    if (pending := flag_value_pending(cmd)) is not None:
        # If a flag value is pending,
        # we expect the user to enter a value for the flag,
        # so we don't provide any choices but set the active prompt.
        choices = []
        flag, remaining = pending
        active_prompt = (
            f"value for {flag} ({remaining} left)",
            "",
            PromptKind.REQUIRED,
        )
        return

    # positional arguments
    for token, desc, kind in all_valid_choices:
        if kind is PromptKind.REQUIRED and not is_flag(token, cmd):
            # If there is a required positional token,
            # we expect the user to enter it next,
            # so we don't provide any choices but set the active prompt.
            choices = []
            active_prompt = (token, desc, kind)
            return

    active_prompt = None

    # REQUIRED flags: only keep the first one (auto-selected by the GUI).
    # Iterate all_valid_choices (the CURRENT set), not choices (the previous
    # round's set) -- choices may still hold stale values here.
    for c in all_valid_choices:
        # Since we have already filtered out required positional tokens above,
        # the only required tokens left are required flags.
        if c[2] is PromptKind.REQUIRED:
            choices = [c]
            choice = 0  # auto-select it; the GUI confirms it automatically
            return

    # normal choices
    if cur_token:
        # If cur_token is not empty, filter all_valid_choices to those that start with cur_token.
        choices = [
            (token, desc, kind)
            for token, desc, kind in all_valid_choices
            if token.startswith(cur_token)
        ]
    else:
        # If cur_token is empty, all valid choices are possible.
        choices = all_valid_choices.copy()

    if choices and choices[0][2] is PromptKind.CHOICE:
        # If there are any non-optional choices, select the first one by default.
        choice = 0


def get_choices() -> list[tuple[str, str, PromptKind]]:
    return choices


def _clean_token():
    global cur_token, cursor
    cur_token = ""
    cursor = -1


def pop_token():
    global cmd, cur_token, cursor
    if cmd:
        cur_token = cmd.pop()
        cursor = len(cur_token) - 1
    else:
        _clean_token()

    update_all_valid_choices()
    update_choices()


def remove_letter():
    global cur_token, cursor
    if cursor >= 0:
        cur_token = cur_token[:cursor] + cur_token[cursor + 1 :]
        cursor -= 1
        update_choices()


def add_letter(letter: str):
    global cur_token, cursor
    cur_token = cur_token[: cursor + 1] + letter + cur_token[cursor + 1 :]
    cursor += 1
    update_choices()


def change_choice(direction: Literal[-1, 1]):
    # +1 for next choice, -1 for previous choice
    global choice
    if choices:
        choice = (choice + direction) % len(choices)


def set_choice(index: int):
    global choice
    if 0 <= index < len(choices):
        choice = index


def confirm_token() -> tuple[bool, str]:
    global cmd, cur_token, cursor

    # add the current token/choice to the command.
    if is_token_valid(cur_token):
        cmd.append(cur_token)
    elif choices and choice >= 0:
        cmd.append(choices[choice][0])
    else:
        t = cur_token
        _clean_token()  # clean the invalid token.
        update_all_valid_choices()
        update_choices()
        return False, f"Invalid token: {t}"

    _clean_token()
    update_all_valid_choices()
    update_choices()
    return True, ""


def run_cmd(client: RemoteCore):
    if cmd[0] in NORMAL_COMMANDS.keys():
        client.send_command(" ".join(cmd))
    elif cmd[0] == "solve":
        pass  # TODO: implement solve command


def confirm_command(client: RemoteCore) -> tuple[bool, str]:
    global cmd, cur_token, cursor

    if is_cmd_valid():
        run_cmd(client)
        cmd.clear()
        _clean_token()
        update_all_valid_choices()
        update_choices()
        return True, ""

    return False, "Invalid command"


def confirm(client: RemoteCore) -> tuple[bool, str]:
    global cmd, cur_token, cursor

    if cur_token or (choices and choice >= 0):
        # If there is a current token or a valid choice, confirm it.
        success, message = confirm_token()
        if not success:
            return False, message

        if active_prompt is not None:
            # Just confirmed a token; now a value is pending (flag value
            # or positional). The command is not complete yet.
            return True, ""

        if not all_valid_choices:
            # If there are no more valid choices,
            # we can confirm the command.
            return confirm_command(client)

        # If there are still valid choices left,
        # we cannot confirm the command yet.
        # Since the user has confirmed a token,
        # we should return true.
        return True, ""

    if active_prompt is not None:
        # If there is an active prompt, we cannot confirm the command yet.
        return False, f"required: {active_prompt[0]}"

    return confirm_command(client)
