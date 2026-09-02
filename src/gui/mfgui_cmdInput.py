"""Command input widget for the MechForge GUI.

A read-only QLineEdit that forwards every key press to the mfgui_commandLine
model. The model owns the token state; this widget only renders it and emits
stateChanged so the MainWindow can refresh the hints.
"""

from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtWidgets import QLineEdit
from PySide6.QtGui import QKeyEvent

import mfgui_commandLine as cl
from mf_remote import ensure_server, RemoteCore

client = ensure_server()  # for test


class MFGuiCmdInput(QLineEdit):
    hintsChanged_choices = Signal(list)  # list[tuple[str, str, cl.PromptKind]])
    hintsChanged_infos = Signal(tuple)  # tuple[str, str, cl.PromptKind])
    setHintChoice = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.last_hints: (
            list[tuple[str, str, cl.PromptKind]] | tuple[str, str, cl.PromptKind] | None
        ) = None
        self.client: RemoteCore | None = None

    def set_client(self, client: RemoteCore):
        self.client = client

    def _judge_hints_changed(self):
        """Judge if the hints have changed and emit signals accordingly."""
        if cl.active_prompt is None:
            if self.last_hints != cl.choices:
                self.last_hints = cl.choices
                self.hintsChanged_choices.emit(cl.choices)
                self.setHintChoice.emit(cl.choice)
        else:
            if self.last_hints != cl.active_prompt:
                self.last_hints = cl.active_prompt
                self.hintsChanged_infos.emit(cl.active_prompt)

    def _confirm_input(self):
        if self.client is None:
            raise RuntimeError("Client is not set for MFGuiCmdInput.")
        ok, msg = cl.confirm(self.client)
        if ok:
            self.setText(cl.text() + " ")
        else:
            self.setText(cl.text())

    @Slot()
    def confirm_input(self):
        self._confirm_input()
        self._judge_hints_changed()

    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        mods = event.modifiers()
        no_modifier = mods == Qt.KeyboardModifier.NoModifier

        event_done = False

        # arrow keys
        if key == Qt.Key.Key_Left and no_modifier and cl.active_prompt is None:
            cl.change_choice(-1)
            self.setHintChoice.emit(cl.choice)
            event_done = True
        elif key == Qt.Key.Key_Right and no_modifier and cl.active_prompt is None:
            cl.change_choice(1)
            self.setHintChoice.emit(cl.choice)
            event_done = True

        # backspace
        elif key == Qt.Key.Key_Backspace:
            if mods & Qt.KeyboardModifier.ShiftModifier:
                cl.pop_letter()
            else:
                cl.pop_token()

            self.setText(cl.text())
            event_done = True

        # space
        elif key == Qt.Key.Key_Space and no_modifier:
            self._confirm_input()
            event_done = True

        # text input
        elif c := event.text():
            cl.add_letter(c)
            # We need to update the text in the QLineEdit, so event is not done yet.
            # It will be done in super().keyPressEvent(event)

        self._judge_hints_changed()
        if event_done:
            event.accept()
        else:
            super().keyPressEvent(event)

        # Debugging output
        # print(f"cursor: {cl.cursor}, cur_token: {cl.cur_token}, cmd: {cl.cmd}")
