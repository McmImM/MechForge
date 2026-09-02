from enum import Enum
from typing import TYPE_CHECKING, cast

if TYPE_CHECKING:
    from mf_gui import MainWindow

from PySide6.QtCore import Slot, Signal
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import QListWidgetItem, QWidget, QApplication

import ui_hints
import mfgui_commandLine as cl


class MFGuiHints(QWidget):
    choice_confirmed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = ui_hints.Ui_Hints()
        self.ui.setupUi(self)
        self.descriptionWidget = self.ui.description
        self.choicesWidget = self.ui.choices
        # self.choicesWidget.setStyleSheet("QListWidget { border: none; outline: none; }")
        self.choicesWidget.itemClicked.connect(self.do_when_choice_clicked)
        self.choicesWidget.itemDoubleClicked.connect(self.do_when_choice_double_clicked)

        self.mode = self.Mode.NONE
        self.choices: list[tuple[str, str, cl.PromptKind]] | None = None
        self.infos: tuple[str, str, cl.PromptKind] | None = None

    class Mode(Enum):
        CHOICES = 1
        INFOS = 2
        NONE = 3

    @Slot(QListWidgetItem)
    def do_when_choice_clicked(self, item: QListWidgetItem):
        index = self.choicesWidget.row(item)
        cl.set_choice(index)
        self.set_hint_choice(index)

    @Slot(QListWidgetItem)
    def do_when_choice_double_clicked(self, item: QListWidgetItem):
        index = self.choicesWidget.row(item)
        cl.set_choice(index)
        self.set_hint_choice(index)
        self.choice_confirmed.emit()  # Emit the signal when a choice is confirmed

    @Slot(list)
    def update_hints_choices(self, choices: list[tuple[str, str, cl.PromptKind]]):
        self.choicesWidget.clear()
        self.mode = self.Mode.CHOICES
        self.choices = choices
        for label, help, kind in choices:
            self.choicesWidget.addItem(label)

    @Slot(tuple)
    def update_hints_infos(self, info: tuple[str, str, cl.PromptKind]):
        self.choicesWidget.clear()
        self.mode = self.Mode.INFOS
        self.infos = info
        self.descriptionWidget.setText(info[0] + ": " + info[1])

    @Slot(int)
    def set_hint_choice(self, index: int):
        # This slot should only be called when the mode is CHOICES.
        assert self.choices is not None

        for i in range(self.choicesWidget.count()):
            self.choicesWidget.item(i).setSelected(i == index)

        self.descriptionWidget.setText(self.choices[index][1])

    def keyPressEvent(self, event: QKeyEvent) -> None:
        # send all key events to the command input,
        # so the arrow keys work correctly even when the hints widget has focus.
        main_window = cast("MainWindow", self.window())
        cmdInput = main_window.ui.cmdInput
        QApplication.sendEvent(cmdInput, event)
        event.accept()
