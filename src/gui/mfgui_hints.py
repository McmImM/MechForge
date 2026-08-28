from PySide6.QtCore import Signal
from PySide6.QtWidgets import QListWidget, QListWidgetItem


class MFGuiHints(QListWidget):
    """Floating suggestion list anchored above the command line."""

    optionChosen = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.www: str
