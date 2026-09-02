import sys

from PySide6.QtCore import Signal, Slot, QObject, QEvent, Qt
from PySide6.QtWidgets import QMainWindow, QApplication
from PySide6.QtGui import QKeyEvent

from ui_mainWindow import Ui_MainWindow

from mf_remote import ensure_server


class MainWindow(QMainWindow):
    server_gone_signal = Signal()

    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.client = ensure_server()
        self.client.monitor(self.on_server_gone)
        self.server_gone_signal.connect(self.do_when_server_gone)

        cmdInput = self.ui.cmdInput
        hints = self.ui.hints
        cmdInput.set_client(self.client)
        cmdInput.hintsChanged_choices.connect(hints.update_hints_choices)
        cmdInput.hintsChanged_infos.connect(hints.update_hints_infos)
        cmdInput.setHintChoice.connect(hints.set_hint_choice)
        hints.choice_confirmed.connect(cmdInput.confirm_input)

        app = QApplication.instance()
        assert app is not None
        app.installEventFilter(self)

    # [notice] not a qt slot.
    def on_server_gone(self):
        print("Server is gone!")
        self.server_gone_signal.emit()

    @Slot()
    def do_when_server_gone(self):
        print("Server is gone! (slot)")
        self.ui.statusbar.showMessage("Server is gone!")

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        if isinstance(event, QKeyEvent) and event.type() == QEvent.Type.KeyPress:
            # If the command input has focus, let it handle the key press.
            if self.ui.cmdInput.hasFocus():
                return False

            # If any modifier key is pressed, ignore the event.
            mods = event.modifiers()
            if mods & (
                Qt.KeyboardModifier.ControlModifier
                | Qt.KeyboardModifier.AltModifier
                | Qt.KeyboardModifier.MetaModifier
            ):
                return False

            # If the event has text, forward it to the command input.
            if event.text():
                self.ui.cmdInput.setFocus()
                # QApplication.sendEvent(self.ui.cmdInput, event)
                return False

        return False


if __name__ == "__main__":
    app = QApplication([])
    mainWindow = MainWindow()
    mainWindow.resize(800, 600)
    mainWindow.show()

    sys.exit(app.exec())
