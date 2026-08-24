import sys
from PySide6.QtWidgets import QMainWindow, QApplication
from PySide6.QtCore import Signal, Slot

from ui_mainWindow import Ui_MainWindow

from mf_remote import RemoteCore, ensure_server


class MainWindow(QMainWindow):
    server_gone_signal = Signal()

    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.client = ensure_server()
        self.client.monitor(self.on_server_gone)
        self.server_gone_signal.connect(self.on_server_gone_slot)

    # [notice] not a qt slot.
    def on_server_gone(self):
        print("Server is gone!")
        self.server_gone_signal.emit()

    @Slot()
    def on_server_gone_slot(self):
        print("Server is gone! (slot)")
        self.ui.statusbar.showMessage("Server is gone!")


if __name__ == "__main__":
    app = QApplication([])
    mainWindow = MainWindow()
    mainWindow.resize(800, 600)
    mainWindow.show()

    sys.exit(app.exec())
