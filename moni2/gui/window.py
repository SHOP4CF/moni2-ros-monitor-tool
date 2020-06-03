import logging
from PyQt5.QtWidgets import (
    QMainWindow,
    QDockWidget,
    QWidget,
    QVBoxLayout,
    QLabel,
    QListWidget
)
from PyQt5.QtCore import Qt


class MonitorWindow(QMainWindow):

    def __init__(self, log: logging.Logger, parent=None):
        super().__init__(parent)
        self.log = log

        self.label = QLabel("hejsa")

        self.init_components()
        self.init_ui()

    def init_components(self):
        pass

    def init_ui(self):
        self.setWindowTitle("Moni2")
        self.setGeometry(100, 100, 640, 480)

        widget = QWidget(self)
        layout = QVBoxLayout(widget)
        self.setCentralWidget(widget)
        layout.addWidget(self.label)

        dockWidget = QDockWidget('Dock test', self)
        listWidget = QListWidget()
        listWidget.addItem('Google')
        listWidget.addItem('Apple')
        dockWidget.setWidget(listWidget)
        self.addDockWidget(Qt.RightDockWidgetArea, dockWidget)

        self.show()

    def set_text(self, text: str):
        self.label.setText(text)
