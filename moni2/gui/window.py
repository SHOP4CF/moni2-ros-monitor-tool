import logging
from PyQt5.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QLabel
)


class MonitorWindow(QMainWindow):

    def __init__(self, log: logging.Logger, parent=None):
        super().__init__(parent)
        self.log = log

        self.setWindowTitle("Moni2")
        self.setGeometry(100, 100, 640, 480)

        widget = QWidget(self)
        layout = QVBoxLayout(widget)
        self.setCentralWidget(widget)

        self.label = QLabel("hejsa")
        layout.addWidget(self.label)

        self.show()

    def set_text(self, text: str):
        self.label.setText(text)
