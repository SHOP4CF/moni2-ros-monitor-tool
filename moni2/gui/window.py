import sys
import logging
from typing import Callable
from PyQt5.QtWidgets import (
    QMainWindow,
    QDockWidget,
    QWidget,
    QVBoxLayout,
    QLabel,
    QListWidget,
    QAction,
    QMenu,
)
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import Qt

from rcl_interfaces.msg import Log
from moni2.gui.log_widget import LogWidget


class MonitorWindow(QMainWindow):

    APP_NAME = "Socket tester"

    STATUSBAR_MESSAGE_TYPE = {
        'error': ('255,0,0', 'black'),
        'warning': ('255,255,0', 'black'),
        'info': ('255,255,255', 'black'),
    }

    def __init__(self, log: logging.Logger, parent=None):
        super().__init__(parent)
        self.log = log

        self.label = QLabel("hejsa")

        self.log_widget: LogWidget = None

        self.init_components()
        self.init_ui()
        self.init_menu()
        self.message("Initialized")

    def init_components(self):
        self.log_widget = LogWidget(self.log.get_child("LogWidget"), self)

    def init_ui(self):
        self.log.info("Initializing UI...")
        self.setWindowTitle(self.APP_NAME)
        self.setGeometry(100, 100, 640, 480)

        widget = QWidget(self)
        layout = QVBoxLayout(widget)
        self.setCentralWidget(widget)
        layout.addWidget(self.label)

        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_widget)

        dockWidget = QDockWidget('Dock test', self)
        listWidget = QListWidget()
        listWidget.addItem('Google')
        listWidget.addItem('Apple')
        dockWidget.setWidget(listWidget)
        self.addDockWidget(Qt.RightDockWidgetArea, dockWidget)

        self.show()

    def init_menu(self):
        self.log.info("Initializing menu...")
        main_menu = self.menuBar()
        file_menu = main_menu.addMenu("&File")
        self._add_menu_action(file_menu, "Save config", callback=lambda: self.message("TODO"))
        self._add_menu_action(file_menu, "Load config", callback=lambda: self.message("TODO"))
        file_menu.addSeparator()
        self._add_menu_action(file_menu, "Settings", callback=lambda: self.message("TODO"))
        file_menu.addSeparator()
        self._add_menu_action(file_menu, "&Quit", self.close_application, "Exit the application", "Ctrl+Q")

        views_menu = main_menu.addMenu("&Views")
        views_menu.addAction(self.log_widget.toggleViewAction())

    def toggle_log_view(self):
        self.message("toggle log view")

    def log_widget_visible(self, visible: bool):
        self.log.info(f"Log widget visible: {visible}")

    @staticmethod
    def _add_menu_action(menu: QMenu, name: str, callback: Callable, status_tip="", shortcut="") -> QAction:
        action = QAction(name, menu)
        if shortcut:
            action.setShortcut(shortcut)
        if status_tip:
            action.setStatusTip(status_tip)
        action.triggered.connect(callback)
        menu.addAction(action)
        return action

    def set_text(self, text: str):
        self.label.setText(text)

    def received_log(self, log: Log):
        self.log_widget.received_log(log)

    @pyqtSlot(str, str)
    def message(self, message: str, message_type='info'):
        assert message_type in self.STATUSBAR_MESSAGE_TYPE, f"message_type is: {message_type}, " \
            f"should be one of the following: {list(self.STATUSBAR_MESSAGE_TYPE.keys())}"

        background_color, text_color = self.STATUSBAR_MESSAGE_TYPE[message_type]

        style = f"QStatusBar{{background:rgba({background_color},255);color:{text_color};font-weight:bold;}}"
        self.statusBar().setStyleSheet(style)
        self.statusBar().showMessage(message)
        self.log.info(message)

    def close_application(self):
        self.message("Closing the application")
        sys.exit()
