import sys
import logging
from typing import Callable
from PyQt5.QtWidgets import (
    QMainWindow,
    QTabWidget,
    QWidget,
    QVBoxLayout,
    QAction,
    QMenu,
)
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import Qt

from rcl_interfaces.msg import Log
from moni2.node_info import NodeInfo
from moni2.gui.log_widget import LogWidget
from moni2.gui.node_widget import NodeWidget
from moni2.gui.node_detail_widget import NodeDetailWidget


class MonitorWindow(QMainWindow):

    APP_NAME = "Moni2"

    STATUSBAR_MESSAGE_TYPE = {
        'error': ('255,0,0', 'black'),
        'warning': ('255,255,0', 'black'),
        'info': ('255,255,255', 'black'),
    }

    def __init__(self, log: logging.Logger, parent=None):
        super().__init__(parent)
        self.log = log

        self.log_widget: LogWidget = None
        self.node_widget: NodeWidget = None
        self.node_detail_widget: NodeDetailWidget = None

        self.init_components()
        self.init_ui()
        self.init_menu()
        self.message("Initialized")

    def init_components(self):
        self.log_widget = LogWidget(self.log.get_child("LogWidget"), self)
        self.node_widget = NodeWidget(self.log.get_child("NodeWidget"), self)
        self.node_detail_widget = NodeDetailWidget(self.log.get_child("NodeDetailWidget"), self)

        self.node_widget.node_clicked.connect(self.node_detail_widget.update_info)

    def init_ui(self):
        self.log.info("Initializing UI...")
        self.setWindowTitle(self.APP_NAME)
        self.setGeometry(100, 100, 640, 480)

        widget = QWidget(self)
        layout = QVBoxLayout(widget)
        self.setCentralWidget(widget)

        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_widget)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.node_widget)
        self.addDockWidget(Qt.RightDockWidgetArea, self.node_detail_widget)
        self.setTabPosition(Qt.AllDockWidgetAreas, QTabWidget.North)

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
        views_menu.addAction(self.node_widget.toggleViewAction())
        views_menu.addAction(self.node_detail_widget.toggleViewAction())
        views_menu.addAction(self.log_widget.toggleViewAction())

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

    def received_log(self, log: Log):
        self.log_widget.received_log(log)

    def update_node(self, node: NodeInfo):
        self.node_widget.update_node(node)

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
