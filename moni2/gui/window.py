import sys
import os
import logging
from typing import Callable
from PyQt5.QtWidgets import (
    QMainWindow,
    QTabWidget,
    QAction,
    QMenu,
)
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot, pyqtSignal
from PyQt5.QtCore import Qt

from rcl_interfaces.msg import Log
from moni2.node_info import NodeInfo
from moni2.gui.log_widget import LogWidget
from moni2.gui.node_dialog import EditNodeListDialog
from moni2.gui.node_model import NodeModel


class MonitorWindow(QMainWindow):

    APP_NAME = "Moni2"

    node_updated = pyqtSignal(NodeInfo)

    STATUSBAR_MESSAGE_TYPE = {
        'error': ('255,0,0', 'black'),
        'warning': ('255,255,0', 'black'),
        'info': ('255,255,255', 'black'),
    }

    def __init__(self, log: logging.Logger, image_path: str, parent=None):
        super().__init__(parent)
        self.log = log
        self.image_path = image_path

        self.log_widget: LogWidget = None
        self.node_model: NodeModel = None

        self.init_components()
        self.init_ui()
        self.init_menu()
        self.message("Initialized")

    def init_components(self):
        self.log_widget = LogWidget(self.log.get_child("LogWidget"), self)
        self.node_model = NodeModel(self.log.get_child("NodeModel"), self)

        self.log_widget.warning_counter.connect(self.node_model.log_warning_count)
        self.log_widget.error_counter.connect(self.node_model.log_error_count)

    def init_ui(self):
        self.log.info("Initializing UI...")
        self.setWindowTitle(self.APP_NAME)
        self.setGeometry(100, 100, 800, 600)

        logo_path = self.image_path + os.path.sep + 'logo.png'
        self.setWindowIcon(QIcon(logo_path))

        self.setCentralWidget(self.node_model)

        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_widget)
        self.setTabPosition(Qt.AllDockWidgetAreas, QTabWidget.North)

        self.show()

    def init_menu(self):
        self.log.info("Initializing menu...")
        main_menu = self.menuBar()
        file_menu = main_menu.addMenu("&File")
        self._add_menu_action(file_menu, "Save config", callback=lambda: self.message("TODO"))
        self._add_menu_action(file_menu, "Edit config", callback=self._edit_nodes)
        self._add_menu_action(file_menu, "Load config", callback=lambda: self.message("TODO"))
        file_menu.addSeparator()
        self._add_menu_action(file_menu, "Settings", callback=lambda: self.message("TODO"))
        file_menu.addSeparator()
        self._add_menu_action(file_menu, "&Quit", self.close_application, "Exit the application", "Ctrl+Q")

        views_menu = main_menu.addMenu("&Views")
        views_menu.addAction(self.log_widget.toggleViewAction())

    def _edit_nodes(self):
        self.log.info("Editing list of nodes")
        dialog = EditNodeListDialog(self.log.get_child("EditNodeListDialog"), self)
        self.node_updated.connect(dialog.node_updated)
        dialog.node_list_updated.connect(self.node_model.node_list_updated)
        dialog.exec()

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
        self.node_updated.emit(node)

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
