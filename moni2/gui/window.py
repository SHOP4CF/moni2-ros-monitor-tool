import sys
import os
import logging
from PyQt5.QtWidgets import (
    QMainWindow,
    QTabWidget,
    QAction,
)
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot, pyqtSignal, Qt

from rcl_interfaces.msg import Log
from moni2.node_info import NodeInfo
from moni2.gui.log_widget import LogWidget
from moni2.gui.node_model import NodeModel
from moni2.gui.config_handler import ConfigHandler


class MonitorWindow(QMainWindow):

    VERSION = "0.0.1"
    APP_NAME = "Moni2"
    ORGANIZATION = "dti.dk"

    node_updated = pyqtSignal(NodeInfo)
    online_nodes = pyqtSignal(list)

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
        self.config: ConfigHandler = None

        self.watched_nodes: [str] = []

        self.init_components()
        self.init_ui()
        self.init_menu()
        self.message("Initialized")
        self.config.open_recent()

    def init_components(self):
        self.log_widget = LogWidget(self.log.get_child("LogWidget"), self)
        self.node_model = NodeModel(self.log.get_child("NodeModel"), self.node_updated, self)
        self.config = ConfigHandler(self.log.get_child("ConfigHandler"),
                                    self.ORGANIZATION, self.APP_NAME, self.VERSION, self)

        self.log_widget.warning_counter.connect(self.node_model.log_warning_count)
        self.log_widget.error_counter.connect(self.node_model.log_error_count)

        self.online_nodes.connect(self.node_model.online_nodes)

        self.config.message.connect(self.message)
        self.config.node_list_updated.connect(self.node_model.node_list_updated)
        self.config.node_list_updated.connect(self.node_list_updated)

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
        self.config.create_menu(file_menu)
        file_menu.addSeparator()
        settings_action = QAction(QIcon.fromTheme('preferences-other'), 'Settings', file_menu)
        settings_action.triggered.connect(lambda: self.message("TODO"))
        file_menu.addAction(settings_action)
        file_menu.addSeparator()
        exit_action = QAction(QIcon.fromTheme('application-exit'), "&Quit", file_menu)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.setStatusTip("Exit the application")
        exit_action.triggered.connect(self.close_application)
        file_menu.addAction(exit_action)

        views_menu = main_menu.addMenu("&Views")
        views_menu.addAction(self.log_widget.toggleViewAction())

    def received_log(self, log: Log):
        self.log_widget.received_log(log)

    def update_node(self, node: NodeInfo):
        self.node_updated.emit(node)

    def update_online_nodes(self, nodes: [str]) -> [str]:
        self.online_nodes.emit(nodes)
        return self.watched_nodes

    @pyqtSlot(str, str)
    def message(self, message: str, message_type='info'):
        assert message_type in self.STATUSBAR_MESSAGE_TYPE, f"message_type is: {message_type}, " \
            f"should be one of the following: {list(self.STATUSBAR_MESSAGE_TYPE.keys())}"

        background_color, text_color = self.STATUSBAR_MESSAGE_TYPE[message_type]

        style = f"QStatusBar{{background:rgba({background_color},255);color:{text_color};font-weight:bold;}}"
        self.statusBar().setStyleSheet(style)
        self.statusBar().showMessage(message)
        self.log.info(message)

    @pyqtSlot(list)
    def node_list_updated(self, nodes: [str]):
        self.watched_nodes = nodes

    def close_application(self):
        self.message("Closing the application")
        sys.exit()
