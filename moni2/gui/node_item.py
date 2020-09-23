import logging
from typing import Optional
from PyQt5.QtWidgets import (
    QWidget,
    QLabel,
    QHBoxLayout,
    QVBoxLayout,
    QFrame,
    QTreeWidget,
    QTreeWidgetItem,
)
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtGui import QFont
from moni2.node_info import NodeInfo, NodeName, TopicInfo
from moni2.gui.settings_handler import SettingsReader


class HLine(QFrame):

    def __init__(self, height=2):
        super().__init__()
        self.setFixedHeight(height)
        self.setStyleSheet("background-color: #333333;")
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)


class NodeItem(QWidget):

    DEFAULT_PUBLISHERS = ['/rosout', '/parameter_events']
    DEFAULT_SERVICES = ['/describe_parameters', '/get_parameter_types',
                        '/get_parameters', '/list_parameters',
                        '/set_parameters', '/set_parameters_atomically']

    def __init__(self,
                 node_name: NodeName,
                 log: logging.Logger,
                 settings: SettingsReader,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.log = log
        self.settings = settings

        self.node_name: NodeName = node_name
        self.node_info: NodeInfo = None

        self.status = QLabel("")
        self.node_name_label = QLabel(node_name.full_name)
        self.n_errors = QLabel("🛑0")
        self.n_warnings = QLabel("⚠0")
        self.topics: QTreeWidget = None

        self.init_ui()

    def init_ui(self):
        self.log.info("Initializing UI...")
        font = QFont("ubuntu", 15, QFont.Bold)
        for label in [self.status, self.node_name_label, self.n_errors, self.n_warnings]:
            label.setFont(font)

        status_layout = QHBoxLayout()
        status_layout.addWidget(self.status)
        status_layout.addWidget(self.node_name_label)
        status_layout.addStretch(1)
        status_layout.addWidget(self.n_errors)
        status_layout.addWidget(self.n_warnings)
        self.n_errors.setStyleSheet("color: #a64452")
        self.n_warnings.setStyleSheet("color: #FFBF00")

        self.topics = QTreeWidget()
        self.topics.setHeaderLabel("Topics")

        main_layout = QVBoxLayout(self)
        main_layout.addLayout(status_layout)
        main_layout.addWidget(HLine())
        main_layout.addWidget(self.topics)

        self.setLayout(main_layout)
        self.setObjectName("NodeItem")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: #dddddd; border-radius: 15px;")
        self.adjustSize()

    @pyqtSlot(NodeInfo)
    def update_node(self, node: NodeInfo):
        if node.name == self.node_name and self.node_info != node:
            self.node_info = node
            self.update_ui()

    def update_ui(self):
        self.topics.clear()

        self._insert_topics("Publishers", self.node_info.publishers)
        self._insert_topics("Subscribers", self.node_info.subscribers)
        self._insert_topics("Service servers", self.node_info.service_server)
        self._insert_topics("Service clients", self.node_info.service_client)
        self._insert_topics("Action servers", self.node_info.action_server)
        self._insert_topics("Action clients", self.node_info.action_client)

    def _insert_topics(self, title: str, topics: [TopicInfo]):
        parent = QTreeWidgetItem(self.topics)
        parent.setText(0, title)
        for topic in topics:
            if self.settings.hide_default_publishers() and topic.name in self.DEFAULT_PUBLISHERS:
                continue
            if self.settings.hide_parameter_services() and \
                    topic.name.replace(self.node_name.full_name, '') in self.DEFAULT_SERVICES:
                continue
            child = QTreeWidgetItem(parent)
            child.setText(0, topic.name)
        parent.setHidden(parent.childCount() == 0)

    def set_warning_count(self, count: int):
        self.n_warnings.setText(f"⚠{count}")

    def set_error_count(self, count: int):
        self.n_errors.setText(f"🛑{count}")

    def set_online(self, online: bool):
        text, color = ("✅", "green") if online else ("❌", "red")
        self.status.setText(text)
        self.status.setStyleSheet(f"color: {color}")
        self.node_name_label.setStyleSheet(f"color: {color}")

        if not online:
            self.topics.clear()
