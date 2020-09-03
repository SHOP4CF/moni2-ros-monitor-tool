import logging
from typing import Optional
from PyQt5.QtWidgets import (
    QWidget,
    QLabel,
    QListWidget,
    QHBoxLayout,
    QVBoxLayout,
    QAbstractItemView,
    QLayout,
    QListView,
    QFrame
)
from PyQt5.QtCore import Qt, pyqtSlot
from moni2.node_info import NodeInfo, NodeName


class HLine(QFrame):

    def __init__(self, height=2):
        super().__init__()
        self.setFixedHeight(height)
        self.setStyleSheet("background-color: #333333;")
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)


class NodeItem(QWidget):

    def __init__(self, node_name: NodeName, log: logging.Logger, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.log = log

        self.node_name: NodeName = node_name
        self.node_info: NodeInfo = None

        self.status = QLabel("")
        self.node_name_label = QLabel(node_name.full_name)
        self.n_errors = QLabel("🛑0")
        self.n_warnings = QLabel("⚠0")
        self.publishers: QListWidget = None
        self.subscribers: QListWidget = None
        self.service_server: QListWidget = None
        self.service_client: QListWidget = None
        self.action_server: QListWidget = None
        self.action_client: QListWidget = None

        self.init_ui()

    def init_ui(self):
        self.log.info("Initializing UI...")

        status_layout = QHBoxLayout()
        status_layout.addWidget(self.status)
        status_layout.addWidget(self.node_name_label)
        status_layout.addStretch(1)
        status_layout.addWidget(self.n_errors)
        status_layout.addWidget(self.n_warnings)
        self.n_errors.setStyleSheet("color: #a64452")
        self.n_warnings.setStyleSheet("color: #FFBF00")

        topic_layout = QHBoxLayout()
        self.publishers, pub_layout = self._create_list("Publishers")
        self.subscribers, sub_layout = self._create_list("Subscribers")
        topic_layout.addLayout(pub_layout)
        topic_layout.addLayout(sub_layout)

        service_layout = QHBoxLayout()
        self.service_server, serv_layout = self._create_list("Service server")
        self.service_client, cli_layout = self._create_list("Service client")
        service_layout.addLayout(serv_layout)
        service_layout.addLayout(cli_layout)

        action_layout = QHBoxLayout()
        self.action_server, serv_layout = self._create_list("Action server")
        self.action_client, cli_layout = self._create_list("Action client")
        action_layout.addLayout(serv_layout)
        action_layout.addLayout(cli_layout)

        main_layout = QVBoxLayout(self)
        main_layout.addLayout(status_layout)
        main_layout.addWidget(HLine())
        main_layout.addLayout(topic_layout)
        main_layout.addWidget(HLine(1))
        main_layout.addLayout(service_layout)
        main_layout.addWidget(HLine(1))
        main_layout.addLayout(action_layout)
        main_layout.addStretch(10)

        self.setLayout(main_layout)
        self.setObjectName("NodeItem")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: #dddddd; border-radius: 10px;")
        self.adjustSize()

    @pyqtSlot(NodeInfo)
    def update_node(self, node: NodeInfo):
        if node.name == self.node_name and self.node_info != node:
            self.node_info = node
            self.publishers.clear()
            for pub in node.publishers:
                self.publishers.addItem(pub.name)
            self.subscribers.clear()
            for sub in node.subscribers:
                self.subscribers.addItem(sub.name)
            self.service_server.clear()
            for serv in node.service_server:
                self.service_server.addItem(serv.name)
            self.service_client.clear()
            for client in node.service_client:
                self.service_client.addItem(client.name)
            self.action_server.clear()
            for server in node.action_server:
                self.action_server.addItem(server.name)
            self.action_client.clear()
            for client in node.action_client:
                self.action_client.addItem(client.name)

    def _create_list(self, name: str) -> (QListWidget, QLayout):
        topic_list = QListWidget()
        topic_list.setSelectionMode(QAbstractItemView.NoSelection)
        topic_list.setResizeMode(QListView.Adjust)
        topic_list.setStyleSheet("QListView{border:none}")
        topic_list.setUniformItemSizes(True)
        topic_list.addItem("None")

        layout = QVBoxLayout()
        layout.addWidget(QLabel(name))
        layout.addWidget(topic_list)

        return topic_list, layout

    def set_warning_count(self, count: int):
        self.n_warnings.setText(f"⚠{count}")

    def set_error_count(self, count: int):
        self.n_errors.setText(f"🛑{count}")

    def set_online(self, online: bool):
        text, color = ("✅", "green") if online else ("❌", "red")
        self.status.setText(text)
        self.status.setStyleSheet(f"color: {color}")
        self.node_name_label.setStyleSheet(f"color: {color}")
