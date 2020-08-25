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
    QAbstractScrollArea,
)
from PyQt5.QtCore import Qt
from moni2.node_info import NodeInfo


class NodeItem(QWidget):

    def __init__(self, node_name: str, log: logging.Logger, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.log = log

        self.status = QLabel("❌✅")
        self.node_name = QLabel(node_name)
        self.n_errors = QLabel("🛑0")
        self.n_warnings = QLabel("⚠0")
        self.publishers: QListWidget = None
        self.subscribers: QListWidget = None
        self.services: QListWidget = None
        self.clients: QListWidget = None

        self.init_ui()

    def init_ui(self):
        self.log.info("Initializing UI...")

        status_layout = QHBoxLayout()
        status_layout.addWidget(self.status)
        status_layout.addWidget(self.node_name)
        status_layout.addStretch(1)
        status_layout.addWidget(self.n_errors)
        status_layout.addWidget(self.n_warnings)
        self.n_errors.setStyleSheet("color: #a64452")
        self.n_warnings.setStyleSheet("color: #FFBF00")

        topic_layout = QHBoxLayout()
        self.publishers, pub_layout = self._create_list("Publishers")
        self.subscribers, sub_layout = self._create_list("Subscribers")
        self.services, serv_layout = self._create_list("Services")
        self.clients, cli_layout = self._create_list("Clients")
        topic_layout.addLayout(pub_layout)
        topic_layout.addLayout(sub_layout)
        topic_layout.addLayout(serv_layout)
        topic_layout.addLayout(cli_layout)

        main_layout = QVBoxLayout(self)
        main_layout.addLayout(status_layout)
        main_layout.addLayout(topic_layout)

        self.setLayout(main_layout)
        self.setObjectName("NodeItem")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: #dddddd; border-radius: 10px;")

    def update_node(self, node: NodeInfo):
        pass

    def _create_list(self, name: str) -> (QListWidget, QLayout):
        list = QListWidget()
        list.setSelectionMode(QAbstractItemView.NoSelection)
        list.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        list.setStyleSheet("QListView{border:none}")

        layout = QVBoxLayout()
        layout.addWidget(QLabel(name))
        layout.addWidget(list)

        return list, layout

    def set_warning_count(self, count: int):
        self.n_warnings.setText(f"⚠{count}")

    def set_error_count(self, count: int):
        self.n_errors.setText(f"🛑{count}")

    def set_online(self, online: bool):
        text, color = ("✅", "green") if online else ("❌", "red")
        self.status.setText(text)
        self.status.setStyleSheet(f"color: {color}")
