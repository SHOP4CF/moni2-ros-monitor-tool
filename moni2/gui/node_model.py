import logging
from typing import Optional, Dict
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QGridLayout,
    QScrollArea,
    QScroller,
    QLabel
)
from PyQt5.QtCore import pyqtSlot, pyqtSignal, Qt
from moni2.node_info import NodeInfo
from moni2.gui.node_item import NodeItem


class NodeModel(QWidget):

    def __init__(self, log: logging.Logger, node_updated: pyqtSignal(NodeInfo), parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.log = log

        self.node_updated_callback = node_updated

        self.nodes: Dict[str, NodeItem] = {}
        self.status_label = QLabel()
        self.layout: QGridLayout = None
        self.cols = 2

        self.init_components()
        self.init_ui()

    def init_components(self):
        self.log.info("Initializing components...")

    def init_ui(self):
        self.log.info("Initializing UI...")

        layout = QVBoxLayout(self)
        self.status_label.setAlignment(Qt.AlignHCenter)
        layout.addWidget(self.status_label)
        layout.addSpacing(10)
        scroll_area = QScrollArea(self)
        scroll_area.setWidgetResizable(True)
        scroll_area.setStyleSheet("QScrollArea{border:none}")
        QScroller.grabGesture(scroll_area.viewport(), QScroller.LeftMouseButtonGesture)
        scroll_area_widget = QWidget()
        self.layout = QGridLayout(scroll_area_widget)
        scroll_area.setWidget(scroll_area_widget)
        layout.addWidget(scroll_area)
        self.setLayout(layout)

    @pyqtSlot(list)
    def node_list_updated(self, nodes: [str]):
        self.log.info(f"New list of nodes: {nodes}")
        for node in self.nodes:
            # TODO: only delete if removed from list
            self.nodes[node].deleteLater()
        self.nodes.clear()

        position = 0
        for node in nodes:
            # TODO: only create if not in self.nodes
            node_item = NodeItem(node, self.log.get_child(node))
            self.node_updated_callback.connect(node_item.update_node)
            self.nodes[node] = node_item
            self.layout.addWidget(node_item, position//self.cols, position % self.cols)
            position += 1

    @pyqtSlot(str, int)
    def log_warning_count(self, log_name: str, warning_count: int):
        if log_name in self.nodes:
            self.nodes[log_name].set_warning_count(warning_count)

    @pyqtSlot(str, int)
    def log_error_count(self, log_name: str, warning_count: int):
        if log_name in self.nodes:
            self.nodes[log_name].set_error_count(warning_count)

    @pyqtSlot(list)
    def online_nodes(self, nodes: [str]):
        online = set(self.nodes).intersection(nodes)
        offline = set(self.nodes).difference(nodes)
        self.status_label.setText(f"{len(online)} out of {len(online) + len(offline)} nodes is online")
        for node in offline:
            self.nodes[node].set_online(False)
        for node in online:
            self.nodes[node].set_online(True)
