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
from moni2.node_info import NodeInfo, NodeName
from moni2.gui.node_item import NodeItem


class NodeModel(QWidget):

    def __init__(self, log: logging.Logger, node_updated: pyqtSignal(NodeInfo), parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.log = log

        self.node_updated_callback = node_updated

        self.nodes: Dict[NodeName, NodeItem] = {}
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
    def node_list_updated(self, nodes: [NodeName]):
        self.log.info(f"New list of nodes: {nodes}")
        obsolete_nodes = set(self.nodes.keys()).difference(nodes)
        new_nodes = set(nodes).difference(self.nodes.keys())

        while self.layout.count():  # Remove all NodeItems from the layout
            self.layout.takeAt(0)

        for node in obsolete_nodes:  # delete NodeItems no longer needed
            self.nodes[node].deleteLater()
            del self.nodes[node]

        for node in new_nodes:  # create new nodes if needed
            node_item = NodeItem(node, self.log.get_child(node.name))
            self.node_updated_callback.connect(node_item.update_node)
            self.nodes[node] = node_item

        position = 0  # insert all NodeItems
        for node in nodes:
            self.layout.addWidget(self.nodes[node], position // self.cols, position % self.cols)
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
    def online_nodes(self, nodes: [NodeName]):
        online = set(self.nodes).intersection(nodes)
        offline = set(self.nodes).difference(nodes)
        self.status_label.setText(f"{len(online)} out of {len(online) + len(offline)} nodes is online")
        for node in offline:
            self.nodes[node].set_online(False)
        for node in online:
            self.nodes[node].set_online(True)
