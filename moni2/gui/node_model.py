import logging
from typing import Optional, Dict
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QGridLayout,
    QTableView,
    QScrollArea
)
from PyQt5.QtCore import pyqtSlot

from moni2.gui.node_item import NodeItem


class NodeModel(QWidget):

    def __init__(self, log: logging.Logger, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.log = log

        self.nodes: Dict[str, NodeItem] = {}

        self.node_list_view = QTableView()

        self.layout: QGridLayout = None
        self.cols = 2

        self.init_components()
        self.init_ui()

    def init_components(self):
        self.log.info("Initializing components...")

    def init_ui(self):
        self.log.info("Initializing UI...")

        layout = QVBoxLayout(self)
        scroll_area = QScrollArea(self)
        scroll_area.setWidgetResizable(True)
        scroll_area_widget = QWidget()
        self.layout = QGridLayout(scroll_area_widget)
        scroll_area.setWidget(scroll_area_widget)
        layout.addWidget(scroll_area)
        self.setLayout(layout)

    @pyqtSlot(list)
    def node_list_updated(self, nodes: [str]):
        self.log.warning(f"New list of nodes: {nodes}")
        for node in self.nodes:
            # TODO: only delete if removed from list
            self.nodes[node].deleteLater()
        self.nodes.clear()

        position = 0
        for node in nodes:
            # TODO: only create if not in self.nodes
            node_item = NodeItem(node, self.log.get_child(node))
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
