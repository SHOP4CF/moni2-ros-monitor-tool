import logging
from typing import Optional, Dict
from PyQt5.QtWidgets import (
    QWidget,
    QDockWidget,
    QListWidgetItem,
    QListWidget
)
from PyQt5.QtCore import pyqtSignal

from moni2.node_info import NodeInfo


class NodeWidget(QDockWidget):

    node_clicked = pyqtSignal(NodeInfo)

    def __init__(self, log: logging.Logger, parent: Optional[QWidget] = None):
        super().__init__("Nodes", parent)
        self.log = log

        self.nodes: Dict[str, NodeInfo] = {}
        self.node_list_items: Dict[str, QListWidgetItem] = {}
        self.listWidget = QListWidget()

        self.init_components()
        self.init_ui()
        self.log.info("NodeWidget initialized!")

    def init_components(self):
        self.log.info("Initializing components...")

        self.listWidget.itemClicked.connect(self._node_clicked)

    def init_ui(self):
        self.log.info("Initializing UI...")

        self.setWidget(self.listWidget)

    def update_node(self, node: NodeInfo):
        self.nodes[node.name] = node

        if node.name in self.node_list_items:
            print(f"Update: {node.name}")
        else:
            item = QListWidgetItem(node.name, self.listWidget)
            self.node_list_items[node.name] = item
            self.listWidget.addItem(item)

    def _node_clicked(self, item: QListWidgetItem):
        self.node_clicked.emit(self.nodes[item.text()])
