import logging
from typing import Optional
from PyQt5.QtWidgets import (
    QWidget,
    QDialog,
    QDialogButtonBox,
    QVBoxLayout,
    QHBoxLayout,
    QListWidget,
    QAbstractItemView,
    QLineEdit,
    QPushButton,
    QLabel,
)
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from moni2.node_info import NodeInfo


# https://stackoverflow.com/a/22546488
class EditNodeListDialog(QDialog):
    node_list_updated = pyqtSignal(list)

    def __init__(self, log: logging.Logger, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.log = log

        self.nodes: [str] = []
        self.node_list = QListWidget()
        self.new_node_input: QLineEdit = QLineEdit()
        self.nodes_selected_label = QLabel()

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Add node")

        self.node_list.setSelectionMode(QAbstractItemView.MultiSelection)
        self.node_list.selectionModel().selectionChanged.connect(self._update_selected_nodes)

        self.new_node_input.setPlaceholderText("Other node")
        new_node_button = QPushButton("Add node")
        new_node_button.clicked.connect(self._manual_add_node)
        new_node_layout = QHBoxLayout()
        new_node_layout.addWidget(self.new_node_input)
        new_node_layout.addWidget(new_node_button)

        button = QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        button_box = QDialogButtonBox(button)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        layout = QVBoxLayout()
        layout.addWidget(self.node_list)
        layout.addStretch(2)
        layout.addLayout(new_node_layout)
        layout.addWidget(self.nodes_selected_label)
        layout.addSpacing(20)
        layout.addWidget(button_box)
        self.setLayout(layout)

    def set_nodes(self, nodes: [str]):
        self.nodes = nodes
        for node in nodes:
            self.node_list.addItem(node)
        self._update_selected_nodes()

    def accept(self) -> None:
        nodes = [node.text() for node in self.node_list.selectedItems()]
        self.log.info(f"Nodes: {nodes}")
        self.node_list_updated.emit(nodes)
        super().accept()

    @pyqtSlot(NodeInfo)
    def node_updated(self, node: NodeInfo):
        if node.name not in self.nodes:
            self.nodes.append(node.name)
            self.node_list.addItem(node.name)
        self._update_selected_nodes()

    def _manual_add_node(self):
        node = self.new_node_input.text()
        self.node_list.addItem(node)
        self._update_selected_nodes()

    def _update_selected_nodes(self):
        selected = len(self.node_list.selectedItems())
        total = self.node_list.count()
        self.nodes_selected_label.setText(f"{selected} out of {total} selected")
