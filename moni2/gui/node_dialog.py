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
from PyQt5.QtGui import QDragEnterEvent, QDragMoveEvent, QDropEvent
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt


class DragListWidget(QListWidget):
    # https://stackoverflow.com/a/22546488

    dropped = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setDragDropMode(QAbstractItemView.DragDrop)
        self.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.setAcceptDrops(True)

    def dragEnterEvent(self, event: QDragEnterEvent):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            super(DragListWidget, self).dragEnterEvent(event)

    def dragMoveEvent(self, event: QDragMoveEvent):
        if event.mimeData().hasUrls():
            event.setDropAction(Qt.CopyAction)
            event.accept()
        else:
            super(DragListWidget, self).dragMoveEvent(event)

    def dropEvent(self, event: QDropEvent):
        if event.mimeData().hasUrls():
            event.setDropAction(Qt.CopyAction)
            event.accept()
            links = []
            for url in event.mimeData().urls():
                links.append(str(url.toLocalFile()))
            self.dropped.emit()
        else:
            event.setDropAction(Qt.MoveAction)
            super(DragListWidget, self).dropEvent(event)
            self.dropped.emit()


class EditNodeListDialog(QDialog):
    node_list_updated = pyqtSignal(list)

    def __init__(self, log: logging.Logger, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.log = log

        self.online_node_list = QListWidget()
        self.node_list = DragListWidget()
        self.new_node_input = QLineEdit()
        self.new_node_button = QPushButton()
        self.nodes_selected_label = QLabel()

        self.init_components()
        self.init_ui()
        self.log.info("Initialized")

    def init_components(self):
        self.log.info("Initializing components...")

        self.online_node_list.setDragDropMode(QAbstractItemView.DragOnly)
        self.online_node_list.setDragEnabled(True)
        self.online_node_list.setSelectionMode(QAbstractItemView.ExtendedSelection)

        self.node_list.dropped.connect(self._node_dropped)

        self.new_node_input.setPlaceholderText("node name")
        self.new_node_button.setText("Add node")
        self.new_node_button.clicked.connect(self._manual_add_node)

    def init_ui(self):
        self.log.info("Initializing UI...")
        self.setWindowTitle("Select nodes to be monitored")

        online_layout = QVBoxLayout()
        online_layout.addWidget(QLabel("Online nodes"))
        online_layout.addWidget(self.online_node_list)

        selected_layout = QVBoxLayout()
        selected_layout.addWidget(QLabel("Selected nodes"))
        selected_layout.addWidget(self.node_list)

        new_node_layout = QVBoxLayout()
        new_node_layout.addWidget(QLabel("Add offline node"))
        new_node_layout.addWidget(self.new_node_input)
        new_node_layout.addWidget(self.new_node_button)
        new_node_layout.addStretch(10)

        layout = QHBoxLayout()
        layout.addLayout(online_layout)
        layout.addLayout(selected_layout)
        layout.addLayout(new_node_layout)

        button = QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        button_box = QDialogButtonBox(button)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        main_layout = QVBoxLayout()
        main_layout.addLayout(layout)
        main_layout.addWidget(self.nodes_selected_label)
        main_layout.addSpacing(20)
        main_layout.addWidget(button_box)
        self.setLayout(main_layout)

    def _node_dropped(self):
        nodes = set()
        delete_list = []
        for i in range(self.node_list.count()):
            node = self.node_list.item(i).text()
            if node in nodes:
                delete_list.insert(0, i)
            else:
                nodes.add(node)
        for i in delete_list:
            self.node_list.takeItem(i)
        self._update_selected_nodes()

    def set_nodes(self, nodes: [str]):
        for node in nodes:
            self.node_list.addItem(node)
        self._update_selected_nodes()

    def accept(self) -> None:
        nodes = [self.node_list.item(i).text() for i in range(self.node_list.count())]
        self.log.info(f"Nodes: {nodes}")
        self.node_list_updated.emit(nodes)
        super().accept()

    @pyqtSlot(list)
    def node_updated(self, nodes: [str]):
        self.online_node_list.clear()
        for node in nodes:
            self.online_node_list.addItem(node)

    def _manual_add_node(self):
        node = self.new_node_input.text()
        self.node_list.addItem(node)
        self.new_node_input.setText("")
        self._update_selected_nodes()

    def _update_selected_nodes(self):
        total = self.node_list.count()
        self.nodes_selected_label.setText(f"{total} nodes selected")
