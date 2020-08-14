import logging
from typing import Optional
from PyQt5.QtWidgets import (
    QWidget,
    QDockWidget,
    QVBoxLayout,
    QLabel
)
from PyQt5.QtCore import pyqtSlot

from moni2.node_info import NodeInfo


class NodeDetailWidget(QDockWidget):

    def __init__(self, log: logging.Logger, parent: Optional[QWidget] = None):
        super().__init__("Node details", parent)
        self.log = log

        self.node_name_label = QLabel()
        self.n_topics_label = QLabel()

        self.init_components()
        self.init_ui()
        self.log.info("NodeDetailWidget initialized!")

    def init_components(self):
        self.log.info("Initializing components...")

    def init_ui(self):
        self.log.info("Initializing UI...")
        main_widget = QWidget(self)
        main_layout = QVBoxLayout(main_widget)

        main_layout.addWidget(QLabel("node name"))
        main_layout.addWidget(self.node_name_label)
        main_layout.addWidget(QLabel("#publishers"))
        main_layout.addWidget(self.n_topics_label)
        main_layout.addWidget(QLabel("#subs"))
        main_layout.addWidget(QLabel("#services"))
        main_layout.addWidget(QLabel("#actions"))
        main_layout.addWidget(QLabel("#params"))

        self.setWidget(main_widget)

    @pyqtSlot(NodeInfo)
    def update_info(self, node: NodeInfo):
        self.node_name_label.setText(node.name)
        self.n_topics_label.setText(len(node.publishers))
