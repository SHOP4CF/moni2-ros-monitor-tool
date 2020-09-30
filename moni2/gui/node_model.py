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
from moni2.node_info import NodeInfo, NodeName, parse_node_name
from moni2.gui.node_item import NodeItem
from moni2.gui.settings_handler import SettingsReader


class NodeModel(QWidget):

    def __init__(self, log: logging.Logger,
                 settings: SettingsReader,
                 node_updated: pyqtSignal(NodeInfo),
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.log = log
        self.settings = settings

        self.node_updated_callback = node_updated

        self.nodes: Dict[NodeName, NodeItem] = {}
        self.status_label = QLabel()
        self.layout: QGridLayout = None
        self.columns = self.settings.columns()
        self.hide_default_publishers = self.settings.hide_default_publishers()
        self.hide_parameter_services = self.settings.hide_parameter_services()

        self.init_components()
        self.init_ui()

    def init_components(self):
        self.log.info("Initializing components...")

    def init_ui(self):
        self.log.info("Initializing UI...")

        layout = QVBoxLayout(self)
        self.status_label.setAlignment(Qt.AlignHCenter)
        layout.addSpacing(10)
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
        self.log.info(f"New list of nodes: {[node.full_name for node in nodes]}")
        obsolete_nodes = set(self.nodes.keys()).difference(nodes)
        new_nodes = set(nodes).difference(self.nodes.keys())

        while self.layout.count():  # Remove all NodeItems from the layout
            self.layout.takeAt(0)

        for node in obsolete_nodes:  # delete NodeItems no longer needed
            self.nodes[node].deleteLater()
            del self.nodes[node]

        for node in new_nodes:  # create new nodes if needed
            try:
                log = self.log.getChild(node.name)
            except AttributeError:
                log = self.log.get_child(node.name)
            node_item = NodeItem(node, log, self.settings)
            self.node_updated_callback.connect(node_item.update_node)
            self.nodes[node] = node_item

        position = 0  # insert all NodeItems
        for node in nodes:
            self.layout.addWidget(self.nodes[node], position // self.columns, position % self.columns)
            position += 1

    @pyqtSlot(str, int)
    def log_warning_count(self, log_name: str, warning_count: int):
        node_name = parse_node_name(log_name)
        if node_name in self.nodes:
            self.nodes[node_name].set_warning_count(warning_count)

    @pyqtSlot(str, int)
    def log_error_count(self, log_name: str, warning_count: int):
        node_name = parse_node_name(log_name)
        if node_name in self.nodes:
            self.nodes[node_name].set_error_count(warning_count)

    @pyqtSlot(list)
    def online_nodes(self, nodes: [NodeName]):
        online = set(self.nodes).intersection(nodes)
        offline = set(self.nodes).difference(nodes)
        self._update_status_label(len(online), len(offline))
        for node in offline:
            self.nodes[node].set_online(False)
        for node in online:
            self.nodes[node].set_online(True)

    def _update_status_label(self, n_online: int, n_offline: int):
        self.status_label.setText(f"{n_online} out of {n_online + n_offline} nodes is online")
        background_color = '#00dd00' if n_offline == 0 else '#dd0000'
        text_color = '#000000' if n_offline == 0 else '#ffffff'

        self.status_label.setStyleSheet(
            f"background-color: {background_color}; color: {text_color};"
            f"border-radius: 15px; padding: 15px; margin-left: 10px; margin-right: 10px;")

    @pyqtSlot()
    def settings_changed(self):
        if self.columns != self.settings.columns():
            self.columns = self.settings.columns()
            self.node_list_updated(self.nodes.keys())
        if self.hide_default_publishers != self.settings.hide_default_publishers():
            self.hide_default_publishers = self.settings.hide_default_publishers()
            for node in self.nodes.values():
                node.update_ui()
        if self.hide_parameter_services != self.settings.hide_parameter_services():
            self.hide_parameter_services = self.settings.hide_parameter_services()
            for node in self.nodes.values():
                node.update_ui()
