import logging
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import pyqtSignal

from moni2.gui.node_model import NodeModel
from moni2.gui.settings_handler import SettingsReader
from moni2.node_info import NodeInfo, NodeName, parse_node_name


class FakeSettingsHandler(SettingsReader):

    def __init__(self):
        self.n_columns = 2

    def columns(self) -> int:
        return self.n_columns

    def hide_default_publishers(self) -> bool:
        return False

    def hide_parameter_services(self) -> bool:
        return False

    def hide_moni2_logs(self) -> bool:
        return False

    def hide_unmonitored_nodes(self) -> bool:
        return False

    def default_log_level(self) -> str:
        return False


class FakeWindow(QMainWindow):

    node_updated = pyqtSignal(NodeInfo)

    def __init__(self):
        super().__init__(None)
        self.watched_nodes: [NodeName] = []

    def update_node(self, node: NodeInfo):
        self.node_updated.emit(node)


class TestNodeModel:

    node_model: NodeModel
    node_name_1: NodeName
    node_name_2: NodeName
    node_names: [NodeName]

    def setup(self):
        print("Setup")
        self.window = FakeWindow()
        self.settings = FakeSettingsHandler()

        self.node_model = NodeModel(logging.getLogger("Test_NodeModel"), self.settings,
                                    self.window.node_updated, self.window)
        self.node_name_1 = parse_node_name('node_1')
        self.node_name_2 = parse_node_name('node_2')
        self.node_names = [self.node_name_1, self.node_name_2]
        self.node_model.node_list_updated(self.node_names)
        print("Setup complete")

    def teardown(self):
        print("Teardown")

    def test_initial_state(self):
        assert len(self.node_model.nodes) == 2, "There should be 2 nodes in the list"
        assert self.node_name_1 in self.node_model.nodes
        assert self.node_name_2 in self.node_model.nodes

    def test_node_list_updated(self):
        new_node_name = parse_node_name('node_3')
        self.node_model.node_list_updated([self.node_name_1, new_node_name])

        assert len(self.node_model.nodes) == 2, "There should be 2 nodes in the list"
        assert self.node_name_1 in self.node_model.nodes
        assert new_node_name in self.node_model.nodes

    def test_log_warning_count(self):
        self.node_model.log_warning_count('node_3', 1)
        assert "0" in self.node_model.nodes[self.node_name_1].n_warnings.text()
        assert "0" in self.node_model.nodes[self.node_name_2].n_warnings.text()

        self.node_model.log_warning_count('node_1', 1)
        assert "1" in self.node_model.nodes[self.node_name_1].n_warnings.text()
        assert "0" in self.node_model.nodes[self.node_name_2].n_warnings.text()

    def test_log_error_count(self):
        self.node_model.log_error_count('node_3', 1)
        assert "0" in self.node_model.nodes[self.node_name_1].n_errors.text()
        assert "0" in self.node_model.nodes[self.node_name_2].n_errors.text()

        self.node_model.log_error_count('node_1', 1)
        assert "1" in self.node_model.nodes[self.node_name_1].n_errors.text()
        assert "0" in self.node_model.nodes[self.node_name_2].n_errors.text()

    def test_online_nodes(self):
        assert "❌" in self.node_model.nodes[self.node_name_1].status.text()
        assert "❌" in self.node_model.nodes[self.node_name_2].status.text()

        self.node_model.online_nodes([self.node_name_1])
        assert "✅" in self.node_model.nodes[self.node_name_1].status.text()
        assert "❌" in self.node_model.nodes[self.node_name_2].status.text()

    def test_settings_changed(self):
        assert self.node_model.columns == 2
        self.settings.n_columns = 3
        self.node_model.settings_changed()
        assert self.node_model.columns == 3
