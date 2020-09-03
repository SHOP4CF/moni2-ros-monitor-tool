import sys
import logging
from moni2.gui.node_dialog import EditNodeListDialog
from moni2.node_info import parse_node_name
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt

app = QApplication(sys.argv)


class TestNodeDialog:

    node_dialog: EditNodeListDialog

    def setup(self):
        print("Setup")

        self.node_dialog = EditNodeListDialog(logging.getLogger("Test_NodeDialog"))
        self.test_nodes = [
            parse_node_name('test_node'),
            parse_node_name('other_node')
        ]

    def teardown(self):
        print("Teardown")
        assert self.node_dialog

    def test_lists_is_empty(self):
        assert len(self.node_dialog.selected_nodes()) == 0

    def test_set_nodes(self):
        self.node_dialog.set_nodes(self.test_nodes)

        assert self.node_dialog.selected_nodes() == self.test_nodes

    def test_set_online_nodes(self):
        self.node_dialog.node_updated(self.test_nodes)

        assert self.node_dialog.online_node_list.count() == 2
        for i in range(self.node_dialog.online_node_list.count()):
            assert self.node_dialog.online_node_list.item(i).data(Qt.UserRole) == self.test_nodes[i]

    def test_manual_add_node(self):
        self.node_dialog.new_node_input.setText("new_node")
        self.node_dialog.new_node_button.click()

        assert self.node_dialog.new_node_input.text() == ""
        assert self.node_dialog.selected_nodes() == [parse_node_name("new_node")]
