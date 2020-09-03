import sys
import logging
from PyQt5.QtWidgets import QApplication

from moni2.gui.node_item import NodeItem
from moni2.node_info import parse_node_name, NodeInfo, TopicInfo


app = QApplication(sys.argv)


class TestNodeItem:

    node_item: NodeItem

    def setup(self):
        print("Setup")

        self.node_name = parse_node_name('test_node')
        self.node_item = NodeItem(self.node_name, logging.getLogger("Test_NodeItems"))

    def teardown(self):
        print("Teardown")
        assert self.node_item

    def test_name(self):
        assert self.node_item.node_name_label.text() == self.node_name.full_name

    def test_online(self):
        self.node_item.set_online(True)
        assert self.node_item.status.text() == "✅"

    def test_warning_count(self):
        self.node_item.set_warning_count(10)
        assert "10" in self.node_item.n_warnings.text()

    def test_error_count(self):
        self.node_item.set_error_count(3)
        assert "3" in self.node_item.n_errors.text()

    def test_update_node(self):
        info = NodeInfo(self.node_name)
        self.node_item.update_node(info)
        assert self.node_item.node_info == info

    def test_update_node2(self):
        topic = [TopicInfo("/some/topic", ['/type1', '/type2'])]
        info = NodeInfo(self.node_name, topic, topic, topic, topic, topic, topic)
        self.node_item.update_node(info)
        assert self.node_item.node_info == info
        assert self.node_item.action_client.count() == 1

    def test_update_node_other_name(self):
        info = NodeInfo(parse_node_name('other_node'))
        self.node_item.update_node(info)
        assert self.node_item.node_info != info
