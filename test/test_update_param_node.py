import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
from moni2.param_update_node import ParamUpdateNode
from moni2.node_info import parse_node_name, NodeName, ParamInfo


class TestUpdateParamNode:

    param_update_node: ParamUpdateNode
    node_name: NodeName
    online_watched_nodes: [NodeName]
    node_params: dict

    def setup_class(self):
        print("Setup class")
        rclpy.init()

    def setup(self):
        print("Setup")
        self.node_name = parse_node_name('test_node')
        self.online_watched_nodes = [self.node_name]
        self.node_params = {}
        self.param_update_node = ParamUpdateNode(self.online_watched_nodes, self.node_params)
        self.test_node = rclpy.create_node('test_node')
        self.test_node.declare_parameter('test', 1)
        x = threading.Thread(target=rclpy.spin, args=(self.test_node, MultiThreadedExecutor()), daemon=True)
        x.start()

    def teardown(self):
        print("Teardown")
        self.test_node.destroy_node()
        self.param_update_node.destroy_node()

    def teardown_class(self):
        print("Teardown class")
        rclpy.shutdown()

    def test_get_value(self):
        self.param_update_node.update_param_list()
        assert self.node_name in self.node_params, f"{self.node_name} not in {self.node_params}"
        expected_params = [ParamInfo('use_sim_time', 'bool', False), ParamInfo('test', 'int', 1)]
        print(expected_params)
        print(self.node_params[self.node_name])
        assert self.node_params[self.node_name] == expected_params

    def test_get_more_values(self):
        node_name = parse_node_name('node2')
        node = rclpy.create_node(node_name.name)
        node.declare_parameter('string', "hejsa")
        node.declare_parameter('double', 2.0)
        x = threading.Thread(target=rclpy.spin, args=(node, MultiThreadedExecutor()), daemon=True)
        x.start()
        self.online_watched_nodes.append(node_name)

        self.param_update_node.update_param_list()
        assert node_name in self.node_params, f"## {node_name} not in {self.node_params} ##"
        assert self.node_name in self.node_params, f"## {self.node_name} not in {self.node_params} ##"

        node.destroy_node()

    def test_waiting_for_node(self):
        node_name = parse_node_name('node2')
        self.online_watched_nodes.append(node_name)

        self.param_update_node.update_param_list()
        assert node_name in self.node_params, f"## {node_name} not in {self.node_params} ##"
        assert self.node_name in self.node_params, f"## {self.node_name} not in {self.node_params} ##"

        assert self.node_params[node_name] == []
