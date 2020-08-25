import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication
from moni2.gui import MonitorWindow
from moni2.node_info import NodeInfo, Topic, Service
from std_srvs.srv import SetBool
from rcl_interfaces.msg import Log
from ament_index_python.packages import get_package_share_directory


class Moni2Node(Node):

    def __init__(self):
        super().__init__("moni2")
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.window = MonitorWindow(self.get_logger().get_child('gui'), get_package_share_directory('moni2'))
        self.text_service = self.create_service(SetBool, 'set_bool', self.set_bool_callback)
        self.log_sub = self.create_subscription(Log, '/rosout', self.received_log, 10)
        self.timer = self.create_timer(2.0, self.check_nodes)

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def check_nodes(self):
        nodes = self.get_node_names_and_namespaces()
        print(f"## Nodes: {nodes}")
        for node, namespace in nodes:
            publishers = []
            subscribers = []
            services = []
            clients = []
            for name, types in self.get_publisher_names_and_types_by_node(node, namespace):
                publishers.append(Topic(name, types))
            for name, types in self.get_subscriber_names_and_types_by_node(node, namespace):
                subscribers.append(Topic(name, types))
            for name, types in self.get_service_names_and_types_by_node(node, namespace):
                services.append(Service(name, types))
            for name, types in self.get_client_names_and_types_by_node(node, namespace):
                clients.append(Service(name, types))
            node_info = NodeInfo(node, publishers, subscribers, services, clients)
            self.window.update_node(node_info)

    def set_bool_callback(self, request: SetBool.Request, response: SetBool.Response):
        text = "Soo true!" if request.data else "Soo false!"
        self.window.set_text(text)
        return response

    def received_log(self, log: Log):
        self.window.received_log(log)

    def destroy_node(self) -> bool:
        self.get_logger().info("Closing down")
        self.window.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication([])

    node = Moni2Node()

    try:
        x = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        x.start()
        app.exec()
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
