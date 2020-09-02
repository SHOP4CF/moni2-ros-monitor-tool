import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication
from moni2.gui import MonitorWindow
from moni2.node_info import NodeInfoHandler
from rcl_interfaces.msg import Log
from ament_index_python.packages import get_package_share_directory


class Moni2Node(Node):

    def __init__(self):
        super().__init__("moni2")
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.window = MonitorWindow(self.get_logger().get_child('gui'), get_package_share_directory('moni2'))
        self.log_sub = self.create_subscription(Log, '/rosout', self.received_log, 10)
        self.timer = self.create_timer(3.0, self.check_nodes)
        self.node_info = NodeInfoHandler(self)

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def check_nodes(self):
        nodes = self.node_info.get_node_names()
        watched_nodes = self.window.update_online_nodes(nodes)

        for node in set(watched_nodes).intersection(nodes):
            self.window.update_node(self.node_info.get_node_info(node))

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
