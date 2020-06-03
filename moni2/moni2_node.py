import rclpy
from rclpy.node import Node


class Moni2Node(Node):

    def __init__(self):
        super().__init__("moni2")
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.get_logger().info(f"{self.get_name()} Initialized!")


def main(args=None):
    rclpy.init(args=args)
    node = Moni2Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
