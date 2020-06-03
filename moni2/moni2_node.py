import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication
from moni2.gui import MonitorWindow
from std_srvs.srv import SetBool


class Moni2Node(Node):

    def __init__(self, window):
        super().__init__("moni2")
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.window = window
        self.text_service = self.create_service(SetBool, 'set_bool', self.set_bool_callback)

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def set_bool_callback(self, request: SetBool.Request, response: SetBool.Response):
        text = "Soo true!" if request.data else "Soo false!"
        self.window.set_text(text)
        return response


def main(args=None):
    rclpy.init(args=args)
    app = QApplication([])

    window = MonitorWindow(rclpy.create_node('test').get_logger())
    node = Moni2Node(window)

    try:
        x = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        x.start()
        app.exec()
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        window.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
