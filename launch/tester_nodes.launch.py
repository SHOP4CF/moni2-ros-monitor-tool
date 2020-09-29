from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for this example.

    To run this launch file, call:
      ros2 launch moni2 moni2.launch.py
    To see the list of arguments and how to pass them:
      ros2 launch moni2 moni2.launch.py --show-args

    :return:
    """
    publisher_node = Node(
        package='examples_rclpy_minimal_publisher',
        executable='publisher_member_function',
        output='screen',
        emulate_tty=True,
        parameters=[{}]
    )
    subscriber_node = Node(
        package='examples_rclpy_minimal_subscriber',
        executable='subscriber_member_function',
        output='screen',
        emulate_tty=True,
        parameters=[{}]
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node
    ])
