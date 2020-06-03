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
    moni2_node = Node(
        package='moni2',
        node_executable='moni2',
        output='screen',
        emulate_tty=True,
        parameters=[{}]
    )

    return LaunchDescription([
        moni2_node
    ])
