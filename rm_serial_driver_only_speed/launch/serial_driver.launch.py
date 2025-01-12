import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_serial_driver_only_speed'), 'config', 'serial_driver.yaml')

    rm_serial_driver_node = Node(
        package='rm_serial_driver_only_speed',
        executable='rm_serial_driver_only_speed_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([rm_serial_driver_node])
