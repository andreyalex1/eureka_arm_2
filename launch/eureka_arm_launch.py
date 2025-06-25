from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eureka_arm_2',
            executable='arm_usb_2',
            name='arm_usb_2',
            shell=True,
            respawn=True,
            respawn_delay=10,
        ),
    ])