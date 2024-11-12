from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eureka_arm_2',
            executable='arm_usb',
            name='arm_usb',
            shell=True,
        ),
    ])