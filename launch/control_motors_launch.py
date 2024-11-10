import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'kit_teleoperation'
    return LaunchDescription([
        Node(
            package=package_name,
            executable='control_motors_node',
            name='kit_motor_teleop',
        ),
    ])