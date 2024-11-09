import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'kit_teleoperation'
    tf_rviz_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'tf.rviz')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='kit_tf_visualizer',
            output='screen',
            arguments=['-d', tf_rviz_file],
        ),
    ])