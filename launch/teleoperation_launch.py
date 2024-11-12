# multi_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'kit_teleoperation'
    launch_files = [
        'control_motors_launch.py',
        'view_compressed_launch.py',
        'view_ekf_launch.py'
    ]

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package_name), 'launch', launch_file)
            )
        ) for launch_file in launch_files
    ])
