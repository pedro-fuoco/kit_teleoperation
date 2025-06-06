import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'kit_teleoperation'
    calibration_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'extrinsinc_camera_calibration_params.yaml')

    return LaunchDescription([
        Node(
            package=package_name,
            executable='calibrate_camera_extrinsic_node',
            name='camera_calibrator_extrinsic',
            parameters=[calibration_config_file],
        ),
    ])
