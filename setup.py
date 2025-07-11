import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kit_teleoperation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedro-fuoco',
    maintainer_email='pedrofuoco6@gmail.com',
    description='ROS 2 teleoperation package for Kit de Robotica',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'view_compressed_node = kit_teleoperation.view_compressed_img:main',
            'calibrate_camera_intrinsic_node = kit_teleoperation.calibrate_camera_intrinsic:main',
            'calibrate_camera_extrinsic_node = kit_teleoperation.calibrate_camera_extrinsic:main',
            'control_motors_node = kit_teleoperation.control_motors:main',
        ],
    },
)