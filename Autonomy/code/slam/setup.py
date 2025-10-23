from setuptools import setup
import os
from glob import glob

package_name = 'autonomy_slam'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC 2026 Team',
    maintainer_email='team@urc2026.edu',
    description='RGB-D SLAM subsystem with GPS integration for URC 2026 autonomy (RTAB-Map based)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_node = autonomy_slam.slam_node:main',
            'depth_processor = autonomy_slam.depth_processor:main',
            'gps_fusion_node = autonomy_slam.gps_fusion_node:main',
        ],
    },
)