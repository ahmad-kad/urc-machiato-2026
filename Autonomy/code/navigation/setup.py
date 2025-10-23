from setuptools import setup
import os
from glob import glob

package_name = 'autonomy_navigation'

setup(
    name=package_name,
    version='1.0.0',
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
    description='Navigation subsystem for URC 2026 autonomy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = autonomy_navigation.navigation_node:main',
            'gnss_processor = autonomy_navigation.gnss_processor:main',
            'path_planner = autonomy_navigation.path_planner:main',
            'motion_controller = autonomy_navigation.motion_controller:main',
            'sensor_integration = autonomy_navigation.sensor_integration:main',
        ],
    },
)