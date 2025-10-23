from setuptools import setup
import os
from glob import glob

package_name = 'autonomy_computer_vision'

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
    description='Computer vision subsystem for URC 2026 autonomy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'computer_vision_node = autonomy_computer_vision.computer_vision_node:main',
        ],
    },
)
