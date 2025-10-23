from setuptools import setup
import os
from glob import glob

package_name = 'autonomy_simulation'

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
        (os.path.join('share', package_name, 'digitaltwins/config'), glob('digitaltwins/config/*.yaml')),
        (os.path.join('share', package_name, 'digitaltwins/launch'), glob('digitaltwins/launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC 2026 Team',
    maintainer_email='team@urc2026.edu',
    description='Simulation and Digital Twin subsystem for URC 2026 autonomy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_simulator = autonomy_simulation.sensor_simulator:main',
            'digital_twin_manager = autonomy_simulation.digital_twin_manager:main',
        ],
    },
)
