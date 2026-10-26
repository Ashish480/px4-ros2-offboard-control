import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ashish Shajan',
    maintainer_email='ashishshajan480@gmail.com',
    description='ROS 2 offboard velocity control, keyboard teleop and GPS-failsafe '
                'testing for a PX4 quadrotor in Gazebo SITL.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'teleop = drone_control.teleop:main',
            'velocity_control = drone_control.velocity_control:main',
            'visualizer = drone_control.visualizer:main',
            'processes = drone_control.processes:main',
            'debug = drone_control.debug:main',
            'failsafe = drone_control.failsafe:main',
            'failsafe_mode_switch = drone_control.failsafe_mode_switch:main',
        ],
    },
)
