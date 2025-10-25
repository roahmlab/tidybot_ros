from setuptools import find_packages, setup

from glob import glob
import os

package_name = 'tidybot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*')),
        ('share/' + package_name + '/config/', glob('config/*')),
    ],
    install_requires=['setuptools',
                      'flask',
                      'flask_socketio',
                      'simple-websocket',
                      'tidybot_utils',
                      'numpy',
                      'opencv-python',],
    zip_safe=True,
    maintainer='janchen',
    maintainer_email='janchen@umich.edu',
    description='Tidybot remote control package for ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'teleop_server = tidybot_control.teleop_server:main',
            'teleop_controller = tidybot_control.teleop_controller:main',
            'state_controller = tidybot_control.state_controller:main',
            'reset_env = tidybot_control.reset_env:main',
            'remote_controller = tidybot_control.remote_controller:main',
            'joystick_controller = tidybot_control.joystick_controller:main',
        ],
    },
)
