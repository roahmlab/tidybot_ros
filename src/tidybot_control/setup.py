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
                      'tidybot_msgs'],
    zip_safe=True,
    maintainer='janchen',
    maintainer_email='janchen@umich.edu',
    description='Tidybot remote control package for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_states_listener = tidybot_control.joint_states_listener:main',
            'twist_to_multiarray = tidybot_control.twist_to_multiarray:main',
            'web_server_publisher = tidybot_control.web_server_publisher:main',
            'ws_relay = tidybot_control.ws_relay:main',
            'state_controller = tidybot_control.state_controller:main',
        ],
    },
)
