from setuptools import find_packages, setup

from glob import glob

package_name = 'tidybot_teleop'

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
    description='Tidybot teleoperation package for ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'phone_teleop_server = tidybot_teleop.phone_teleop_server:main',
            'phone_teleop = tidybot_teleop.phone_teleop:main',
            'state_controller = tidybot_teleop.state_controller:main',
            'reset_env = tidybot_teleop.reset_env:main',
            'remote_teleop = tidybot_teleop.remote_teleop:main',
            'joystick_teleop = tidybot_teleop.joystick_teleop:main',
        ],
    },
)
