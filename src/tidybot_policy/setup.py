from setuptools import find_packages, setup

from glob import glob

package_name = 'tidybot_policy'

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
    description='Tidybot policy package for ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'phone_teleop_server = tidybot_policy.phone_teleop_server:main',
            'phone_policy = tidybot_policy.phone_policy:main',
            'state_controller = tidybot_policy.state_controller:main',
            'reset_env = tidybot_policy.reset_env:main',
            'remote_policy_diffusion = tidybot_policy.remote_policy_diffusion:main',
            'remote_policy_vla = tidybot_policy.remote_policy_vla:main',
            'gamepad_policy = tidybot_policy.gamepad_policy:main',
        ],
    },
)
