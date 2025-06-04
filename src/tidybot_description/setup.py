from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tidybot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*')),
        ('share/' + package_name + '/urdf/', ['urdf/tidybot.xacro', 'urdf/bot.xacro', 'urdf/ros2_control.yaml']),
        ('share/' + package_name + '/urdf/base/', glob('urdf/base/*')),
        ('share/' + package_name + '/urdf/arms/gen3_lite/', glob('urdf/arms/gen3_lite/*')),
        ('share/' + package_name + '/urdf/grippers/robotiq_2f_85/', glob('urdf/grippers/robotiq_2f_85/*')),
        ('share/' + package_name + '/urdf/grippers/gen3_lite_2f/', glob('urdf/grippers/gen3_lite_2f/*')),
        ('share/' + package_name + '/meshes/arms/gen3_lite/', glob('meshes/arms/gen3_lite/*')),
        ('share/' + package_name + '/meshes/grippers/gen3_lite_2f/', glob('meshes/grippers/gen3_lite_2f/*')),
        ('share/' + package_name + '/meshes/base/', glob('meshes/base/*')),
        ('share/' + package_name + '/config/', glob('config/*')),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='janchen',
    maintainer_email='janchen@umich.edu',
    description='description package for the tidybot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
