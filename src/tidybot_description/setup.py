from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tidybot_description'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*')),
        ('share/' + package_name + '/urdf/', ['urdf/tidybot.xacro', 'urdf/bot.xacro', 'urdf/tidybot.ros2_control.xacro', 'urdf/tidybot.gazebo.xacro', 'urdf/arm.xacro']),
        ('share/' + package_name + '/urdf/world/', glob('urdf/world/*')),
        ('share/' + package_name + '/urdf/base/tidybot/', glob('urdf/base/tidybot/*')),
        ('share/' + package_name + '/urdf/base/tidybot++/', glob('urdf/base/tidybot++/*')),
        ('share/' + package_name + '/urdf/arms/gen3_lite/', glob('urdf/arms/gen3_lite/*')),
        ('share/' + package_name + '/urdf/arms/gen3_7dof/', glob('urdf/arms/gen3_7dof/*')),
        ('share/' + package_name + '/urdf/grippers/robotiq_2f_85/', glob('urdf/grippers/robotiq_2f_85/*')),
        ('share/' + package_name + '/urdf/grippers/gen3_lite_2f/', glob('urdf/grippers/gen3_lite_2f/*')),
        ('share/' + package_name + '/meshes/arms/gen3_lite/', glob('meshes/arms/gen3_lite/*')),
        ('share/' + package_name + '/meshes/arms/gen3_7dof/', glob('meshes/arms/gen3_7dof/*')),
        ('share/' + package_name + '/meshes/grippers/gen3_lite_2f/', glob('meshes/grippers/gen3_lite_2f/*')),
        ('share/' + package_name + '/meshes/grippers/robotiq_2f_85/visual', glob('meshes/grippers/robotiq_2f_85/visual/*')),
        ('share/' + package_name + '/meshes/grippers/robotiq_2f_85/collision', glob('meshes/grippers/robotiq_2f_85/collision/*')),
        ('share/' + package_name + '/meshes/base/tidybot', glob('meshes/base/tidybot/*')),
        ('share/' + package_name + '/meshes/base/tidybot++', glob('meshes/base/tidybot++/*')),
        ('share/' + package_name + '/config/', glob('config/*')),
]

data_files+=[
        (                                                  # install_path
            os.path.join('share', package_name, 'models',
                         os.path.relpath(os.path.dirname(f), 'models')),
            [f]                                            # single file
        )
        for f in glob('models/**/*', recursive=True)
        if os.path.isfile(f)                              # skip directories
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools',
                      'tidybot_moveit_config',],
    zip_safe=True,
    maintainer='janchen',
    maintainer_email='janchen@umich.edu',
    description='description package for the tidybot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tf_relay = tidybot_description.tf_relay:main',
            'isaac_sim_bridge = tidybot_description.isaac_sim_bridge:main',
        ],
    },
)
