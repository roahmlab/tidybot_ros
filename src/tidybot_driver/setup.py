from setuptools import find_packages, setup
from glob import glob

package_name = 'tidybot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*')),
        ('share/' + package_name + '/models/', ['models/gen3_robotiq_2f_85.urdf']),
        ('share/' + package_name + '/models/assets/base', glob('models/assets/base/*')),
        ('share/' + package_name + '/models/assets/2f85', glob('models/assets/2f85/*')),
        ('share/' + package_name + '/models/assets/gen3', glob('models/assets/gen3/*')),
        ('share/' + package_name + '/models/kinova_gen3/', glob('models/kinova_gen3/*')),
        ('share/' + package_name + '/models/stanford_tidybot/', glob('stanford_tidybot/*')),
        ('share/' + package_name + '/config/', glob('config/*')),
    ],
    install_requires=['setuptools',
                      'tidybot_utils',
                      'phoenix6',
                      'ruckig',
                      'threadpoolctl',
                      'numpy'],
    zip_safe=True,
    maintainer='janchen',
    maintainer_email='janchen@umich.edu',
    description='Package for interfacing with Tidybot hardware',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'base_server = tidybot_driver.base_server:main',
            'base_demo = tidybot_driver.base:main',
            'arm_server = tidybot_driver.arm_server:main',
            'camera_wrist = tidybot_driver.camera_wrist:main',
            'camera_ext = tidybot_driver.camera_external:main',
            'joint_state_publisher = tidybot_driver.joint_state_publisher:main',
            'play_csv_delta = tidybot_driver.play_csv_delta:main',
            'play_csv_pose = tidybot_driver.play_csv_pose:main'
        ],
    },
)
