from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tidybot_openvla'

def recursive_glob(root_dir):
    return [f for f in glob(os.path.join(root_dir, '**'), recursive=True) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/openvla', recursive_glob('openvla')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    zip_safe=True,
    maintainer='yuandi',
    maintainer_email='yuandi@umich.edu',
    description='OpenVLA deployment',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'openvla = openvla.tidybot_openvla:main',
        ]
    },
)
