from setuptools import find_packages, setup

package_name = 'tidybot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_server = tidybot_driver.base_server:main',
            'base_demo = tidybot_driver.base:main',
        ],
    },
)
