import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'chassis_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'deploy'),
            glob(os.path.join('deploy', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="Chih-Pin, Huang",
    maintainer_email="itriB40528@itri.org.tw",
    description='System-level ROS2 services for the chassis onboard computer (guarded shutdown)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_service_node = chassis_system.system_service_node:main',
        ],
    },
)
