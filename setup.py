import os
from glob import glob
from setuptools import setup

package_name = 'tilt_lidar_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shati',
    maintainer_email='shati@example.com',
    description='Tilting LDS-02 sweep control and tilt-compensated point cloud generation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_sweep_node = tilt_lidar_mapping.servo_sweep_node:main',
            'scan_to_cloud_node = tilt_lidar_mapping.scan_to_cloud_node:main',
        ],
    },
)
