from setuptools import setup
import os
from glob import glob

package_name = 'amr_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tnguyenmanht',
    maintainer_email='tnguyenmanht@gmail.com',
    description='Perception nodes: lane detection và LiDAR filter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lane_detection_node = amr_perception.lane_detection_node:main',
            'lidar_filter_node = amr_perception.lidar_filter_node:main',
        ],
    },
)
