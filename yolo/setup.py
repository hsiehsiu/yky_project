from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsiu',
    maintainer_email='billy940702@gmail.com',
    description='YOLOv8 detector for RealSense camera',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'node = yolo.node:main',
            'launch = launch.yolo.launch:generate_launch_description.'
        ],
    },
)
