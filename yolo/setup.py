from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/resource', [
            'resource/yolo_model.pt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsiu',
    maintainer_email='billy940702@gmail.com',
    description='YOLOv8 detector for RealSense camera',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'analysis = yolo.analysis:main',
            'yolo_visualization = yolo.yolo_visualization:main',
        ],
    },
)
