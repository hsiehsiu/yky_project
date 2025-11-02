from setuptools import setup

package_name = 'transfer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/transfer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tmrdriver',
    maintainer_email='tmrdriver@example.com',
    description='camera→base tf + YOLO detections to JSON',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trans = transfer.transfer:main',
        ],
    },
)
