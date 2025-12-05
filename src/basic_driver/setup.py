from setuptools import setup
import os
from glob import glob

package_name = 'basic_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qyx',
    maintainer_email='qyx@todo.todo',
    description='Basic driver utilities for robotics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar2nt = basic_driver.lidar2nt:main',
            'camera_viewer = basic_driver.camera_viewer:main',
            'twist2twiststamped = basic_driver.twist2twiststamped:main',
            'wr1_control = basic_driver.wr1_control:main',
            'wr1_server = basic_driver.wr1_server:main',
        ],
    },
)
