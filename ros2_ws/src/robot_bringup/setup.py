from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # urdf files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='t',
    maintainer_email='n.thanadett@gmail.com',
    description='Robot bringup package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wheel_odometry_node = robot_bringup.wheel_odometry_node:main',
            'imu_node = robot_bringup.imu_node:main',
            'heading_pid_node = robot_bringup.heading_pid_node:main',
        ],
    },
)
