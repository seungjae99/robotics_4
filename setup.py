from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'termproject'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='choi',
    maintainer_email='seungjaechoi9@gmail.com',
    description='Mobile Robotics Term Project TEAM 4',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "read_lidar=termproject.read_lidar:main",
            "collision_avoid=termproject.collision_avoid:main",
            "visual_pub=termproject.visual_pub:main",
            "parking2=termproject.parking2:main",
            "waypoint=termproject.waypoint:main",
            "waypoint_const_vel=termproject.waypoint_const_vel:main",
            "waypoint_col_avoid=termproject.waypoint_col_avoid:main",
        ],
    }, 
)
