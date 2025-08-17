from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hexapod_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # ('share/' + package_name + '/urdf', ['urdf/hexapod.urdf']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='myat',
    maintainer_email='myatnoeoo772985799@gmail.com',
    description='Control package for hexapod robot with tripod gait in ROS 2 Humble + Gazebo Harmonic',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # gait control node
            'tripod_gait_node = hexapod_control.tripod_gait_node:main',
            'hopf_cpg_tripod_node = hexapod_control.hopf_cpg_tripod_node:main',
        ],
    },
)
