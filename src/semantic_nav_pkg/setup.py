from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'semantic_nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drone_user',
    maintainer_email='drone_user@todo.todo',
    description='Semantic Navigation Autopilot',
    license='All Rights Reserved',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autopilot_node = semantic_nav_pkg.autopilot_node:main',
            'lidar_processor_node = semantic_nav_pkg.lidar_processor_node:main'
        ],
    },
)
