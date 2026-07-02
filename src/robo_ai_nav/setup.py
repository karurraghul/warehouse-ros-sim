import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robo_ai_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
    maintainer='raghul',
    maintainer_email='raghul@todo.todo',
    description='Nav2 waypoint-follower driving the delivery robot through warehouse aisles.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator = robo_ai_nav.waypoint_navigator:main',
        ],
    },
)
