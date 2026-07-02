import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robo_ai_vision'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raghul',
    maintainer_email='raghul@todo.todo',
    description='ArUco/QR marker detection from the delivery robot camera feed.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_detector_node = robo_ai_vision.marker_detector_node:main',
        ],
    },
)
