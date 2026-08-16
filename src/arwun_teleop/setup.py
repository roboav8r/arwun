import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'arwun_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboav8r',
    maintainer_email='jaduncan86@gmail.com',
    description='Joystick teleop and joystick-driven rosbag2 recording for the Arwun rover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_controller = arwun_teleop.record_controller:main',
            'record_indicator = arwun_teleop.record_indicator:main',
        ],
    },
)
