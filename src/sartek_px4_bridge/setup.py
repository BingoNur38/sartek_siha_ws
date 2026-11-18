from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'sartek_px4_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mert',
    maintainer_email='mert@todo.todo',
    description='SARTEK Savaşan İHA — PX4 Micro XRCE-DDS telemetri ve kontrol sınır köprüsü',
    license='BSD-3-Clause',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'px4_telemetry_bridge = sartek_px4_bridge.px4_telemetry_bridge:main',
            'px4_control_bridge = sartek_px4_bridge.px4_control_bridge:main',
        ],
    },
)
