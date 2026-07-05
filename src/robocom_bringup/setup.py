from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robocom_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=['robocom_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocom-team',
    maintainer_email='team@robocom.local',
    description='Bringup package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autostart = robocom_bringup.autostart:main',
        ],
    },
)
