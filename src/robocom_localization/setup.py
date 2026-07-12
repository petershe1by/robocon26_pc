from setuptools import setup
from glob import glob

package_name = 'robocom_localization'

setup(
    name=package_name,
    version='1.0.0',
    packages=['robocom_localization'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocom-team',
    maintainer_email='team@robocom.local',
    description='LiDAR localization for RoboCom',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_node = robocom_localization.localization_node:main',
            'imu_filter = robocom_localization.imu_filter:main',
        ],
    },
)
