from setuptools import setup, find_packages

package_name = 'robocom_arm_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocom-team',
    maintainer_email='team@robocom.local',
    description='5-state arm state machine',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_control_node = robocom_arm_control.arm_control_node:main',
        ],
    },
)
