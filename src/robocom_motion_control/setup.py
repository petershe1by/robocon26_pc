from setuptools import setup, find_packages

package_name = 'robocom_motion_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/motion_params.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='robocom-team',
    maintainer_email='team@robocom.local',
    description='Motion control & USB CDC SBUS bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_control_node = robocom_motion_control.motion_control_node:main',
        ],
    },
)
