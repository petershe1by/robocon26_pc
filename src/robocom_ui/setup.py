from setuptools import setup, find_packages

package_name = 'robocom_ui'

setup(
    name=package_name,
    version='1.0.0',
    packages=['robocom_ui'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ui.launch.py']),
    ],
    install_requires=['setuptools', 'PySide6'],
    zip_safe=True,
    maintainer='robocom-team',
    maintainer_email='team@robocom.local',
    description='PySide6 UI for RoboCom',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robocom_ui = robocom_ui.ui_main:main',
        ],
    },
)
