from setuptools import setup, find_packages

package_name = 'robocom_task_scheduler'

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
    description='Task scheduler',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_scheduler_node = robocom_task_scheduler.task_scheduler_node:main',
        ],
    },
)
