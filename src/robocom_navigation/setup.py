from setuptools import setup, find_packages

package_name = 'robocom_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nav_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocom-team',
    maintainer_email='team@robocom.local',
    description='Navigation, safety & task cycle',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = robocom_navigation.navigation_node:main',
        ],
    },
)
