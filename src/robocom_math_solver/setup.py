from setuptools import setup, find_packages

package_name = 'robocom_math_solver'

setup(
    name=package_name,
    version='1.0.0',
    packages=['robocom_math_solver'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/math_solver_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocom-team',
    maintainer_email='team@robocom.local',
    description='OCR-based math quiz solver for RoboCom mission task',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'math_solver_node = robocom_math_solver.math_solver_node:main',
        ],
    },
)
