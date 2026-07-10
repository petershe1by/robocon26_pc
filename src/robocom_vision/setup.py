from setuptools import setup, find_packages

package_name = 'robocom_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=['robocom_vision'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/vision_params.yaml']),
        ('share/' + package_name + '/models', ['models/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocom-team',
    maintainer_email='team@robocom.local',
    description='Vision: YOLO, color mask, depth',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_block_detector = robocom_vision.yolo_block_detector:main',
            'color_mask_detector = robocom_vision.color_mask_detector:main',
            'depth_helper = robocom_vision.depth_helper:main',
        ],
    },
)
