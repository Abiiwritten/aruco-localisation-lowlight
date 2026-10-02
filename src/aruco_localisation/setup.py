import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aruco_localisation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav',
    maintainer_email='you@example.com',
    description=(
        'ROS2 ArUco marker detection, pose estimation, TF broadcasting, '
        'and OptiTrack-based extrinsic calibration for localisation.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = aruco_localisation.detector:main',
            'tf_publisher = aruco_localisation.tf_publisher:main',
            'optitrack_transform = aruco_localisation.optitrack_transform:main',
            'marker_rigidbody_extrinsic_calibrator = aruco_localisation.marker_rigidbody_extrinsic_calibrator:main',
        ],
    },
)
