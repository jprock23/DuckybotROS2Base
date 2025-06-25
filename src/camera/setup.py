import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.dae')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name, 'config/calibration'), glob('config/calibration/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jproque',
    maintainer_email='juanproque@tamu.edu',
    description='Package to detect cv2 aruco tags.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_detector=camera.tag_detector_node:main',
            'static_broadcaster=camera.static_camera_to_map_broadcaster:main'
        ],
    },
)

