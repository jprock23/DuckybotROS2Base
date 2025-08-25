import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'deliberation'

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
    maintainer_email='juanproque@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "broadcast=deliberation.static_broadcaster:main",
            "chassis=deliberation.chassis_broadcaster:main",
            "state_publisher=deliberation.robot_state_publisher:main"
        ],
    },
)

