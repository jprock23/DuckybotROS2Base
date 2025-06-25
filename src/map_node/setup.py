from setuptools import find_packages, setup

package_name = 'map_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jproque',
    maintainer_email='jproque@todo.todo',
    description='TODO: Package description',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_gen=map_node.map_generator_node:main'
        ],
    },
)
