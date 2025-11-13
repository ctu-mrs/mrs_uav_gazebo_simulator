import os
from setuptools import find_packages, setup

package_name = 'mrs_uav_gazebo_simulator'
data_directories = ['launch', 'config', 'models', 'ROMFS', 'tmux']

data_files_list = []

for directory in data_directories:
    for dirpath, _, filenames in os.walk(directory):
        install_dir = os.path.join('share', package_name, dirpath)
        source_files = [os.path.join(dirpath, f) for f in filenames]
        data_files_list.append((install_dir, source_files))

setup(
    name='mrs-uav-gazebo-simulator',
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']), *data_files_list],
    install_requires=[
        'setuptools',
        'jinja2',
    ],
    zip_safe=True,
    maintainer='Vojtech Spurny',
    maintainer_email='vojtech.spurny@fel.cvut.cz',
    description='The Metapackage for MRS UAV Gazebo simulation pipeline.',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mrs_drone_spawner = mrs_uav_gazebo_simulator.mrs_drone_spawner:main',
        ],
    },
)
