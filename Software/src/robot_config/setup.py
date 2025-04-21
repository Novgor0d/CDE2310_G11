from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install configuration files
        (os.path.join('share', package_name, 'robot_config'), glob('robot_config/*.yaml')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unknown',
    maintainer_email='unknown@todo.todo',
    description='Custom bringup package for robot autonomy',
    license='Apache License 2.0',  # Set your license here
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any console scripts here, if needed
        ],
    },
)
