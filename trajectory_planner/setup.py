from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'trajectory_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ralph',
    maintainer_email='ralph@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_planning_linear = trajectory_planner.trajectory_planning_linear:main',
            'trajectory_with_library = trajectory_planner.trajectory_with_library:main',
            'testing_circle = trajectory_planner.testing_circle:main'
        ],
    },
)
