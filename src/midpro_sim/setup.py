import os
from glob import glob
from setuptools import setup

package_name = 'midpro_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='midpro',
    maintainer_email='dev@example.com',
    description='Gazebo-free 2D simulator and detour bridge for the midpro Nav2 stack',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_robot = midpro_sim.fake_robot_node:main',
            'detour_bridge = midpro_sim.detour_bridge:main',
        ],
    },
)
