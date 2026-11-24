from glob import glob
from setuptools import setup

package_name = 'midpro_safety'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='midpro',
    description='Direction-aware reactive obstacle guard',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['obstacle_guard = midpro_safety.obstacle_guard:main']},
)
