from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all files in the 'config' directory
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'output'), glob('output/*')),
        (os.path.join('share', package_name, 'satellite_map'), glob('satellite_map/*')),
        (os.path.join('share', package_name, 'waypoints'), glob('waypoints/*')),
        ('lib/' + package_name, [package_name+'/canusb.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='junekyoopark@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_position_subscriber = dashboard.global_position_subscriber:main',
            'lap_visualizer = dashboard.lap_visualizer:main',
            'lap_visualizer_test = dashboard.lap_visualizer_test:main',
            'dashboard = dashboard.dashboard:main',
            'lap_counter = dashboard.lap_counter:main',
            'lap_counter_test = dashboard.lap_counter_test:main',
        ],
    },
)
