from glob import glob
from setuptools import find_packages, setup

package_name = 'kanga_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hunter',
    maintainer_email='hunter.dejong03@gmail.com',
    description='Nav2 bringup and waypoint navigation for Kanga simulation.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'send_waypoints = kanga_nav2.send_waypoints:main',
            'test_send_waypoints = kanga_nav2.test_send_waypoints:main',
        ],
    },
)
