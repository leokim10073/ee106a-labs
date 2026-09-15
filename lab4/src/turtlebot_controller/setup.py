import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'turtlebot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Janus Sucharitakul',
    maintainer_email='janus_sucharitakul@berkeley.edu',
    description='Lab 4 nodes: open-loop square driving and a Nav2 goal client.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'square_drive = turtlebot_controller.square_drive:main',
            'nav_client = turtlebot_controller.nav_client:main',
        ],
    },
)
