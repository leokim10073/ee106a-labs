from setuptools import find_packages, setup

package_name = 'straight_line'

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
    maintainer='ee106a-tae',
    maintainer_email='reid@xz.ax',
    description='the realization that life is more than primal and dual',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'straight_line_server = straight_line.straight_line_server:main',
        ],
    },
)
