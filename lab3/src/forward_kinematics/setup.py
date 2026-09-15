from setuptools import find_packages, setup

package_name = "forward_kinematics"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/urdf",
            ["urdf/ur7e_flattened.urdf"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ee106a-tab",
    maintainer_email="aarsh.shroff@berkeley.edu",
    description="Forward kinematics utilities and ROS 2 node for the UR7e used in EECS C106A.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
