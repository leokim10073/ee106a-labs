from setuptools import find_packages, setup

package_name = "joint_control"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ee106a-tab",
    maintainer_email="aarsh.shroff@berkeley.edu",
    description="ROS 2 nodes for commanding and validating UR7e joint trajectories in EECS C106A.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "validate_trajectory = joint_control.validate_trajectory:main",
        ],
    },
)
