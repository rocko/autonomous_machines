
"""Package configuration for ROS 2."""

from setuptools import setup

package_name = "waypoint_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/mission.yaml"]),
    ],
    install_requires=["setuptools"],
    # Tell colcon to use pytest for this Python package.
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="Simulation-only autonomous waypoint navigation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "waypoint_controller = "
            "waypoint_navigation.waypoint_controller:main",
        ],
    },
)
