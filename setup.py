from setuptools import setup
import os

package_name = "sgengine"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Colorado School of Mines - Robotics Club",
    maintainer_email="jcdavis@mines.edu",
    description="SpaceGrantEngine - ROS2 Utilities Package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test = sgengine.test_node:main",
        ],
    },
)
