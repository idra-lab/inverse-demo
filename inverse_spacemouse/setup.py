from glob import glob

from setuptools import setup


package_name = "inverse_spacemouse"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/udev", glob("udev/*.rules")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Victor Kowalski",
    maintainer_email="victor.kowalski.m@gmail.com",
    description="SpaceMouse (hidraw) to CartesianTrajectoryPoint velocity commands.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "spacemouse_node = inverse_spacemouse.spacemouse_node:main",
        ],
    },
)
