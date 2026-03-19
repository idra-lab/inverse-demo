from setuptools import find_packages, setup
import os
from glob import glob 
package_name = "inverse_orchestrator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hydran00",
    maintainer_email="davide.nardi-1@unitn.it",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orchestrator = scripts.orchestrator:main",
            "move_relative_test = scripts.tests.move_relative_test:main",
            "learn_skill_test = scripts.tests.learn_skill_test:main",
            "execute_skill_test = scripts.tests.execute_skill_test:main",
            "gripper_test = scripts.tests.gripper_test:main",
            "point_to_point_test = scripts.tests.point_to_point_test:main",
            "tf_publisher = scripts.tf_publisher:main",
        ],
    },
)
