from setuptools import find_packages, setup

package_name = "inverse_orchestrator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "orchestrator = inverse_orchestrator.orchestrator:main",
            "move_relative_test = inverse_orchestrator.tests.move_relative_test:main",
            "learn_skill_test = inverse_orchestrator.tests.learn_skill_test:main",
            "execute_skill_test = inverse_orchestrator.tests.execute_skill_test:main",
        ],
    },
)
