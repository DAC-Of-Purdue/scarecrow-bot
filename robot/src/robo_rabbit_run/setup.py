from setuptools import find_packages, setup

package_name = "robo_rabbit_run"

setup(
    name=package_name,
    version="0.0.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tam Bureetes",
    maintainer_email="tbureete@purdue.edu",
    description="Robo rabbit run game engine",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rabbit_detector = robo_rabbit_run.detector:main",
            "navigation = robo_rabbit_run.navigation:main",
        ],
    },
)
