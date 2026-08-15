from setuptools import find_packages, setup

package_name = "cmd_vel_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot user",
    maintainer_email="user@example.com",
    description="Standalone PyQt5 GUI for monitoring and publishing ROS 2 cmd_vel.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cmd_vel_gui = cmd_vel_gui.cmd_vel_gui:main",
        ],
    },
)
