from setuptools import find_packages, setup

package_name = "motor_pid_gui"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot user",
    maintainer_email="user@example.com",
    description="Standalone PyQt5 GUI for ESP32 motor PID tuning over ROS 2.",
    license="MIT",
    entry_points={"console_scripts": ["pid_tuning_gui = motor_pid_gui.pid_tuning_gui:main"]},
)
