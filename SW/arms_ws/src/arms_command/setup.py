from setuptools import find_packages, setup

package_name = "arms_command"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arms",
    maintainer_email="dev@arms.local",
    description="A.R.M.S. 조종 입력 → /arms/command (실기체 ESP32 HW / SITL 가상 조종기). 튜닝 콘솔은 arms_sim/panel.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arms_command_node = arms_command.arms_command_node:main",
        ],
    },
)
