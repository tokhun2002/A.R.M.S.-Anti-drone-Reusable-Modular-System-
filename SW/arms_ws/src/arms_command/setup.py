from setuptools import find_packages, setup
from glob import glob

package_name = "arms_command"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arms",
    maintainer_email="dev@arms.local",
    description="A.R.M.S. command interface (GUI panel, GPIO button)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arms_command_node = arms_command.arms_command_node:main",
            "arms_command_gpio_node = arms_command.arms_command_gpio_node:main",
        ],
    },
)
