from setuptools import find_packages, setup
from glob import glob

package_name = "arms_ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/sounds", glob("sounds/*.mp3")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arms",
    maintainer_email="dev@arms.local",
    description="OpenCV UI for A.R.M.S.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arms_ui_node = arms_ui.arms_ui_node:main",
            "flight_recorder = arms_ui.flight_recorder:main",
        ],
    },
)
