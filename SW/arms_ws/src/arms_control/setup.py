from setuptools import setup

setup(
    name="arms_control",
    version="0.1.0",
    packages=["arms_control"],
    data_files=[],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "sitl_bridge_node = arms_control.sitl_bridge_node:main",
        ],
    },
)
