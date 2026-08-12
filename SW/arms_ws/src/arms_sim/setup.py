from setuptools import find_packages, setup

package_name = "arms_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arms",
    maintainer_email="dev@arms.local",
    description="A.R.M.S. SITL 전용 지원: 표적 심판(referee) + 튜닝/개발 콘솔(panel). 실기체 미사용.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "referee = arms_sim.referee:main",
            "panel = arms_sim.panel:main",
        ],
    },
)
