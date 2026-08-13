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

    # console_scripts 는 두지 않는다 — 이 패키지는 ament_cmake 라 실행되지 않는다.
    #   ament_python_install_package() 는 모듈만 깔고 setup.py 의 entry_points 는
    #   무시한다. 실행파일 `arms_command_node` 는 CMakeLists 의
    #   install(PROGRAMS scripts/arms_command_node) 가 깐다.
    #   여기 entry_point 를 되살려도 아무 일도 일어나지 않으며, "선언은 있는데
    #   왜 안 되지" 로 시간만 쓴다 (실제로 그랬다).
)
