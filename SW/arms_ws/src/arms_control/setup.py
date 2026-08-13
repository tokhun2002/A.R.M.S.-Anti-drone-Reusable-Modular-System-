from setuptools import setup

setup(
    name="arms_control",
    version="0.1.0",
    packages=["arms_control"],
    data_files=[],
    install_requires=["setuptools"],

    # console_scripts 는 두지 않는다 — 이 패키지는 ament_cmake 라 실행되지 않는다.
    #   ament_python_install_package() 는 모듈만 깔고 setup.py 의 entry_points 는
    #   무시한다. 실행파일 `sitl_bridge_node` 는 CMakeLists 의
    #   install(PROGRAMS scripts/sitl_bridge_node) 가 깐다.
    #   여기 entry_point 를 되살려도 아무 일도 일어나지 않으며, "선언은 있는데
    #   왜 안 되지" 로 시간만 쓴다 (실제로 그랬다).
)
