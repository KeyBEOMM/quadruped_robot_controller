# CMakeLists.txt - ESP32 Quadruped Robot Controller
#
# 두 개의 main 파일이 src/ 안에 공존합니다:
#   - main.cpp              : 보행 제어 (#ifndef CALIBRATION_MODE 으로 보호)
#   - main_calibration.cpp  : 캘리브레이션 (#ifdef CALIBRATION_MODE 로 보호)
#
# PIOENV에 따라 CALIBRATION_MODE 매크로를 주입하여 하나만 활성화합니다.

if("$ENV{PIOENV}" STREQUAL "esp32dev_calibration")
    message(STATUS ">>> CALIBRATION MODE: CALIBRATION_MODE macro injected")
    add_compile_definitions(CALIBRATION_MODE=1)
else()
    message(STATUS ">>> NORMAL LOCOMOTION MODE")
endif()

FILE(GLOB_RECURSE app_sources "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "." "../include")
