# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

# ${CMAKE_SOURCE_DIR} :프로젝트 최상단 위치
FILE(GLOB_RECURSE app_sources "*.cpp" "*.c") # 경로 안주면 현재 디렉토리에서 검색함
# FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*) - 경로 주는 경우

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "." "../include")













