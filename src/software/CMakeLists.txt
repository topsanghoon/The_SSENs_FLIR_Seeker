cmake_minimum_required(VERSION 3.16)
project(AGM850 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Threads REQUIRED)

# EO 스위치만 여기서 선언
option(ENABLE_EO "Build EO (OpenCV) components" ON)

# 타깃은 아래에서 생성됨
add_subdirectory(app)
