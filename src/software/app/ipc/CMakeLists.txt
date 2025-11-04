# src/software/app/ipc/CMakeLists.txt

find_package(OpenCV 4 REQUIRED)  # core만 쓰더라도 그냥 REQUIRED가 안전

add_library(ipc STATIC
  event_bus_impl.cpp
)

# include 경로 전파
target_include_directories(ipc
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(ipc PUBLIC opencv_core)

