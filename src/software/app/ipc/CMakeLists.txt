# src/software/app/ipc/CMakeLists.txt

find_package(OpenCV 4 REQUIRED)  # core만 쓰더라도 그냥 REQUIRED가 안전

add_library(ipc STATIC
  event_bus_impl.cpp
)

# include 경로 전파
target_include_directories(ipc
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/..   # .../app → #include "ipc/xxx.hpp" 가능
)

# 라이브러리 연결
# 최소 core만 쓸 거면 opencv_core만 걸어도 됨. (아래 둘 중 택1)
target_link_libraries(ipc PUBLIC opencv_core)
# 또는 전체 변수(환경 따라 자동으로 world/core 등 맞춰짐)
# target_link_libraries(ipc PUBLIC ${OpenCV_LIBS})
