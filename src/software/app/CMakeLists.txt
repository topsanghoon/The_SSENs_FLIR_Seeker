# 하위 모듈
add_subdirectory(ipc)
add_subdirectory(components)
add_subdirectory(threads)



# 실행파일 추가
add_executable(seeker_demo main.cpp)

# OpenCV (필요 모듈만)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

# GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

# include 경로와 링크
target_include_directories(seeker_demo
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${GST_INCLUDE_DIRS}
)

target_link_directories(seeker_demo
  PRIVATE
    ${GST_LIBRARY_DIRS}
)

target_link_libraries(seeker_demo
  PRIVATE
    threads
    components
    ipc
    ${OpenCV_LIBS}
    ${GST_LIBRARIES}
    Threads::Threads
)
