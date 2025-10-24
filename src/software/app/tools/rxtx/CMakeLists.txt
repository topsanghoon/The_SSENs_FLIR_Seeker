# src/software/app/tools/ir_thread/CMakeLists.txt

add_executable(rxtx_test
  rxtx_test.cpp
  ../../threads/EO_CaptureThread.cpp
  ../../threads/IR_CaptureThread.cpp
  ../../threads/EO_TxThread.cpp
  ../../threads/IR_TxThread.cpp
  ../../threads/Meta_TxThread.cpp
  ../../threads/Net_RxThread.cpp
)

target_compile_features(rxtx_test PRIVATE cxx_std_17)

target_include_directories(rxtx_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..                 # app/tools/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../threads_includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../components/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../ipc
)

# ---------- GStreamer 추가 ----------
find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED
  gstreamer-1.0>=1.14
  gstreamer-base-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

target_include_directories(rxtx_test PRIVATE
  ${GST_INCLUDE_DIRS}
)

# 일부 환경에서 필요 (링커가 못 찾는 경우)
link_directories(${GST_LIBRARY_DIRS})

# pkg-config가 주는 컴파일 플래그 적용(경고무시 등)
target_compile_options(rxtx_test PRIVATE ${GST_CFLAGS_OTHER})

# ---------- 링크 ----------
target_link_libraries(rxtx_test PRIVATE
  components
  ipc

  # OpenCV (기존 방식 유지)
  opencv_core
  opencv_imgproc
  opencv_highgui
  opencv_imgcodecs
  opencv_video
  opencv_tracking

  # GStreamer
  ${GST_LIBRARIES}

  pthread
)

# (선택) 만약 특정 아키텍처에서 glib-2.0 헤더 경로 이슈가 있다면 아래를 풀어주세요.
# if (EXISTS "/usr/lib/aarch64-linux-gnu/glib-2.0/include")
#   target_include_directories(rxtx_test PRIVATE /usr/lib/aarch64-linux-gnu/glib-2.0/include)
# endif()
# if (EXISTS "/usr/lib/x86_64-linux-gnu/glib-2.0/include")
#   target_include_directories(rxtx_test PRIVATE /usr/lib/x86_64-linux-gnu/glib-2.0/include)
# endif()
