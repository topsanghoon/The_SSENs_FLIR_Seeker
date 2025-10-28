# src/software/app/tools/eo_capture_test/CMakeLists.txt
add_executable(eo_capture_test
  eo_capture_test.cpp
  ../../threads/EO_CaptureThread.cpp   # EO_CaptureThread source
  ../../threads/EO_TxThread.cpp        # EO_TxThread source
)

target_compile_features(eo_capture_test PRIVATE cxx_std_17)

# Find and link GStreamer for EO_TxThread
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

target_include_directories(eo_capture_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..         # app/tools/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../threads_includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../components/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../ipc
  ${GSTREAMER_INCLUDE_DIRS}
)

target_link_libraries(eo_capture_test PRIVATE
  components
  ipc
  opencv_core
  opencv_imgproc
  opencv_highgui
  opencv_imgcodecs
  opencv_video
  opencv_videoio
  pthread
  ${GSTREAMER_LIBRARIES}
)

target_compile_options(eo_capture_test PRIVATE ${GSTREAMER_CFLAGS_OTHER})