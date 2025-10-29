# src/software/app/tools/ir_capture_test/CMakeLists.txt
add_executable(ir_capture_test
  ir_capture_test.cpp
  ../../threads/IR_CaptureThread.cpp   # IR_CaptureThread source
  ../../threads/IR_TxThread.cpp        # IR_TxThread source
)

target_compile_features(ir_capture_test PRIVATE cxx_std_17)

# Find and link GStreamer for IR_TxThread
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

target_include_directories(ir_capture_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..         # app/tools/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../threads_includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../components/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../ipc
  ${GSTREAMER_INCLUDE_DIRS}
)

target_link_libraries(ir_capture_test PRIVATE
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

target_compile_options(ir_capture_test PRIVATE ${GSTREAMER_CFLAGS_OTHER})
