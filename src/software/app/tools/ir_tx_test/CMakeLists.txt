# src/software/app/tools/ir_thread/CMakeLists.txt
add_executable(ir_tx_test
  ir_tx_test.cpp
  ../../threads/IR_TxThread.cpp   # ← 필요한 스레드 소스만 직접 포함
)

set_property(TARGET ir_tx_test PROPERTY CXX_STANDARD 17)
set_property(TARGET ir_tx_test PROPERTY CXX_STANDARD_REQUIRED ON)

# Find and link GStreamer for IR_TxThread
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

target_include_directories(ir_tx_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..         # app/tools/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../threads_includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../components/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../ipc
  ${GSTREAMER_INCLUDE_DIRS}
)

target_link_libraries(ir_tx_test PRIVATE
  components
  ipc
  opencv_core
  opencv_imgproc
  opencv_highgui
  opencv_imgcodecs
  opencv_video
  opencv_tracking
  pthread
  ${GSTREAMER_LIBRARIES}
)

target_compile_options(ir_tx_test PRIVATE ${GSTREAMER_CFLAGS_OTHER})