add_executable(ir_trackthread_sanity
  IR_TrackThread_sanity.cpp
)

target_include_directories(ir_trackthread_sanity PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(ir_trackthread_sanity PRIVATE
  threads
  components
  ipc
  opencv_core
  opencv_imgproc
  opencv_video
  opencv_tracking
  pthread
)
