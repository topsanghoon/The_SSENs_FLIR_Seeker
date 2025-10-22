add_executable(control_test 
  main_control.cpp
  ../../threads/ControlThread.cpp   # ← 필요한 스레드 소스만 직접 포함
)

target_compile_features(control_test PRIVATE cxx_std_17)

target_include_directories(control_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..         # app/tools/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../threads_includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../components/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../ipc
)

target_link_libraries(control_test PRIVATE
  components
  ipc
  opencv_core
  opencv_imgproc
  opencv_highgui
  opencv_imgcodecs
  opencv_video
  opencv_tracking
  pthread
)
