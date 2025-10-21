# src/software/app/tools/ir_thread/CMakeLists.txt
add_executable(meta_txthread_test 
  meta_txthread_test.cpp
  ../../threads/Meta_TxThread.cpp   # ← 필요한 스레드 소스만 직접 포함
)

target_compile_features(meta_txthread_test PRIVATE cxx_std_17)

target_include_directories(meta_txthread_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..         # app/tools/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../threads_includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../components/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/../../ipc
)

target_link_libraries(meta_txthread_test PRIVATE
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
