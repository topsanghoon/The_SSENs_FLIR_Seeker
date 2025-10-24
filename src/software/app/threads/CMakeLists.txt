file(GLOB THREADS_SRC CONFIGURE_DEPENDS
  *.cpp
)

# Remove ControlThread.cpp since it depends on IController which we don't use
list(REMOVE_ITEM THREADS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/ControlThread.cpp)

add_library(threads STATIC ${THREADS_SRC})

target_include_directories(threads
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/
)

target_link_libraries(threads
  PUBLIC components ipc
  PRIVATE Threads::Threads
)

# Find and link GStreamer for IR_TxThread
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

target_include_directories(threads PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(threads PRIVATE ${GSTREAMER_LIBRARIES})
target_compile_options(threads PRIVATE ${GSTREAMER_CFLAGS_OTHER})

if (ENABLE_EO)
  # 스레드 쪽에서도 OpenCV 심볼을 쓸 수 있으니 동일하게 추가s
  find_package(OpenCV 4 REQUIRED)
  target_include_directories(threads PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(threads  PUBLIC ${OpenCV_LIBS})
endif()
