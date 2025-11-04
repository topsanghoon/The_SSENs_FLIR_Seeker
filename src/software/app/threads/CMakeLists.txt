file(GLOB THREADS_SRC CONFIGURE_DEPENDS *.cpp)

add_library(threads STATIC ${THREADS_SRC})

target_include_directories(threads
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..     # app/threads_includes 포함
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 공통 링크: components, ipc, thread
target_link_libraries(threads
  PUBLIC components ipc
  PRIVATE Threads::Threads
)

# --- GStreamer (IR/EO TxThread)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)
target_include_directories(threads PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(threads PRIVATE ${GSTREAMER_LIBRARIES})
target_compile_options(threads PRIVATE ${GSTREAMER_CFLAGS_OTHER})

# --- OpenCV (EO/IR TrackThread, ArUco, Capture)
find_package(OpenCV 4 REQUIRED)
target_include_directories(threads PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(threads PUBLIC ${OpenCV_LIBS})

# --- 경고 최소화
target_compile_features(threads PUBLIC cxx_std_17)
