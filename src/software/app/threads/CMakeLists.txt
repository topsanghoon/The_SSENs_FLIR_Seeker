file(GLOB THREADS_SRC CONFIGURE_DEPENDS
  *.cpp
)

add_library(threads STATIC ${THREADS_SRC})

target_include_directories(threads
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    # ${CMAKE_CURRENT_SOURCE_DIR}/../components/includes
    # ${CMAKE_CURRENT_SOURCE_DIR}/../ipc
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(threads
  PUBLIC components ipc
  PRIVATE Threads::Threads
)

if (ENABLE_EO)
  # 스레드 쪽에서도 OpenCV 심볼을 쓸 수 있으니 동일하게 추가s
  find_package(OpenCV 4 REQUIRED)
  target_include_directories(threads PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(threads  PUBLIC ${OpenCV_LIBS})
endif()
