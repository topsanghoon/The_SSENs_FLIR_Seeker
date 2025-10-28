# 소스 자동수집(편의) - 원하면 명시 나열로 바꿔도 됩니다.
file(GLOB COMPONENTS_SRC CONFIGURE_DEPENDS
  core/*.cpp
)

add_library(components STATIC ${COMPONENTS_SRC}
  ../util/csv_sink.cpp
  )

target_include_directories(components
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..       # app/components/includes/*.hpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/../ipc          # ipc_types.hpp 등을 직접 include할 때 대비
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/core
)

target_link_libraries(components
  PUBLIC Threads::Threads
  PUBLIC ipc
)

# ---- OpenCV 는 '타깃 생성 후' 조건부로 붙이기 ----
if (ENABLE_EO)
  find_package(OpenCV 4 REQUIRED)  # COMPONENTS 없이 전체 묶음이 실패확률 ↓
  message(STATUS "OpenCV include: ${OpenCV_INCLUDE_DIRS}")
  message(STATUS "OpenCV libs   : ${OpenCV_LIBS}")

  target_include_directories(components PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(components  PUBLIC ${OpenCV_LIBS})
endif()
