add_library(ipc STATIC
  event_bus_impl.cpp
)

target_include_directories(ipc
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}   # app/ipc/*.hpp
)
