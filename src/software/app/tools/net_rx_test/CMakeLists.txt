cmake_minimum_required(VERSION 3.16)

# Net_RxThread Test - Simple test for receiving click commands
add_executable(net_rx_test net_rx_test.cpp)

target_link_libraries(net_rx_test
    PRIVATE
    components
    threads
    ipc
    pthread
)

target_include_directories(net_rx_test
    PRIVATE
    ${CMAKE_SOURCE_DIR}/app
)