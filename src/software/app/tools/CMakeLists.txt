# src/software/app/tools/CMakeLists.txt
# tools 루트에서는 서브디렉토리만 관리
add_subdirectory(ir_trackthread)
add_subdirectory(ir_tx_test)
add_subdirectory(ir_capture_test)
add_subdirectory(eo_tx_test)
add_subdirectory(eo_capture_test)
add_subdirectory(net_rx_test)

# 필요해지면 아래 주석 해제해서 확장
# add_subdirectory(eo_aruco_thread)
# add_subdirectory(meta_tx_thread)
