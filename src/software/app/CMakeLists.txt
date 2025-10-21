# 하위 모듈(정적/인터페이스 라이브러리)로 분리
add_subdirectory(ipc)
add_subdirectory(components)
add_subdirectory(threads)

#test모듈 
add_subdirectory(tools)

# (선택) 실행파일이 생기면 여기서 추가
# add_executable(seeker_demo seeker_demo/main.cpp)
# target_link_libraries(seeker_demo PRIVATE threads components ipc Threads::Threads)
