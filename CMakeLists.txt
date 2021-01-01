cmake_minimum_required(VERSION 3.0.0)
project(astarpath VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(astarpath main.cpp)
target_link_libraries(${PROJECT_NAME} X11 GL pthread png stdc++fs)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
