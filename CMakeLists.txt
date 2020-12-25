 cmake_minimum_required(VERSION 2.8.12)
 project(BouncingBalls)

 add_definitions("-std=c++11")

 include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
 conan_basic_setup()

 add_executable(BouncingBalls BouncingBalls.cpp)
 target_link_libraries(BouncingBalls ${CONAN_LIBS})
