cmake_minimum_required(VERSION 2.8.3)
project(serial_test)
find_package(cmake_modules)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC})

include_directories( ${includeDirs} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(DCMAKE_build_type debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -g3 -o -O2 -m64 -Ofast -flto -msse2 -msse3 -pthread -march=native -mfpmath=sse")

add_executable (serial_test src/main_serial.cpp )

