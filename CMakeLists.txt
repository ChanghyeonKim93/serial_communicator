cmake_minimum_required(VERSION 3.16.0)
project(serial_communicator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread")
set(CMAKE_BUILD_TYPE Release)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

set(INCLUDE_DIR ${INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
set(INCLUDE_DIR ${INCLUDE_DIR} include)

set(SRC ${SRC}
  src/serial_communicator.cc
  src/crc16.cc
)

include_directories( ${INCLUDE_DIR} )

add_executable(serial_communicator src/main.cc ${SRC})
add_executable(get_ebimu ebimu/main_parse.cc ${SRC})

