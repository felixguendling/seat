cmake_minimum_required(VERSION 3.20)
project(seat)

set(BUILD_DEPS ON)
include(cmake/pkg.cmake)

file(GLOB_RECURSE seat-src-files src/*.cc)
add_library(seatl ${seat-src-files})
target_link_libraries(seatl PUBLIC utl cista ortools::ortools)
target_compile_features(seatl PUBLIC cxx_std_20)
target_include_directories(seatl PUBLIC include)

file(GLOB_RECURSE seat-server-files exe/server.cc)
add_executable(seat-server ${seat-server-files})
target_link_libraries(seat-server seatl web-server nlohmann_json)

file(GLOB_RECURSE seat-exe-files exe/main.cc)
add_executable(seat ${seat-exe-files})
target_link_libraries(seat seatl)

file(GLOB_RECURSE seat-test-files test/*.cc)
add_executable(seat-test ${seat-test-files})
target_link_libraries(seat-test seatl gtest gtest_main)