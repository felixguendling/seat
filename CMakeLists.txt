cmake_minimum_required(VERSION 3.20)
project(seat)

include(cmake/pkg.cmake)
add_subdirectory(glucose-4.2.1)

file(GLOB_RECURSE seat-src-files src/*.cc)
add_library(seatl ${seat-src-files})
target_link_libraries(seatl PUBLIC utl cista glucose)
target_compile_features(seatl PUBLIC cxx_std_20)
target_include_directories(seatl PUBLIC include)

file(GLOB_RECURSE seat-exe-files exe/*.cc)
add_executable(seat ${seat-exe-files})
target_link_libraries(seat seatl)

file(GLOB_RECURSE seat-test-files test/*.cc)
add_executable(seat-test ${seat-test-files})
target_link_libraries(seat-test seatl gtest gtest_main)
