cmake_minimum_required(VERSION 3.20)
project(seat)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

set(BUILD_DEPS ON)
set(protobuf_BUILD_SHARED_LIBS OFF)
set(protobuf_MSVC_STATIC_RUNTIME ON)

include(cmake/pkg.cmake)

file(GLOB_RECURSE seat-src-files src/*.cc)
add_library(seatl ${seat-src-files} src/heuristic_state.cc src/simulated_annealing.cc src/solver_pathbased.cc src/simulate_booking_series.cc src/solver_full_path.cc src/seat_assignment.cc src/train.cc src/seat_assignment2.cc)
target_link_libraries(seatl PUBLIC utl cista ortools::ortools)
target_compile_features(seatl PUBLIC cxx_std_20)
target_include_directories(seatl PUBLIC include)

add_executable(seat-server exe/server.cc)
target_link_libraries(seat-server web-server seatl nlohmann_json)

file(GLOB_RECURSE seat-exe-files exe/main.cc)
add_executable(seat ${seat-exe-files} include/seat/annealing.h include/seat/neighbourhood.h include/seat/heuristic_state.h include/seat/simulated_annealing.h exe/main.cc include/seat/solver_pathbased.h include/seat/mcf_solver_i.h include/seat/simulate_booking_series.h include/seat/solver_full_path.h include/seat/seat_assignment.h include/seat/train.h include/seat/seat_assignment2.h)
target_link_libraries(seat seatl)

file(GLOB_RECURSE seat-test-files test/*.cc)
add_executable(seat-test ${seat-test-files} test/sa_test.cc)
target_link_libraries(seat-test seatl gtest gtest_main)
