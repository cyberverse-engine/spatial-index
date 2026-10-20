cmake_minimum_required(VERSION 4.0.0)
project(cyberverse_spatial_index CXX)

option(CYBERVERSE_SPATIAL_INDEX_BUILD_TESTS "Build unit test cases" ON)
option(CYBERVERSE_SPATIAL_INDEX_BUILD_BENCHMARKS "Build benchmarks" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/engine_dep_geometry.cmake)
resolve_engine_dep_geometry()

add_subdirectory(src)

if(CYBERVERSE_SPATIAL_INDEX_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

if(CYBERVERSE_SPATIAL_INDEX_BUILD_BENCHMARKS)
  add_subdirectory(benchmark)
endif()
