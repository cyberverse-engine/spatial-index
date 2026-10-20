cmake_minimum_required(VERSION 4.0.0)

set(BENCHMARK_ENABLE_TESTING OFF)

include(FetchContent)

FetchContent_Declare(
  benchmark
  GIT_REPOSITORY https://github.com/google/benchmark
  GIT_TAG v1.9.5
)
FetchContent_MakeAvailable(benchmark)

add_executable(cyberverse_spatial_index_benchmarks uniform_grid_benchmark.cpp)

target_link_libraries(
  cyberverse_spatial_index_benchmarks
  PRIVATE
  cyberverse::spatial_index::static_lib
  benchmark::benchmark
)
