/**
 * @file uniform_grid_benchmark.cpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Uniform grid benchmarks
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <glm/vec3.hpp>

#include <benchmark/benchmark.h>

#include "cyberverse/spatial_index/dense_uniform_grid.hpp"
#include "cyberverse/spatial_index/sparse_uniform_grid.hpp"

namespace {

using namespace cyberverse;

constexpr int k_extent = 16;

template <typename Grid>
void fill(Grid& grid) {
  for (int z = 0; z < k_extent; ++z) {
    for (int y = 0; y < k_extent; ++y) {
      for (int x = 0; x < k_extent; ++x) {
        grid.set(glm::vec3(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f, static_cast<float>(z) + 0.5f),
                 x + y + z);
      }
    }
  }
}

template <typename Grid>
void benchmark_set(benchmark::State& state) {
  for (auto _ : state) {
    Grid grid(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));
    fill(grid);
    benchmark::DoNotOptimize(grid.grid_num());
  }
}

template <typename Grid>
void benchmark_get(benchmark::State& state) {
  Grid grid(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));
  fill(grid);

  for (auto _ : state) {
    int sum = 0;
    for (int z = 0; z < k_extent; ++z) {
      for (int y = 0; y < k_extent; ++y) {
        for (int x = 0; x < k_extent; ++x) {
          const int* value = grid.get(
              glm::vec3(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f, static_cast<float>(z) + 0.5f));
          sum += value ? *value : 0;
        }
      }
    }
    benchmark::DoNotOptimize(sum);
  }
}

template <typename Grid>
void benchmark_range_query(benchmark::State& state) {
  Grid grid(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));
  fill(grid);

  const geometry::aabb range(glm::vec3(2.0f), glm::vec3(14.0f));
  for (auto _ : state) {
    int sum = 0;
    grid.query(range, [&](const int& value, const geometry::aabb&) {
      sum += value;
      return true;
    });
    benchmark::DoNotOptimize(sum);
  }
}

template <typename Grid>
void benchmark_ray_query(benchmark::State& state) {
  Grid grid(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));
  fill(grid);

  const geometry::ray ray(glm::vec3(-10.0f, 8.5f, 8.5f), glm::vec3(1.0f, 0.0f, 0.0f));
  for (auto _ : state) {
    int sum = 0;
    grid.query(ray, [&](const int& value, const geometry::aabb&) {
      sum += value;
      return true;
    });
    benchmark::DoNotOptimize(sum);
  }
}

} // namespace

BENCHMARK(benchmark_set<spatial_index::sparse_uniform_grid<int>>);
BENCHMARK(benchmark_set<spatial_index::dense_uniform_grid<int>>);
BENCHMARK(benchmark_get<spatial_index::sparse_uniform_grid<int>>);
BENCHMARK(benchmark_get<spatial_index::dense_uniform_grid<int>>);
BENCHMARK(benchmark_range_query<spatial_index::sparse_uniform_grid<int>>);
BENCHMARK(benchmark_range_query<spatial_index::dense_uniform_grid<int>>);
BENCHMARK(benchmark_ray_query<spatial_index::sparse_uniform_grid<int>>);
BENCHMARK(benchmark_ray_query<spatial_index::dense_uniform_grid<int>>);

BENCHMARK_MAIN();
