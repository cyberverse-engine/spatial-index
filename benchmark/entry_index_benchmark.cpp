/**
 * @file entry_index_benchmark.cpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Entry index benchmarks
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <cstdint>
#include <vector>

#include <glm/vec3.hpp>

#include <benchmark/benchmark.h>

#include "cyberverse/spatial_index.hpp"

namespace {

using namespace cyberverse;

constexpr int k_extent = 46;
constexpr int k_objects = k_extent * k_extent * k_extent;
constexpr float k_half = 0.1f;

geometry::aabb object_box(int x, int y, int z) {
  const glm::vec3 center(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f, static_cast<float>(z) + 0.5f);
  return geometry::aabb(center - glm::vec3(k_half), center + glm::vec3(k_half));
}

template <typename Index>
void fill(Index& index) {
  for (int z = 0; z < k_extent; ++z) {
    for (int y = 0; y < k_extent; ++y) {
      for (int x = 0; x < k_extent; ++x) {
        const std::uint64_t handle = static_cast<std::uint64_t>((z * k_extent + y) * k_extent + x);
        index.insert(handle, object_box(x, y, z), static_cast<int>(handle & 3u));
      }
    }
  }
}

template <typename CellIndex>
void benchmark_cell_entry_insert(benchmark::State& state) {
  for (auto _ : state) {
    spatial_index::cell_entry_index<CellIndex> index(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));
    fill(index);
    benchmark::DoNotOptimize(index.size());
  }
}

template <typename CellIndex>
void benchmark_cell_entry_query(benchmark::State& state) {
  spatial_index::cell_entry_index<CellIndex> index(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));
  fill(index);
  const geometry::aabb range(glm::vec3(0.0f), glm::vec3(16.0f));
  for (auto _ : state) {
    std::size_t count = 0;
    index.query(range, [&](std::uint64_t, int, const geometry::aabb&) {
      ++count;
      return true;
    });
    benchmark::DoNotOptimize(count);
  }
}

template <typename CellIndex>
void benchmark_cell_entry_frustum(benchmark::State& state) {
  spatial_index::cell_entry_index<CellIndex> index(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));
  fill(index);
  const geometry::frustum fr{};
  typename spatial_index::cell_entry_index<CellIndex>::query_scratch_type scratch;
  std::vector<std::uint64_t> visible;
  visible.reserve(static_cast<std::size_t>(k_objects));
  for (auto _ : state) {
    visible.clear();
    index.query(fr, scratch, [&](std::uint64_t handle, int, const geometry::aabb&) {
      visible.push_back(handle);
      return true;
    });
    benchmark::DoNotOptimize(visible.size());
  }
}

template <typename Index>
void benchmark_entry_index(benchmark::State& state) {
  Index index;
  fill(index);
  const geometry::aabb range(glm::vec3(0.0f), glm::vec3(16.0f));
  for (auto _ : state) {
    std::size_t count = 0;
    index.query(range, [&](std::uint64_t, int, const geometry::aabb&) {
      ++count;
      return true;
    });
    benchmark::DoNotOptimize(count);
  }
}

using cell_bucket = spatial_index::linear_index<std::uint64_t, int>;
using sparse_cells = spatial_index::sparse_uniform_grid<cell_bucket>;
using dense_cells = spatial_index::dense_uniform_grid<cell_bucket>;

} // namespace

BENCHMARK(benchmark_cell_entry_insert<sparse_cells>);
BENCHMARK(benchmark_cell_entry_insert<dense_cells>);
BENCHMARK(benchmark_cell_entry_query<sparse_cells>);
BENCHMARK(benchmark_cell_entry_query<dense_cells>);
BENCHMARK(benchmark_cell_entry_frustum<sparse_cells>);
BENCHMARK(benchmark_cell_entry_frustum<dense_cells>);
BENCHMARK(benchmark_entry_index<spatial_index::bvh<std::uint64_t, int>>);
