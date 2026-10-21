/**
 * @file cell_entry_index_test.cpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Cell entry index tests
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <ranges>
#include <thread>
#include <vector>

#include <glm/vec3.hpp>

#include <catch2/catch_test_macros.hpp>

#include "cyberverse/spatial_index.hpp"

namespace {

using namespace cyberverse;

using bucket = spatial_index::linear_index<std::uint64_t, int>;
using sparse_cells = spatial_index::sparse_uniform_grid<bucket>;
using dense_cells = spatial_index::dense_uniform_grid<bucket>;
using sparse_index = spatial_index::cell_entry_index<sparse_cells>;
using dense_index = spatial_index::cell_entry_index<dense_cells>;

geometry::aabb box(const glm::vec3& min, const glm::vec3& max) { return geometry::aabb(min, max); }

template <spatial_index::EntryIndexType Index>
void check_index_contract(Index& index) {
  REQUIRE(index.empty());
  REQUIRE(index.insert(1, box(glm::vec3(0.1f), glm::vec3(0.2f)), 11));
  REQUIRE(index.insert(2, box(glm::vec3(-3.0f), glm::vec3(3.0f)), 22));
  REQUIRE(index.insert(3, box(glm::vec3(5.0f), glm::vec3(6.0f)), 33));
  REQUIRE_FALSE(index.insert(1, box(glm::vec3(0.0f), glm::vec3(1.0f)), 0));
  REQUIRE(index.contains(1));
  REQUIRE(index.bounds(1) != nullptr);
  REQUIRE(index.value(1) != nullptr);
  REQUIRE(*index.value(1) == 11);

  std::vector<std::uint64_t> seen;
  int filter_calls = 0;
  index.query(
      box(glm::vec3(-10.0f), glm::vec3(10.0f)),
      [&](std::uint64_t, int value) {
        ++filter_calls;
        return value > 0;
      },
      [&](std::uint64_t handle, int value, const geometry::aabb&) {
        REQUIRE(value > 0);
        seen.push_back(handle);
        return true;
      });
  REQUIRE(std::ranges::all_of(seen, [](std::uint64_t handle) {
    return handle >= 1 && handle <= 3;
  }));
  REQUIRE(std::ranges::adjacent_find(seen) == seen.end());
  REQUIRE(seen.size() == 3);
  REQUIRE(filter_calls == 3);

  const auto nearest = index.nearest(glm::vec3(0.1f), 100.0f);
  REQUIRE(nearest == 1);
  const auto nearest_filtered = index.nearest(
      glm::vec3(0.1f),
      [](std::uint64_t, int value) {
        return value == 22;
      },
      100.0f);
  REQUIRE(nearest_filtered == 2);
  REQUIRE(index.k_nearest(glm::vec3(0.1f), 2, 100.0f).size() == 2);

  const auto hit = index.raycast(geometry::ray(glm::vec3(-10.0f, 0.1f, 0.1f), glm::vec3(1.0f, 0.0f, 0.0f)), 100.0f);
  REQUIRE(hit.has_value());

  REQUIRE(index.set_value(1, 123));
  REQUIRE(*index.value(1) == 123);
  REQUIRE(index.update(3, box(glm::vec3(7.0f), glm::vec3(8.0f))));
  std::vector<std::uint64_t> visible;
  REQUIRE(spatial_index::collect_visible(index, geometry::frustum{}, visible) == seen.size());
  REQUIRE(visible.size() == seen.size());

  int early_out = 0;
  index.query(box(glm::vec3(-10.0f), glm::vec3(10.0f)), [&](std::uint64_t, int, const geometry::aabb&) {
    ++early_out;
    return false;
  });
  REQUIRE(early_out == 1);

  REQUIRE(index.erase(2));
  REQUIRE_FALSE(index.contains(2));
  REQUIRE(index.size() == 2);
  index.clear();
  REQUIRE(index.empty());
}

} // namespace

TEST_CASE("cell entry index concepts are satisfied") {
  static_assert(spatial_index::CellIndexType<sparse_cells>);
  static_assert(spatial_index::CellIndexType<dense_cells>);
  static_assert(spatial_index::EntryBucketType<bucket>);
  static_assert(spatial_index::EntryIndexType<sparse_index>);
  static_assert(spatial_index::EntryIndexType<dense_index>);
}

TEST_CASE("sparse cell entry index contract") {
  sparse_index index(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));
  check_index_contract(index);
}

TEST_CASE("dense cell entry index contract") {
  dense_index index(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));
  check_index_contract(index);
}

TEST_CASE("cell entry index deduplicates across cells") {
  sparse_index index(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));
  REQUIRE(index.insert(7, box(glm::vec3(-3.0f), glm::vec3(3.0f)), 0));

  const auto query_box = box(glm::vec3(-10.0f), glm::vec3(10.0f));
  int count = 0;
  index.query(query_box, [&](std::uint64_t handle, int, const geometry::aabb&) {
    REQUIRE(handle == 7);
    ++count;
    return true;
  });
  REQUIRE(count == 1);
}

TEST_CASE("cell entry index query with external value map") {
  using mono_bucket = spatial_index::linear_index<std::uint64_t>;
  using mono_cells = spatial_index::sparse_uniform_grid<mono_bucket>;
  spatial_index::cell_entry_index<mono_cells> index(glm::vec3(1.0f), glm::ivec3(4));
  REQUIRE(index.insert(1, box(glm::vec3(0.0f), glm::vec3(1.0f))));
  REQUIRE(index.insert(2, box(glm::vec3(2.0f), glm::vec3(3.0f))));

  const std::vector<int> external = { 42, 7 };
  int count = 0;
  index.query(
      box(glm::vec3(-1.0f), glm::vec3(4.0f)),
      [&](std::uint64_t handle, const std::monostate&) {
        return external[handle - 1] == 42;
      },
      [&](std::uint64_t handle, const std::monostate&, const geometry::aabb&) {
        REQUIRE(handle == 1);
        ++count;
        return true;
      });
  REQUIRE(count == 1);
}

TEST_CASE("cell entry index queries can run concurrently") {
  sparse_index index(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));
  for (std::uint64_t i = 0; i < 64; ++i) {
    const float x = static_cast<float>(i);
    REQUIRE(index.insert(i, box(glm::vec3(x), glm::vec3(x + 0.5f)), static_cast<int>(i)));
  }

  const auto query_box = box(glm::vec3(-1.0f), glm::vec3(100.0f));
  std::atomic<int> failures{ 0 };
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([&] {
      sparse_index::query_scratch_type scratch;
      for (int repeat = 0; repeat < 20; ++repeat) {
        int count = 0;
        index.query(query_box, scratch, [&](std::uint64_t, int, const geometry::aabb&) {
          ++count;
          return true;
        });
        if (count != 64) {
          ++failures;
        }
      }
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  REQUIRE(failures.load() == 0);
}
