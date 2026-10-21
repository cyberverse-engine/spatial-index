/**
 * @file entry_index_test.cpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Entry index tests
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include <glm/vec3.hpp>

#include <catch2/catch_test_macros.hpp>

#include "cyberverse/spatial_index.hpp"

namespace {

struct custom_handle {
  std::uint64_t value = 0;

  friend bool operator==(const custom_handle&, const custom_handle&) noexcept = default;
  friend auto operator<=>(const custom_handle&, const custom_handle&) noexcept = default;
};

} // namespace

template <>
struct std::hash<custom_handle> {
  std::size_t operator()(const custom_handle& handle) const noexcept {
    return std::hash<std::uint64_t>{}(handle.value);
  }
};

namespace {

using namespace cyberverse;

geometry::aabb box(const glm::vec3& min, const glm::vec3& max) { return geometry::aabb(min, max); }

template <spatial_index::EntryIndexType Index>
void check_basic_contract() {
  Index index;
  REQUIRE(index.empty());
  REQUIRE(index.insert(1, box(glm::vec3(0.1f), glm::vec3(0.2f))));
  REQUIRE(index.insert(2, box(glm::vec3(-3.0f), glm::vec3(3.0f))));
  REQUIRE(index.insert(3, box(glm::vec3(5.0f), glm::vec3(6.0f))));
  REQUIRE_FALSE(index.insert(1, box(glm::vec3(0.0f), glm::vec3(1.0f))));
  REQUIRE(index.contains(1));
  REQUIRE(index.bounds(1) != nullptr);
  REQUIRE(index.value(1) != nullptr);

  int visited = 0;
  index.query(box(glm::vec3(-10.0f), glm::vec3(10.0f)), [&](std::uint64_t handle, const auto&, const auto&) {
    REQUIRE(handle <= 3);
    ++visited;
    return true;
  });
  REQUIRE(visited >= 2);

  const auto nearest = index.nearest(glm::vec3(0.1f), 10.0f);
  REQUIRE(nearest.has_value());

  const auto k_nearest = index.k_nearest(glm::vec3(0.1f), 2, 10.0f);
  REQUIRE(k_nearest.size() == 2);

  const auto hit = index.raycast(geometry::ray(glm::vec3(-10.0f, 0.1f, 0.1f), glm::vec3(1.0f, 0.0f, 0.0f)), 100.0f);
  REQUIRE(hit.has_value());

  REQUIRE(index.update(3, box(glm::vec3(7.0f), glm::vec3(8.0f))));
  REQUIRE(index.erase(2));
  REQUIRE_FALSE(index.contains(2));
  REQUIRE(index.size() == 2);
  index.clear();
  REQUIRE(index.empty());
}

} // namespace

TEST_CASE("entry index concepts are satisfied") {
  static_assert(spatial_index::EntryHandleType<std::uint64_t>);
  static_assert(spatial_index::EntryHandleType<custom_handle>);
  static_assert(spatial_index::EntryBucketType<spatial_index::linear_index<std::uint64_t>>);
  static_assert(spatial_index::EntryIndexType<spatial_index::linear_index<std::uint64_t>>);
  static_assert(spatial_index::EntryIndexType<spatial_index::bvh<std::uint64_t>>);
  static_assert(spatial_index::EntryIndexType<spatial_index::hash_index<std::uint64_t>>);
  static_assert(spatial_index::EntryIndexType<spatial_index::linear_index<custom_handle>>);
}

TEST_CASE("linear index basic contract") { check_basic_contract<spatial_index::linear_index<std::uint64_t>>(); }

TEST_CASE("bvh basic contract") { check_basic_contract<spatial_index::bvh<std::uint64_t>>(); }

TEST_CASE("hash index basic contract") {
  using hash_index = spatial_index::hash_index<std::uint64_t>;
  hash_index index(glm::vec3(1.0f));
  REQUIRE(index.insert(1, box(glm::vec3(0.1f), glm::vec3(0.2f))));
  REQUIRE(index.insert(2, box(glm::vec3(-3.0f), glm::vec3(3.0f))));
  REQUIRE(index.insert(3, box(glm::vec3(5.0f), glm::vec3(6.0f))));
  REQUIRE_FALSE(index.insert(1, box(glm::vec3(0.0f), glm::vec3(1.0f))));
  REQUIRE(index.nearest(glm::vec3(0.1f), 10.0f) == 1);
  REQUIRE(index.k_nearest(glm::vec3(0.1f), 2, 10.0f).size() == 2);
  REQUIRE(index.raycast(geometry::ray(glm::vec3(-10.0f, 0.1f, 0.1f), glm::vec3(1.0f, 0.0f, 0.0f)), 100.0f));
  REQUIRE(index.update(3, box(glm::vec3(7.0f), glm::vec3(8.0f))));
  REQUIRE(index.erase(2));
  REQUIRE_FALSE(index.contains(2));
}

TEST_CASE("entry value is exposed to filters and visitors") {
  spatial_index::linear_index<std::uint64_t, int> index;
  REQUIRE(index.insert(1, box(glm::vec3(0.0f), glm::vec3(1.0f)), 42));
  REQUIRE(index.value(1) != nullptr);
  REQUIRE(*index.value(1) == 42);
  int visited = 0;
  index.query(
      box(glm::vec3(-1.0f), glm::vec3(2.0f)),
      [](std::uint64_t, int value) {
        return value == 42;
      },
      [&](std::uint64_t, int value, const geometry::aabb&) {
        REQUIRE(value == 42);
        ++visited;
        return true;
      });
  REQUIRE(visited == 1);
  REQUIRE(index.set_value(1, 7));
  REQUIRE(*index.value(1) == 7);
}

TEST_CASE("query scratch deduplicates handles") {
  spatial_index::entry_query_scratch<std::uint64_t> scratch;
  REQUIRE(scratch.mark(1));
  REQUIRE_FALSE(scratch.mark(1));
  REQUIRE(scratch.mark(2));
  scratch.clear();
  REQUIRE(scratch.mark(1));
}

TEST_CASE("queries can run concurrently with per-thread scratch") {
  spatial_index::linear_index<std::uint64_t, int> index;
  for (std::uint64_t i = 0; i < 128; ++i) {
    const float x = static_cast<float>(i);
    REQUIRE(index.insert(i, box(glm::vec3(x), glm::vec3(x + 0.5f)), static_cast<int>(i)));
  }

  const auto query_box = box(glm::vec3(-1.0f), glm::vec3(200.0f));
  std::atomic<int> failures{ 0 };
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([&] {
      for (int repeat = 0; repeat < 20; ++repeat) {
        int count = 0;
        index.query(query_box, [&](std::uint64_t, int, const geometry::aabb&) {
          ++count;
          return true;
        });
        if (count != 128) {
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

TEST_CASE("custom handle type works end to end") {
  spatial_index::linear_index<custom_handle, int> index;
  const custom_handle first{ 1 };
  const custom_handle second{ 2 };
  REQUIRE(index.insert(first, box(glm::vec3(0.0f), glm::vec3(1.0f)), 7));
  REQUIRE(index.insert(second, box(glm::vec3(2.0f), glm::vec3(3.0f)), 9));
  REQUIRE(index.contains(first));
  REQUIRE(index.nearest(glm::vec3(0.5f), 10.0f) == first);
  int count = 0;
  index.query(box(glm::vec3(-1.0f), glm::vec3(4.0f)), [&](const custom_handle&, int, const geometry::aabb&) {
    ++count;
    return true;
  });
  REQUIRE(count == 2);
}
