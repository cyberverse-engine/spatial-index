/**
 * @file uniform_grid_test.cpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Uniform grid tests
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include <glm/vec3.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include "cyberverse/spatial_index/dense_uniform_grid.hpp"
#include "cyberverse/spatial_index/sparse_uniform_grid.hpp"

namespace {

using namespace cyberverse;

template <typename Grid>
void test_cells_and_expansion() {
  Grid grid(glm::vec3(1.0f), glm::ivec3(2), glm::vec3(0.0f));

  REQUIRE(grid.empty());
  REQUIRE(grid.grid_num() == 0);
  REQUIRE(grid.get(glm::vec3(0.5f)) == nullptr);
  REQUIRE_FALSE(grid.contains(glm::vec3(0.5f)));

  grid.set(glm::vec3(0.5f), 42);
  REQUIRE(grid.grid_num() == 1);
  REQUIRE(grid.get(glm::vec3(0.5f)) != nullptr);
  REQUIRE(*grid.get(glm::vec3(0.5f)) == 42);

  // A cell below the current root grows the tree towards -infinity.
  grid.set(glm::vec3(-1000.5f, -1000.5f, -1000.5f), 7);
  REQUIRE(*grid.get(glm::vec3(-1000.5f, -1000.5f, -1000.5f)) == 7);

  // A cell above the current root grows it towards +infinity.
  grid.set(glm::vec3(2000.5f, 2000.5f, 2000.5f), 9);
  REQUIRE(*grid.get(glm::vec3(2000.5f, 2000.5f, 2000.5f)) == 9);

  // The old cells survive both expansions.
  REQUIRE(*grid.get(glm::vec3(0.5f)) == 42);
  REQUIRE(*grid.get(glm::vec3(-1000.5f, -1000.5f, -1000.5f)) == 7);

  // ensure() returns a mutable reference and counts a new cell only once.
  int& value = grid.ensure(glm::vec3(3000.5f));
  value = 11;
  REQUIRE(grid.grid_num() == 4);
  REQUIRE(grid.ensure(glm::vec3(3000.5f)) == 11);
  REQUIRE(grid.grid_num() == 4);

  // Setting a cell from a value owned by the grid is safe even when it expands the tree.
  grid.set(glm::vec3(-5000.5f), *grid.get(glm::vec3(0.5f)));
  REQUIRE(*grid.get(glm::vec3(-5000.5f)) == 42);
  REQUIRE(grid.grid_num() == 5);

  // Erase prunes the leaf and the empty ancestors.
  REQUIRE(grid.erase(glm::vec3(-1000.5f, -1000.5f, -1000.5f)));
  REQUIRE(grid.grid_num() == 4);
  REQUIRE(grid.get(glm::vec3(-1000.5f, -1000.5f, -1000.5f)) == nullptr);
  REQUIRE_FALSE(grid.erase(glm::vec3(-1000.5f, -1000.5f, -1000.5f)));

  grid.clear();
  REQUIRE(grid.empty());
  REQUIRE(grid.grid_num() == 0);
  REQUIRE(grid.get(glm::vec3(0.5f)) == nullptr);
}

template <typename Grid>
void test_ranges_and_generators() {
  Grid grid(glm::vec3(1.0f), glm::ivec3(2), glm::vec3(0.0f));

  // A range ending exactly on a cell boundary does not include the next cell.
  grid.set(geometry::aabb(glm::vec3(0.0f), glm::vec3(1.0f)), 5);
  REQUIRE(grid.grid_num() == 1);
  REQUIRE(*grid.get(glm::vec3(0.5f)) == 5);
  REQUIRE(grid.get(glm::vec3(1.5f)) == nullptr);

  // A degenerate range still selects the containing cell.
  grid.set(geometry::aabb(glm::vec3(2.0f), glm::vec3(2.0f)), 6);
  REQUIRE(*grid.get(glm::vec3(2.5f)) == 6);

  grid.clear();

  // Overlapping 2x2x2 cells in both the negative and positive directions.
  const geometry::aabb range(glm::vec3(-2.0f), glm::vec3(2.0f));
  grid.set(range, 9);
  REQUIRE(grid.grid_num() == 4 * 4 * 4);
  for (int z = -2; z < 2; ++z) {
    for (int y = -2; y < 2; ++y) {
      for (int x = -2; x < 2; ++x) {
        const int* value = grid.get(glm::vec3(x + 0.5f, y + 0.5f, z + 0.5f));
        REQUIRE(value != nullptr);
        REQUIRE(*value == 9);
      }
    }
  }

  // Generators receive the unit cell box, or take no argument at all.
  std::size_t visited = 0;
  grid.query(geometry::aabb(glm::vec3(-2.0f), glm::vec3(2.0f)), [&](const int& value, const geometry::aabb& box) {
    REQUIRE(value == 9);
    REQUIRE(box.is_valid());
    ++visited;
    return true;
  });
  REQUIRE(visited == grid.grid_num());

  // The non-const overload exposes mutable values.
  grid.query(range, [](int& value, const geometry::aabb&) {
    value = 10;
    return true;
  });
  REQUIRE(*grid.get(glm::vec3(0.5f)) == 10);

  const Grid& const_grid = grid;
  std::size_t const_count = 0;
  const_grid.query(range, [&](const int& value, const geometry::aabb&) {
    REQUIRE(value == 10);
    ++const_count;
    return true;
  });
  REQUIRE(const_count == 64);

  grid.set(geometry::aabb(glm::vec3(10.25f), glm::vec3(10.75f)), [](const geometry::aabb& box) {
    return static_cast<int>(box.min().x);
  });
  REQUIRE(*grid.get(glm::vec3(10.5f)) == 10);

  int generated = 0;
  grid.set(geometry::aabb(glm::vec3(-20.75f), glm::vec3(-20.25f)), [&generated] {
    return ++generated;
  });
  REQUIRE(generated > 0);
  REQUIRE(grid.get(glm::vec3(-20.5f)) != nullptr);

  // Erasing a range removes exactly the overlapped cells.
  const std::size_t before = grid.grid_num();
  REQUIRE(grid.erase(geometry::aabb(glm::vec3(-2.0f), glm::vec3(0.0f))));
  REQUIRE(grid.grid_num() < before);
  REQUIRE(grid.get(glm::vec3(-0.5f)) == nullptr);

  // Query with an early-out visits one cell.
  std::size_t early = 0;
  grid.query(range, [&](const int&, const geometry::aabb&) {
    ++early;
    return false;
  });
  REQUIRE(early == 1);
}

template <typename Grid>
void test_ray_and_frustum() {
  Grid grid(glm::vec3(1.0f), glm::ivec3(2), glm::vec3(0.0f));
  grid.set(glm::vec3(0.5f, 0.5f, 0.5f), 1);
  grid.set(glm::vec3(5.5f, 0.5f, 0.5f), 2);
  grid.set(glm::vec3(-5.5f, 0.5f, 0.5f), 3);

  const geometry::ray ray(glm::vec3(-10.0f, 0.5f, 0.5f), glm::vec3(1.0f, 0.0f, 0.0f));
  std::vector<int> hits;
  grid.query(ray, [&](const int& value, const geometry::aabb& box) {
    float distance = 0.0f;
    REQUIRE(geometry::ray_aabb(ray, box, distance));
    hits.push_back(value);
    return true;
  });
  REQUIRE(hits.size() == 3);
  REQUIRE(std::find(hits.begin(), hits.end(), 2) != hits.end());
  REQUIRE(std::find(hits.begin(), hits.end(), 3) != hits.end());

  // A ray that misses everything visits nothing.
  const geometry::ray miss(glm::vec3(-10.0f, 100.0f, 100.0f), glm::vec3(1.0f, 0.0f, 0.0f));
  std::size_t miss_count = 0;
  grid.query(miss, [&](const int&, const geometry::aabb&) {
    ++miss_count;
    return true;
  });
  REQUIRE(miss_count == 0);

  // A default-constructed frustum culls nothing.
  std::size_t frustum_count = 0;
  grid.query(geometry::frustum{}, [&](const int&, const geometry::aabb&) {
    ++frustum_count;
    return true;
  });
  REQUIRE(frustum_count == grid.grid_num());

  // A frustum that only contains the cell around the origin.
  geometry::frustum fr;
  fr.planes = { {
      { glm::vec3(1, 0, 0), 0.0f },
      { glm::vec3(-1, 0, 0), -1.0f },
      { glm::vec3(0, 1, 0), 0.0f },
      { glm::vec3(0, -1, 0), -1.0f },
      { glm::vec3(0, 0, 1), 0.0f },
      { glm::vec3(0, 0, -1), -1.0f },
  } };
  std::size_t box_count = 0;
  grid.query(fr, [&](const int& value, const geometry::aabb&) {
    REQUIRE(value == 1);
    ++box_count;
    return true;
  });
  REQUIRE(box_count == 1);
}

template <typename Grid>
void test_sparse_promotion() {
  Grid grid(glm::vec3(1.0f), glm::ivec3(4), glm::vec3(0.0f));

  // 8^3 cells force every internal node to use all 4^3 child slots, which promotes the sparse inline array into the
  // hash map representation.
  grid.set(geometry::aabb(glm::vec3(0.1f), glm::vec3(7.9f)), 5);
  REQUIRE(grid.grid_num() == 8 * 8 * 8);
  for (int z = 0; z < 8; ++z) {
    for (int y = 0; y < 8; ++y) {
      for (int x = 0; x < 8; ++x) {
        REQUIRE(*grid.get(glm::vec3(x + 0.5f, y + 0.5f, z + 0.5f)) == 5);
      }
    }
  }

  REQUIRE(grid.erase(geometry::aabb(glm::vec3(0.0f), glm::vec3(6.0f))));
  REQUIRE(grid.grid_num() == 8 * 8 * 8 - 6 * 6 * 6);
  REQUIRE(grid.get(glm::vec3(0.5f, 0.5f, 0.5f)) == nullptr);
  REQUIRE(*grid.get(glm::vec3(7.5f, 7.5f, 7.5f)) == 5);
}

template <typename Grid>
void test_cell_api() {
  using cell_coord = typename Grid::cell_coord;
  Grid grid(glm::vec3(1.0f), glm::ivec3(2), glm::vec3(0.0f));

  const cell_coord origin_cell{ 0, 0, 0 };
  REQUIRE(grid.get_cell(origin_cell) == nullptr);

  int& value = grid.ensure_cell(origin_cell);
  value = 7;
  REQUIRE(grid.contains_cell(origin_cell));
  REQUIRE(grid.get_cell(origin_cell) != nullptr);
  REQUIRE(*grid.get_cell(origin_cell) == 7);
  REQUIRE(grid.ensure_cell(origin_cell) == 7);

  cell_coord lo{};
  cell_coord hi{};
  REQUIRE(grid.cell_range(geometry::aabb(glm::vec3(0.0f), glm::vec3(1.0f)), lo, hi));
  REQUIRE(lo == origin_cell);
  REQUIRE(hi == origin_cell);

  REQUIRE(grid.cell_range(geometry::aabb(glm::vec3(0.0f), glm::vec3(2.0f)), lo, hi));
  REQUIRE(lo == origin_cell);
  REQUIRE(hi == cell_coord(1, 1, 1));

  const cell_coord far_cell{ 10, 0, 0 };
  grid.ensure_cell(far_cell) = 9;
  REQUIRE(grid.contains_cell(far_cell));
  REQUIRE(*grid.get_cell(far_cell) == 9);

  REQUIRE(grid.erase_cell(origin_cell));
  REQUIRE(grid.get_cell(origin_cell) == nullptr);
  REQUIRE_FALSE(grid.erase_cell(origin_cell));
  REQUIRE(grid.contains_cell(far_cell));
  REQUIRE(*grid.get_cell(far_cell) == 9);
}

} // namespace

TEMPLATE_TEST_CASE("uniform grid cells and root expansion",
                   "[uniform_grid]",
                   cyberverse::spatial_index::sparse_uniform_grid<int>,
                   cyberverse::spatial_index::dense_uniform_grid<int>) {
  test_cells_and_expansion<TestType>();
}

TEMPLATE_TEST_CASE("uniform grid ranges and generators",
                   "[uniform_grid]",
                   cyberverse::spatial_index::sparse_uniform_grid<int>,
                   cyberverse::spatial_index::dense_uniform_grid<int>) {
  test_ranges_and_generators<TestType>();
}

TEMPLATE_TEST_CASE("uniform grid ray and frustum queries",
                   "[uniform_grid]",
                   cyberverse::spatial_index::sparse_uniform_grid<int>,
                   cyberverse::spatial_index::dense_uniform_grid<int>) {
  test_ray_and_frustum<TestType>();
}

TEMPLATE_TEST_CASE("uniform grid sparse promotion",
                   "[uniform_grid]",
                   cyberverse::spatial_index::sparse_uniform_grid<int>,
                   cyberverse::spatial_index::dense_uniform_grid<int>) {
  test_sparse_promotion<TestType>();
}

TEST_CASE("uniform grid accessors and invalid configuration", "[uniform_grid]") {
  using sparse_uniform_grid = cyberverse::spatial_index::sparse_uniform_grid<int>;
  using dense_uniform_grid = cyberverse::spatial_index::dense_uniform_grid<int>;

  REQUIRE_THROWS_AS(sparse_uniform_grid(glm::vec3(0.0f), glm::ivec3(2)), std::invalid_argument);
  REQUIRE_THROWS_AS(sparse_uniform_grid(glm::vec3(1.0f), glm::ivec3(1)), std::invalid_argument);
  REQUIRE_THROWS_AS(dense_uniform_grid(glm::vec3(1.0f), glm::ivec3(2, 0, 2)), std::invalid_argument);
  REQUIRE_THROWS_AS(dense_uniform_grid(glm::vec3(1.0f), glm::ivec3(2), glm::vec3(0.0f, NAN, 0.0f)),
                    std::invalid_argument);
  REQUIRE_THROWS_AS(dense_uniform_grid(glm::vec3(std::numeric_limits<float>::max()),
                                       glm::ivec3(2),
                                       glm::vec3(std::numeric_limits<float>::max())),
                    std::invalid_argument);

  sparse_uniform_grid grid(glm::vec3(2.0f, 3.0f, 4.0f), glm::ivec3(2, 3, 4), glm::vec3(1.0f, 2.0f, 3.0f));
  REQUIRE(grid.unit() == glm::vec3(2.0f, 3.0f, 4.0f));
  REQUIRE(grid.branch_factor() == glm::ivec3(2, 3, 4));
  REQUIRE(grid.origin() == glm::vec3(1.0f, 2.0f, 3.0f));

  const geometry::aabb box = grid.get_unit_box(glm::vec3(2.0f, 3.0f, 4.0f));
  REQUIRE(box.is_valid());
  REQUIRE(box.min() == glm::vec3(1.0f, 2.0f, 3.0f));
  REQUIRE(box.max() == glm::vec3(3.0f, 5.0f, 7.0f));

  REQUIRE(grid.memory_usage_in_bytes() > 0);
  REQUIRE(grid.get(glm::vec3(NAN)) == nullptr);
  REQUIRE_FALSE(grid.erase(glm::vec3(NAN)));
}

TEMPLATE_TEST_CASE("uniform grid cell API",
                   "[uniform_grid]",
                   cyberverse::spatial_index::sparse_uniform_grid<int>,
                   cyberverse::spatial_index::dense_uniform_grid<int>) {
  test_cell_api<TestType>();
}
