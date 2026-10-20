# spatial-index

Cyberverse Engine spatial index library.

The library targets spatial coarse indices (uniform grids, spatial hashes, BVHs) and fine-grained local block
indices. The current implementation provides two uniform grid variants that share the same tree layout, dynamic root
expansion and query algorithms:

- `spatial_index::sparse_uniform_grid<T>` — children are stored in an inline `keyed_array` first and promoted to a
  hash map when a node has many children.
- `spatial_index::dense_uniform_grid<T>` — every branch slot of a materialized node is allocated in a contiguous
  vector. The grid still grows dynamically with exactly the same expansion algorithm as the sparse variant.

Both types satisfy `spatial_index::PartitionIndexType`.

## Configuration

A grid is defined by:

- `unit`: per-axis unit cell size.
- `branch_factor`: number of children per node and per axis, at least `2` on every axis.
- `origin`: world position of the lower corner of cell `(0, 0, 0)`.

Cell coordinates are signed integers, so positions below the origin are supported. The root is grown on demand: a cell
outside the current root inserts the old root at the appropriate child index of a new root, extending the tree in the
direction of the requested cell.

## Usage

```cpp
#include "cyberverse/spatial_index.hpp"

using namespace cyberverse;

spatial_index::sparse_uniform_grid<int> grid(glm::vec3(1.0f), glm::ivec3(4));

grid.set(glm::vec3(0.5f), 42);
grid.set(geometry::aabb(glm::vec3(-2.0f), glm::vec3(2.0f)), 7);
grid.set(geometry::aabb(glm::vec3(10.25f), glm::vec3(10.75f)), [](const geometry::aabb& cell) {
  return static_cast<int>(cell.min().x);
});

const int* value = grid.get(glm::vec3(0.5f));

grid.query(geometry::ray(glm::vec3(-10.0f, 0.5f, 0.5f), glm::vec3(1.0f, 0.0f, 0.0f)),
           [](const int& value, const geometry::aabb& cell) {
             // return false to stop the query
             return true;
           });

grid.erase(glm::vec3(0.5f));
grid.clear();
```

Range operations use half-open cells: a range ending exactly on a cell boundary does not include the next cell, while
a degenerate range (`min == max`) still selects the containing cell. Range queries use the same rule.

`get_unit_box(pos)` returns the cell box for any finite position, whether or not the cell exists. Any insertion may move
the tree and invalidate pointers / references previously returned by `get()` or `ensure()`.

## Build

```bash
cmake -S . -B build
cmake --build build

# Tests must be built and run under /tmp, never inside the repo.
cmake -S . -B /tmp/cyberverse-spatial-index-test
cmake --build /tmp/cyberverse-spatial-index-test
ctest --test-dir /tmp/cyberverse-spatial-index-test --output-on-failure
```

- `CYBERVERSE_SPATIAL_INDEX_BUILD_TESTS` (default `ON`) builds `cyberverse_spatial_index_tests` with Catch2.
- `CYBERVERSE_SPATIAL_INDEX_BUILD_BENCHMARKS` (default `ON`) builds `cyberverse_spatial_index_benchmarks`.
- The geometry dependency is resolved from an installed `cyberverse_geometry` package, `CYBERVERSE_GEOMETRY_DIR`, the
  sibling `../geometry` source tree, or `FetchContent` (`CYBERVERSE_GEOMETRY_GIT_REPOSITORY`).
- Presets: `cmake --preset debug` / `cmake --preset release`, then `cmake --build --preset debug|release`.
