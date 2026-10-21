# spatial-index

Cyberverse Engine spatial index library (C++23, GLM).

The library separates two spatial-index layers:

- **`CellIndexType`** — a cell/subspace partition that stores one user value per cell.
  `sparse_uniform_grid<T>` and `dense_uniform_grid<T>` implement it.
- **`EntryIndexType`** — a spatial index over entries described by an AABB, an externally owned
  `entry_handle`, and an optional `entry_value`. `linear_index`, `bvh`, `hash_index` and
  `cell_entry_index` implement it.

`cell_entry_index<CellIndex>` composes a cell index with a per-cell `EntryBucketType`
(`CellIndex::value_type`) to provide an `EntryIndexType`. Cell buckets are themselves entry indexes,
so there is no separate `object_bucket` type.

## Uniform grids

A grid is defined by:

- `unit`: per-axis unit cell size;
- `branch_factor`: children per node and per axis, at least `2` on every axis;
- `origin`: world position of the lower corner of cell `(0, 0, 0)`.

Cells are signed int64 coordinates and half-open. The root grows on demand; inserting a cell outside
the current root extends the tree in that direction.

```cpp
#include "cyberverse/spatial_index.hpp"

using namespace cyberverse;

spatial_index::sparse_uniform_grid<int> grid(glm::vec3(1.0f), glm::ivec3(4));

grid.set(glm::vec3(0.5f), 42);
grid.set(geometry::aabb(glm::vec3(-2.0f), glm::vec3(2.0f)), 7);

const int* value = grid.get(glm::vec3(0.5f));

grid.query(geometry::ray(glm::vec3(-10.0f, 0.5f, 0.5f), glm::vec3(1.0f, 0.0f, 0.0f)),
           [](const int& value, const geometry::aabb& cell) {
             return true; // false stops the query
           });
```

`CellIndexType` additionally exposes cell-level operations used by `cell_entry_index`:
`cell_range`, `get_cell`, `ensure_cell`, `contains_cell` and `erase_cell`. `contains_cell` reports
whether the coordinate is inside the current root; `get_cell` reports whether a value exists.

Range operations use half-open cells: a range ending exactly on a cell boundary does not include the
next cell, while a degenerate range selects the containing cell. Insertions may invalidate previously
returned pointers/references.

## Entry indexes

### Handles

`entry_handle` is not a library-defined type. It is a caller-supplied type satisfying
`spatial_index::EntryHandleType`:

```cpp
template <typename H>
concept EntryHandleType =
    std::copyable<H> &&
    std::default_initializable<H> &&
    std::equality_comparable<H> &&
    std::totally_ordered<H> &&
    requires(const H& h) {
      { std::hash<H>{}(h) } -> std::convertible_to<std::size_t>;
    };
```

The index never interprets handle bits, never reuses handles, and never stores a generation. The caller
owns handle allocation and uniqueness. Inserting an already-present handle returns `false`.

### Entry value

Every entry can carry an optional `entry_value` (default `std::monostate`). The value is a lightweight,
cache-friendly filter field:

- use a small value type to store it inline;
- use `std::monostate` plus an external `handle -> value` map when a copy per cell is undesirable.

Filters and visitors receive both the handle and the value:

```cpp
bool(handle, const entry_value&)                    // filter
bool(handle, const entry_value&, const aabb&)       // visitor
```

### Linear, BVH and hash

```cpp
using handle = std::uint64_t;

spatial_index::linear_index<handle> linear;                // vector scan; best as a cell bucket
spatial_index::bvh<handle> bvh;                            // dynamic AABB tree; direct EntryIndexType
spatial_index::hash_index<handle> hash(glm::vec3(1.0f));   // spatial hash; direct EntryIndexType
```

All three implement `EntryIndexType`. `bvh` intentionally does **not** implement `CellIndexType`;
there is no `bvh<linear_index<...>>` composition. A BVH's leaf layout/SOO is an internal detail.

### Cell-composed entry index

```cpp
using bucket = spatial_index::linear_index<std::uint64_t>;
using cells  = spatial_index::sparse_uniform_grid<bucket>;
using index  = spatial_index::cell_entry_index<cells>;

index objects(glm::vec3(1.0f), glm::ivec3(4));
objects.insert(std::uint64_t{ 42 }, geometry::aabb(glm::vec3(0.4f), glm::vec3(0.6f)));
```

`cell_entry_index` inserts each entry into every overlapped cell. It deduplicates cross-cell visits with
a caller-owned query scratch; the index itself has no mutable query state.

```cpp
index::query_scratch_type scratch;

std::vector<std::uint64_t> visible;
visible.reserve(1024);
spatial_index::collect_visible(objects, geometry::frustum{}, visible);
```

## Queries and parallelism

`EntryIndexType` provides AABB, sphere, ray and frustum queries, `nearest`, `k_nearest` and `raycast`.
Every query is `const`, visits each entry at most once, and calls the filter once per entry.

Queries are safe to run concurrently on the same index. Writes are not thread-safe and require external
synchronization. `cell_entry_index` uses one scratch per query/thread; do not share a scratch between
threads. Passing a scratch explicitly avoids per-query allocation:

```cpp
index::query_scratch_type scratch;
objects.query(range, scratch, [](std::uint64_t, const auto&, const auto&) {
  return true;
});
```

## Build

```bash
cmake -S . -B /tmp/cyberverse-spatial-index-test
cmake --build /tmp/cyberverse-spatial-index-test
ctest --test-dir /tmp/cyberverse-spatial-index-test --output-on-failure
```

- `CYBERVERSE_SPATIAL_INDEX_BUILD_TESTS` (default `ON`) builds `cyberverse_spatial_index_tests` with
  Catch2.
- `CYBERVERSE_SPATIAL_INDEX_BUILD_BENCHMARKS` (default `ON`) builds
  `cyberverse_spatial_index_benchmarks`.
- Geometry is resolved from an installed package, `CYBERVERSE_GEOMETRY_DIR`, sibling `../geometry`, or
  `FetchContent`.
- Tests must be built and run under `/tmp`, never inside the repository.

## Design notes

See [docs/design.md](docs/design.md) for the design principles and roadmap.
