# AGENTS.md

`spatial-index` is the standalone spatial index module of Cyberverse Engine: C++23, GLM, built as the
static library `cyberverse::spatial_index::static_lib`. It depends on
`cyberverse::geometry::static_lib`. The public API lives in `src/cyberverse/spatial_index/`;
`src/cyberverse/spatial_index.hpp` is the umbrella header, and
`src/cyberverse/spatial_index/spatial_index.cpp` is only a translation unit that includes the umbrella
header.

The public API is header-only. The library has two index layers:

- `CellIndexType` — a cell/subspace partition storing one user value per cell.
  `sparse_uniform_grid<T>` and `dense_uniform_grid<T>` implement it.
- `EntryIndexType` — a spatial index over entries with an externally owned `entry_handle`, an AABB
  and an optional `entry_value`. `linear_index`, `bvh`, `hash_index` and `cell_entry_index` implement
  it.

`EntryBucketType` is a **requirement subset** of `EntryIndexType`: every `EntryIndexType` also
satisfies `EntryBucketType`. A cell bucket is any such type that is also default constructible; there
is no `object_bucket` class.

## Build and test

Run from this directory. Build and run tests under `/tmp`, never inside the repository.

```bash
cmake -S . -B /tmp/cyberverse-spatial-index-test
cmake --build /tmp/cyberverse-spatial-index-test
ctest --test-dir /tmp/cyberverse-spatial-index-test --output-on-failure
```

- The first configure resolves `geometry` and fetches GLM through it, plus Catch2 v3.5.2 and Google
  Benchmark v1.9.5. Skip the latter two with
  `-DCYBERVERSE_SPATIAL_INDEX_BUILD_TESTS=OFF -DCYBERVERSE_SPATIAL_INDEX_BUILD_BENCHMARKS=OFF`.
- `cmake/engine_dep_geometry.cmake` resolves `cyberverse::geometry::static_lib` idempotently, in order:
  an existing target, an installed `cyberverse_geometry` package, `CYBERVERSE_GEOMETRY_DIR` (CMake or
  environment), the sibling `../geometry`, or `FetchContent`. When geometry is vendored, its tests and
  benchmarks are forced off.
- Entry coverage and queries use `geometry::aabb_overlaps`, `geometry::sphere_overlaps_aabb`,
  `geometry::point_aabb_distance_sq`, `geometry::ray_aabb` and `geometry::is_outside`. A standalone
  FetchContent build must use a geometry revision containing these predicates.
- Presets: `cmake --preset debug` / `cmake --preset release`, then
  `cmake --build --preset debug|release`; outputs go to `build/` and `build-release/`. Tests still must
  be built and run under `/tmp`.
- `test/` builds `cyberverse_spatial_index_tests` (Catch2, discovered by CTest); `benchmark/` builds
  `cyberverse_spatial_index_benchmarks` (Google Benchmark).
- Format with the repository `.clang-format` (LLVM base, 2-space indent, 120 columns, attached braces,
  left-aligned pointers): `clang-format -i <files>`.
- The repo's `build/` directory is ignored; do not commit build artifacts. On macOS,
  `xcrun clang-format` is available even when `clang-format` is not on `PATH`.

## Layout

```text
src/cyberverse/spatial_index.hpp              umbrella header
src/cyberverse/spatial_index/
  type_traits.hpp                             EntryHandleType, EntryBucketType, EntryIndexType, CellIndexType
  entry_query_scratch.hpp                     caller-owned handle dedup set
  linear_index.hpp / .inl                     sequential entry index
  bvh.hpp / .inl                              dynamic AABB tree entry index
  hash_index.hpp / .inl                       spatial hash entry index
  cell_entry_index.hpp / .inl                 cell index + per-cell entry buckets
  algorithms.hpp                              collect_visible
  sparse_uniform_grid.hpp / .inl              sparse cell grid
  dense_uniform_grid.hpp / .inl               dense cell grid
  detail/cell_range.hpp                       shared half-open cell range/center math
  detail/keyed_array.hpp                      small inline map used by sparse nodes
  detail/uniform_grid_node.hpp / .inl         node + dense/sparse child collections
  detail/uniform_grid_impl.hpp                shared grid implementation
  spatial_index.cpp                           translation unit including the umbrella header
```

Out-of-line template definitions live next to the public header as `<name>.inl`, **not** in an `impl/`
subdirectory. Public headers include their `.inl` at the bottom. Add new public headers and `.inl` files
to `HEADER_FILES` in `src/CMakeLists.txt` and to the umbrella header when appropriate.

## Cell grids

`sparse_uniform_grid` and `dense_uniform_grid` are thin, non-copyable/non-movable public wrappers around
`detail::uniform_grid_impl<T, Sparse>`. The wrapper owns a single `impl_` member and forwards every
method.

`detail::uniform_grid_impl<T, Sparse>` contains cell addressing, root growth, range set/erase, all
queries and statistics. The only storage difference is `uniform_grid_node<T, Sparse>`. Do not fork the
algorithm between sparse and dense; extend the shared implementation instead.

Cell coordinates are signed int64 and independent of tree depth. `cell_range` is half-open. `ensure_cell`
creates a path and value, growing the root if needed, and throws on overflow. `get_cell` and
`contains_cell` never grow the root; `get_cell` returns nullptr when no value exists, while
`contains_cell` reports whether the coordinate is inside the current root. `erase_cell` removes one value,
prunes ancestors, keeps the root and never shrinks it.

## Entry indexes

`entry_handle` is a caller-defined type satisfying `EntryHandleType`: copyable, default-initializable,
equality comparable, totally ordered and hashable through `std::hash`. The library does not interpret
handle bits, does not allocate handles and does not track generations. The caller guarantees unique
handles. A duplicate `insert` returns `false`.

`entry_value_type` defaults to `std::monostate`. A non-monostate value is stored inline per entry.
Filters and visitors always receive `(handle, value[, bounds])`. A `std::monostate` handle index pairs
with an external `handle -> value` map when the caller wants one value copy.

`linear_index` is a vector scan and is intended as a cell bucket. `bvh` is a dynamic binary AABB tree and
directly implements `EntryIndexType`; it must **not** implement `CellIndexType`, and there is no
`bvh<linear_index<...>>` composition. `hash_index` is a direct spatial-hash `EntryIndexType`.

`cell_entry_index<CellIndex>` requires `CellIndex::value_type` to satisfy `EntryBucketType` and be
default constructible. It stores a `record` map keyed by handle, inserts each entry into every overlapped
cell, and queries each bucket. It has no mutable query state: cross-cell deduplication uses a
caller-owned `entry_query_scratch<Handle>`. The no-scratch overloads create a local scratch and are safe
for parallel reads. Writes require external synchronization. `record` contains bounds/value/cell-count
only; do not add `visit_epoch` or other query-mutated fields.

An entry's value is stored in the record and in each covered bucket. This is the accepted copy cost; use
`std::monostate` plus an external map to avoid it. `set_value` updates the record and all covered
buckets.

All queries are const, visit each entry at most once, call the filter once after deduplication, and stop
when the visitor returns `false`. `collect_visible` is a generic free function over `EntryIndexType`.

## Tests

- `test/uniform_grid_test.cpp` covers sparse/dense grids and public cell APIs.
- `test/entry_index_test.cpp` covers concepts, linear/bvh/hash, values, scratch and concurrent reads.
- `test/cell_entry_index_test.cpp` covers sparse/dense composition, cross-cell dedup, filters, external
  values, `collect_visible`, early-out and concurrent reads.

## Conventions

- Public types/functions are `lower_snake_case`; private members end with `_`.
- Headers use `#pragma once` and are included as `"cyberverse/spatial_index/<name>.hpp"`.
- Prefer `std::size_t`, `std::uint32_t`, `std::int64_t`; include `<cstddef>`/`<cstdint>`.
- Keep concepts small and document requirement subsets. `EntryBucketType` must stay a subset of
  `EntryIndexType`.
- Do not add storage-specific statistics (`grid_num`, `memory_usage_in_bytes`) to the concepts.
- No commits or builds inside the repository; tests and benchmarks use `/tmp`.
