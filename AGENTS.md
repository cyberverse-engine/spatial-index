# AGENTS.md

`spatial-index` is the standalone spatial index module of Cyberverse Engine: C++23, GLM, built as the static
library `cyberverse::spatial_index::static_lib`. It depends on `cyberverse::geometry::static_lib`. The public API
lives in `src/cyberverse/spatial_index/`; `src/cyberverse/spatial_index.hpp` is the umbrella header, and
`src/cyberverse/spatial_index/spatial_index.cpp` is only a translation unit that includes the umbrella header.

The public API is header-only. The current implementation provides two uniform grid variants:

- `spatial_index::sparse_uniform_grid<T>` — children are kept in an inline `keyed_array` and promoted to an
  `unordered_map` when a node has many children.
- `spatial_index::dense_uniform_grid<T>` — every branch slot of a materialized node is allocated contiguously.

Both variants share the tree layout, dynamic root expansion and all query algorithms. Spatial hashes and BVHs are
planned; a fine-grained local block index is not implemented yet.

## Build and test

Run from this directory. Build and run tests under `/tmp`, never inside the repository.

```bash
cmake -S . -B /tmp/cyberverse-spatial-index-test
cmake --build /tmp/cyberverse-spatial-index-test
ctest --test-dir /tmp/cyberverse-spatial-index-test --output-on-failure
```

- The first configure resolves `geometry` and fetches GLM through it, plus Catch2 v3.5.2 and Google Benchmark
  v1.9.5. Skip the latter two with
  `-DCYBERVERSE_SPATIAL_INDEX_BUILD_TESTS=OFF -DCYBERVERSE_SPATIAL_INDEX_BUILD_BENCHMARKS=OFF`.
- `cmake/engine_dep_geometry.cmake` resolves `cyberverse::geometry::static_lib` idempotently, in order: an existing
  target, an installed `cyberverse_geometry` package, `CYBERVERSE_GEOMETRY_DIR` (CMake or environment), the sibling
  `../geometry`, or `FetchContent`. When geometry is vendored, its tests and benchmarks are forced off.
- Presets: `cmake --preset debug` / `cmake --preset release`, then `cmake --build --preset debug|release`; outputs go
  to `build/` and `build-release/`. Tests still must be built and run under `/tmp`.
- `test/` builds `cyberverse_spatial_index_tests` (Catch2, discovered by CTest); `benchmark/` builds
  `cyberverse_spatial_index_benchmarks` (Google Benchmark).
- Format with the repository `.clang-format` (LLVM base, 2-space indent, 120 columns, attached braces, left-aligned
  pointers): `clang-format -i <files>`.
- The repo's `build/` directory is ignored; do not commit build artifacts. On macOS, `xcrun clang-format` is
  available even when `clang-format` is not on `PATH`.

## Layout

```
src/cyberverse/spatial_index.hpp              umbrella header
src/cyberverse/spatial_index/
  sparse_uniform_grid.hpp / .inl              public sparse grid + out-of-line forwarding
  dense_uniform_grid.hpp  / .inl              public dense grid + out-of-line forwarding
  type_traits.hpp                             PartitionIndexType concept
  detail/keyed_array.hpp                      small inline map used by sparse nodes
  detail/uniform_grid_node.hpp / .inl         node + dense/sparse child collections
  detail/uniform_grid_impl.hpp                shared grid implementation
  spatial_index.cpp                           translation unit including the umbrella header
```

Out-of-line template definitions live next to the public header as `<name>.inl`, **not** in an `impl/`
subdirectory. Public headers include their `.inl` at the bottom. Add new public headers and `.inl` files to
`HEADER_FILES` in `src/CMakeLists.txt` and to the umbrella header when appropriate.

## Architecture

### Public grids

`sparse_uniform_grid` and `dense_uniform_grid` are thin, non-copyable/non-movable public wrappers around
`detail::uniform_grid_impl<T, Sparse>`. The wrapper owns a single `impl_` member and forwards every method. The two
public APIs are intentionally identical; keep them that way.

`detail::uniform_grid_impl<T, Sparse>` in `detail/uniform_grid_impl.hpp` contains the entire algorithm: cell
addressing, root growth, range set/erase, all queries and statistics. The only storage difference is
`uniform_grid_node<T, Sparse>`. Do not fork the algorithm between sparse and dense; extend the shared implementation
instead.

`grid_num()` returns the number of occupied unit cells. `memory_usage_in_bytes()` accounts for the structure in
detail for the dense storage and is approximate for the sparse hash-map storage. Both are stats APIs and are
deliberately not part of `PartitionIndexType`.

### Nodes and child collections

- A node represents a sub-box. A **leaf node is exactly one unit cell** and may own a value; an internal node owns
  children and never owns a value. Internal nodes whose value has been erased are pruned, so materialized leaves
  normally have values.
- `box_.is_valid()` marks whether a node is materialized. A lazy child collection is held by
  `std::unique_ptr<collection_type>`; this breaks the recursive type dependency and lets an empty leaf avoid paying
  for a child array.
- The node is move-only. Its move operations and destructor are defined out-of-line in `uniform_grid_node.inl` after
  both collection types are complete.
- `uniform_grid_node_collection<T, false>` (the primary template, dense) stores `std::vector<node_type>` sized once
  to `branch_factor` product; unused slots are non-existing default nodes.
- `uniform_grid_node_collection<T, true>` (sparse) stores the first entries inline in
  `keyed_array<size_t, node_type>` (capacity 8), then promotes all entries into `std::unordered_map<size_t,
  node_type>` with `keyed_array::drain()`.

- `uniform_grid_node::ensure_child()` and `insert_child()` allocate the collection lazily and mark the node
  internal. `erase_child()` resets the collection pointer when it becomes empty.
- Keep `keyed_array`'s small inline capacity unless a benchmark justifies changing it; its `find` is a linear scan,
  which is deliberate for small child counts.

### Cell addressing and root growth

- `unit` is per-axis cell size, `branch_factor` is per-axis children per node (each component >= 2), and `origin`
  is the world position of the lower corner of cell `(0, 0, 0)`.
- Cell coordinates are signed `int64_t`, stored internally as `glm::vec<3, std::int64_t>` (`cell_vec`). A cell is
  half-open: cell `c` covers `[origin + c * unit, origin + (c + 1) * unit)`.
- `to_cell()` converts a world position with double-precision division and `floor`, then bounds-checks before
  converting to `int64_t`. Non-finite or unrepresentable positions are rejected.
- The root starts as the single unit cell at `(0, 0, 0)`: `base_cell_ = 0`, `span_ = 1`. `span_` is the number of
  cells covered by the root on each axis and is always `branch_factor^depth` component-wise.
- `contains_cell()` relies on the invariant `base_cell_ + span_` fitting in `int64_t`. Keep the overflow checks in
  `expand_root()` when changing the growth math.
- Root growth is **directional**. For a target outside the root, the old root is inserted into the new root at a
  child index chosen per axis:
  - target below the old range: child `branch_factor - 1`;
  - target at/above the old upper bound: child `0`;
  - target inside that axis: child `(branch_factor - 1) / 2`.

  Then `new_span = old_span * branch_factor` and `new_base = old_base - child_index * old_span`. This is required
  for correctness: with `branch_factor == 2`, a fixed child index can only grow the root in one direction, so the
  grid would never reach cells on the opposite side. Do not replace this with a fixed "old root at child 1" rule.
- A new root box must be representable as `geometry::aabb`; root growth throws `std::overflow_error` when the
  integer range or the float box can no longer be represented.
- Constructor validation throws `std::invalid_argument` for non-finite/zero/negative unit, branch factor < 2, a
  non-finite origin, an overflowing branch-factor product, or a non-finite initial cell box.

### Range and query semantics

- `set(aabb, ...)`, `erase(aabb)` and `query(aabb, ...)` select every unit cell **overlapped** by the range. A range
  ending exactly on a cell boundary excludes the cell starting at that boundary; a degenerate range (`min == max`)
  still selects its containing cell. All three operations go through the shared `cell_range()`.
- `set(range, value)` copies `value` once before expanding the tree; it is safe to pass a value owned by the grid.
  The generator overload is invoked once per selected cell, either with no arguments or with the unit cell
  `const geometry::aabb&`, and must return a value convertible to `value_type`.
- `set(pos, args...)` forwards arguments to `value_type`; those arguments must not alias a value owned by this grid
  because expansion may move nodes. `set(pos, const value_type&)` exists for the alias-safe copy case.
- An insertion that grows the tree can move nodes and invalidates pointers/references previously returned by
  `get()`/`ensure()`.
- `query` visits every materialized cell whose box intersects the query shape. The visitor returns `false` to stop.
  Ray/frustum/aabb traversal order is not guaranteed to be distance-sorted. Do not modify the grid from a visitor.
- `get_unit_box(pos)` returns the cell box whether or not the cell exists; it returns an invalid box for a
  non-finite/unrepresentable position.
- `erase` prunes empty leaves and empty ancestors. The root itself is never removed; it may become an empty internal
  node.
- The structure is not thread-safe.

## PartitionIndexType

`PartitionIndexType` in `type_traits.hpp` is the common coarse-index interface. It currently requires:

- `value_type`;
- `get(pos)` const/non-const, `contains(pos)`;
- `ensure(pos)`, `set(pos, value_type)`, `set(aabb, const value_type&)`;
- `erase(pos)`, `erase(aabb)`, `clear()`, `empty()`;
- `query(ray/aabb/frustum, visitor)` const/non-const.

It deliberately does **not** require storage-specific statistics (`grid_num()`, `memory_usage_in_bytes()`) or the
configuration/box accessors (`unit()`, `branch_factor()`, `origin()`, `get_unit_box()`). Those remain available on
the uniform grids as extra API. Keep the concept focused on the interface every partition index must provide, and
add a `static_assert(PartitionIndexType<...>)` for new index types.

## Style and conventions

- Namespace `cyberverse::spatial_index`; internals in `cyberverse::spatial_index::detail`.
- `.clang-format` is authoritative: LLVM base, 2-space indent, 120 columns, attached braces, left-aligned pointers.
- Public types/functions are `lower_snake_case`; private members end with `_`.
- Use `#pragma once`, include public headers as `"cyberverse/spatial_index/<name>.hpp"`, and keep the Doxygen
  `@file` header at the top of every new file.
- `src/` is the include root. Add public headers and `.inl` files to `HEADER_FILES`; add `.cpp` files to
  `SOURCE_FILES`. Include new public headers from `spatial_index.hpp` when they are part of the umbrella API.
- Prefer `noexcept` only when the operation really cannot throw; visitors and value constructors may throw, so do
  not blanket-mark new APIs `noexcept`.
- GLM integer vector arithmetic requires matching component types. `glm::vec<3, std::int64_t>` does not mix with
  `glm::ivec3`; the grid keeps a converted `branch_cell_` for cell math. Keep new cell arithmetic in `cell_vec` and
  branch count in `size_t`.

## Tests

- Catch2 v3; `test/uniform_grid_test.cpp` covers both grids with `TEMPLATE_TEST_CASE`.
- Cover at least: positive and negative root growth, `branch_factor == 2` (the directional-expansion case),
  half-open range boundaries and degenerate ranges, generators, ray/aabb/frustum queries, early-out visitors,
  sparse inline-array promotion (> 8 children, e.g. branch factor 4), erase pruning, `clear`, invalid
  configurations, and const/non-const query overloads.
- Build and run tests under `/tmp`. For changes to traversal, expansion or storage, also run an ASan/UBSan build and
  a randomized model comparison (a `std::map` of cell indices is sufficient) before considering the change done.

## Git

- Existing repo remotes (`origin`) use the local SSH alias `github-simon2077s.com`; do not rewrite them. The alias
  is not available on other machines or CI.
- Build-time fetches must use the real public host `git@github.com:<org>/<repo>.git`, never the local alias.
- Commit directly on `main` with Conventional Commits (`feat:`, `fix:`, `docs:`, `chore:`, `test:`).
