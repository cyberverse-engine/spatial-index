# 空间索引设计说明与 Roadmap

本文记录 `spatial-index` 的设计原则、关键取舍和后续路线。目标场景是游戏运行时的大量对象：

- 物品拾取/附近查询；
- 视锥裁剪，只渲染可见对象；
- 射线拾取；
- 10 万到 100 万级对象的动态索引；
- 查询可以多线程并行读，更新由外部同步。

## 1. 两层索引抽象

### 1.1 CellIndexType

`CellIndexType` 描述“子空间/cell → 一个用户值”的索引：

- cell 坐标是 signed int64；
- cell 是半开区间；
- 一个 cell 只保存一个 `value_type`；
- 提供 `cell_range`、`get_cell`、`ensure_cell`、`contains_cell`、`erase_cell`；
- 提供 aabb/ray/frustum 的 cell 级 query；
- 不要求 `value_type` 带 AABB。

当前实现：

- `sparse_uniform_grid<T>`：子节点先内联存储，子节点多时提升为 hash map；
- `dense_uniform_grid<T>`：所有 branch slot 连续分配。

两者共享 `detail::uniform_grid_impl<T, Sparse>`，只改变节点子集合的存储方式。不要为 sparse/dense
分叉查询和扩展算法。

### 1.2 EntryIndexType

`EntryIndexType` 描述“空间索引一个 Entry”：

- 每个 entry 有外部提供的 `entry_handle`；
- 每个 entry 有 AABB；
- 可选 `entry_value`，默认 `std::monostate`；
- 按 handle 插入、更新 bounds、删除、查询；
- query 是 const，访问每个 entry 至多一次；
- filter 在去重之后调用一次；
- visitor 返回 `false` 立即停止。

`EntryBucketType` 是 `EntryIndexType` 的**需求子集**：所有满足 `EntryIndexType` 的类型也满足
`EntryBucketType`。bucket 只要求局部的 insert/erase/set_value/query/clear/empty/size，不要求
全局 `update`、`contains`、`bounds`、`value`、`nearest`、`k_nearest`、`raycast`。

这样：

- 从 concept 角度，`linear_index`、`bvh`、`hash_index` 以及 `cell_entry_index` 都可以作为 bucket；
- 作为 `CellIndex::value_type` 还要求可默认构造；`linear_index` 和 `bvh` 直接满足，`hash_index` /
  `cell_entry_index` 带有配置参数，需要默认配置或 bucket factory；
- bucket 不需要成为特殊类型，`object_bucket` 不再存在；
- 需要更严格能力时使用完整的 `EntryIndexType`。

## 2. Handle 设计原则

`entry_handle` 不是库内固定类型，而是一个 concept：

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

原则：

1. **handle 由调用方构造和管理**。库不分配 handle，不维护 free list，不保存 generation。
2. **handle 唯一性由调用方保证**。重复 `insert` 返回 `false`，不覆盖。
3. **stale handle 由调用方负责**。如果调用方复用 handle，旧 handle 会指向新 entry。
4. **handle 位域语义由调用方决定**。库不假设高 32 位是 id、低 32 位是 tag。
5. **全序用于确定性 tie-break**。`k_nearest` 按 `(distance, handle)` 排序。
6. **hash 通过 `std::hash` 定制**。自定义 handle 必须提供特化。

这避免了之前讨论中的几个问题：

- 32 位 id + 32 位 tag 没有 generation，slot 复用会产生 stale alias；
- 如果每个嵌套索引都自动生成 handle，外层和每个 cell bucket 会得到不同 handle；
- 索引自动生成 handle 会让 `cell_entry_index` 必须维护 outer → inner 映射。

把 handle 的所有权交给调用方后，嵌套索引共享同一个 handle，`cell_entry_index` 不需要 handle 映射。

## 3. EntryValue 设计原则

`entry_value` 的定位是“非常轻量级的快速过滤字段”，不是对象负载：

- 默认 `std::monostate`，表示不保存额外用户数据；
- 非 monostate 时内联保存在 entry 中；
- filter 和 visitor 都能直接读取；
- 复制代价由调用方接受；
- 调用方也可以选择 monostate + 外部 `handle -> value` map。

在 `cell_entry_index` 中，entry 会插入多个 cell：

- `record` 保存一份 value；
- 每个覆盖 cell 的 bucket entry 也保存一份 value；
- 因此非 monostate 时 value 会按覆盖 cell 数复制；
- 这是有意的 CPU cache 取舍；如果不可接受，使用 monostate + 外部 map。

`entry_value` 不承担对象生命周期、类型擦除或大对象存储的职责。

## 4. cell_entry_index 设计

`cell_entry_index<CellIndex>` 把 cell 索引和 entry bucket 组合起来：

- `CellIndex::value_type` 必须是 `EntryBucketType` 且可默认构造；
- 每个 entry 按其 AABB 覆盖的所有 cell 插入；
- 每个 cell 的 bucket 保存 `(handle, bounds, value)`；
- `record` 按 handle 保存 `bounds`、`value`、`multi_cell`；
- 跨 cell 访问用 `entry_query_scratch<Handle>` 去重；
- `record` 不保存 `visit_epoch`，query 不修改索引；
- 所有 query 都是 const，多个线程可以同时读；
- 写操作需要外部同步。

选择“覆盖所有 cell”而不是“只插入中心 cell”的原因：

- 对象 AABB 可能跨 cell；
- 只按中心插入，查询只访问相邻 cell 时可能漏对象；
- 覆盖插入可以用 bucket 的真实 AABB 做 narrowphase，语义更直接；
- 小对象通常只覆盖一个 cell，成本接近 1；
- 大对象可以后续单独走 overflow/block 策略。

去重策略：

- 单 cell entry 不查 scratch；
- 多 cell entry 用 scratch.mark(handle)；
- 无 scratch 的重载创建局部 scratch，因此也是并行读安全的；
- 显式 scratch 只避免分配，不改变语义。

## 5. BVH 设计原则

`bvh` 直接实现 `EntryIndexType`：

- 不实现 `CellIndexType`；
- 不存在 `bvh<linear_index<...>>` 这种把 bucket 当 value 的用法；
- 叶子容量、SOO、树平衡/refit 属于 BVH 内部实现；
- 本次只实现基础动态 AABB 树，不做高级树质量优化。

原因：

- BVH 没有“位置 → bucket”的寻址能力，不能直接决定一个 handle 应该进哪个 linear_index leaf；
- 如果确实需要 block BVH，应显式提供 block 映射和 bucket 管理，而不是让普通 BVH 猜测；
- 对 100 万小对象，未来更合适的是 block/子空间 BVH + bucket，而不是每对象一个 leaf。

## 6. 并行读与线程模型

原则：

- 查询是 const，不修改索引；
- `cell_entry_index` 没有 mutable query epoch；
- 去重状态属于调用方/线程，不属于索引；
- scratch 不能跨线程共享；
- 写操作（insert/update/erase/set_value/clear）不是线程安全的；
- `value()`/`bounds()` 返回的指针在并发写期间不安全；
- 外部 `handle -> value` map 的线程安全由调用方负责。

这允许渲染线程、拾取线程等并行查询同一个索引，而更新由主线程/外部锁串行化。

## 7. 查询语义

所有 `EntryIndexType` 查询：

- 按 AABB/sphere/ray/frustum 做 broadphase；
- 每个 entry 每次查询至多访问一次；
- filter 在去重后调用一次；
- visitor 返回 false 立即停止；
- `nearest`/`k_nearest` 使用点到 AABB 的距离；
- `k_nearest` 按 `(distance, handle)` 确定性排序；
- `raycast` 返回最近 AABB 命中。

`cell_entry_index` 的 query 使用 cell 索引做粗筛，再查 bucket；`nearest`/`k_nearest` 对有限距离使用
cell 索引的 sphere 查询，无限距离退化为 record 扫描；`raycast` 使用 cell 索引的 ray 查询。

## 8. 性能取舍

当前实现的关键成本：

- `cell_entry_index` 的 record 表是 `std::unordered_map<handle, record>`；
- 非 monostate value 会按覆盖 cell 数复制；
- inner bucket 默认 `linear_index`，cell 内对象多时扫描成本上升；
- `hash_index` 的 ray/frustum 查询当前遍历所有 cell；
- `bvh` 是基础动态树，没有 bulk load、SOO、rotation；
- `k_nearest` 在部分路径会收集候选后排序，而不是 best-first；
- `nearest` 无限距离会扫描 record；
- 大对象跨很多 cell 时，覆盖插入成本高。

这些是有意的阶段性取舍，后续 Roadmap 处理。

## 9. Roadmap

### 近期（正确性与易用性）

1. **随机化/属性测试**
   - cell_entry 与 brute force 对拍 query/nearest/raycast；
   - bvh/hash 与 brute force 对拍；
   - 覆盖跨 cell、更新、删除、空索引、退化 AABB。

2. **TSAN 并行读测试**
   - 多线程 query 同一索引；
   - 每线程独立 scratch；
   - 确认无隐藏 mutable 状态。

3. **API 文档**
   - 每个 concept 的精确要求；
   - `EntryBucketType` 子集关系；
   - scratch 生命周期和线程约束；
   - 错误处理（重复 handle、无效 bounds、空 bucket）。

4. **查询 early-out 审计**
   - 所有 cell/bucket 路径都正确传播 visitor false；
   - 确保 filter 不会被重复调用。

### 中期（性能）

5. **cell_entry 单 cell 快路径**
   - 已记录 `multi_cell`；
   - 继续减少单 cell 查询的 scratch/间接访问；
   - 评估 `record` 是否可用更紧凑的 slot 表。

6. **bucket 策略**
   - cell 内对象数超过阈值时，将 `linear_index` 切换为 `bvh`/分块；
   - 或提供 `cell_entry_index` 的 bucket 选择策略；
   - benchmark cell size × bucket type。

7. **block/子空间 BVH**
   - 对 100 万对象，粗层 BVH over blocks + 每 block bucket；
   - 需要显式 block 映射，不能复用普通 `bvh<linear_index<...>>`；
   - 目标是降低 leaf 数、改善 frustum/ray 剪枝。

8. **hash_index 粗层剪枝**
   - ray/frustum 不再遍历所有 cell；
   - 引入 block/BVH 或分层 hash；
   - 保持直接 `EntryIndexType` 语义。

9. **BVH 内部优化**
   - leaf capacity/SOO；
   - SAH 插入、rotation、refit/rebuild；
   - bulk load；
   - 这些不改变 `EntryIndexType` 接口。

10. **nearest/k_nearest/raycast**
    - best-first 遍历；
    - 避免全候选排序；
    - 支持有限半径的早停；
    - cell_entry 无限距离查询优化。

11. **内存**
    - record 表 reserve/arena；
    - bucket 预留；
    - 1M 对象 benchmark：RSS、insert、update、query、frustum、nearest；
    - 评估 monostate + 外部 map 与内联 value 的曲线。

### 长期（能力）

12. **多级/混合索引**
    - cell_entry、block BVH、hash 作为可组合 backend；
    - 统一 `EntryIndexType`；
    - 不引入新的特殊 bucket 类型。

13. **并发更新**
    - 当前只支持并行读；
    - 如需要并发写，评估锁分片、snapshot/RCU 或 per-thread index；
    - 不为了并发写牺牲查询语义。

14. **持久化/序列化**
    - 仅在有明确需求时考虑；
    - handle 与对象生命周期仍由调用方管理。

## 10. 非目标

- 不管理 handle 分配、复用、generation、tag 位域；
- 不保存对象负载/生命周期；
- 不实现并发写；
- 不在本次重构中实现 BVH 高级树质量优化；
- 不把 `bvh` 变成 `CellIndexType`；
- 不恢复 `object_bucket` 特殊类型。

## 11. 相关文件

- `src/cyberverse/spatial_index/type_traits.hpp`
- `src/cyberverse/spatial_index/entry_query_scratch.hpp`
- `src/cyberverse/spatial_index/linear_index.hpp`
- `src/cyberverse/spatial_index/bvh.hpp`
- `src/cyberverse/spatial_index/hash_index.hpp`
- `src/cyberverse/spatial_index/cell_entry_index.hpp`
- `src/cyberverse/spatial_index/algorithms.hpp`
- `test/entry_index_test.cpp`
- `test/cell_entry_index_test.cpp`
- `benchmark/entry_index_benchmark.cpp`
