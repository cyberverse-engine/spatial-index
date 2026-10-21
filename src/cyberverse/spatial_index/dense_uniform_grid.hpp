/**
 * @file dense_uniform_grid.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Dense uniform grid
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/geometry/intersect.hpp"
#include "cyberverse/spatial_index/detail/uniform_grid_impl.hpp"
#include "cyberverse/spatial_index/type_traits.hpp"

namespace cyberverse {
namespace spatial_index {

/**
 * @brief Dense uniform grid
 *
 *  这是一个稠密的均匀网格空间索引实现。与 sparse_uniform_grid 使用完全相同的树结构、动态根节点扩展算法和查询
 *  算法，唯一的区别是节点使用连续的 vector 保存所有分支槽位，而不是稀疏的 keyed_array + unordered_map。
 *  稠密网格同样可以动态扩展，扩展策略与稀疏版本一致。
 *
 *  该类型满足 CellIndexType concept 的类型约束
 *
 *  NOTE: This structure is not thread-safe.
 *  NOTE: The constructor throws std::invalid_argument for an invalid unit, branch factor or origin. ensure() and the
 *        set(position, ...) overloads throw std::invalid_argument for a non-finite / unrepresentable position and
 *        std::overflow_error when the coordinate range can no longer be represented.
 *  NOTE: Inserting a cell outside the current tree may move the nodes; pointers and references previously returned by
 *        get() / ensure() are invalidated by such an insertion.
 *  NOTE: A query visitor must not modify the grid.
 *
 * @tparam T Value type
 */
template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
class dense_uniform_grid {
public:
  using value_type = T;
  using cell_coord = detail::cell_vec;

  /**
   * @brief Construct a dense uniform grid
   *
   * @param unit Unit cell size, every component must be finite and greater than zero
   * @param branch_factor Number of children per node on each axis, every component must be at least 2
   * @param origin World position of the lower corner of cell (0, 0, 0)
   */
  dense_uniform_grid(glm::vec3 unit, glm::ivec3 branch_factor, glm::vec3 origin = glm::vec3(0));

  dense_uniform_grid(const dense_uniform_grid&) = delete;
  dense_uniform_grid(dense_uniform_grid&&) = delete;

  ~dense_uniform_grid() = default;

  dense_uniform_grid& operator=(const dense_uniform_grid&) = delete;
  dense_uniform_grid& operator=(dense_uniform_grid&&) = delete;

  /**
   * @brief Get value of grid which containing pos
   *
   * @param pos Position
   * @return const value_type* Pointer to value, nullptr will be returned if not found
   */
  const value_type* get(const glm::vec3& pos) const noexcept;

  /**
   * @brief Get value of grid which containing pos
   *
   * @param pos Position
   * @return value_type* Pointer to value, nullptr will be returned if not found
   */
  value_type* get(const glm::vec3& pos) noexcept;

  /**
   * @brief Whether the grid containing pos owns a value
   *
   * @param pos Position
   * @return bool
   */
  bool contains(const glm::vec3& pos) const noexcept;

  /**
   * @brief Get value of grid which containing pos, will create new value by it's default constructor if not exists
   *
   * @param pos Position
   * @return T& Value reference
   */
  value_type& ensure(const glm::vec3& pos);

  /**
   * @brief Set the value of grid containing pos, will create new value or overwrite existing value
   *
   *  NOTE: The arguments must not reference a value owned by this grid; expanding the tree may invalidate such a
   *  reference. Use the `(pos, const value_type&)` overload to copy an existing value safely.
   *
   * @tparam Args Value constructor / assignment argument types
   * @param pos Position
   * @param args Arguments forwarded to the value constructor
   */
  template <typename... Args>
  void set(const glm::vec3& pos, Args&&... args);

  /**
   * @brief Set the value of grid containing pos, will create new value or overwrite existing value
   *
   *  The value is copied before the tree is expanded, so it may alias a value already owned by this grid.
   *
   * @param pos Position
   * @param value Value
   */
  void set(const glm::vec3& pos, const value_type& value);

  /**
   * @brief Set the grids in range to value. The value will be copied n times, will create new value or overwrite
   * existing value
   *
   *  Every unit cell overlapped by @p range is set. A range ending exactly on a cell boundary does not include the
   *  next cell.
   *
   * @param range Range
   * @param value Value
   */
  void set(const geometry::aabb& range, const value_type& value);

  /**
   * @brief Set the grids in range by a generator function, will create new value or overwrite existing value
   *
   *  The generator is invoked once per unit cell. It may either take no argument or take the unit cell box
   *  (`const geometry::aabb&`), and must return a value convertible to `value_type`.
   *
   * @tparam Fn Generator function type
   * @param range Range
   * @param generator Generator function
   */
  template <typename Fn>
    requires(std::is_invocable_r_v<value_type, std::remove_cvref_t<Fn>&> ||
             std::is_invocable_r_v<value_type, std::remove_cvref_t<Fn>&, const geometry::aabb&>)
  void set(const geometry::aabb& range, Fn&& generator);

  /**
   * @brief Erase the grid containing pos
   *
   * @param pos Position
   * @return bool, actually erased or not
   */
  bool erase(const glm::vec3& pos) noexcept;

  /**
   * @brief Erase the grids which are in range
   *
   * @param range Range
   * @return bool, actually erased at least one grid or not
   */
  bool erase(const geometry::aabb& range) noexcept;

  /**
   * @brief Clear, all resources will be released
   *
   */
  void clear() noexcept;

  /**
   * @brief Whether the grid owns no value
   *
   * @return bool
   */
  bool empty() const noexcept;

  /**
   * @brief Query by ray
   *
   *  Every unit cell whose box intersects the ray is visited. The visit function returns false to stop the query.
   *
   * @tparam Visitor Visit function, returns false to stop visiting
   * @param ry Ray
   * @param f Visit function
   */
  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, const value_type&, const geometry::aabb&>)
  void query(const geometry::ray& ry, Visitor&& f) const noexcept;

  /**
   * @brief Query by ray, values are mutable
   *
   * @tparam Visitor Visit function, returns false to stop visiting
   * @param ry Ray
   * @param f Visit function
   */
  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, value_type&, const geometry::aabb&>)
  void query(const geometry::ray& ry, Visitor&& f) noexcept;

  /**
   * @brief Query by range
   *
   *  Every unit cell overlapped by @p range is visited. The visit function returns false to stop the query.
   *
   * @tparam Visitor Visit function, returns false to stop visiting
   * @param range Range
   * @param f Visit function
   */
  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, const value_type&, const geometry::aabb&>)
  void query(const geometry::aabb& range, Visitor&& f) const noexcept;

  /**
   * @brief Query by range, values are mutable
   *
   * @tparam Visitor Visit function, returns false to stop visiting
   * @param range Range
   * @param f Visit function
   */
  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, value_type&, const geometry::aabb&>)
  void query(const geometry::aabb& range, Visitor&& f) noexcept;

  /**
   * @brief Query by frustum
   *
   *  Every unit cell whose box is not culled by the frustum is visited. The visit function returns false to stop the
   *  query.
   *
   * @tparam Visitor Visit function, returns false to stop visiting
   * @param fr Frustum
   * @param f Visit function
   */
  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, const value_type&, const geometry::aabb&>)
  void query(const geometry::frustum& fr, Visitor&& f) const noexcept;

  /**
   * @brief Query by frustum, values are mutable
   *
   * @tparam Visitor Visit function, returns false to stop visiting
   * @param fr Frustum
   * @param f Visit function
   */
  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, value_type&, const geometry::aabb&>)
  void query(const geometry::frustum& fr, Visitor&& f) noexcept;

  /**
   * @brief Get grid unit
   *
   * @return const glm::vec3&
   */
  const glm::vec3& unit() const noexcept;

  /**
   * @brief Get branch factor
   *
   * @return const glm::ivec3&
   */
  const glm::ivec3& branch_factor() const noexcept;

  /**
   * @brief Get origin
   *
   * @return const glm::vec3&
   */
  const glm::vec3& origin() const noexcept;

  /**
   * @brief Get the unit box object
   *
   *  NOTE: This method will return the aabb regardless whether the actual grid exists or not.
   *
   * @param pos Position
   * @return geometry::aabb, invalid when @p pos is non-finite or its cell coordinate is not representable
   */
  geometry::aabb get_unit_box(const glm::vec3& pos) const noexcept;

  /**
   * @brief Return number of unit grid which own a value
   *
   * @return size_t
   */
  size_t grid_num() const noexcept;

  /**
   * @brief Approximate memory footprint of the grid, its nodes and its values
   *
   *  The dense storage allocates every branch slot of a materialized internal node, so this is typically much larger
   *  than the sparse grid for the same contents.
   *
   * @return size_t
   */
  size_t memory_usage_in_bytes() const noexcept;

  /**
   * @brief Cell range overlapped by an AABB
   *
   *  Half-open cells; a range ending exactly on a cell boundary excludes the next cell.
   */
  bool cell_range(const geometry::aabb& range, cell_coord& lo, cell_coord& hi) const noexcept;

  const value_type* get_cell(const cell_coord& cell) const noexcept;

  value_type* get_cell(const cell_coord& cell) noexcept;

  value_type& ensure_cell(const cell_coord& cell);

  bool erase_cell(const cell_coord& cell) noexcept;

  bool contains_cell(const cell_coord& cell) const noexcept;

private:
  using impl_type = detail::uniform_grid_impl<value_type, false>;

  impl_type impl_;
};

static_assert(CellIndexType<dense_uniform_grid<int>>);

} // namespace spatial_index
} // namespace cyberverse

#include "cyberverse/spatial_index/dense_uniform_grid.inl"
