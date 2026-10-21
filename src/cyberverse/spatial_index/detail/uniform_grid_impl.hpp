/**
 * @file uniform_grid_impl.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Shared implementation of the sparse / dense uniform grid
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <glm/common.hpp>
#include <glm/vec3.hpp>

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/geometry/intersect.hpp"
#include "cyberverse/spatial_index/detail/cell_range.hpp"
#include "cyberverse/spatial_index/detail/uniform_grid_node.hpp"

namespace cyberverse {
namespace spatial_index {
namespace detail {

/**
 * @brief Shared uniform grid implementation
 *
 *  The only difference between the sparse and the dense grid is the node collection type selected by @p Sparse. The
 *  tree layout, the dynamic root expansion, the cell addressing and every query algorithm are shared here.
 *
 * @tparam T Value type
 * @tparam Sparse Sparse or not
 */
template <typename T, bool Sparse>
class uniform_grid_impl {
public:
  using value_type = T;
  using node_type = uniform_grid_node<T, Sparse>;

  // Signed per-axis cell coordinate. A cell coordinate is independent from the tree depth.
  using cell_vec = glm::vec<3, std::int64_t>;
  using cell_coord = cell_vec;

  uniform_grid_impl(glm::vec3 unit, glm::ivec3 branch_factor, glm::vec3 origin);

  uniform_grid_impl(const uniform_grid_impl&) = delete;
  uniform_grid_impl(uniform_grid_impl&&) = delete;

  ~uniform_grid_impl() = default;

  uniform_grid_impl& operator=(const uniform_grid_impl&) = delete;
  uniform_grid_impl& operator=(uniform_grid_impl&&) = delete;

  const value_type* get(const glm::vec3& pos) const noexcept;

  value_type* get(const glm::vec3& pos) noexcept;

  value_type& ensure(const glm::vec3& pos);

  template <typename... Args>
  void set(const glm::vec3& pos, Args&&... args);

  void set(const glm::vec3& pos, const value_type& value);

  void set(const geometry::aabb& range, const value_type& value);

  template <typename Fn>
    requires(std::is_invocable_r_v<value_type, std::remove_cvref_t<Fn>&> ||
             std::is_invocable_r_v<value_type, std::remove_cvref_t<Fn>&, const geometry::aabb&>)
  void set(const geometry::aabb& range, Fn&& generator);

  bool erase(const glm::vec3& pos) noexcept;

  bool erase(const geometry::aabb& range) noexcept;

  void clear() noexcept;

  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, const value_type&, const geometry::aabb&>)
  void query(const geometry::ray& ry, Visitor&& f) const noexcept;

  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, value_type&, const geometry::aabb&>)
  void query(const geometry::ray& ry, Visitor&& f) noexcept;

  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, const value_type&, const geometry::aabb&>)
  void query(const geometry::aabb& range, Visitor&& f) const noexcept;

  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, value_type&, const geometry::aabb&>)
  void query(const geometry::aabb& range, Visitor&& f) noexcept;

  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, const value_type&, const geometry::aabb&>)
  void query(const geometry::frustum& fr, Visitor&& f) const noexcept;

  template <typename Visitor>
    requires(std::is_invocable_r_v<bool, std::remove_cvref_t<Visitor>, value_type&, const geometry::aabb&>)
  void query(const geometry::frustum& fr, Visitor&& f) noexcept;

  const glm::vec3& unit() const noexcept { return unit_; }

  const glm::ivec3& branch_factor() const noexcept { return branch_factor_; }

  const glm::vec3& origin() const noexcept { return origin_; }

  geometry::aabb get_unit_box(const glm::vec3& pos) const noexcept;

  size_t grid_num() const noexcept { return grid_num_; }

  size_t memory_usage_in_bytes() const noexcept { return sizeof(*this) + root_.owned_memory_usage_in_bytes(); }

  bool cell_range(const geometry::aabb& range, cell_coord& lo, cell_coord& hi) const noexcept;

  const value_type* get_cell(const cell_coord& cell) const noexcept;

  value_type* get_cell(const cell_coord& cell) noexcept;

  value_type& ensure_cell(const cell_coord& cell);

  bool erase_cell(const cell_coord& cell) noexcept;

  bool contains_cell(const cell_coord& cell) const noexcept;

private:
  static bool is_finite(const glm::vec3& value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
  }

  static glm::vec3 checked_unit(glm::vec3 unit) {
    if (!is_finite(unit) || unit.x <= 0.0f || unit.y <= 0.0f || unit.z <= 0.0f) {
      throw std::invalid_argument("uniform grid unit must be finite and greater than zero");
    }
    return unit;
  }

  static glm::vec3 checked_origin(glm::vec3 origin) {
    if (!is_finite(origin)) {
      throw std::invalid_argument("uniform grid origin must be finite");
    }
    return origin;
  }

  static glm::ivec3 checked_branch_factor(glm::ivec3 branch_factor) {
    if (branch_factor.x < 2 || branch_factor.y < 2 || branch_factor.z < 2) {
      throw std::invalid_argument("uniform grid branch factor must be at least 2 on every axis");
    }
    return branch_factor;
  }

  static size_t checked_branch_count(const glm::ivec3& branch_factor) {
    size_t count = 1;
    const size_t max = std::numeric_limits<size_t>::max();
    for (int axis = 0; axis < 3; ++axis) {
      const size_t factor = static_cast<size_t>(branch_factor[axis]);
      if (count > max / factor) {
        throw std::invalid_argument("uniform grid branch factor product overflows size_t");
      }
      count *= factor;
    }
    return count;
  }

  static glm::dvec3 to_dvec3(const glm::vec3& value) noexcept {
    return glm::dvec3(static_cast<double>(value.x), static_cast<double>(value.y), static_cast<double>(value.z));
  }

  static glm::dvec3 to_dvec3(const cell_vec& value) noexcept {
    return glm::dvec3(static_cast<double>(value.x), static_cast<double>(value.y), static_cast<double>(value.z));
  }

  static bool double_to_cell(double value, std::int64_t& out) noexcept {
    // [-2^63, 2^63) is exactly representable as a lower / exclusive upper bound in double.
    constexpr double lower = -9223372036854775808.0;
    constexpr double upper = 9223372036854775808.0;
    if (!(value >= lower && value < upper)) {
      return false;
    }
    out = static_cast<std::int64_t>(value);
    return true;
  }

  static bool is_unit_span(const cell_vec& span) noexcept { return span.x == 1 && span.y == 1 && span.z == 1; }

  static geometry::aabb
  make_cell_box(const glm::vec3& origin, const glm::vec3& unit, const cell_vec& cell, const cell_vec& span) noexcept {
    return detail::cell_box(origin, unit, cell, span);
  }

  geometry::aabb cell_box(const cell_vec& cell, const cell_vec& span) const noexcept {
    return make_cell_box(origin_, unit_, cell, span);
  }

  bool to_cell(const glm::vec3& pos, cell_vec& cell) const noexcept {
    if (!is_finite(pos)) {
      return false;
    }

    const glm::dvec3 relative = (to_dvec3(pos) - to_dvec3(origin_)) / to_dvec3(unit_);
    const glm::dvec3 floored = glm::floor(relative);
    return double_to_cell(floored.x, cell.x) && double_to_cell(floored.y, cell.y) && double_to_cell(floored.z, cell.z);
  }

  size_t flat_index(std::int64_t x, std::int64_t y, std::int64_t z) const noexcept {
    const size_t bx = static_cast<size_t>(branch_factor_.x);
    const size_t by = static_cast<size_t>(branch_factor_.y);
    return static_cast<size_t>(x) + bx * (static_cast<size_t>(y) + by * static_cast<size_t>(z));
  }

  size_t flat_index(const cell_vec& index) const noexcept { return flat_index(index.x, index.y, index.z); }

  bool clip_to_root(cell_vec& lo, cell_vec& hi) const noexcept {
    const cell_vec root_hi = base_cell_ + span_ - cell_vec(1);
    lo = glm::max(lo, base_cell_);
    hi = glm::min(hi, root_hi);
    return lo.x <= hi.x && lo.y <= hi.y && lo.z <= hi.z;
  }

  static std::int64_t floor_div(std::int64_t value, std::int64_t divisor) noexcept {
    std::int64_t quotient = value / divisor;
    if (value % divisor < 0) {
      --quotient;
    }
    return quotient;
  }

  static void child_index_range(std::int64_t lo,
                                std::int64_t hi,
                                std::int64_t node_cell,
                                std::int64_t child_span,
                                std::int64_t branch_factor,
                                std::int64_t& first,
                                std::int64_t& last) noexcept {
    first = std::clamp<std::int64_t>(floor_div(lo - node_cell, child_span), 0, branch_factor - 1);
    last = std::clamp<std::int64_t>(floor_div(hi - node_cell, child_span), 0, branch_factor - 1);
  }

  void expand_root(const cell_vec& target) {
    const cell_vec old_span = span_;
    const cell_vec old_base = base_cell_;
    const std::int64_t max = std::numeric_limits<std::int64_t>::max();
    const std::int64_t min = std::numeric_limits<std::int64_t>::min();

    cell_vec new_span{};
    for (int axis = 0; axis < 3; ++axis) {
      if (old_span[axis] > max / static_cast<std::int64_t>(branch_factor_[axis])) {
        throw std::overflow_error("uniform grid cell span overflows int64_t");
      }
      new_span[axis] = old_span[axis] * static_cast<std::int64_t>(branch_factor_[axis]);
    }

    // Put the old root into the child that grows the root towards the target on every axis. With a branch factor of
    // two the old root has to sit either at the first or the last child, so the expansion direction depends on where
    // the target lies; a fixed child index would never grow towards the opposite side.
    cell_vec child_index{};
    for (int axis = 0; axis < 3; ++axis) {
      const std::int64_t old_upper = old_base[axis] + old_span[axis];
      if (target[axis] < old_base[axis]) {
        child_index[axis] = static_cast<std::int64_t>(branch_factor_[axis]) - 1;
      } else if (target[axis] >= old_upper) {
        child_index[axis] = 0;
      } else {
        child_index[axis] = static_cast<std::int64_t>((branch_factor_[axis] - 1) / 2);
      }
    }

    cell_vec new_base{};
    for (int axis = 0; axis < 3; ++axis) {
      const std::int64_t offset = child_index[axis] * old_span[axis];
      if (old_base[axis] < min + offset) {
        throw std::overflow_error("uniform grid root origin underflows int64_t");
      }
      new_base[axis] = old_base[axis] - offset;
    }

    for (int axis = 0; axis < 3; ++axis) {
      if (new_base[axis] > max - new_span[axis]) {
        throw std::overflow_error("uniform grid root upper bound overflows int64_t");
      }
    }

    const geometry::aabb new_box = make_cell_box(origin_, unit_, new_base, new_span);
    if (!new_box.is_valid()) {
      throw std::overflow_error("uniform grid root box is not representable");
    }

    node_type new_root(new_box, false, branch_count_);
    new_root.insert_child(flat_index(child_index), std::move(root_));
    root_ = std::move(new_root);
    base_cell_ = new_base;
    span_ = new_span;
  }

  void ensure_root_contains(const cell_vec& cell) {
    while (!contains_cell(cell)) {
      expand_root(cell);
    }
  }

  node_type* descend(const cell_vec& cell) noexcept {
    cell_vec span = span_;
    cell_vec relative = cell - base_cell_;
    node_type* node = &root_;
    while (!is_unit_span(span)) {
      const cell_vec child_span = span / branch_cell_;
      const cell_vec child_index = relative / child_span;
      node = node->child(flat_index(child_index));
      if (!node) {
        return nullptr;
      }
      relative = relative % child_span;
      span = child_span;
    }
    return node;
  }

  const node_type* descend(const cell_vec& cell) const noexcept {
    cell_vec span = span_;
    cell_vec relative = cell - base_cell_;
    const node_type* node = &root_;
    while (!is_unit_span(span)) {
      const cell_vec child_span = span / branch_cell_;
      const cell_vec child_index = relative / child_span;
      node = node->child(flat_index(child_index));
      if (!node) {
        return nullptr;
      }
      relative = relative % child_span;
      span = child_span;
    }
    return node;
  }

  node_type& ensure_leaf(const cell_vec& cell) {
    cell_vec node_cell = base_cell_;
    cell_vec node_span = span_;
    cell_vec relative = cell - base_cell_;
    node_type* node = &root_;

    while (!is_unit_span(node_span)) {
      const cell_vec child_span = node_span / branch_cell_;
      const cell_vec child_index = relative / child_span;
      const cell_vec child_cell = node_cell + child_index * child_span;
      node = &node->ensure_child(flat_index(child_index), cell_box(child_cell, child_span), is_unit_span(child_span));
      node_cell = child_cell;
      relative = relative % child_span;
      node_span = child_span;
    }
    return *node;
  }

  void assign_value(node_type& node, const value_type& value) {
    const bool existed = node.has_value();
    node.set_value(value);
    if (!existed) {
      ++grid_num_;
    }
  }

  template <typename Fn>
  void assign_generated_value(node_type& node, const cell_vec& cell, const cell_vec& span, Fn& generator) {
    const bool existed = node.has_value();
    if constexpr (std::is_invocable_r_v<value_type, Fn&>) {
      node.set_value(std::invoke(generator));
    } else {
      node.set_value(std::invoke(generator, cell_box(cell, span)));
    }
    if (!existed) {
      ++grid_num_;
    }
  }

  void set_range_node(node_type& node,
                      const cell_vec& node_cell,
                      const cell_vec& node_span,
                      const cell_vec& lo,
                      const cell_vec& hi,
                      const value_type& value) {
    if (is_unit_span(node_span)) {
      assign_value(node, value);
      return;
    }

    const cell_vec child_span = node_span / branch_cell_;
    std::int64_t x0 = 0;
    std::int64_t x1 = 0;
    std::int64_t y0 = 0;
    std::int64_t y1 = 0;
    std::int64_t z0 = 0;
    std::int64_t z1 = 0;
    child_index_range(lo.x, hi.x, node_cell.x, child_span.x, branch_factor_.x, x0, x1);
    child_index_range(lo.y, hi.y, node_cell.y, child_span.y, branch_factor_.y, y0, y1);
    child_index_range(lo.z, hi.z, node_cell.z, child_span.z, branch_factor_.z, z0, z1);

    for (std::int64_t z = z0; z <= z1; ++z) {
      for (std::int64_t y = y0; y <= y1; ++y) {
        for (std::int64_t x = x0; x <= x1; ++x) {
          const cell_vec child_cell(
              node_cell.x + x * child_span.x, node_cell.y + y * child_span.y, node_cell.z + z * child_span.z);
          node_type& child =
              node.ensure_child(flat_index(x, y, z), cell_box(child_cell, child_span), is_unit_span(child_span));
          set_range_node(child, child_cell, child_span, lo, hi, value);
        }
      }
    }
  }

  template <typename Fn>
  void set_range_generator_node(node_type& node,
                                const cell_vec& node_cell,
                                const cell_vec& node_span,
                                const cell_vec& lo,
                                const cell_vec& hi,
                                Fn& generator) {
    if (is_unit_span(node_span)) {
      assign_generated_value(node, node_cell, node_span, generator);
      return;
    }

    const cell_vec child_span = node_span / branch_cell_;
    std::int64_t x0 = 0;
    std::int64_t x1 = 0;
    std::int64_t y0 = 0;
    std::int64_t y1 = 0;
    std::int64_t z0 = 0;
    std::int64_t z1 = 0;
    child_index_range(lo.x, hi.x, node_cell.x, child_span.x, branch_factor_.x, x0, x1);
    child_index_range(lo.y, hi.y, node_cell.y, child_span.y, branch_factor_.y, y0, y1);
    child_index_range(lo.z, hi.z, node_cell.z, child_span.z, branch_factor_.z, z0, z1);

    for (std::int64_t z = z0; z <= z1; ++z) {
      for (std::int64_t y = y0; y <= y1; ++y) {
        for (std::int64_t x = x0; x <= x1; ++x) {
          const cell_vec child_cell(
              node_cell.x + x * child_span.x, node_cell.y + y * child_span.y, node_cell.z + z * child_span.z);
          node_type& child =
              node.ensure_child(flat_index(x, y, z), cell_box(child_cell, child_span), is_unit_span(child_span));
          set_range_generator_node(child, child_cell, child_span, lo, hi, generator);
        }
      }
    }
  }

  bool
  erase_cell_node(node_type& node, const cell_vec& node_cell, const cell_vec& node_span, const cell_vec& relative) {
    if (is_unit_span(node_span)) {
      if (!node.erase_value()) {
        return false;
      }
      --grid_num_;
      return true;
    }

    const cell_vec child_span = node_span / branch_cell_;
    const cell_vec child_index = relative / child_span;
    const size_t index = flat_index(child_index);
    node_type* child = node.child(index);
    if (!child) {
      return false;
    }

    const cell_vec child_cell = node_cell + child_index * child_span;
    const bool erased = erase_cell_node(*child, child_cell, child_span, relative % child_span);
    if (erased && child->empty()) {
      node.erase_child(index);
    }
    return erased;
  }

  bool erase_range_node(
      node_type& node, const cell_vec& node_cell, const cell_vec& node_span, const cell_vec& lo, const cell_vec& hi) {
    if (is_unit_span(node_span)) {
      if (!node.erase_value()) {
        return false;
      }
      --grid_num_;
      return true;
    }

    const cell_vec child_span = node_span / branch_cell_;
    std::int64_t x0 = 0;
    std::int64_t x1 = 0;
    std::int64_t y0 = 0;
    std::int64_t y1 = 0;
    std::int64_t z0 = 0;
    std::int64_t z1 = 0;
    child_index_range(lo.x, hi.x, node_cell.x, child_span.x, branch_factor_.x, x0, x1);
    child_index_range(lo.y, hi.y, node_cell.y, child_span.y, branch_factor_.y, y0, y1);
    child_index_range(lo.z, hi.z, node_cell.z, child_span.z, branch_factor_.z, z0, z1);

    bool erased = false;
    for (std::int64_t z = z0; z <= z1; ++z) {
      for (std::int64_t y = y0; y <= y1; ++y) {
        for (std::int64_t x = x0; x <= x1; ++x) {
          const size_t index = flat_index(x, y, z);
          node_type* child = node.child(index);
          if (!child) {
            continue;
          }
          const cell_vec child_cell(
              node_cell.x + x * child_span.x, node_cell.y + y * child_span.y, node_cell.z + z * child_span.z);
          if (erase_range_node(*child, child_cell, child_span, lo, hi)) {
            erased = true;
            if (child->empty()) {
              node.erase_child(index);
            }
          }
        }
      }
    }
    return erased;
  }

  template <typename Node, typename Visitor>
  bool query_range_node(Node& node,
                        const cell_vec& node_cell,
                        const cell_vec& node_span,
                        const cell_vec& lo,
                        const cell_vec& hi,
                        Visitor& visitor) const noexcept {
    if (is_unit_span(node_span)) {
      if (!node.has_value()) {
        return true;
      }
      return std::invoke(visitor, *node.value(), node.box());
    }

    const cell_vec child_span = node_span / branch_cell_;
    std::int64_t x0 = 0;
    std::int64_t x1 = 0;
    std::int64_t y0 = 0;
    std::int64_t y1 = 0;
    std::int64_t z0 = 0;
    std::int64_t z1 = 0;
    child_index_range(lo.x, hi.x, node_cell.x, child_span.x, branch_factor_.x, x0, x1);
    child_index_range(lo.y, hi.y, node_cell.y, child_span.y, branch_factor_.y, y0, y1);
    child_index_range(lo.z, hi.z, node_cell.z, child_span.z, branch_factor_.z, z0, z1);

    for (std::int64_t z = z0; z <= z1; ++z) {
      for (std::int64_t y = y0; y <= y1; ++y) {
        for (std::int64_t x = x0; x <= x1; ++x) {
          Node* child = node.child(flat_index(x, y, z));
          if (!child) {
            continue;
          }
          const cell_vec child_cell(
              node_cell.x + x * child_span.x, node_cell.y + y * child_span.y, node_cell.z + z * child_span.z);
          if (!query_range_node(*child, child_cell, child_span, lo, hi, visitor)) {
            return false;
          }
        }
      }
    }
    return true;
  }

  template <typename Node, typename Visitor>
  bool query_ray_node(Node& node, const geometry::ray& ry, Visitor& visitor) const noexcept {
    float distance = 0.0f;
    if (!geometry::ray_aabb(ry, node.box(), distance)) {
      return true;
    }

    if (node.is_leaf()) {
      if (!node.has_value()) {
        return true;
      }
      return std::invoke(visitor, *node.value(), node.box());
    }

    bool keep_going = true;
    node.for_each_child([&](size_t, auto& child) -> bool {
      if (!query_ray_node(child, ry, visitor)) {
        keep_going = false;
        return false;
      }
      return true;
    });
    return keep_going;
  }

  template <typename Node, typename Visitor>
  bool query_frustum_node(Node& node, const geometry::frustum& fr, Visitor& visitor) const noexcept {
    if (geometry::is_outside(fr, node.box())) {
      return true;
    }

    if (node.is_leaf()) {
      if (!node.has_value()) {
        return true;
      }
      return std::invoke(visitor, *node.value(), node.box());
    }

    bool keep_going = true;
    node.for_each_child([&](size_t, auto& child) -> bool {
      if (!query_frustum_node(child, fr, visitor)) {
        keep_going = false;
        return false;
      }
      return true;
    });
    return keep_going;
  }

  glm::vec3 unit_;
  glm::ivec3 branch_factor_;
  glm::vec3 origin_;
  cell_vec branch_cell_;
  cell_vec base_cell_{ 0, 0, 0 };
  cell_vec span_{ 1, 1, 1 };
  size_t branch_count_ = 0;
  node_type root_;
  size_t grid_num_ = 0;
};

template <typename T, bool Sparse>
uniform_grid_impl<T, Sparse>::uniform_grid_impl(glm::vec3 unit, glm::ivec3 branch_factor, glm::vec3 origin)
    : unit_(checked_unit(unit))
    , branch_factor_(checked_branch_factor(branch_factor))
    , origin_(checked_origin(origin))
    , branch_cell_(branch_factor.x, branch_factor.y, branch_factor.z)
    , branch_count_(checked_branch_count(branch_factor))
    , root_(make_cell_box(origin, unit, cell_vec(0), cell_vec(1)), true, branch_count_) {
  if (!root_.box().is_valid()) {
    throw std::invalid_argument("uniform grid origin and unit produce a non-finite cell box");
  }
}

template <typename T, bool Sparse>
const typename uniform_grid_impl<T, Sparse>::value_type*
uniform_grid_impl<T, Sparse>::get(const glm::vec3& pos) const noexcept {
  cell_vec cell{};
  if (!to_cell(pos, cell) || !contains_cell(cell)) {
    return nullptr;
  }

  const node_type* node = descend(cell);
  return node ? node->value() : nullptr;
}

template <typename T, bool Sparse>
typename uniform_grid_impl<T, Sparse>::value_type* uniform_grid_impl<T, Sparse>::get(const glm::vec3& pos) noexcept {
  cell_vec cell{};
  if (!to_cell(pos, cell) || !contains_cell(cell)) {
    return nullptr;
  }

  node_type* node = descend(cell);
  return node ? node->value() : nullptr;
}

template <typename T, bool Sparse>
typename uniform_grid_impl<T, Sparse>::value_type& uniform_grid_impl<T, Sparse>::ensure(const glm::vec3& pos) {
  cell_vec cell{};
  if (!to_cell(pos, cell)) {
    throw std::invalid_argument("uniform grid position must be finite and representable");
  }
  ensure_root_contains(cell);

  node_type& leaf = ensure_leaf(cell);
  if (!leaf.has_value()) {
    leaf.ensure_value();
    ++grid_num_;
  }
  return *leaf.value();
}

template <typename T, bool Sparse>
template <typename... Args>
void uniform_grid_impl<T, Sparse>::set(const glm::vec3& pos, Args&&... args) {
  cell_vec cell{};
  if (!to_cell(pos, cell)) {
    throw std::invalid_argument("uniform grid position must be finite and representable");
  }
  ensure_root_contains(cell);

  node_type& leaf = ensure_leaf(cell);
  const bool existed = leaf.has_value();
  leaf.set_value(std::forward<Args>(args)...);
  if (!existed) {
    ++grid_num_;
  }
}

template <typename T, bool Sparse>
void uniform_grid_impl<T, Sparse>::set(const glm::vec3& pos, const value_type& value) {
  // Copy before expanding: the source may alias a value owned by this grid and expanding moves the whole tree.
  const value_type value_copy = value;

  cell_vec cell{};
  if (!to_cell(pos, cell)) {
    throw std::invalid_argument("uniform grid position must be finite and representable");
  }
  ensure_root_contains(cell);

  node_type& leaf = ensure_leaf(cell);
  const bool existed = leaf.has_value();
  leaf.set_value(value_copy);
  if (!existed) {
    ++grid_num_;
  }
}

template <typename T, bool Sparse>
void uniform_grid_impl<T, Sparse>::set(const geometry::aabb& range, const value_type& value) {
  cell_vec lo{};
  cell_vec hi{};
  if (!cell_range(range, lo, hi)) {
    return;
  }

  // Copy before expanding: the source may alias a value owned by this grid and expanding moves the whole tree.
  const value_type value_copy = value;

  ensure_root_contains(lo);
  ensure_root_contains(hi);
  set_range_node(root_, base_cell_, span_, lo, hi, value_copy);
}

template <typename T, bool Sparse>
template <typename Fn>
  requires(std::is_invocable_r_v<typename uniform_grid_impl<T, Sparse>::value_type, std::remove_cvref_t<Fn>&> ||
           std::is_invocable_r_v<typename uniform_grid_impl<T, Sparse>::value_type,
                                 std::remove_cvref_t<Fn>&,
                                 const geometry::aabb&>)
void uniform_grid_impl<T, Sparse>::set(const geometry::aabb& range, Fn&& generator) {
  cell_vec lo{};
  cell_vec hi{};
  if (!cell_range(range, lo, hi)) {
    return;
  }

  ensure_root_contains(lo);
  ensure_root_contains(hi);
  Fn& generator_ref = generator;
  set_range_generator_node(root_, base_cell_, span_, lo, hi, generator_ref);
}

template <typename T, bool Sparse>
bool uniform_grid_impl<T, Sparse>::cell_range(const geometry::aabb& range,
                                              cell_coord& lo,
                                              cell_coord& hi) const noexcept {
  return detail::cell_range(origin_, unit_, range, lo, hi);
}

template <typename T, bool Sparse>
bool uniform_grid_impl<T, Sparse>::contains_cell(const cell_coord& cell) const noexcept {
  // The invariant base + span is kept inside int64_t by expand_root(), so no subtraction can overflow here.
  const cell_vec upper = base_cell_ + span_;
  return cell.x >= base_cell_.x && cell.x < upper.x && cell.y >= base_cell_.y && cell.y < upper.y &&
         cell.z >= base_cell_.z && cell.z < upper.z;
}

template <typename T, bool Sparse>
const typename uniform_grid_impl<T, Sparse>::value_type*
uniform_grid_impl<T, Sparse>::get_cell(const cell_coord& cell) const noexcept {
  if (!contains_cell(cell)) {
    return nullptr;
  }
  const node_type* node = descend(cell);
  return node ? node->value() : nullptr;
}

template <typename T, bool Sparse>
typename uniform_grid_impl<T, Sparse>::value_type*
uniform_grid_impl<T, Sparse>::get_cell(const cell_coord& cell) noexcept {
  if (!contains_cell(cell)) {
    return nullptr;
  }
  node_type* node = descend(cell);
  return node ? node->value() : nullptr;
}

template <typename T, bool Sparse>
typename uniform_grid_impl<T, Sparse>::value_type& uniform_grid_impl<T, Sparse>::ensure_cell(const cell_coord& cell) {
  ensure_root_contains(cell);
  node_type& leaf = ensure_leaf(cell);
  if (!leaf.has_value()) {
    leaf.ensure_value();
    ++grid_num_;
  }
  return *leaf.value();
}

template <typename T, bool Sparse>
bool uniform_grid_impl<T, Sparse>::erase_cell(const cell_coord& cell) noexcept {
  if (!contains_cell(cell)) {
    return false;
  }
  return erase_cell_node(root_, base_cell_, span_, cell - base_cell_);
}

template <typename T, bool Sparse>
bool uniform_grid_impl<T, Sparse>::erase(const glm::vec3& pos) noexcept {
  cell_vec cell{};
  if (!to_cell(pos, cell) || !contains_cell(cell)) {
    return false;
  }
  return erase_cell_node(root_, base_cell_, span_, cell - base_cell_);
}

template <typename T, bool Sparse>
bool uniform_grid_impl<T, Sparse>::erase(const geometry::aabb& range) noexcept {
  cell_vec lo{};
  cell_vec hi{};
  if (!cell_range(range, lo, hi) || !clip_to_root(lo, hi)) {
    return false;
  }
  return erase_range_node(root_, base_cell_, span_, lo, hi);
}

template <typename T, bool Sparse>
void uniform_grid_impl<T, Sparse>::clear() noexcept {
  root_.reset(make_cell_box(origin_, unit_, cell_vec(0), cell_vec(1)), true, branch_count_);
  base_cell_ = cell_vec(0);
  span_ = cell_vec(1);
  grid_num_ = 0;
}

template <typename T, bool Sparse>
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 const typename uniform_grid_impl<T, Sparse>::value_type&,
                                 const geometry::aabb&>)
void uniform_grid_impl<T, Sparse>::query(const geometry::ray& ry, Visitor&& f) const noexcept {
  if (!ry.is_valid()) {
    return;
  }
  Visitor& visitor = f;
  query_ray_node(root_, ry, visitor);
}

template <typename T, bool Sparse>
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 typename uniform_grid_impl<T, Sparse>::value_type&,
                                 const geometry::aabb&>)
void uniform_grid_impl<T, Sparse>::query(const geometry::ray& ry, Visitor&& f) noexcept {
  if (!ry.is_valid()) {
    return;
  }
  Visitor& visitor = f;
  query_ray_node(root_, ry, visitor);
}

template <typename T, bool Sparse>
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 const typename uniform_grid_impl<T, Sparse>::value_type&,
                                 const geometry::aabb&>)
void uniform_grid_impl<T, Sparse>::query(const geometry::aabb& range, Visitor&& f) const noexcept {
  cell_vec lo{};
  cell_vec hi{};
  if (!cell_range(range, lo, hi) || !clip_to_root(lo, hi)) {
    return;
  }
  Visitor& visitor = f;
  query_range_node(root_, base_cell_, span_, lo, hi, visitor);
}

template <typename T, bool Sparse>
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 typename uniform_grid_impl<T, Sparse>::value_type&,
                                 const geometry::aabb&>)
void uniform_grid_impl<T, Sparse>::query(const geometry::aabb& range, Visitor&& f) noexcept {
  cell_vec lo{};
  cell_vec hi{};
  if (!cell_range(range, lo, hi) || !clip_to_root(lo, hi)) {
    return;
  }
  Visitor& visitor = f;
  query_range_node(root_, base_cell_, span_, lo, hi, visitor);
}

template <typename T, bool Sparse>
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 const typename uniform_grid_impl<T, Sparse>::value_type&,
                                 const geometry::aabb&>)
void uniform_grid_impl<T, Sparse>::query(const geometry::frustum& fr, Visitor&& f) const noexcept {
  Visitor& visitor = f;
  query_frustum_node(root_, fr, visitor);
}

template <typename T, bool Sparse>
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 typename uniform_grid_impl<T, Sparse>::value_type&,
                                 const geometry::aabb&>)
void uniform_grid_impl<T, Sparse>::query(const geometry::frustum& fr, Visitor&& f) noexcept {
  Visitor& visitor = f;
  query_frustum_node(root_, fr, visitor);
}

template <typename T, bool Sparse>
geometry::aabb uniform_grid_impl<T, Sparse>::get_unit_box(const glm::vec3& pos) const noexcept {
  cell_vec cell{};
  if (!to_cell(pos, cell)) {
    return {};
  }
  return cell_box(cell, cell_vec(1));
}

} // namespace detail
} // namespace spatial_index
} // namespace cyberverse
