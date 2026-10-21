/**
 * @file sparse_uniform_grid.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Sparse uniform grid implementation
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "cyberverse/spatial_index/sparse_uniform_grid.hpp"

namespace cyberverse {
namespace spatial_index {

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
sparse_uniform_grid<T>::sparse_uniform_grid(glm::vec3 unit, glm::ivec3 branch_factor, glm::vec3 origin)
    : impl_(unit, branch_factor, origin) {}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
const typename sparse_uniform_grid<T>::value_type* sparse_uniform_grid<T>::get(const glm::vec3& pos) const noexcept {
  return impl_.get(pos);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
typename sparse_uniform_grid<T>::value_type* sparse_uniform_grid<T>::get(const glm::vec3& pos) noexcept {
  return impl_.get(pos);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
bool sparse_uniform_grid<T>::contains(const glm::vec3& pos) const noexcept {
  return impl_.get(pos) != nullptr;
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
typename sparse_uniform_grid<T>::value_type& sparse_uniform_grid<T>::ensure(const glm::vec3& pos) {
  return impl_.ensure(pos);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
template <typename... Args>
void sparse_uniform_grid<T>::set(const glm::vec3& pos, Args&&... args) {
  impl_.set(pos, std::forward<Args>(args)...);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
void sparse_uniform_grid<T>::set(const glm::vec3& pos, const value_type& value) {
  impl_.set(pos, value);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
void sparse_uniform_grid<T>::set(const geometry::aabb& range, const value_type& value) {
  impl_.set(range, value);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
template <typename Fn>
  requires(std::is_invocable_r_v<typename sparse_uniform_grid<T>::value_type, std::remove_cvref_t<Fn>&> ||
           std::is_invocable_r_v<typename sparse_uniform_grid<T>::value_type,
                                 std::remove_cvref_t<Fn>&,
                                 const geometry::aabb&>)
void sparse_uniform_grid<T>::set(const geometry::aabb& range, Fn&& generator) {
  impl_.set(range, std::forward<Fn>(generator));
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
bool sparse_uniform_grid<T>::erase(const glm::vec3& pos) noexcept {
  return impl_.erase(pos);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
bool sparse_uniform_grid<T>::erase(const geometry::aabb& range) noexcept {
  return impl_.erase(range);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
void sparse_uniform_grid<T>::clear() noexcept {
  impl_.clear();
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
bool sparse_uniform_grid<T>::empty() const noexcept {
  return impl_.grid_num() == 0;
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 const typename sparse_uniform_grid<T>::value_type&,
                                 const geometry::aabb&>)
void sparse_uniform_grid<T>::query(const geometry::ray& ry, Visitor&& f) const noexcept {
  impl_.query(ry, std::forward<Visitor>(f));
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 typename sparse_uniform_grid<T>::value_type&,
                                 const geometry::aabb&>)
void sparse_uniform_grid<T>::query(const geometry::ray& ry, Visitor&& f) noexcept {
  impl_.query(ry, std::forward<Visitor>(f));
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 const typename sparse_uniform_grid<T>::value_type&,
                                 const geometry::aabb&>)
void sparse_uniform_grid<T>::query(const geometry::aabb& range, Visitor&& f) const noexcept {
  impl_.query(range, std::forward<Visitor>(f));
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 typename sparse_uniform_grid<T>::value_type&,
                                 const geometry::aabb&>)
void sparse_uniform_grid<T>::query(const geometry::aabb& range, Visitor&& f) noexcept {
  impl_.query(range, std::forward<Visitor>(f));
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 const typename sparse_uniform_grid<T>::value_type&,
                                 const geometry::aabb&>)
void sparse_uniform_grid<T>::query(const geometry::frustum& fr, Visitor&& f) const noexcept {
  impl_.query(fr, std::forward<Visitor>(f));
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
template <typename Visitor>
  requires(std::is_invocable_r_v<bool,
                                 std::remove_cvref_t<Visitor>,
                                 typename sparse_uniform_grid<T>::value_type&,
                                 const geometry::aabb&>)
void sparse_uniform_grid<T>::query(const geometry::frustum& fr, Visitor&& f) noexcept {
  impl_.query(fr, std::forward<Visitor>(f));
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
const glm::vec3& sparse_uniform_grid<T>::unit() const noexcept {
  return impl_.unit();
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
const glm::ivec3& sparse_uniform_grid<T>::branch_factor() const noexcept {
  return impl_.branch_factor();
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
const glm::vec3& sparse_uniform_grid<T>::origin() const noexcept {
  return impl_.origin();
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
geometry::aabb sparse_uniform_grid<T>::get_unit_box(const glm::vec3& pos) const noexcept {
  return impl_.get_unit_box(pos);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
size_t sparse_uniform_grid<T>::grid_num() const noexcept {
  return impl_.grid_num();
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
size_t sparse_uniform_grid<T>::memory_usage_in_bytes() const noexcept {
  return impl_.memory_usage_in_bytes();
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
bool sparse_uniform_grid<T>::cell_range(const geometry::aabb& range, cell_coord& lo, cell_coord& hi) const noexcept {
  return impl_.cell_range(range, lo, hi);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
const typename sparse_uniform_grid<T>::value_type*
sparse_uniform_grid<T>::get_cell(const cell_coord& cell) const noexcept {
  return impl_.get_cell(cell);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
typename sparse_uniform_grid<T>::value_type* sparse_uniform_grid<T>::get_cell(const cell_coord& cell) noexcept {
  return impl_.get_cell(cell);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
typename sparse_uniform_grid<T>::value_type& sparse_uniform_grid<T>::ensure_cell(const cell_coord& cell) {
  return impl_.ensure_cell(cell);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
bool sparse_uniform_grid<T>::erase_cell(const cell_coord& cell) noexcept {
  return impl_.erase_cell(cell);
}

template <typename T>
  requires(std::is_default_constructible_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> &&
           std::is_move_constructible_v<T> && std::is_move_assignable_v<T>)
bool sparse_uniform_grid<T>::contains_cell(const cell_coord& cell) const noexcept {
  return impl_.contains_cell(cell);
}

} // namespace spatial_index
} // namespace cyberverse
