/**
 * @file uniform_grid_node.inl
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Uniform grid node implementation
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "cyberverse/spatial_index/detail/uniform_grid_node.hpp"

namespace cyberverse {
namespace spatial_index {
namespace detail {

// ---------------------------------------------------------------------------------------------------------------------
// uniform_grid_node
// ---------------------------------------------------------------------------------------------------------------------

template <typename T, bool Sparse>
uniform_grid_node<T, Sparse>::uniform_grid_node(const geometry::aabb& box, bool is_leaf, size_t child_capacity)
    : box_(box), child_capacity_(child_capacity), leaf_(is_leaf) {}

template <typename T, bool Sparse>
uniform_grid_node<T, Sparse>::~uniform_grid_node() = default;

template <typename T, bool Sparse>
uniform_grid_node<T, Sparse>::uniform_grid_node(uniform_grid_node&&) = default;

template <typename T, bool Sparse>
uniform_grid_node<T, Sparse>& uniform_grid_node<T, Sparse>::operator=(uniform_grid_node&&) = default;

template <typename T, bool Sparse>
bool uniform_grid_node<T, Sparse>::empty() const noexcept {
  return !value_.has_value() && (!collection_ || collection_->empty());
}

template <typename T, bool Sparse>
void uniform_grid_node<T, Sparse>::clear() noexcept {
  box_ = geometry::aabb{};
  collection_.reset();
  value_.reset();
  child_capacity_ = 0;
  leaf_ = true;
}

template <typename T, bool Sparse>
void uniform_grid_node<T, Sparse>::reset(const geometry::aabb& box, bool is_leaf, size_t child_capacity) noexcept {
  box_ = box;
  collection_.reset();
  value_.reset();
  child_capacity_ = child_capacity;
  leaf_ = is_leaf;
}

template <typename T, bool Sparse>
size_t uniform_grid_node<T, Sparse>::child_count() const noexcept {
  return collection_ ? collection_->size() : 0;
}

template <typename T, bool Sparse>
bool uniform_grid_node<T, Sparse>::has_children() const noexcept {
  return collection_ && !collection_->empty();
}

template <typename T, bool Sparse>
const uniform_grid_node<T, Sparse>* uniform_grid_node<T, Sparse>::child(size_t index) const noexcept {
  return collection_ ? collection_->find(index) : nullptr;
}

template <typename T, bool Sparse>
uniform_grid_node<T, Sparse>* uniform_grid_node<T, Sparse>::child(size_t index) noexcept {
  return collection_ ? collection_->find(index) : nullptr;
}

template <typename T, bool Sparse>
uniform_grid_node<T, Sparse>&
uniform_grid_node<T, Sparse>::ensure_child(size_t index, const geometry::aabb& box, bool is_leaf) {
  assert(index < child_capacity_ && "child index is outside of the branch factor");
  assert(!value_.has_value() && "an internal node cannot own a value");

  if (!collection_) {
    collection_ = std::make_unique<collection_type>(child_capacity_);
  }
  leaf_ = false;
  return collection_->ensure(index, box, is_leaf);
}

template <typename T, bool Sparse>
uniform_grid_node<T, Sparse>& uniform_grid_node<T, Sparse>::insert_child(size_t index, uniform_grid_node&& node) {
  assert(index < child_capacity_ && "child index is outside of the branch factor");
  assert(!value_.has_value() && "an internal node cannot own a value");

  if (!collection_) {
    collection_ = std::make_unique<collection_type>(child_capacity_);
  }
  leaf_ = false;
  return collection_->insert(index, std::move(node));
}

template <typename T, bool Sparse>
bool uniform_grid_node<T, Sparse>::erase_child(size_t index) noexcept {
  if (!collection_ || !collection_->erase(index)) {
    return false;
  }
  if (collection_->empty()) {
    collection_.reset();
  }
  return true;
}

template <typename T, bool Sparse>
template <typename F>
void uniform_grid_node<T, Sparse>::for_each_child(F&& f) const noexcept {
  if (collection_) {
    collection_->for_each(std::forward<F>(f));
  }
}

template <typename T, bool Sparse>
template <typename F>
void uniform_grid_node<T, Sparse>::for_each_child(F&& f) noexcept {
  if (collection_) {
    collection_->for_each(std::forward<F>(f));
  }
}

template <typename T, bool Sparse>
size_t uniform_grid_node<T, Sparse>::memory_usage_in_bytes() const noexcept {
  return sizeof(*this) + owned_memory_usage_in_bytes();
}

template <typename T, bool Sparse>
size_t uniform_grid_node<T, Sparse>::owned_memory_usage_in_bytes() const noexcept {
  return collection_ ? collection_->memory_usage_in_bytes() : 0;
}

// ---------------------------------------------------------------------------------------------------------------------
// uniform_grid_node_collection (dense)
// ---------------------------------------------------------------------------------------------------------------------

template <typename T, bool Sparse>
uniform_grid_node_collection<T, Sparse>::uniform_grid_node_collection(size_t capacity) : container_(capacity) {}

template <typename T, bool Sparse>
void uniform_grid_node_collection<T, Sparse>::clear() noexcept {
  for (node_type& node : container_) {
    if (node.exists()) {
      node.clear();
    }
  }
  size_ = 0;
}

template <typename T, bool Sparse>
typename uniform_grid_node_collection<T, Sparse>::node_type*
uniform_grid_node_collection<T, Sparse>::find(size_t index) noexcept {
  if (index >= container_.size()) {
    return nullptr;
  }
  node_type& node = container_[index];
  return node.exists() ? &node : nullptr;
}

template <typename T, bool Sparse>
const typename uniform_grid_node_collection<T, Sparse>::node_type*
uniform_grid_node_collection<T, Sparse>::find(size_t index) const noexcept {
  if (index >= container_.size()) {
    return nullptr;
  }
  const node_type& node = container_[index];
  return node.exists() ? &node : nullptr;
}

template <typename T, bool Sparse>
typename uniform_grid_node_collection<T, Sparse>::node_type&
uniform_grid_node_collection<T, Sparse>::ensure(size_t index, const geometry::aabb& box, bool is_leaf) {
  assert(index < container_.size() && "child index is outside of the branch factor");

  node_type& node = container_[index];
  if (!node.exists()) {
    node.reset(box, is_leaf, container_.size());
    ++size_;
  }
  return node;
}

template <typename T, bool Sparse>
typename uniform_grid_node_collection<T, Sparse>::node_type&
uniform_grid_node_collection<T, Sparse>::insert(size_t index, node_type&& node) {
  assert(index < container_.size() && "child index is outside of the branch factor");

  node_type& slot = container_[index];
  const bool existed = slot.exists();
  slot = std::move(node);
  if (!existed) {
    ++size_;
  }
  return slot;
}

template <typename T, bool Sparse>
bool uniform_grid_node_collection<T, Sparse>::erase(size_t index) noexcept {
  if (index >= container_.size()) {
    return false;
  }

  node_type& node = container_[index];
  if (!node.exists()) {
    return false;
  }
  node.clear();
  --size_;
  return true;
}

template <typename T, bool Sparse>
template <typename F>
void uniform_grid_node_collection<T, Sparse>::for_each(F&& f) const noexcept {
  for (size_t i = 0; i < container_.size(); ++i) {
    const node_type& node = container_[i];
    if (node.exists() && !f(i, node)) {
      break;
    }
  }
}

template <typename T, bool Sparse>
template <typename F>
void uniform_grid_node_collection<T, Sparse>::for_each(F&& f) noexcept {
  for (size_t i = 0; i < container_.size(); ++i) {
    node_type& node = container_[i];
    if (node.exists() && !f(i, node)) {
      break;
    }
  }
}

template <typename T, bool Sparse>
size_t uniform_grid_node_collection<T, Sparse>::memory_usage_in_bytes() const noexcept {
  size_t total = sizeof(*this) + container_.capacity() * sizeof(node_type);
  for (const node_type& node : container_) {
    if (node.exists()) {
      total += node.owned_memory_usage_in_bytes();
    }
  }
  return total;
}

// ---------------------------------------------------------------------------------------------------------------------
// uniform_grid_node_collection (sparse)
// ---------------------------------------------------------------------------------------------------------------------

template <typename T>
uniform_grid_node_collection<T, true>::uniform_grid_node_collection(size_t capacity) : capacity_(capacity) {}

template <typename T>
void uniform_grid_node_collection<T, true>::clear() noexcept {
  array_.clear();
  map_.clear();
}

template <typename T>
typename uniform_grid_node_collection<T, true>::node_type*
uniform_grid_node_collection<T, true>::find(size_t index) noexcept {
  if (node_type* node = array_.find(index)) {
    return node;
  }
  auto it = map_.find(index);
  return it == map_.end() ? nullptr : &it->second;
}

template <typename T>
const typename uniform_grid_node_collection<T, true>::node_type*
uniform_grid_node_collection<T, true>::find(size_t index) const noexcept {
  if (const node_type* node = array_.find(index)) {
    return node;
  }
  auto it = map_.find(index);
  return it == map_.end() ? nullptr : &it->second;
}

template <typename T>
void uniform_grid_node_collection<T, true>::promote_to_map() {
  array_.drain([this](size_t&& key, node_type&& node) {
    map_.emplace(std::move(key), std::move(node));
  });
}

template <typename T>
typename uniform_grid_node_collection<T, true>::node_type&
uniform_grid_node_collection<T, true>::ensure(size_t index, const geometry::aabb& box, bool is_leaf) {
  assert(index < capacity_ && "child index is outside of the branch factor");

  if (node_type* node = find(index)) {
    return *node;
  }

  if (!array_.full()) {
    const auto result = array_.set(index, node_type(box, is_leaf, capacity_));
    assert(result == array_type::set_result::ok);
    (void)result;
    return *array_.find(index);
  }

  promote_to_map();
  auto [it, inserted] = map_.try_emplace(index, box, is_leaf, capacity_);
  (void)inserted;
  return it->second;
}

template <typename T>
typename uniform_grid_node_collection<T, true>::node_type&
uniform_grid_node_collection<T, true>::insert(size_t index, node_type&& node) {
  assert(index < capacity_ && "child index is outside of the branch factor");

  if (node_type* existing = find(index)) {
    *existing = std::move(node);
    return *existing;
  }

  if (!array_.full()) {
    const auto result = array_.set(index, std::move(node));
    assert(result == array_type::set_result::ok);
    (void)result;
    return *array_.find(index);
  }

  promote_to_map();
  auto [it, inserted] = map_.emplace(index, std::move(node));
  (void)inserted;
  return it->second;
}

template <typename T>
bool uniform_grid_node_collection<T, true>::erase(size_t index) noexcept {
  if (array_.erase(index)) {
    return true;
  }
  return map_.erase(index) > 0;
}

template <typename T>
template <typename F>
void uniform_grid_node_collection<T, true>::for_each(F&& f) const noexcept {
  bool stopped = false;
  array_.for_each([&](const size_t& key, const node_type& node) -> bool {
    if (f(key, node)) {
      return true;
    }
    stopped = true;
    return false;
  });
  if (stopped) {
    return;
  }

  for (const auto& entry : map_) {
    if (!f(entry.first, entry.second)) {
      break;
    }
  }
}

template <typename T>
template <typename F>
void uniform_grid_node_collection<T, true>::for_each(F&& f) noexcept {
  bool stopped = false;
  array_.for_each([&](const size_t& key, node_type& node) -> bool {
    if (f(key, node)) {
      return true;
    }
    stopped = true;
    return false;
  });
  if (stopped) {
    return;
  }

  for (auto& entry : map_) {
    if (!f(entry.first, entry.second)) {
      break;
    }
  }
}

template <typename T>
size_t uniform_grid_node_collection<T, true>::memory_usage_in_bytes() const noexcept {
  // The inline keyed_array slots are part of sizeof(*this); the map entries and the child collections are not.
  size_t total = sizeof(*this);
  total += map_.bucket_count() * sizeof(void*);
  total += map_.size() * (sizeof(std::pair<const size_t, node_type>) + sizeof(void*));

  array_.for_each([&](const size_t&, const node_type& node) -> bool {
    total += node.owned_memory_usage_in_bytes();
    return true;
  });
  for (const auto& entry : map_) {
    total += entry.second.owned_memory_usage_in_bytes();
  }
  return total;
}

} // namespace detail
} // namespace spatial_index
} // namespace cyberverse
