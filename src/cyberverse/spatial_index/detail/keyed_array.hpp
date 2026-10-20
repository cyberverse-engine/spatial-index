/**
 * @file keyed_array.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Keyed array
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <optional>
#include <type_traits>
#include <utility>

namespace cyberverse {
namespace spatial_index {
namespace detail {

template <typename Key, typename Value, size_t Capacity = 8>
class keyed_array {
public:
  using key_type = Key;
  using value_type = Value;
  using pair_type = std::pair<key_type, value_type>;
  using slot_type = std::optional<pair_type>;

  static constexpr size_t capacity = Capacity; // Maximum number of inline entries

  static_assert(Capacity > 0, "keyed_array capacity must be positive");

  /**
   * @brief Result of an insertion
   *
   */
  enum class set_result {
    ok,        // Inserted or overwritten
    duplicate, // The name already exists and overwrite was false, nothing changed
    full,      // No free slot left, the caller has to promote the entries into the map representation
  };

  keyed_array() = default;

  keyed_array(const keyed_array&) = delete;
  keyed_array(keyed_array&&) = default;

  ~keyed_array() = default;

  keyed_array& operator=(const keyed_array&) = delete;
  keyed_array& operator=(keyed_array&&) = default;

  /**
   * @brief Whether the array is empty
   *
   * @return bool
   */
  bool empty() const noexcept { return size_ == 0; }

  /**
   * @brief Number of live entries
   *
   * @return size_t
   */
  size_t size() const noexcept { return size_; }

  /**
   * @brief Whether every slot is used
   *
   * @return bool
   */
  bool full() const noexcept { return size_ == Capacity; }

  /**
   * @brief Find a value by key
   *
   * @param key Key
   * @return value_type*, nullptr when no such entry exists
   */
  template <typename T>
  const value_type* find(T&& key) const noexcept {
    for (size_t i = 0; i < size_; ++i) {
      if (at(i).first == key) {
        return &at(i).second;
      }
    }
    return nullptr;
  }

  /**
   * @brief Find a value by key
   *
   * @param key Key
   * @return value_type*, nullptr when no such entry exists
   */
  template <typename T>
  value_type* find(T&& key) noexcept {
    for (size_t i = 0; i < size_; ++i) {
      if (at(i).first == key) {
        return &at(i).second;
      }
    }
    return nullptr;
  }

  /**
   * @brief Whether an entry with the key exists
   *
   * @param key Key
   * @return bool
   */
  template <typename T>
  bool contains(T&& key) const noexcept {
    for (size_t i = 0; i < size_; ++i) {
      if (at(i).first == key) {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Insert or overwrite an entry, appended at the end of the live prefix
   *
   * @param key Key
   * @param value Value
   * @param overwrite Overwrite an existing entry, false by default
   * @return set_result, `full` means the caller has to promote the entries into the map representation
   */
  template <typename T, typename U>
  set_result set(T&& key, U&& value, bool overwrite = false) {
    for (size_t i = 0; i < size_; ++i) {
      if (at(i).first == key) {
        if (!overwrite) {
          return set_result::duplicate;
        }
        at(i).second = std::forward<U>(value);
        return set_result::ok;
      }
    }
    if (full()) {
      return set_result::full;
    }
    slots_[size_].emplace(std::forward<T>(key), std::forward<U>(value));
    ++size_;
    return set_result::ok;
  }

  /**
   * @brief Erase an entry, moving the entries behind it down to keep the live prefix dense
   *
   * @param key Key of the entry to erase
   * @return bool, erased or not
   */
  template <typename T>
  bool erase(T&& key) noexcept {
    for (size_t i = 0; i < size_; ++i) {
      if (at(i).first != key) {
        continue;
      }
      // No holes: shift the entries behind the erased one down, then drop the last slot
      for (size_t k = i + 1; k < size_; ++k) {
        slots_[k - 1] = std::move(slots_[k]);
      }
      slots_[size_ - 1].reset();
      --size_;
      return true;
    }
    return false;
  }

  /**
   * @brief Erase all entries
   *
   */
  void clear() noexcept {
    for (size_t i = 0; i < size_; ++i) {
      slots_[i].reset();
    }
    size_ = 0;
  }

  /**
   * @brief Visit every live entry in insertion order
   *
   * @tparam F Visit function type, signature is `bool(const key_type& name, const value_type& value)`
   * @param f Visit function
   */
  template <typename F>
    requires(std::is_same_v<std::invoke_result_t<std::remove_cvref_t<F>, const key_type&, const value_type&>, bool>)
  void for_each(F&& f) const noexcept {
    for (size_t i = 0; i < size_; ++i) {
      if (!f(at(i).first, at(i).second)) {
        break;
      }
    }
  }

  /**
   * @brief Visit every live entry in insertion order
   *
   * @tparam F Visit function type, signature is `bool(const key_type& name, value_type& value)`
   * @param f Visit function
   */
  template <typename F>
    requires(std::is_same_v<std::invoke_result_t<std::remove_cvref_t<F>, const key_type&, value_type&>, bool>)
  void for_each(F&& f) noexcept {
    for (size_t i = 0; i < size_; ++i) {
      if (!f(const_cast<const key_type&>(at(i).first), at(i).second)) {
        break;
      }
    }
  }

  /**
   * @brief Move every live entry out through the function, then clear the array
   *
   *  Used to promote the entries into the hash map representation without copying the names and the nodes.
   *
   * @tparam F Visit function type, signature is `void(std::string&& name, node_type&& p_node)`
   * @param f Visit function
   */
  template <typename F>
    requires(std::is_same_v<std::invoke_result_t<std::remove_cvref_t<F>, key_type &&, value_type &&>, void>)
  void drain(F&& f) {
    for (size_t i = 0; i < size_; ++i) {
      f(std::move(at(i).first), std::move(at(i).second));
    }
    clear();
  }

private:
  /**
   * @brief Get a live entry, every index below `size()` must be engaged
   *
   * @param index Slot index
   * @return value_type&
   */
  pair_type& at(size_t index) noexcept {
    assert(slots_[index].has_value() && "live entries are the dense prefix [0, size_)");
    return *slots_[index];
  }

  /**
   * @brief Get a live entry, every index below `size()` must be engaged
   *
   * @param index Slot index
   * @return value_type&
   */
  const pair_type& at(size_t index) const noexcept {
    assert(slots_[index].has_value() && "live entries are the dense prefix [0, size_)");
    return *slots_[index];
  }

  std::array<slot_type, Capacity> slots_{}; // Engaged slots are always the dense prefix [0, size_)
  size_t size_ = 0;                         // Number of live entries
};

} // namespace detail
} // namespace spatial_index
} // namespace cyberverse
