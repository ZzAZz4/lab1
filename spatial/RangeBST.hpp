#pragma once

#include "SpatialBase.h"

namespace utec {
namespace spatial {
namespace detail {

#ifndef SPATIAL_IMPL_POINT_LESS_EQ
#define SPATIAL_IMPL_POINT_LESS_EQ
template <class Point>
struct point_less_equals {
  constexpr bool operator()(const Point& lhs, const Point& rhs) const {
    return !(rhs < lhs);
  }
};
#endif

template <class Key_, class Value_>
struct bst_node_t {
  using key_type = Key_;
  using value_type = Value_;
  using pointer = bst_node_t*;

  key_type m_key;
  value_type m_value;

  bst_node_t* m_left = nullptr;
  bst_node_t* m_right = nullptr;

  bst_node_t(const key_type& key, const value_type& value,
             pointer left = nullptr, pointer right = nullptr)
      : m_key(key), m_value(value), m_left(left), m_right(right) {}

  /* Deletes the subtree */
  ~bst_node_t() {
    delete m_left;
    delete m_right;
  }
};

template <class Key_, class Value_, class Compare_ = std::less<Key_>>
struct bs_tree_t {
  using node_type = bst_node_t<Key_, Value_>;
  using node_pointer = node_type*;
  using key_type = typename node_type::key_type;
  using value_type = typename node_type::value_type;
  using binary_func = Compare_;

  node_type* root = nullptr;

  /* Defines the ordering of the keys */
  binary_func less = binary_func();

  /* Deletes the root */
  ~bs_tree_t() { delete root; }

  /* Inserts a key-value pair in the tree.
   * If a previous node exists with the existing key, false is returned.
   * Else, the key-value pair is inserted and true is returned*/
  bool insert(const key_type& key, const value_type& value) noexcept {
    return insert_helper(root, key, value);
  }

  /* Performs an insert of a key-value pair on the subtree rooted at node.
   * Returns true if the insert was successful, and false on failure */
  bool insert_helper(node_pointer& node, const key_type& key,
                     const value_type& value) noexcept {
    if (!node) {
      node = new node_type(key, value);
      return true;
    }
    if (this->less(node->m_key, key)) {
      return insert_helper(node->m_right, key, value);
    }
    if (this->less(key, node->m_key)) {
      return insert_helper(node->m_left, key, value);
    }
    return false;
  }

  /* Writes the range containing all the values associated to the keys ranged
   * from [first, last] that exist on the tree into out. Returns the position of
   * out after the insertions were executed */
  template <class OutIter>
  OutIter between_range(const key_type& first, const key_type& last,
                        OutIter out) const noexcept {
    return range_helper(root, first, last, out);
  }

  /* Executes a range search from [first, last] on the subtree rooted at node.
   * Returns the position of out after all the insertions are executed. */
  template <class OutIter>
  OutIter range_helper(const node_type* node, const key_type& first,
                       const key_type& last, OutIter out) const noexcept {
    if (!node) return out;

    bool occupies_left = this->less(first, node->m_key);
    bool occupies_right = this->less(node->m_key, last);

    if (occupies_left) out = range_helper(node->m_left, first, last, out);
    if (occupies_left && occupies_right) *out++ = node->m_value;
    if (occupies_right) out = range_helper(node->m_right, first, last, out);

    return out;
  }
};

}  // namespace detail

/**
 * RangeBST implementation
 */
template <typename Point>
class RangeBST : public SpatialBase<Point> {
 private:
  detail::bs_tree_t<Point, Point, detail::point_less_equals<Point>> tree;

 public:
  RangeBST() = default;

  void insert(const Point& new_point) override {
    tree.insert(new_point, new_point);
  }

  // El punto de referencia no necesariamente es parte del dataset
  Point nearest_neighbor(const Point& reference) override { return Point({0}); }
  std::vector<Point> range(const Point& min, const Point& max) override {
    std::vector<Point> ret;
    tree.between_range(min, max, std::back_inserter(ret));
    return ret;
  };
};

}  // namespace spatial
}  // namespace utec
