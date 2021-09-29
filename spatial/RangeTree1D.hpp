#pragma once

#include <algorithm>
#include <cassert>

#include "SpatialBase.h"

namespace detail {
template <class Key_, class Value_>
struct range_tree_node_t;

template <class Key_, class Value_>
struct range_tree_leaf_t;

template <class Key_, class Value_>
struct range_tree_inner_t;

template <class Key_, class Value_>
struct range_tree_node_t {
  using key_type = Key_;
  using value_type = Value_;
  using size_type = ssize_t;
  using difference_type = long;

  using base_type = range_tree_node_t;
  using leaf_type = range_tree_leaf_t<Key_, Value_>;
  using inner_type = range_tree_inner_t<Key_, Value_>;

  key_type m_key;
  size_type m_height;

  /* Calculates the height of the tree rooted at node.
   * Returns -1 if the node is nullptr.
   * Returns 0 if the node is a leaf
   * Returns >0 if the node is an inner node */
  [[nodiscard]] static size_type height(const base_type* node) noexcept {
    return !node ? -1 : node->m_height;
  }

  /* Determines if a node is a leaf (height of the node is 0) */
  [[nodiscard]] static bool is_leaf(const base_type* node) noexcept {
    return height(node) == 0;
  }

  /* Determines if a node is an inner node (height of the node is >0) */
  [[nodiscard]] static bool is_inner(const base_type* node) noexcept {
    return height(node) > 0;
  }

  /* Executes the destruction of the subtree rooted at node */
  static void delete_node(base_type* node) noexcept {
    if (!is_leaf(node))
      delete static_cast<inner_type*>(node);
    else
      delete static_cast<leaf_type*>(node);
  }
};

template <class Key_, class Value_>
struct range_tree_leaf_t : range_tree_node_t<Key_, Value_> {
  using base = range_tree_node_t<Key_, Value_>;

  using base_type = typename base::base_type;
  using value_type = typename base::value_type;
  using key_type = typename base::key_type;

  using leaf_type = typename base::leaf_type;
  using inner_type = typename base::inner_type;

  value_type m_value;

  range_tree_leaf_t(const key_type& key, const value_type& value)
      : base_type{key, 0}, m_value{value} {}
};

template <class Key_, class Value_>
struct range_tree_inner_t : range_tree_node_t<Key_, Value_> {
  using base = range_tree_node_t<Key_, Value_>;

  using base_type = typename base::base_type;
  using key_type = typename base::key_type;
  using size_type = typename base::size_type;
  using difference_type = typename base::difference_type;
  using leaf_type = typename base::leaf_type;
  using inner_type = typename base::inner_type;

  base_type* p_left;
  base_type* p_right;

  explicit range_tree_inner_t(const key_type& key, base_type* left = nullptr,
                              base_type* right = nullptr)
      : base{key, height_if_roots(left, right)}, p_left{left}, p_right{right} {}

  ~range_tree_inner_t() {
    base::delete_node(p_right);
    base::delete_node(p_left);
  }

  /* Changes the left child of the node and updates the node's height. */
  void set_left(base_type* left) {
    this->p_left = left;
    this->m_height = height_if_roots(p_left, p_right);
  }

  /* Changes the right child of the node and updates the node's height. */
  void set_right(base_type* right) {
    this->p_right = right;
    this->m_height = height_if_roots(p_left, p_right);
  }

  /* Returns the node after attempting a re-balancing operation.
   * If no re-balancing is needed, the node is returned as-is.
   * Else, the new root of the rebalanced subtree is returned. */
  base_type* rebalanced() {
    if (this->balance_factor() > 1) {
      return this->rotate_left();
    }
    if (this->balance_factor() < -1) {
      return this->rotate_right();
    }
    return this;
  }

  /* Performs a left rotation on the node. Returns the new root of the
   * subtree.*/
  base_type* rotate_left() {
    assert(this->p_right != nullptr && base::is_inner(this->p_right));

    auto* old_root = this;
    auto* new_root = static_cast<inner_type*>(this->p_right);
    auto* moved = new_root->p_left;

    old_root->set_right(moved);
    new_root->set_left(old_root);
    return new_root;
  }

  /* Performs a right rotation on the node. Returns the new root of the
   * subtree.*/
  base_type* rotate_right() {
    assert(this->p_left != nullptr && base::is_inner(this->p_left));

    auto* old_root = this;
    auto* new_root = static_cast<inner_type*>(this->p_left);
    auto* moved = new_root->p_right;

    old_root->set_left(moved);
    new_root->set_right(old_root);

    return new_root;
  }

  /* Returns the balance factor of the node.
   * If the node is perfectly balanced, returns 0
   * If the node is slightly skewed, abs(balance_factor) == 1
   * If the node should be rebalanced, abs(balance_factor) == 2 */
  [[nodiscard]] difference_type balance_factor() const noexcept {
    auto ans = (difference_type)base::height(this->p_right) -
               (difference_type)base::height(this->p_left);
    return ans;
  }

  /* Returns the height of a subtree that would root both left and right*/
  [[nodiscard]] static size_type height_if_roots(
      const base_type* left, const base_type* right) noexcept {
    return 1 + std::max(base::height(left), base::height(right));
  }
};
template <class Key_, class Value_, class Compare_ = std::less<Key_>>
    struct range_tree {
      using node_type = detail::range_tree_node_t<Key_, Value_>;

      using key_type = typename node_type::key_type;
      using value_type = typename node_type::value_type;
      using leaf_type = typename node_type::leaf_type;
      using inner_type = typename node_type::inner_type;

      using binary_fn = Compare_;
      //    using range_container_type = std::vector<value_type>;
      using node_ptr = node_type*;

      using const_node = const node_type;
      using const_leaf = const leaf_type;
      using const_inner = const inner_type;

      node_type* root = nullptr;
      binary_fn less = binary_fn();

      ~range_tree() { node_type::delete_node(root); }

      /* Inserts a key-value pair in the tree.
       * If a previous node exists with the existing key, false is returned.
       * Else, the key-value pair is inserted and true is returned*/
      bool insert(const key_type& key, const value_type& value) {
        // handle the base case
        if (this->root == nullptr) {
          this->root = new leaf_type(key, value);
          return true;
        }

        auto insert_success = insert_helper(this->root, key, value);
        if (!insert_success) return false;

        this->root = insert_success;
        return true;
      }

      /* Performs an insert of a key-value pair on the subtree rooted at node.
       * Returns the root of the subtree that contains the inserted node, or nullptr
       * on failure. */
      node_ptr insert_helper(node_ptr node, const key_type& key,
                             const value_type& value) noexcept {
        assert(node);
        auto node_key = node->m_key;
        if (node_type::is_leaf(node)) {
          // try to create a new inner node with the greatest key at its left
          if (this->less(key, node_key)) {
            return new inner_type(key, new leaf_type(key, value), node);
          }
          if (this->less(node_key, key)) {
            return new inner_type(node_key, node, new leaf_type(key, value));
          }
          // the key already exists
          return nullptr;
        }
        // else, it's an inner node
        auto inner_root = static_cast<inner_type*>(node);

        // try to insert and re-balance the tree
        if (this->less(node_key, key)) {
          auto ans = insert_helper(inner_root->p_right, key, value);
          if (!ans) return nullptr;

          inner_root->set_right(ans);
          return inner_root->rebalanced();
        }
        if (this->less(key, node_key)) {
          auto ans = insert_helper(inner_root->p_left, key, value);
          if (!ans) return nullptr;

          inner_root->set_left(ans);
          return inner_root->rebalanced();
        }
        return nullptr;
      }

      /* Returns a vector containing all the values associated to the keys ranged
       * from [first, last] that exist on the tree. */
      template <class OIter>
          OIter range_between(const key_type& first, const key_type& last,
                              OIter out) const noexcept {
            const_node* split = find_split(root, first, last);
            if (node_type::is_leaf(split)) {
              auto casted_node = static_cast<const_leaf*>(split);
              if (!this->less(casted_node->m_key, first) &&
              !this->less(last, casted_node->m_key)) {
                *out++ = casted_node->m_value;
              }
            } else if (node_type::is_inner(split)) {
              auto casted_node = static_cast<const_inner*>(split);

              range_left_helper(casted_node->p_left, first, out);
              range_right_helper(casted_node->p_right, last, out);
            }
            return out;
          }

          /* Returns the root of the lowest subtree that contains the range [first,
           * last] In case the subtree doesn't exist, or if root is already nullptr,
           * nullptr is returned */
          const_node* find_split(const_node* node, const key_type& first,
                                 const key_type& last) const noexcept {
            if (node_type::is_leaf(node)) {
              return node;
            } else if (node_type::is_inner(node)) {
              auto casted_node = static_cast<const_inner*>(node);
              bool first_on_left = !this->less(casted_node->m_key, first);
              bool last_on_right = this->less(casted_node->m_key, last);

              if (first_on_left && last_on_right) {
                return casted_node;
              } else if (!first_on_left && last_on_right) {
                return find_split(casted_node->p_right, first, last);
              } else if (first_on_left /* last is at the right*/) {
                return find_split(casted_node->p_left, first, last);
              } else
                return nullptr;
            } else
              return nullptr;
          }

          /* Executes a range search from [first, node.key] and stores the result into
           * *out */
          /* Returns the position of out after all the insertions are executed. */
          template <class OIter>
              OIter range_left_helper(const_node* node, const key_type& first,
                                      OIter out) const {
                if (node_type::is_leaf(node)) {
                  auto casted_node = static_cast<const_leaf*>(node);
                  if (!this->less(casted_node->m_key, first)) *out++ = casted_node->m_value;
                  return out;
                }
                // else, it's an inner node
                auto casted_node = static_cast<const_inner*>(node);
                if (!this->less(casted_node->m_key, first)) {
                  out = range_left_helper(casted_node->p_left, first, out);
                  return report_all(casted_node->p_right, out);
                } else {
                  return range_left_helper(casted_node->p_right, first, out);
                }
              }

              /* Executes a range search from (node.key, last] and stores the result into
               * *out */
              /* Returns the position of out after all the insertions are executed. */
              template <class OutputIter>
                  OutputIter range_right_helper(const_node* node, const key_type& last,
                                                OutputIter out) const {
                    if (node_type::is_leaf(node)) {
                      auto casted_node = static_cast<const_leaf*>(node);
                      if (!this->less(last, casted_node->m_key)) {
                        *out++ = casted_node->m_value;
                      }
                      return out;
                    };
                    // else, it's an inner node
                    auto casted_node = static_cast<const_inner*>(node);
                    if (this->less(casted_node->m_key, last)) {
                      out = report_all(casted_node->p_left, out);
                      return range_right_helper(casted_node->p_right, last, out);
                    } else {
                      return range_right_helper(casted_node->p_left, last, out);
                    }
                  }

                  /* Stores all the values of the subtree rooted at node into *out.
                   * Returns the position of out after all the insertions are executed */
                  template <class OutputIter>
                      OutputIter report_all(const_node* node, OutputIter out) const {
                        assert(node);
                        if (node_type::is_leaf(node)) {
                          *out++ = static_cast<const leaf_type*>(node)->m_value;
                          return out;
                        }
                        auto casted_node = static_cast<const inner_type*>(node);
                        auto end_pos = report_all(casted_node->p_left, out);
                        return report_all(casted_node->p_right, end_pos);
                      }
    };

}  // namespace detail



namespace utec {
namespace spatial {

template <class Point>
struct point_less_equals {
  constexpr bool operator () (const Point& lhs, const Point& rhs) const {
    return !(rhs < lhs);
  }
};

/**
 * RangeTree1D implementation
 */
template <typename Point>
class RangeTree1D : public SpatialBase<Point> {
 private:
  detail::range_tree<Point, Point, point_less_equals<Point>> tree;

 public:
  RangeTree1D(){};
  void insert(const Point& new_point) override {
    tree.insert(new_point, new_point);
  }

  // El punto de referencia no necesariamente es parte del dataset
  Point nearest_neighbor(const Point& reference) override { return Point({0}); }
  std::vector<Point> range(const Point& min, const Point& max) override {
    std::vector<Point> ret;
    tree.range_between(min, max, std::back_inserter(ret));
    return ret;
  };
};

}  // namespace spatial
}  // namespace utec
