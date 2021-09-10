#pragma once

#include "SpatialBase.h"

namespace utec {
namespace spatial {

/**
 * BasicSpatial implementation
 */
template <typename Point>
class BasicSpatial : public SpatialBase<Point> {
 private:
  std::vector<Point> values;
 public:
  BasicSpatial() = default;
  void insert(const Point& new_point) override;

  // El punto de referencia no necesariamente es parte del dataset
  Point nearest_neighbor(const Point& reference) override;
};


}  // namespace spatial
}  // namespace utec

#include <algorithm>

namespace utec {
namespace spatial {

template <typename Point>
void BasicSpatial<Point>::insert(const Point& new_point) {
  this->values.push_back(new_point);
}

template <typename Point>
Point BasicSpatial<Point>::nearest_neighbor(const Point& reference) {
  const auto closest_to_ref = [&reference](const Point& a, const Point& b) {
    return a.distance(reference) < b.distance(reference);
  };
  return *std::min_element(values.cbegin(), values.cend(), closest_to_ref);
}

}  // namespace spatial
}  // namespace utec

