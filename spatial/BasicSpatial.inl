#ifndef SPATIAL_IMPLEMENTATION_BASICSPATIAL_INL
#define SPATIAL_IMPLEMENTATION_BASICSPATIAL_INL

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

#endif  // SPATIAL_IMPLEMENTATION_BASICSPATIAL_INL
