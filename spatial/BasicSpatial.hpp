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

#include "BasicSpatial.inl"
