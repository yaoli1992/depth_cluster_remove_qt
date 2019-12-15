

#include "utils/rich_point.h"

namespace depth_clustering {

RichPoint& RichPoint::operator=(const RichPoint& other) {
  if (this != &other) {  // self-assignment check expected
    _point = other.AsEigenVector();
    _ring = other.ring();
  }
  return *this;
}

RichPoint& RichPoint::operator=(const Eigen::Vector3f& other) {
  this->_point = other;
  return *this;
}

bool RichPoint::operator==(const RichPoint& other) const {
  return this->x() == other.x() && this->y() == other.y() &&
         this->z() == other.z() && this->ring() == other.ring();
}

}  // namespace depth_clustering
