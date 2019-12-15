#include "projections/spherical_projection.h"

#include <vector>

namespace depth_clustering {

void SphericalProjection::InitFromPoints( const RichPoint::AlignedVector& points) 
{
  this->CheckCloudAndStorage(points);
  for (size_t index = 0; index < points.size(); ++index) 
  {
    const auto& point = points[index];

    float dist_to_sensor = point.DistToSensor3D();

    if (dist_to_sensor < 0.01f) 
    {
      continue;
    }
    auto angle_rows = Radians::FromRadians(asin(point.z() / dist_to_sensor));
    auto angle_cols = Radians::FromRadians(atan2(point.y(), point.x()));
    size_t bin_rows = this->_params.RowFromAngle(angle_rows);
    size_t bin_cols = this->_params.ColFromAngle(angle_cols);
    
   // std::cout<< "bin_cols and bin_rows "<<bin_cols<< "  "<<bin_rows<<std::endl;

    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);

    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    
    if (current_written_depth < dist_to_sensor) 
    {
      // write this point to the image only if it is closer
      current_written_depth = dist_to_sensor;
    }
  }
//这一步是针对激光雷达数据每一圈数据的一个补偿，针对自己的数据，这里暂时可以不使用，
  //FixDepthSystematicErrorIfNeeded();
}

typename CloudProjection::Ptr SphericalProjection::Clone() const 
{
  return typename CloudProjection::Ptr(new SphericalProjection(*this));
}

}  // namespace depth_clustering
