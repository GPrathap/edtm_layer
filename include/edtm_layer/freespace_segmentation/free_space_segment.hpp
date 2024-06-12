#ifndef FREE_SPACE_SEGMENT_MANAGER_
#define FREE_SPACE_SEGMENT_MANAGER_
#include <vector>
#include <iostream>
#include <cmath>
#include <cassert>
#include <ctime>

#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <decomp_rviz_plugins/data_ros_utils.hpp>
#include <decomp_basis/data_type.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
// #include <tools/utility.hpp>


namespace aoc_mapping {

class FreeSpaceSegmentManager{
 public:
    FreeSpaceSegmentManager(double car_radius, double z_ground, std::vector<double> free_segment_dim);
    ~FreeSpaceSegmentManager() = default;
   
   void pclPtrToVec(const pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud);
   // Convex Decomposition
   bool cvxEllipsoidDecomp(vec_Vecf<3>& path, std::vector<LinearConstraint3D>& l_constraints,
                          vec_E<Polyhedron<3>>& poly_out);
   void setGlobalPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses);
   typedef std::shared_ptr<FreeSpaceSegmentManager> Ptr;
  vec_Vecf<3> global_path_;

  void clear_cloud();
  void add_to_cloud(Vec3f& point);
  void setCloud(vec_Vec3f& poses);

 private:
  vec_Vec3f vec_o_;
  EllipsoidDecomp3D ellip_decomp_util_;
  double car_radius_, z_ground_;
  std::vector<double> free_segment_dim_;
  std::mutex mtx_map;
  
//   int cells_x_, cells_y_, cells_z_;
//   bool visual_;

};

}
#endif //FREE_SPACE_SEGMENT_MANAGER_
