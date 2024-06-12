/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 *********************************************************************/

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

#include "edtm_layer/spatio_temporal_voxel_grid.hpp"

namespace volume_grid
{

/*****************************************************************************/
SpatioTemporalVoxelGrid::SpatioTemporalVoxelGrid(
  rclcpp::Clock::SharedPtr clock,
  const float & voxel_size, const double & background_value,
  const int & decay_model, const double & voxel_decay, const bool & pub_voxels)
: _clock(clock), _decay_model(decay_model), _background_value(background_value),
  _voxel_size(voxel_size), _voxel_decay(voxel_decay), _pub_voxels(pub_voxels),
  _grid_points(std::make_unique<std::vector<geometry_msgs::msg::Point32>>()),
  _cost_map(new std::unordered_map<occupany_cell, uint>)
/*****************************************************************************/
{
  
}


void SpatioTemporalVoxelGrid::initEDTMap(std::string mapper_name_,  MappingParameters map_params){

  mp_ = map_params;
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  // mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_origin_ = Eigen::Vector3d(mp_.map_lef_bottom_x, mp_.map_lef_bottom_y, mp_.ground_height_);


  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  oct_tree = std::make_shared<octomap::OcTree>(mp_.resolution_);
  octomap::point3d max_(mp_.map_max_boundary_[0], mp_.map_max_boundary_[1], mp_.map_max_boundary_[2]);
  octomap::point3d min_(mp_.map_min_boundary_[0], mp_.map_min_boundary_[1], mp_.map_min_boundary_[2]);
  oct_tree->setBBXMax(max_);
  oct_tree->setBBXMin(min_);
  distmap = std::make_shared<DynamicEDTOctomap>(mp_.local_update_range_(0), oct_tree.get(), min_, max_, false);
  vec_obs_ = std::make_shared<vec_Vec3f>();

  free_seg_manager = std::make_shared<aoc_mapping::FreeSpaceSegmentManager>(mp_.free_space_car_radius, mp_.free_space_z_ground, mp_.free_segment_dim_);
  
  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  // pointcloud_processor = std::make_shared<std::thread>(&aoc_mapping::SpatioTemporalVoxelGrid::processPointCloud, this);

}


 int SpatioTemporalVoxelGrid::toAddress(const Eigen::Vector3i& id) {
    return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
  }

  
  int SpatioTemporalVoxelGrid::toAddress(int& x, int& y, int& z) {
    return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
  }

  
  void SpatioTemporalVoxelGrid::boundIndex(Eigen::Vector3i& id) {
    Eigen::Vector3i id1;
    id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
    id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
    id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
    id = id1;
  }


  
  bool SpatioTemporalVoxelGrid::isKnownOccupied(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    int adr = toAddress(id1);
    return md_.occupancy_buffer_inflate_[adr] == 1;
  }

  
  void SpatioTemporalVoxelGrid::setOccupied(Eigen::Vector3d pos) {
    if (!isInMap(pos)) return;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                  id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
  }

  
  int SpatioTemporalVoxelGrid::getInflateOccupancy(Eigen::Vector3d pos) {
    if (!isInMap(pos)) return -1;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    id[2] = 1;
    return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
  }

  int SpatioTemporalVoxelGrid::getInflateOccupancy(const Eigen::Vector3i& idx) {
    if (!isInMap(idx)) return -1;
    return int(md_.occupancy_buffer_inflate_[toAddress(idx)]);
  }

  
  bool SpatioTemporalVoxelGrid::isInMap(const Eigen::Vector3d& pos) {
    if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
        pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
      // cout << "less than min range!" << endl;
      // RCLCPP_INFO_STREAM(logger_, "pos " << pos.transpose()<< " mp_.map_min_boundary_" << mp_.map_min_boundary_);
      return false;
    }
    if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
        pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
          // RCLCPP_INFO_STREAM(logger_, "pos " << pos.transpose()<< " mp_.map_max_boundary_" << mp_.map_max_boundary_);
      return false;
    }
    return true;
  }

  
  bool SpatioTemporalVoxelGrid::isInMap(const Eigen::Vector3i& idx) {
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
      // RCLCPP_INFO_STREAM(logger_, "int pos " << idx.transpose()<< " mp_.map_min_boundary_" << mp_.map_min_boundary_);
      return false;
    }
    if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
        idx(2) > mp_.map_voxel_num_(2) - 1) {
          // RCLCPP_INFO_STREAM(logger_, "int pos " << idx.transpose()<< " mp_.map_voxel_num_" << mp_.map_voxel_num_);
      return false;
    }
    return true;
  }

  
  void SpatioTemporalVoxelGrid::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
  }

  
  void SpatioTemporalVoxelGrid::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
  }

  
  void SpatioTemporalVoxelGrid::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
    int num = 0;
    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z) {
          pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
        }
  }

  
double SpatioTemporalVoxelGrid::getResolution() {
   return mp_.resolution_; 
}


void SpatioTemporalVoxelGrid::getConvexSegmentsFreeSpace(const std::vector<geometry_msgs::msg::PoseStamped>& poses, 
        vec_E<Polyhedron<3>>& poly_whole, std::vector<LinearConstraint3D>& l_constraints_whole){
     
      // std::cout<< "------------num of onstcles----------->>>" << processed_obs_.size() << std::endl;
      free_seg_manager->setCloud(processed_obs_);
      free_seg_manager->setGlobalPath(poses);
      free_seg_manager->cvxEllipsoidDecomp(free_seg_manager->global_path_, l_constraints_whole, poly_whole);
}

std::vector<Eigen::Vector3d> SpatioTemporalVoxelGrid::nearestObstaclesToCurrentPose(Eigen::Vector3d x){
    
  std::vector<Eigen::Vector3d> neighbour_points;
    if(!isInMap(x)){
      std::cout<< "Point outside of the map" << x.transpose() << std::endl;
      return neighbour_points;
    }
    octomap::point3d p(x[0],x[1],x[2]);
    octomap::point3d closestObst;
    float distance;
    distmap->getDistanceAndClosestObstacle(p, distance, closestObst);
    Eigen::Vector3d obs(closestObst.x(), closestObst.y(), closestObst.z());
    neighbour_points.push_back(obs);
    return neighbour_points;
}

double SpatioTemporalVoxelGrid::getFreeDistance(Eigen::Vector3d x){
  octomap::point3d p(x[0],x[1],x[2]);
  double dis = distmap->getDistance(p);
  return dis;
}

void SpatioTemporalVoxelGrid::getCloseObstacle(Eigen::Vector3d x, Eigen::Vector3d& close_obs, double& dis){
  octomap::point3d p(x[0],x[1],x[2]);
  octomap::point3d closestObst;
  float distance;
  distmap->getDistanceAndClosestObstacle(p, distance, closestObst);
  if(distance <0){
      distance = 20;
      closestObst.x() = x[0]+100;
      closestObst.y() = x[1]+100;
      closestObst.z() = x[2]+100;
  }
  Eigen::Vector3d obs_pose(closestObst.x(), closestObst.y(), closestObst.z());
  close_obs = obs_pose;
  dis = distance;
}

void SpatioTemporalVoxelGrid::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SpatioTemporalVoxelGrid::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}


Eigen::Vector3d SpatioTemporalVoxelGrid::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}


bool SpatioTemporalVoxelGrid::odomValid() { return md_.has_odom_; }

bool SpatioTemporalVoxelGrid::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d SpatioTemporalVoxelGrid::getOrigin() { return mp_.map_origin_; }

// int SpatioTemporalVoxelGrid::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }
void SpatioTemporalVoxelGrid::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

/*****************************************************************************/
SpatioTemporalVoxelGrid::~SpatioTemporalVoxelGrid(void)
/*****************************************************************************/
{
  
}


/*****************************************************************************/
void SpatioTemporalVoxelGrid::ClearFrustums(
  const std::vector<observation::MeasurementReading> & clearing_readings,
  std::unordered_set<occupany_cell> & cleared_cells)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);
  _grid_points->clear();
  _cost_map->clear();

  std::vector<frustum_model> obs_frustums;
  if (clearing_readings.size() == 0) {
    TemporalClearAndGenerateCostmap(obs_frustums, cleared_cells);
    return;
  }

  obs_frustums.reserve(clearing_readings.size());

  std::vector<observation::MeasurementReading>::const_iterator it =
    clearing_readings.begin();
  for (; it != clearing_readings.end(); ++it) {
    geometry::Frustum * frustum;
    if (it->_model_type == DEPTH_CAMERA) {
      frustum = new geometry::DepthCameraFrustum(
        it->_vertical_fov_in_rad,
        it->_horizontal_fov_in_rad, it->_min_z_in_m, it->_max_z_in_m);
    } else if (it->_model_type == THREE_DIMENSIONAL_LIDAR) {
      frustum = new geometry::ThreeDimensionalLidarFrustum(
        it->_vertical_fov_in_rad, it->_vertical_fov_padding_in_m,
        it->_horizontal_fov_in_rad, it->_min_z_in_m, it->_max_z_in_m);
    } else {
      // add else if statement for each implemented model
      delete frustum;
      continue;
    }

    frustum->SetPosition(it->_origin);
    frustum->SetOrientation(it->_orientation);
    frustum->TransformModel();
    obs_frustums.emplace_back(frustum, it->_decay_acceleration);
  }
  TemporalClearAndGenerateCostmap(obs_frustums, cleared_cells);
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::TemporalClearAndGenerateCostmap(
  std::vector<frustum_model> & frustums,
  std::unordered_set<occupany_cell> & cleared_cells)
/*****************************************************************************/
{
  // sample time once for all clearing readings
  // const double cur_time = _clock->now().seconds();
}




std::vector<Eigen::Vector3d> SpatioTemporalVoxelGrid::getMapCurrentRange(){
    std::vector<Eigen::Vector3d> current_ranges;
    current_ranges.push_back(mp_.map_min_boundary_);
    current_ranges.push_back(mp_.map_max_boundary_);
    return current_ranges;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::operator()(const observation::MeasurementReading & obs) {

  if (obs._marking) {

    float mark_range_2 = obs._obstacle_range_in_m * obs._obstacle_range_in_m;
    const double cur_time = _clock->now().seconds();

    const sensor_msgs::msg::PointCloud2 & cloud = *(obs._cloud);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    mtx_point_cloud_.lock();
    processed_obs_ = *vec_obs_;
    mtx_point_cloud_.unlock();

    pcl::PointCloud<pcl::PointXYZ> mod_cloud;
    md_.has_cloud_ = true;

    this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_, md_.camera_pos_ + mp_.local_update_range_);
    pcl::PointXYZ pt;
    Eigen::Vector3d p3d, p3d_inf;

    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    int inf_step_z = 1;

    double max_x, max_y, max_z, min_x, min_y, min_z;

    min_x = mp_.map_max_boundary_(0);
    min_y = mp_.map_max_boundary_(1);
    min_z = mp_.map_max_boundary_(2);

    max_x = mp_.map_min_boundary_(0);
    max_y = mp_.map_min_boundary_(1);
    max_z = mp_.map_min_boundary_(2);

    oct_tree->clear();
    // // RCLCPP_INFO_STREAM(logger_, "Map bundary: min_x " << min_x << " min_y " << min_y << " min_z " << min_z << " max_x " << max_x << " max_y " << max_y << " max_z " << max_z); 
    vec_obs_->clear();
    md_.camera_pos_ = {obs._origin.x, obs._origin.y, obs._origin.z};
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
      float distance_2 =
        (*iter_x - obs._origin.x) * (*iter_x - obs._origin.x) +
        (*iter_y - obs._origin.y) * (*iter_y - obs._origin.y) +
        (*iter_z - obs._origin.z) * (*iter_z - obs._origin.z);

      if (distance_2 > mark_range_2 || distance_2 < 0.0001) {
        continue;
      }

      double x = *iter_x < 0 ? *iter_x - _voxel_size : *iter_x;
      double y = *iter_y < 0 ? *iter_y - _voxel_size : *iter_y;
      double z = *iter_y < 0 ? *iter_z - _voxel_size : *iter_z;
      
      p3d(0) = x, p3d(1) = y, p3d(2) = z;

      /* point inside update range */
      Eigen::Vector3d devi = p3d - md_.camera_pos_;
      Eigen::Vector3i inf_pt;
      if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) && fabs(devi(2)) < mp_.local_update_range_(2)){
          /* inflate the point */
          for (int x_i = -inf_step; x_i <= inf_step; ++x_i){
            for (int y_i = -inf_step; y_i <= inf_step; ++y_i){
                for (int z_i = -inf_step_z; z_i <= inf_step_z; ++z_i){
                  
                  p3d_inf(0) = x + x_i * mp_.resolution_;
                  p3d_inf(1) = y + y_i * mp_.resolution_;
                  p3d_inf(2) = z + z_i * mp_.resolution_;

                  max_x = max(max_x, p3d_inf(0));
                  max_y = max(max_y, p3d_inf(1));
                  max_z = max(max_z, p3d_inf(2));

                  min_x = min(min_x, p3d_inf(0));
                  min_y = min(min_y, p3d_inf(1));
                  min_z = min(min_z, p3d_inf(2));

                  posToIndex(p3d_inf, inf_pt);
                  inf_pt[2] = 1; //TODO need to set this properlly 

                  if (!isInMap(inf_pt)){
                    continue;
                  }

                  // std::cout<< "====== adding points to clouds " << inf_pt.transpose() << std::endl;

                  int idx_inf = toAddress(inf_pt);
                  md_.occupancy_buffer_inflate_[idx_inf] = 1;
                  octomap::point3d endpoint(p3d_inf(0), p3d_inf(1), p3d_inf(2));
                  Vec3f next_point = {p3d_inf(0), p3d_inf(1), p3d_inf(2)};
                  vec_obs_->push_back(next_point);
                  oct_tree->updateNode(endpoint, true);
                  pcl::PointXYZ pt = {(float)p3d_inf(0), (float)p3d_inf(1), (float)p3d_inf(2)};
                  mod_cloud.push_back(pt);
              }
            }
          }
      }

    }

    distmap->update();

    min_x = min(min_x, md_.camera_pos_(0));
    min_y = min(min_y, md_.camera_pos_(1));
    min_z = min(min_z, md_.camera_pos_(2));

    max_x = max(max_x, md_.camera_pos_(0));
    max_y = max(max_y, md_.camera_pos_(1));
    max_z = max(max_z, md_.camera_pos_(2));

    max_z = max(max_z, mp_.ground_height_);

    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    // RCLCPP_INFO_STREAM(logger_, " md_.occupancy_buffer_inflate_  " << cloud.points.size());
    mod_cloud.width = mod_cloud.points.size();
    mod_cloud.height = 1;
    mod_cloud.is_dense = true;
    mod_cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(mod_cloud, cloud_msg);
    map_inf_pub_->publish(cloud_msg);
  }
}

/*****************************************************************************/
std::unordered_map<occupany_cell, uint> *
SpatioTemporalVoxelGrid::GetFlattenedCostmap()
/*****************************************************************************/
{
  return _cost_map;
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetTemporalClearingDuration(
  const double & time_delta)
/*****************************************************************************/
{
  // use configurable model to get desired decay time
  if (_decay_model == 0) {  // Linear
    return _voxel_decay - time_delta;
  } else if (_decay_model == 1) {  // Exponential
    return _voxel_decay * std::exp(-time_delta);
  }
  return _voxel_decay;  // PERSISTENT
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetFrustumAcceleration(
  const double & time_delta, const double & acceleration_factor)
/*****************************************************************************/
{
  const double acceleration = 1. / 6. * acceleration_factor *
    (time_delta * time_delta * time_delta);
  return acceleration;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::GetOccupancyPointCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & pc2)
/*****************************************************************************/
{
  // convert the grid points stored in a PointCloud2
  pc2->width = _grid_points->size();
  pc2->height = 1;
  pc2->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*pc2);

  modifier.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2, "z");

  for (std::vector<geometry_msgs::msg::Point32>::iterator it =
    _grid_points->begin();
    it != _grid_points->end(); ++it)
  {
    const geometry_msgs::msg::Point32 & pt = *it;
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    ++iter_x; ++iter_y; ++iter_z;
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::ResetGrid(void)
/*****************************************************************************/
{
  // boost::unique_lock<boost::mutex> lock(_grid_lock);
  return false;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::ResetGridArea(
  const occupany_cell & start, const occupany_cell & end, bool invert_area)
/*****************************************************************************/
{
  // boost::unique_lock<boost::mutex> lock(_grid_lock);
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::Mark(
  const std::vector<observation::MeasurementReading> & marking_readings)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // mark the grid
  if (marking_readings.size() > 0) {
    for (uint i = 0; i != marking_readings.size(); i++) {
        (*this)(marking_readings.at(i));
    }
  }
}



}  // namespace volume_grid
