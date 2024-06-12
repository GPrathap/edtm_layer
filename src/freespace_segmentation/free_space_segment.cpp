#include <edtm_layer/freespace_segmentation/free_space_segment.hpp>


namespace aoc_mapping {

    FreeSpaceSegmentManager::FreeSpaceSegmentManager(double car_radius, double z_ground, std::vector<double> free_segment_dim){
        free_segment_dim_ = free_segment_dim;
        z_ground_ = z_ground;
        car_radius_ = car_radius;
        // RCLCPP_INFO_STREAM(get_logger(), "free segment dim xyz " << free_segment_dim_[0] << "," << free_segment_dim_[1] << "," << free_segment_dim_[2]);
        std::cout<< "Free segment dim xyz " << free_segment_dim_[0] << "," << free_segment_dim_[1] << "," << free_segment_dim_[2] << std::endl;
    }

    void FreeSpaceSegmentManager::setGlobalPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses){
        global_path_.clear();
        double tolerance_ = free_segment_dim_[0]/2.0;
        double dis = 0.0;
        Vecf<3> previous_pose;
        if(poses.size()>0){
            previous_pose = {poses[0].pose.position.x, poses[0].pose.position.y, 1.0};
            global_path_.emplace_back(previous_pose);
        }
        for(auto pose : poses){
            Vecf<3> next_pose = {pose.pose.position.x, pose.pose.position.y, 1.0};
            dis += (previous_pose.col(0) - next_pose.col(0)).norm();
            if(dis > tolerance_){
                global_path_.emplace_back(next_pose);
                dis = 0;
            }
            previous_pose = next_pose;
        }
        if(global_path_.size() < 2 && poses.size()>0){
            global_path_.clear();
            for(auto pose : poses){
                Vecf<3> next_pose = {pose.pose.position.x, pose.pose.position.y, 1.0};
                global_path_.emplace_back(next_pose);
            }
        }
    }

    void FreeSpaceSegmentManager::setCloud( vec_Vec3f& poses){
        vec_o_ = poses;
    }

    void FreeSpaceSegmentManager::pclPtrToVec(const pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud){
        mtx_map.lock();
        vec_o_.clear();
        for (unsigned int i = 0; i < ptr_cloud->points.size(); i++)
        {
            if(std::isinf(ptr_cloud->points[i].x) || std::isinf(ptr_cloud->points[i].y) || std::isinf(ptr_cloud->points[i].z)){
                continue;
            }
            Vec3f point = {ptr_cloud->points[i].x, ptr_cloud->points[i].y, ptr_cloud->points[i].z};
            vec_o_.push_back(point);
        }
        // std::cout<< "num of points in the filtered cloud " << vec_o_.size() << std::endl;
        mtx_map.unlock();
    }

    void FreeSpaceSegmentManager::clear_cloud(){
        mtx_map.lock();
        vec_o_.clear();
        mtx_map.unlock();
    }

    void FreeSpaceSegmentManager::add_to_cloud(Vec3f& point){
        // mtx_map.lock();
        vec_o_.push_back(point);
        // mtx_map.unlock();
    }


    bool FreeSpaceSegmentManager::cvxEllipsoidDecomp(vec_Vecf<3>& path
                , std::vector<LinearConstraint3D>& l_constraints, vec_E<Polyhedron<3>>& poly_out){
        
        if(path.size()<2){
            // RCLCPP_INFO_STREAM(get_logger(), "reference path is not set yet, no free space estimation ..." );
            std::cout<< "Reference path is not set yet, no free space estimation ..." << std::endl;
            return false;
        }
        // mtx_map.lock();
        
        ellip_decomp_util_.set_obs(vec_o_);
        ellip_decomp_util_.set_local_bbox(Vec3f(free_segment_dim_[0], free_segment_dim_[1], free_segment_dim_[2]));  // Only try to find cvx decomp in the Mikowsski sum of JPS and this box (I think) par_.drone_radius
        ellip_decomp_util_.set_inflate_distance(car_radius_);  // The obstacles are inflated by this distance
        ellip_decomp_util_.dilate(path);                         // Find convex polyhedra
        //  mtx_map.unlock();
        // decomp_util.shrink_polyhedrons(par_.drone_radius);  // Shrink polyhedra by the drone radius. NOT RECOMMENDED (leads
        // to lack of continuity in path sometimes)

        // Convert to inequality constraints Ax < b
        // std::vector<polytope> polytopes;
        auto polys = ellip_decomp_util_.get_polyhedrons();
        l_constraints.clear();

        for (size_t i = 0; i < path.size() - 1; i++)
        {
            const auto pt_inside = (path[i] + path[i + 1]) / 2;
            LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());

            // Now add the constraint "Above the ground:"
            cs.A_.conservativeResize(cs.A_.rows() + 1, cs.A_.cols());
            cs.A_.row(cs.A_.rows() - 1) = -Eigen::Vector3d::UnitZ();
            cs.b_.conservativeResize(cs.b_.rows() + 1, cs.b_.cols());
            cs.b_[cs.b_.rows() - 1] = -z_ground_;
            l_constraints.push_back(cs);
        }
        poly_out = ellip_decomp_util_.get_polyhedrons();
        return true;
    }
}