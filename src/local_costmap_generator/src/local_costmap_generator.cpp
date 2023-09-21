#include "local_costmap_generator/local_costmap_generator.hpp"

namespace local_costmap_generator {

LocalCostmapGenerator::LocalCostmapGenerator()
    : nh_(""), private_nh_("~"), tf_listener_(tf_buffer_), cost_map_(std::vector<std::string>({collision_layer_name_})) {
    // set parameter from ros parameter server
    private_nh_.param("robot_frame_id", robot_frame_id_, static_cast<std::string>("base_link"));
    private_nh_.param("sensor_frame_id", sensor_frame_id_, static_cast<std::string>("laser"));
    std::string in_scan_topic;
    std::string out_costmap_topic;
    private_nh_.param("in_scan_topic", in_scan_topic, static_cast<std::string>("scan"));
    private_nh_.param("out_costmap_topic", out_costmap_topic, static_cast<std::string>("local_costmap"));

    // common params
    private_nh_.param("update_rate", update_rate_, 0.01);
    private_nh_.param("thread_num", thread_num_, 4);

    // preprocess params
    private_nh_.param("is_crop_robot", is_crop_robot_, true);
    private_nh_.param("is_remove_outlier", is_remove_outlier_, false);
    private_nh_.param("sor_mean_k", sor_mean_k_, 10);
    private_nh_.param("sor_stddev_mul_thresh", sor_stddev_mul_thresh_, 1.0);
    private_nh_.param("is_downsample", is_downsample_, false);
    private_nh_.param("downsample_resolution", downsample_resolution_, 0.1);
    private_nh_.param("is_pass_through", is_pass_through_, false);
    private_nh_.param("pass_through_min_from_robot", pass_through_min_from_robot_, 0.0);
    private_nh_.param("pass_through_max_from_robot", pass_through_max_from_robot_, 2.0);

    // rigid body shape params
    private_nh_.param("baselink2front", rigid_body_shape_.baselink2front, 0.47);
    private_nh_.param("baselink2rear", rigid_body_shape_.baselink2rear, 0.14);
    private_nh_.param("baselink2right", rigid_body_shape_.baselink2right, 0.15);
    private_nh_.param("baselink2left", rigid_body_shape_.baselink2left, 0.15);

    // costmap params
    double map_x_length = 10.0;
    double map_y_length = 10.0;
    double map_center_offset_x = 3.0;
    double map_center_offset_y = 0.0;
    double map_resolution = 0.1;
    private_nh_.param("map_x_length", map_x_length, 10.0);
    private_nh_.param("map_y_length", map_y_length, 10.0);
    private_nh_.param("map_center_offset_x", map_center_offset_x, 3.0);
    private_nh_.param("map_center_offset_y", map_center_offset_y, 0.0);
    private_nh_.param("map_resolution", map_resolution, 0.1);
    private_nh_.param("max_val", max_val_, 100.0);

    // initialize inner variables
    cost_map_.setFrameId(robot_frame_id_);
    cost_map_.setGeometry(grid_map::Length(map_x_length, map_y_length), map_resolution, grid_map::Position(map_center_offset_x, map_center_offset_y));
    raw_pc2_ptr_ = boost::make_shared<sensor_msgs::PointCloud2>();
    raw_pcl_ptr_ = boost::make_shared<pcl::PointCloud<PointType>>();
    preprocessed_pcl_ptr_ = boost::make_shared<pcl::PointCloud<PointType>>();
    trans_preprocessed_pcl_ptr_ = boost::make_shared<pcl::PointCloud<PointType>>();

    // crop box filter
    const double min_high = 0.0;
    const double max_high = 10.0;
    const double min_x = -rigid_body_shape_.baselink2rear;
    const double max_x = rigid_body_shape_.baselink2front;
    const double min_y = -rigid_body_shape_.baselink2right;
    const double max_y = rigid_body_shape_.baselink2left;
    crop_box_min_ = Eigen::Vector4f(min_x, min_y, min_high, 1.0);
    crop_box_max_ = Eigen::Vector4f(max_x, max_y, max_high, 1.0);

    // set publishers and subscribers
    timer_costmap_ = nh_.createTimer(ros::Duration(update_rate_), &LocalCostmapGenerator::timer_callback, this);
    pub_cost_map_ = nh_.advertise<grid_map_msgs::GridMap>(out_costmap_topic, 1, true);
    pub_rigid_body_shape_ = nh_.advertise<visualization_msgs::MarkerArray>("local_costmap/rigid_body_shape", 1, true);
    sub_scan_ = nh_.subscribe(in_scan_topic, 1, &LocalCostmapGenerator::scan_callback, this);

    // publish rigid body shape
    publish_rigid_body_shape(rigid_body_shape_);

    // debug
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("preprocessed_cloud", 1, true);
    // pub_calc_time_ = nh_.advertise<std_msgs::Float32>("local_costmap/calc_time", 1, true);
}

void LocalCostmapGenerator::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // convert Laser scan to point cloud
    projector_.projectLaser(*scan, *raw_pc2_ptr_);

    // convert pointcloud2 to pcl
    pcl::fromROSMsg(*raw_pc2_ptr_, *raw_pcl_ptr_);

    is_laser_scan_received_ = true;
}

void LocalCostmapGenerator::timer_callback([[maybe_unused]] const ros::TimerEvent& te) {
    /* Status check */
    if (!is_laser_scan_received_) {
        ROS_WARN_THROTTLE(5.0, "[LocalCostmapGenerator] Waiting for laser scan data...");
        return;
    }

    // ======== time measurement ========
    // const auto start_time = std::chrono::system_clock::now();

    // preprocess point cloud
    preprocess_pointcloud(raw_pcl_ptr_, preprocessed_pcl_ptr_);

    // transform coordinate from sensor frame to robot frame
    try {
        transform_stamped_ = tf_buffer_.lookupTransform(robot_frame_id_, sensor_frame_id_, ros::Time(0));
    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(3.0, "[LocalCostmapGenerator] %s", ex.what());
        return;
    }
    const Eigen::Isometry3d transform_matrix = tf2::transformToEigen(transform_stamped_.transform);
    pcl::transformPointCloud(*preprocessed_pcl_ptr_, *trans_preprocessed_pcl_ptr_, transform_matrix.matrix().cast<float>());

    // remove points within the robot
    if (is_crop_robot_) {
        crop_points_within_robot(trans_preprocessed_pcl_ptr_);
    }

    // convert to grid map
    const std::vector<grid_map::Index> occupied_indices = pointcloud_to_costmap(trans_preprocessed_pcl_ptr_, &cost_map_);

    // inflate cost map with rigid body for collision avoidance
    inflate_rigid_body(collision_layer_name_, occupied_indices, &cost_map_);

    // ======== time measurement ========
    // const auto end_time = std::chrono::system_clock::now();
    // const auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000000.0;
    // std_msgs::Float32 calc_time_msg;
    // calc_time_msg.data = elapsed_time;
    // pub_calc_time_.publish(calc_time_msg);

    // publish cost map
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(cost_map_, message);
    message.info.header.stamp = ros::Time::now();
    pub_cost_map_.publish(message);

    // debug
    // Publish point preprocessed could
    sensor_msgs::PointCloud2 preprocessed_pc2;
    pcl::toROSMsg(*trans_preprocessed_pcl_ptr_, preprocessed_pc2);
    preprocessed_pc2.header.frame_id = robot_frame_id_;
    preprocessed_pc2.header.stamp = ros::Time::now();
    pub_cloud_.publish(preprocessed_pc2);
}

/**
 * @brief preprocess_pointcloud
 * @param[in] raw_pcl_ptr_ raw point cloud in sensor frame
 * @param[out] preprocessed_pcl_ptr_ preprocessed point cloud in sensor frame
 */
void LocalCostmapGenerator::preprocess_pointcloud(const pcl::PointCloud<PointType>::ConstPtr& raw_pcl_ptr_,
                                                  pcl::PointCloud<PointType>::Ptr& preprocessed_pcl_ptr_) {
    // NAN remove
    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*raw_pcl_ptr_, *preprocessed_pcl_ptr_, *indices);

    // outlier remove
    if (is_remove_outlier_) {
        sor_.setInputCloud(preprocessed_pcl_ptr_);
        sor_.setMeanK(sor_mean_k_);
        sor_.setStddevMulThresh(sor_stddev_mul_thresh_);
        sor_.filter(*preprocessed_pcl_ptr_);
    }

    // downsample
    if (is_downsample_) {
        voxel_grid_filter_.setInputCloud(preprocessed_pcl_ptr_);
        voxel_grid_filter_.setLeafSize(downsample_resolution_, downsample_resolution_, downsample_resolution_);
        voxel_grid_filter_.filter(*preprocessed_pcl_ptr_);
    }

    // pass through filter to remove too near or too far points
    if (is_pass_through_) {
        pass_through_filter_.setInputCloud(preprocessed_pcl_ptr_);
        pass_through_filter_.setFilterFieldName("z");
        pass_through_filter_.setFilterLimits(pass_through_min_from_robot_, pass_through_max_from_robot_);
        pass_through_filter_.filter(*preprocessed_pcl_ptr_);
    }
}

void LocalCostmapGenerator::crop_points_within_robot(pcl::PointCloud<PointType>::Ptr& pcl_ptr_) {
    crop_box_filter_.setInputCloud(pcl_ptr_);
    crop_box_filter_.setNegative(true);
    crop_box_filter_.setMin(crop_box_min_);
    crop_box_filter_.setMax(crop_box_max_);
    // Eigen::Vector4f min_point, max_point;
    // double min_high = 0.0;
    // double max_high = 2.0;
    // double min_x = -rigid_body_shape_.baselink2rear;
    // double max_x = rigid_body_shape_.baselink2front;
    // double min_y = -rigid_body_shape_.baselink2right;
    // double max_y = rigid_body_shape_.baselink2left;
    // min_point << min_x, min_y, min_high, 1.0;
    // max_point << max_x, max_y, max_high, 1.0;

    crop_box_filter_.filter(*pcl_ptr_);
}

std::vector<grid_map::Index> LocalCostmapGenerator::pointcloud_to_costmap(const pcl::PointCloud<PointType>::ConstPtr& preprocessed_pcl_ptr,
                                                                          grid_map::GridMap* cost_map) const

{
    grid_map::Matrix& cost_map_data = cost_map->get(collision_layer_name_);
    // costmap clear
    cost_map_data.setZero();
    std::vector<grid_map::Index> occupied_indices(preprocessed_pcl_ptr->points.size());

#pragma omp parallel for num_threads(thread_num_)
    for (unsigned int i = 0; i < preprocessed_pcl_ptr->points.size(); ++i) {
        const auto& point = preprocessed_pcl_ptr->points[i];
        if (cost_map->isInside(grid_map::Position(point.x, point.y))) {
            grid_map::Index index;
            cost_map->getIndex(grid_map::Position(point.x, point.y), index);
            cost_map_data(index.x(), index.y()) = max_val_;
            occupied_indices[i] = index;
        } else {
            occupied_indices[i] = grid_map::Index(-1, -1);
        }
    }

    // remove index (-1, -1) which means outside of costmap
    occupied_indices.erase(std::remove_if(occupied_indices.begin(), occupied_indices.end(),
                                          [](const grid_map::Index& index) { return index.x() == -1 && index.y() == -1; }),
                           occupied_indices.end());

    return occupied_indices;
}

// Inflate costmap for rigid body
void LocalCostmapGenerator::inflate_rigid_body(const std::string& layer_name,
                                               const std::vector<grid_map::Index>& occupied_indices,
                                               grid_map::GridMap* cost_map) const {
    grid_map::Matrix& cost_map_data = cost_map->get(layer_name);

    const int front_offset = static_cast<int>(std::ceil(rigid_body_shape_.baselink2front / cost_map->getResolution()));
    const int rear_offset = static_cast<int>(std::ceil(rigid_body_shape_.baselink2rear / cost_map->getResolution()));
    const int right_offset = static_cast<int>(std::ceil(rigid_body_shape_.baselink2right / cost_map->getResolution()));
    const int left_offset = static_cast<int>(std::ceil(rigid_body_shape_.baselink2left / cost_map->getResolution()));

    const grid_map::Size map_size = cost_map->getSize();

// inflate costmap
#pragma omp parallel for num_threads(thread_num_)
    for (size_t i = 0; i < occupied_indices.size(); ++i) {
        const auto index = occupied_indices[i];
        const int start_y = std::max(index.y() - right_offset, 0);
        const int last_y = std::min(index.y() + left_offset, map_size(0));
        const int start_x = std::max(index.x() - rear_offset, 0);
        const int last_x = std::min(index.x() + front_offset, map_size(1));

        for (int x = start_x; x < last_x; ++x) {
            for (int y = start_y; y < last_y; ++y) {
                cost_map_data(x, y) = max_val_;
            }
        }
    }
}

void LocalCostmapGenerator::publish_rigid_body_shape(const RigidBodyShape& rigid_body_shape) const {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = robot_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "rigid_body_shape";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = (rigid_body_shape.baselink2front - rigid_body_shape.baselink2rear) / 2.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    marker.scale.x = rigid_body_shape.baselink2front + rigid_body_shape.baselink2rear;
    marker.scale.y = rigid_body_shape.baselink2right + rigid_body_shape.baselink2left;
    marker.scale.z = 0.1;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    marker_array.markers.push_back(marker);

    pub_rigid_body_shape_.publish(marker_array);
}

}  // namespace local_costmap_generator