// Kohei Honda, 2023

#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <laser_geometry/laser_geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <voxel_grid/voxel_grid.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_filters/MedianFillFilter.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace local_costmap_generator {

class LocalCostmapGenerator {
private:
    using PointType = pcl::PointXYZ;

public:
    LocalCostmapGenerator();

    ~LocalCostmapGenerator(){};

private:
    /* ros system variables */
    ros::NodeHandle nh_;          //!< @brief ros public node handle
    ros::NodeHandle private_nh_;  //!< @brief ros private node handle
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    /* pub sub */
    ros::Subscriber sub_scan_;             //!< @brief laser scan subscriber
    ros::Timer timer_costmap_;             //!< @brief timer for calculate cost map
    ros::Publisher pub_cost_map_;          //!< @brief cost map publisher
    ros::Publisher pub_rigid_body_shape_;  //!< @brief rigid body shape publisher

    // debug
    ros::Publisher pub_cloud_;
    // ros::Publisher pub_calc_time_;

    /*Parameters*/
    std::string robot_frame_id_;
    std::string sensor_frame_id_;
    double update_rate_ = 0.01;  //!< @brief update interval [s]
    int thread_num_ = 4;         //!< @brief number of threads for cost map calculation
    const std::string collision_layer_name_ = "collision_layer";

    // vehicle shape parameters
    struct RigidBodyShape {
        double baselink2front = 0.47;  // m
        double baselink2rear = 0.14;   // m
        double baselink2right = 0.15;  // m
        double baselink2left = 0.15;   // m
    };
    RigidBodyShape rigid_body_shape_;

    // preprocess filters
    bool is_remove_outlier_ = false;
    pcl::StatisticalOutlierRemoval<PointType> sor_;
    int sor_mean_k_ = 10;
    double sor_stddev_mul_thresh_ = 1.0;

    bool is_downsample_ = false;
    pcl::VoxelGrid<PointType> voxel_grid_filter_;
    double downsample_resolution_ = 0.1;

    bool is_pass_through_ = false;
    pcl::PassThrough<PointType> pass_through_filter_;
    double pass_through_min_from_robot_ = 0.0;
    double pass_through_max_from_robot_ = 2.0;

    bool is_crop_robot_ = true;
    pcl::CropBox<PointType> crop_box_filter_;
    Eigen::Vector4f crop_box_min_;
    Eigen::Vector4f crop_box_max_;

    // Inner variables
    // For scan transform
    bool is_laser_scan_received_ = false;
    sensor_msgs::PointCloud2::Ptr raw_pc2_ptr_;
    laser_geometry::LaserProjection projector_;
    geometry_msgs::TransformStamped transform_stamped_;
    Eigen::Matrix4f transform_matrix_;

    // For cost map calculation
    pcl::PointCloud<PointType>::Ptr raw_pcl_ptr_;
    pcl::PointCloud<PointType>::Ptr preprocessed_pcl_ptr_;
    pcl::PointCloud<PointType>::Ptr trans_preprocessed_pcl_ptr_;
    double max_val_ = 100.0;      // max value of cost map
    grid_map::GridMap cost_map_;  // value = [0, max_val_] (0: free, max_val_: occupied)

    /**
     * @brief main loop
     *
     */
    void timer_callback(const ros::TimerEvent&);

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);

    /**
     * @brief preprocess_pointcloud
     * @param[in] raw_pcl_ptr_ raw point cloud in sensor frame
     * @param[out] preprocessed_pcl_ptr_ preprocessed point cloud in sensor frame
     */
    void preprocess_pointcloud(const pcl::PointCloud<PointType>::ConstPtr& in_pcl_ptr_, pcl::PointCloud<PointType>::Ptr& out_pcl_ptr_);

    void crop_points_within_robot(pcl::PointCloud<PointType>::Ptr& pcl_ptr_);

    /**
     * @brief build occupancy grid map from point cloud
     * @param[in] preprocessed_pcl_ptr_ preprocessed point cloud in sensor frame
     * @param[out] cost_map_ occupancy grid map : value = [0, max_val_] (0: free, max_val_: occupied)
     * @param[out] occupied_indices_ indices of occupied grid cells
     */
    std::vector<grid_map::Index> pointcloud_to_costmap(const pcl::PointCloud<PointType>::ConstPtr& in_pcl_ptr, grid_map::GridMap* out_cost_map) const;

    void inflate_rigid_body(const std::string& layer_name, const std::vector<grid_map::Index>& occupied_indices, grid_map::GridMap* cost_map) const;

    void publish_rigid_body_shape(const RigidBodyShape& rigid_body_shape) const;
};

}  // namespace local_costmap_generator
