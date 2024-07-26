/*
 * Copyright 2020 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SCAN_MATCH_PLICP
#define SCAN_MATCH_PLICP

#include <cmath>
#include <vector>
#include <chrono>

// ros
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// #include "sensor_msgs/msg/Imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// tf2
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// csm
#include <csm/csm_all.h>
#include <csm/csm.h>
#undef min
#undef max

class LaserOdomNode : public rclcpp::Node
{
private:
    // TF related
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // subscriber laser, imu
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_; // laser_scan subscriber
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_; // imu subscriber
    
    // publisher laser, odom, point cloud
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr feature_scan_publisher_; // laser_scan publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_; // odometry publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_; // point cloud publisher
    
    // pointer to the laser_odometry
    std::shared_ptr<LaserOdomNode>laser_odometry_ptr;
    
    // laser projector for laserscan to pcd
    laser_geometry::LaserProjection projector_;

    float edge_threshold_;

    
    rclcpp::Time last_icp_time_;
    rclcpp::Time current_time_;

    geometry_msgs::msg::Twist latest_velocity_;

    // tf2_ros::Buffer tf_buffer_(rclcpp::Clock::SharedPtr, tf2::Duration, rclcpp::Node::SharedPtr);
    // tf2_ros::TransformListener tf_listener_;
    // tf2_ros::TransformBroadcaster tf_broadcaster_;

    tf2::Transform base_to_laser_;    
    tf2::Transform laser_to_base_; 
    tf2::Transform prev_laser_in_tf_odom_;
    tf2::Transform prev_base_in_odom_;

    tf2::Transform base_in_odom_;           // base_link在odom坐标系下的坐标
    tf2::Transform base_in_odom_keyframe_;  // base_link在odom坐标系下的keyframe的坐标

    // parameters
    bool initialized_;
    bool use_imu_;

    std::string odom_frame_;
    std::string base_frame_;

    double kf_dist_linear_;
    double kf_dist_linear_sq_;
    double kf_dist_angular_;
    int kf_scan_count_;
    int scan_count_;

    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;

    // csm
    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;

    void initParams();
    void createCache(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    bool getBaseToLaserTf(const std::string &frame_id);
    void laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, LDP &ldp);
    void scanMatchWithPLICP(LDP &curr_ldp_scan, const rclcpp::Time &time);
    void getPrediction(double &prediction_change_x, double &prediction_change_y, double &prediction_change_angle, double dt);
    void createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t);
    void publishTFAndOdometry();
    bool newKeyframeNeeded(const tf2::Transform &d);
    void getCornerPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
public:
    LaserOdomNode();
    ~LaserOdomNode();

}; // class LaserOdomNode

#endif // LESSON2_SCAN_MATCH_PLICP