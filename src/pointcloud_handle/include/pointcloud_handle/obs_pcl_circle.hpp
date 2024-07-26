#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <vector>
#include <sstream>
#include <cmath>
#include <omp.h>

class ObsPclCircle : public rclcpp::Node
{
public:
    ObsPclCircle();
private:
    // ros2 publisher and subscriber
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_info_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    double x_real_ = 0;
    double y_real_ = 0;
    double theta_real_ = 0;

    // 摄像机相对于小车的偏移量
    const double camera_offset_x_ = 0.0;
    const double camera_offset_y_ = 0.0;

    // callback function
    void newOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);

    void publishMarker(const Eigen::Vector2f& position, int id, double r, double g, double b);
    Eigen::Vector2f calculateCentroid(const std::vector<Eigen::Vector2f>& points);
    Eigen::Vector2f findFarthestPoint(const std::vector<Eigen::Vector2f>& points, const Eigen::Vector2f& centroid);
    std::pair<Eigen::Vector2f, double> calculateMinEnclosingCircle(const std::vector<pcl::PointXYZ>& points, const Eigen::Vector2f& pn);
    void publishObstacleInfo(const std::string& info);
    std::string joinStrings(const std::vector<std::string>& strings, const std::string& delimiter);
};





