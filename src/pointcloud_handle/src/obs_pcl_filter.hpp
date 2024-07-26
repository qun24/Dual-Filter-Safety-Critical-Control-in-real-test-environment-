#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointCloudFilter : public rclcpp::Node {

public:

    PointCloudFilter();
    bool sending_empty_clouds_;
    void publishEmptyCloud();

    rclcpp::Time last_received_time_;
    rclcpp::Time empty_cloud_start_time_;
    

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    
};
