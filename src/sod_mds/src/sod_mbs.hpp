#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include "MBS.hpp" 


struct Pixel {
    int x, y;
};

struct Obstacle {
    std::vector<Pixel> pixels;
    std::vector<pcl::PointXYZ> points3D;
    double risk;
};

std::vector<Obstacle> detectSalientObstacles(const cv::Mat& img);
bool isDepthConsistent(const cv::Mat& depthImage, int x, int y, float depth, float threshold);
pcl::PointXYZ convertPixelToPoint(int x, int y, double Z);
void extractObstaclePoints(std::vector<Obstacle>& obstacles, const cv::Mat& depthImage);
void calculateObstacleRisks(std::vector<Obstacle>& obstacles);
void displayDangerousObstacles(const cv::Mat& img, const std::vector<Obstacle>& obstacles);
void PublishDangerousObstacleCloud(
    const std::vector<Obstacle>& obstacles,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    double riskThreshold = 0.15);
void processImageAndPublishCloud(
    const cv::Mat& inputImage,
    const cv::Mat& depthImage,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);

// class ObstacleDetectionNode : public rclcpp::Node
// {
// public:
//     ObstacleDetectionNode();
//     void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

// private:
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
//     std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
//     std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
//     typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
//     std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
// };
class ObstacleDetectionNode : public rclcpp::Node
{
public:
    ObstacleDetectionNode();
    void imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& rgb_msg, const sensor_msgs::msg::CompressedImage::ConstSharedPtr& depth_msg);

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> rgb_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> depth_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};