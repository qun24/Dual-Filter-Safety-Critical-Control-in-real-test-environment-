#include "obs_pcl_filter.hpp"


PointCloudFilter::PointCloudFilter() : Node("pointcloud_filter") 
{
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("Filtered_Dangerous_Obstacle_cloud", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "saliency_point_cloud", 10, std::bind(&PointCloudFilter::cloudCallback, this, std::placeholders::_1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    sending_empty_clouds_ = false;
    last_received_time_ = this->now();
}

void PointCloudFilter::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    last_received_time_ = this->now();
    sending_empty_clouds_ = false;

    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_distance;
    pass_distance.setInputCloud(cloud);
    pass_distance.setFilterFieldName("z");
    pass_distance.setFilterLimits(0, 2.0);
    PointCloud::Ptr cloud_filtered_distance(new PointCloud);
    pass_distance.filter(*cloud_filtered_distance);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered_distance);
    // vg.setLeafSize(0.05f, 0.05f, 0.05f);
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    PointCloud::Ptr cloud_voxel_filtered(new PointCloud);
    vg.filter(*cloud_voxel_filtered);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud_voxel_filtered);
    ror.setRadiusSearch(0.3);
    ror.setMinNeighborsInRadius(10);
    PointCloud::Ptr cloud_outlier_filtered(new PointCloud);
    ror.filter(*cloud_outlier_filtered);

    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer_->lookupTransform("odom", cloud_msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transform;
    pcl_ros::transformAsMatrix(transformStamped, transform);
    pcl::transformPointCloud(*cloud_outlier_filtered, *cloud_transformed, transform);
    // pcl::transformPointCloud(*cloud_voxel_filtered, *cloud_transformed, transform);
    // pcl::transformPointCloud(*cloud_filtered_distance, *cloud_transformed, transform);



    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_transformed);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.01, 2.0);
    PointCloud::Ptr cloud_global_filtered(new PointCloud);
    pass_z.filter(*cloud_global_filtered);

    sensor_msgs::msg::PointCloud2 cloud_output;
    pcl::toROSMsg(*cloud_global_filtered, cloud_output);
    cloud_output.header.frame_id = "odom";
    cloud_output.header.stamp = cloud_msg->header.stamp;
    pub_->publish(cloud_output);
}

void PointCloudFilter::publishEmptyCloud() {
    auto now = this->now();
    sensor_msgs::msg::PointCloud2 empty_cloud;
    empty_cloud.header.frame_id = "odom";
    empty_cloud.header.stamp = now;
    pub_->publish(empty_cloud);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFilter>();

    rclcpp::Rate rate(25);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        auto now = node->now();
        auto time_since_last_received = now - node->last_received_time_;

        if (time_since_last_received.seconds() > 0.5 && !node->sending_empty_clouds_) {
            node->empty_cloud_start_time_ = now;
            node->sending_empty_clouds_ = true;
        }

        if (node->sending_empty_clouds_) {
            auto sending_duration = now - node->empty_cloud_start_time_;
            if (sending_duration.seconds() < 5.0) {
                node->publishEmptyCloud();
            } else {
                node->sending_empty_clouds_ = false;
            }
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
