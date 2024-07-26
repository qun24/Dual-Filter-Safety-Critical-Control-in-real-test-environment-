#include "pointcloud_handle/obs_pcl_circle.hpp"

using namespace std;
using namespace Eigen;
using namespace pcl;

ObsPclCircle::ObsPclCircle() : Node("pointcloud_processor") {

    obstacle_info_pub_ = this->create_publisher<std_msgs::msg::String>("/obstacle_info", 5);
    // marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&ObsPclCircle::newOdomCallback, this, std::placeholders::_1));
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "Filtered_Dangerous_Obstacle_cloud", 5, std::bind(&ObsPclCircle::pointCloudCallback, this, std::placeholders::_1));
}


void ObsPclCircle::publishMarker(const Eigen::Vector2f& position, int id, double r, double g, double b) {
    auto marker = std::make_unique<visualization_msgs::msg::Marker>();
    marker->header.frame_id = "odom";
    marker->header.stamp = this->now();
    marker->ns = "obstacle";
    marker->id = id;
    marker->type = visualization_msgs::msg::Marker::SPHERE;
    marker->action = visualization_msgs::msg::Marker::ADD;
    marker->pose.position.x = position.x();
    marker->pose.position.y = position.y();
    marker->pose.position.z = 0;
    marker->pose.orientation.w = 1.0;
    marker->scale.x = 0.05;
    marker->scale.y = 0.05;
    marker->scale.z = 0.05;
    marker->color.a = 1.0;
    marker->color.r = r;
    marker->color.g = g;
    marker->color.b = b;
    marker_pub_->publish(std::move(marker));
}

Eigen::Vector2f ObsPclCircle::calculateCentroid(const std::vector<Eigen::Vector2f>& points) {
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    for (const auto& point : points) {
        centroid += point;
    }
    return centroid / static_cast<double>(points.size());
}

Eigen::Vector2f ObsPclCircle::findFarthestPoint(const std::vector<Eigen::Vector2f>& points, const Eigen::Vector2f& centroid) {
    Eigen::Vector2f farthest_point;
    double max_distance = 0;
    for (const auto& point : points) {
        double distance = (point - centroid).norm();
        if (distance > max_distance) {
            max_distance = distance;
            farthest_point = point;
        }
    }
    return farthest_point;
}

std::pair<Eigen::Vector2f, double> ObsPclCircle::calculateMinEnclosingCircle(const std::vector<pcl::PointXYZ>& points, const Eigen::Vector2f& pn) {
    std::vector<Eigen::Vector2f> xy_projected_points;
    xy_projected_points.reserve(points.size());
    for (const auto& point : points) {
        xy_projected_points.emplace_back(point.x, point.y);
    }
    Eigen::Vector2f pg = calculateCentroid(xy_projected_points);
    Eigen::Vector2f pm = findFarthestPoint(xy_projected_points, pg);
    double pm_pn = (pm - pn).norm();
    double pg_pn = (pg - pn).norm();
    double pm_pg = (pm - pg).norm();
    double alpha = std::acos((std::pow(pm_pn, 2) + std::pow(pg_pn, 2) - std::pow(pm_pg, 2)) / (2 * pm_pn * pg_pn));
    double beta = M_PI / 2 - alpha;
    double pc_pm = pm_pn * std::tan(alpha);
    double pc_pg = pc_pm * std::sin(beta) / beta;
    double pc_pn = pc_pg + pg_pn;
    Eigen::Vector2f pg_pn_vector = pg - pn;
    Eigen::Vector2f pc_vector = pn + pg_pn_vector.normalized() * pc_pn;
    double radius = (pm - pc_vector).norm();
    return std::make_pair(pc_vector, radius);
}

void ObsPclCircle::publishObstacleInfo(const std::string& info) {
    auto obstacle_info_msg = std_msgs::msg::String();
    obstacle_info_msg.data = info;
    obstacle_info_pub_->publish(obstacle_info_msg);
}

std::string ObsPclCircle::joinStrings(const std::vector<std::string>& strings, const std::string& delimiter) {
    if (strings.empty()) return "";
    std::ostringstream oss;
    std::copy(strings.begin(), strings.end() - 1, std::ostream_iterator<std::string>(oss, delimiter.c_str()));
    oss << strings.back();
    return oss.str();
}

// Callback functions
// void ObsPclCircle::newOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     x_real_ = msg->pose.pose.position.x;
//     y_real_ = msg->pose.pose.position.y;
//     theta_real_ = tf2::getYaw(msg->pose.pose.orientation);
//     tf2::Quaternion q(
//             msg->pose.pose.orientation.x,
//             msg->pose.pose.orientation.y,
//             msg->pose.pose.orientation.z,
//             msg->pose.pose.orientation.w);
//     tf2::Matrix3x3 m(q);
//     double roll, pitch;
//     m.getRPY(roll, pitch, theta_real_);
// }
void ObsPclCircle::newOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    try {
        x_real_ = msg->pose.pose.position.x;
        y_real_ = msg->pose.pose.position.y;
        theta_real_ = tf2::getYaw(msg->pose.pose.orientation);
        tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, theta_real_);

        RCLCPP_DEBUG(this->get_logger(), "Updated odometry: x=%f, y=%f, theta=%f", x_real_, y_real_, theta_real_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in newOdomCallback: %s", e.what());
    }
}
// void ObsPclCircle::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*cloud_msg, *cloud);
//     if (cloud->empty()) {
//         RCLCPP_WARN(this->get_logger(), "Received an empty point cloud. Skipping processing.");
//         publishObstacleInfo("");
//         return;
//     }
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     tree->setInputCloud(cloud);
//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//     ec.setClusterTolerance(0.25);
//     ec.setMinClusterSize(5);
//     ec.setMaxClusterSize(20000);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(cloud);
//     ec.extract(cluster_indices);
//     std::vector<std::string> obstacle_info;
//     Eigen::Vector2f pn = Eigen::Vector2f(x_real_ + camera_offset_x_, y_real_ + camera_offset_y_);
//     #pragma omp parallel for
//     for (const auto& indices : cluster_indices) {
//         if (indices.indices.size() < 3) {
//             continue;
//         }
//         std::vector<pcl::PointXYZ> cluster;
//         for (const auto& index : indices.indices) {
//             cluster.push_back((*cloud)[index]);
//         }

//         auto [center, radius] = calculateMinEnclosingCircle(cluster, pn);
//         obstacle_info.emplace_back("position: (" + std::to_string(center.x()) + ", " + 
//                                    std::to_string(center.y()) + "), radius: " + 
//                                    std::to_string(radius));
//     }
//     std::string obstacle_info_msg_data = joinStrings(obstacle_info, "\n");
//     publishObstacleInfo(obstacle_info_msg_data);
// }
void ObsPclCircle::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
    try {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty point cloud. Skipping processing.");
            publishObstacleInfo("");
            return;
        }
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.25);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(20000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
        std::vector<std::string> obstacle_info;
        Eigen::Vector2f pn = Eigen::Vector2f(x_real_ + camera_offset_x_, y_real_ + camera_offset_y_);

        std::mutex obstacle_info_mutex;
        #pragma omp parallel for
        for (size_t i = 0; i < cluster_indices.size(); ++i) {
            const auto& indices = cluster_indices[i];
            if (indices.indices.size() < 3) {
                continue;
            }
            std::vector<pcl::PointXYZ> cluster;
            for (const auto& index : indices.indices) {
                cluster.push_back((*cloud)[index]);
            }

            auto [center, radius] = calculateMinEnclosingCircle(cluster, pn);
            std::string info = "position: (" + std::to_string(center.x()) + ", " + 
                               std::to_string(center.y()) + "), radius: " + 
                               std::to_string(radius);

            {
                std::lock_guard<std::mutex> lock(obstacle_info_mutex);
                obstacle_info.emplace_back(std::move(info));
            }
        }
        std::string obstacle_info_msg_data = joinStrings(obstacle_info, "\n");
        publishObstacleInfo(obstacle_info_msg_data);

        RCLCPP_DEBUG(this->get_logger(), "Processed point cloud with %zu clusters", cluster_indices.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in pointCloudCallback: %s", e.what());
    }
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<ObsPclCircle>());
    try {
        rclcpp::spin(std::make_shared<ObsPclCircle>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
