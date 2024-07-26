#include "pointcloud_handle/obs_circle_rviz.hpp"
#include <visualization_msgs/msg/marker.hpp> // 确保包含头文件

using namespace std;

ObstacleVisualizer::ObstacleVisualizer() : Node("obstacle_visualizer") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "/processed_obstacles", 10, std::bind(&ObstacleVisualizer::obstacleInfoCallback, this, std::placeholders::_1)
    );
}

void ObstacleVisualizer::deleteMarker(int marker_id) {
    // delete marker with the given id
    visualization_msgs::msg::Marker marker;  // 不使用shared_ptr，直接创建对象
    marker.header.frame_id = "odom";  // 正确访问header
    marker.header.stamp = this->now();  // 设置时间戳
    marker.ns = "obstacle_visualizer";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::DELETE;

    marker_pub_->publish(marker);
}

std::vector<std::vector<float>> ObstacleVisualizer::extractAndFormatData(const std::string& text) {
    std::regex re("[-+]?\\d*\\.\\d+|\\d+");
    std::sregex_iterator next(text.begin(), text.end(), re);
    std::sregex_iterator end;
    std::vector<float> numbers;

    while (next != end) {
        std::smatch match = *next;
        numbers.push_back(std::stof(match.str()));
        next++;
    }

    std::vector<std::vector<float>> formatted_data;
    for (size_t i = 0; i < numbers.size(); i += 5) {
        std::vector<float> obstacle_data = { 
            round(numbers[i] * 100.0f) / 100.0f, 
            round(numbers[i + 1] * 100.0f) / 100.0f, 
            round(numbers[i + 2] * 100.0f) / 100.0f,
            round(numbers[i + 3] * 100.0f) / 100.0f,
            round(numbers[i + 4] * 100.0f) / 100.0f
        };
        formatted_data.push_back(obstacle_data);
    }

    return formatted_data;
}

void ObstacleVisualizer::drawCircles(const std::vector<std::vector<float>>& formatted_data, std::set<int>& new_marker_ids) {
    for (size_t i = 0; i < formatted_data.size(); ++i) {
        const auto& obstacle = formatted_data[i];
        visualization_msgs::msg::Marker marker;  // 不使用shared_ptr，直接创建对象
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();  // 设置时间戳
        marker.ns = "obstacle_visualizer";
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = obstacle[2] * 2;
        marker.scale.z = 0.01f;
        marker.color.a = 1.0f;
        marker.pose.orientation.w = 1.0f;
        marker.pose.position.x = obstacle[0];
        marker.pose.position.y = obstacle[1];
        marker.pose.position.z = 0.0f;
        marker.id = static_cast<int>(i);

        if (obstacle[3] == 0 && obstacle[4] == 0) {
            marker.color.g = 1.0f;
        } else {
            marker.color.r = 1.0f;
            marker_pub_->publish(marker);
            new_marker_ids.insert(static_cast<int>(i));

            for (int j = 1; j <= 3; ++j) {
                marker.pose.position.x += obstacle[3] * 0.3f;
                marker.pose.position.y += obstacle[4] * 0.3f;
                marker.color.r = 0.5f;
                marker.color.g = 0.5f;
                marker.color.b = 0.5f;
                marker.color.a = 0.5f;
                marker.id = static_cast<int>(i) + j * 10;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;
                new_marker_ids.insert(marker.id);
                marker_pub_->publish(marker);
            }
            continue;
        }

        marker_pub_->publish(marker);
        new_marker_ids.insert(static_cast<int>(i));
    }
}

void ObstacleVisualizer::obstacleInfoCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::string obstacle_info = msg->data;
    std::set<int> new_marker_ids;

    std::vector<std::vector<float>> formatted_list = extractAndFormatData(obstacle_info);
    drawCircles(formatted_list, new_marker_ids);

    for (const int marker_id : last_marker_ids_) {
        if (new_marker_ids.find(marker_id) == new_marker_ids.end()) {
            deleteMarker(marker_id);
        }
    }

    last_marker_ids_ = new_marker_ids;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleVisualizer>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
