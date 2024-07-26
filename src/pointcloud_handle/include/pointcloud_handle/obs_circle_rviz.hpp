#ifndef OBS_CIRCLE_RVIZ_HPP
#define OBS_CIRCLE_RVIZ_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp> // 使用 .hpp 而不是 .h
#include <regex>
#include <string>
#include <vector>
#include <cmath>
#include <set>
#include <sstream>

class ObstacleVisualizer : public rclcpp::Node
{
public:
    ObstacleVisualizer();

private:
    // ros2 publisher and subscriber
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> marker_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    // callback function
    void obstacleInfoCallback(const std_msgs::msg::String::SharedPtr msg);

    // class member functions
    void deleteMarker(int marker_id);
    std::vector<std::vector<float>> extractAndFormatData(const std::string& text);
    void drawCircles(const std::vector<std::vector<float>>& format_data, std::set<int>& new_marker_ids);

    std::set<int> last_marker_ids_;  // 添加这个成员变量以存储上次的marker id
};

#endif // OBS_CIRCLE_RVIZ_HPP
