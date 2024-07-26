#include "sod_mbs.hpp"

const double camera_factor = 1000;

// const double camera_fx = 609.564;
// const double camera_fy = 608.282;
// const double camera_cx = 327.980;
// const double camera_cy = 240.780;

const double camera_fx = 304.781;
const double camera_fy = 304.141;
const double camera_cx = 215.990;
const double camera_cy = 120.389;

std::vector<Obstacle> detectSalientObstacles(const cv::Mat& img) {
    cv::Mat saliencyMap = doWork(img, true, true, false); 
    std::vector<Obstacle> obstacles;

    if (!saliencyMap.empty()) {
        if (saliencyMap.depth() != CV_8U) {
            saliencyMap.convertTo(saliencyMap, CV_8UC1, 255.0);
        }
        // Display the original saliency map 
        cv::imshow("Obstacles Saliency Map", saliencyMap);

        // Find contours of the connected components (obstacles)
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(saliencyMap, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        obstacles.reserve(contours.size());
        cv::Mat mask = cv::Mat::zeros(saliencyMap.size(), CV_8UC1);
        // Extract points for each obstacle and store them in obstaclePoints
        for (const auto& contour : contours) {
            mask.setTo(cv::Scalar(0));
            cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{contour}, cv::Scalar(255));

            Obstacle obstacle;
            cv::Mat nonZeroCoordinates;
            cv::findNonZero(mask, nonZeroCoordinates);

            obstacle.pixels.reserve(nonZeroCoordinates.total());
            for (size_t i = 0; i < nonZeroCoordinates.total(); ++i) {
                const auto& point = nonZeroCoordinates.at<cv::Point>(i);
                obstacle.pixels.emplace_back(Pixel{point.x, point.y});
            }
            obstacles.emplace_back(std::move(obstacle));
        }
    }
    // cv::waitKey(20);
    return obstacles;
}

bool isDepthConsistent(const cv::Mat& depthImage, int x, int y, float depth, float threshold) {
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            float neighborDepth = depthImage.at<float>(y + dy, x + dx);
            if (std::abs(depth - neighborDepth) > threshold) {
                return false;
            }
        }
    }
    return true;
}

pcl::PointXYZ convertPixelToPoint(int x, int y, double Z) {
    Z /= camera_factor; // mm转换为m
    return {
        static_cast<float>((x - camera_cx) * Z / camera_fx),
        static_cast<float>((y - camera_cy) * Z / camera_fy),
        static_cast<float>(Z)
    };
}

void extractObstaclePoints(std::vector<Obstacle>& obstacles, const cv::Mat& depthImage) {
    for (auto& obstacle : obstacles) {
        obstacle.points3D.reserve(obstacle.pixels.size());
        for (const auto& pixel : obstacle.pixels) {
            float depth = depthImage.at<float>(pixel.y, pixel.x);

            if (pixel.x > 0 && pixel.x < depthImage.cols - 1 && 
                pixel.y > 0 && pixel.y < depthImage.rows - 1 &&
                // isDepthConsistent(depthImage, pixel.x, pixel.y, depth, 20.f)) { //threshold 单位是mm
                isDepthConsistent(depthImage, pixel.x, pixel.y, depth, 20.f)) { //threshold 单位是mm
                obstacle.points3D.emplace_back(convertPixelToPoint(pixel.x, pixel.y, depth));
            }
        }
    }
}

void calculateObstacleRisks(std::vector<Obstacle>& obstacles) {
    const double characteristic_distance = 2.5; // 可调参数
    const double char_dist_squared = characteristic_distance * characteristic_distance;
    const double max_expected_size = 4000.0; // 预期的最大物体大小（点数）

    for (size_t i = 0; i < obstacles.size(); ++i) {
        auto& obstacle = obstacles[i];
        if (obstacle.points3D.empty()) continue;

        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        for (const auto& point : obstacle.points3D) {
            center += point.getVector3fMap();
        }

        float size = obstacle.points3D.size();
        center /= size;
        double centerDist = center.norm();

        // 归一化的面积危险度 (0-1)
        double sizerisk = std::min(1.0, size / max_expected_size);

        // 方向危险度(0-1)
        Eigen::Vector3f robotDirection(0, 0, 1);
        double dotProduct = robotDirection.dot(center.normalized());
        double angle = std::acos(std::abs(dotProduct)) / M_PI;
        double directionRisk = std::exp(-angle * angle / (2 * 0.5 * 0.5));

        // distanceRisk(0-1)
        double distancerisk = 1.0 / (1.0 + (centerDist * centerDist) / char_dist_squared);

        // 最终风险计算
        obstacle.risk = sizerisk * directionRisk * distancerisk;
    }
}

void displayDangerousObstacles(const cv::Mat& img, const std::vector<Obstacle>& obstacles) {
    cv::Mat displayImage = cv::Mat::zeros(img.size(), img.type());
    for (const auto& obstacle : obstacles) {
        int brightness = static_cast<int>(obstacle.risk * 255.0);
        brightness = std::min(255, std::max(0, brightness));  // 确保亮度在0到255之间
        
        for (const auto& pixel : obstacle.pixels) {
            displayImage.at<cv::Vec3b>(pixel.y, pixel.x) = cv::Vec3b(brightness, brightness, brightness);
        }
    }
    cv::namedWindow("Dangerous Obstacles Map", cv::WINDOW_NORMAL);
    cv::resizeWindow("Dangerous Obstacles Map", 320, 240);
    cv::imshow("Dangerous Obstacles Map", displayImage);
    cv::waitKey(20);
}

void PublishDangerousObstacleCloud(
    const std::vector<Obstacle>& obstacles,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    double riskThreshold)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr dangerousCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& obstacle : obstacles) {
        if (obstacle.risk > riskThreshold) {
            dangerousCloud->insert(dangerousCloud->end(), obstacle.points3D.begin(), obstacle.points3D.end());
        }
    }

    dangerousCloud->width = dangerousCloud->points.size();
    dangerousCloud->height = 1;
    dangerousCloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*dangerousCloud, cloud_msg);
    
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = timestamp;
    publisher->publish(cloud_msg);
}

void processImageAndPublishCloud(
    const cv::Mat& inputImage,
    const cv::Mat& depthImage,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher)
{
    auto obstacles = detectSalientObstacles(inputImage);
    extractObstaclePoints(obstacles, depthImage);
    calculateObstacleRisks(obstacles);
    displayDangerousObstacles(inputImage, obstacles);
    PublishDangerousObstacleCloud(obstacles, publisher, frame_id, timestamp);
}


// ObstacleDetectionNode::ObstacleDetectionNode()
//     : Node("obstacle_detection_node")
//     {
//         point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("saliency_point_cloud", 5);
//         rgb_sub_ =   std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/camera/color/image_raw");
//         depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/camera/depth/image_rect_raw");

//         sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);
//         sync_->registerCallback(std::bind(&ObstacleDetectionNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
// }

// void ObstacleDetectionNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
// {
//         cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_depth;
//         try
//         {
//             cv_ptr_rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
//             cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
//         }
//         catch (cv_bridge::Exception& e)
//         {
//             RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//             return;
//         }

//         processImageAndPublishCloud(cv_ptr_rgb->image, cv_ptr_depth->image, 
//                                     depth_msg->header.frame_id, depth_msg->header.stamp, 
//                                     point_cloud_publisher_);
// }

ObstacleDetectionNode::ObstacleDetectionNode()
    : Node("obstacle_detection_node")
{
    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("saliency_point_cloud", 5);
    
    rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(this, "/camera/compressed_image");
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(this, "/camera/compressed_depth_image");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(5), *rgb_sub_, *depth_sub_);

    sync_->registerCallback(std::bind(&ObstacleDetectionNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
}


void ObstacleDetectionNode::imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& rgb_msg, 
                                          const sensor_msgs::msg::CompressedImage::ConstSharedPtr& depth_msg) {
    cv::Mat rgb_image = cv::imdecode(cv::Mat(rgb_msg->data), cv::IMREAD_COLOR);
    cv::Mat depth_image = cv::imdecode(cv::Mat(depth_msg->data), cv::IMREAD_UNCHANGED);
    
    // 检查原始深度图像的类型和值范围
    std::cout << "Original depth image type: " << depth_image.type() << ", channels: " << depth_image.channels() << std::endl;
    double min_val, max_val;
    cv::minMaxLoc(depth_image, &min_val, &max_val);
    std::cout << "Original depth value range: [" << min_val << ", " << max_val << "]" << std::endl;

    // 根据原始图像类型进行适当的转换
    if (depth_image.type() == CV_8UC1) {
        // 如果是8位图像，假设这是一个归一化的深度图，需要重新缩放
        double depth_scale = 10.0; // 假设最大深度为10米，根据实际情况调整
        depth_image.convertTo(depth_image, CV_32FC1, depth_scale / 255.0);
    } else if (depth_image.type() == CV_16UC1) {
        // 如果是16位图像，假设单位是毫米，转换为米
        // depth_image.convertTo(depth_image, CV_32FC1, 1.0 / 1000.0);
        depth_image.convertTo(depth_image, CV_32FC1);
    } else if (depth_image.type() != CV_32FC1) {
        std::cerr << "Unexpected depth image type: " << depth_image.type() << std::endl;
        return;
    }

    // // 检查转换后的深度图像的值范围
    // cv::minMaxLoc(depth_image, &min_val, &max_val);
    // std::cout << "Converted depth value range: [" << min_val << ", " << max_val << "] meters" << std::endl;
    // 检查深度类型，根据深度类型来判断是否是CV_32FC1
    // int depth = depth_image.depth();
    // std::cout << "Final depth type: " << depth << std::endl;

    // 显示图像
    cv::imshow("Decoded RGB Image", rgb_image);
    

    // 处理图像并发布点云
    processImageAndPublishCloud(rgb_image, depth_image, depth_msg->header.frame_id, depth_msg->header.stamp, point_cloud_publisher_);

    // 及时释放解码后的图像内存
    rgb_image.release();
    depth_image.release();
}


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetectionNode>());
    rclcpp::shutdown();
    return 0;
    
}