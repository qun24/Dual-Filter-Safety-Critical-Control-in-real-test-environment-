#include <cstdio>
#include <map>
#include "laser_geometry/laser_geometry.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_types.h"
#include <pcl/filters/voxel_grid.h>
#include "lidar_odom_from_scratch/plicp_odometry.h"

# define max_scan_count 1500

struct smoothness_t
{
    float value;
    size_t index;
};

// 排序的规则,从小到大进行排序
struct by_value
{
    bool operator()(smoothness_t const &left, smoothness_t const &right)
    {
        return left.value < right.value;
    }
};

// LaserScan class
LaserOdomNode::LaserOdomNode() : Node("laser_odom_node")
                  
{
  auto start = this->now();
  RCLCPP_INFO(this->get_logger(), "LaserScan initial.");
  // 订阅雷达数据
  laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(), std::bind(&LaserOdomNode::scanCallback, this, std::placeholders::_1));
  // 发布雷达数据
//   imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    // "/bno055/imu", rclcpp::SensorDataQoS(), std::bind(&LaserOdomNode::imuCallback, this, std::placeholders::_1));
  feature_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/feature_scan", 1);
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS());
  point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcd", 1);

  LaserOdomNode::initParams();

  scan_count_ = 0;

  // waiting for first lidar frame
  initialized_ = false;
  use_imu_ = false;

  base_in_odom_.setIdentity();
  base_in_odom_keyframe_.setIdentity();
  prev_laser_in_tf_odom_.setIdentity();
  prev_base_in_odom_.setIdentity();

  input_.laser[0] = 0.0;  
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  output_.cov_x_m  = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  auto init_end = this->now();
  RCLCPP_INFO(this->get_logger(), "init time: %f", (init_end - start).seconds());
} // end of constructor

LaserOdomNode::~LaserOdomNode(){};

void LaserOdomNode::initParams()
    {
        // 读取参数
        RCLCPP_INFO(this->get_logger(), "\033[1;32m----> PLICP odometry started.\033[0m");
        odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        
        // **** keyframe params: when to generate the keyframe scan
        // if either is set to 0, reduces to frame-to-frame matching
        kf_dist_linear_ = this->declare_parameter<double>("kf_dist_linear", 0.1);
        kf_dist_angular_ = this->declare_parameter<double>("kf_dist_angular", 5.0 * (M_PI / 180.0));
        kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;
        kf_scan_count_ = this->declare_parameter<int>("kf_scan_count", 10);

        // **** CSM 的参数 - comments copied from algos.h (by Andrea Censi)

        // Maximum angular displacement between scans
        input_.max_angular_correction_deg = this->declare_parameter<double>("max_angular_correction_deg", 45.0);

        // Maximum translation between scans (m)
        input_.max_linear_correction = this->declare_parameter<double>("max_linear_correction", 1.0);

        // Maximum ICP cycle iterations
        input_.max_iterations = this->declare_parameter<int>("max_iterations", 10);

        // A threshold for stopping (m)
        input_.epsilon_xy = this->declare_parameter<double>("epsilon_xy", 0.000001);

        // A threshold for stopping (rad)
        input_.epsilon_theta = this->declare_parameter<double>("epsilon_theta", 0.000001);

        // Maximum distance for a correspondence to be valid
        input_.max_correspondence_dist = this->declare_parameter<double>("max_correspondence_dist", 1.0);

        // Noise in the scan (m)
        input_.sigma = this->declare_parameter<double>("sigma", 0.010);

        // Use smart tricks for finding correspondences.
        input_.use_corr_tricks = this->declare_parameter<int>("use_corr_tricks", 1);

        // Restart: Restart if error is over threshold
        input_.restart = this->declare_parameter<int>("restart", 0);

        // Restart: Threshold for restarting
        input_.restart_threshold_mean_error = this->declare_parameter<double>("restart_threshold_mean_error", 0.01);

        // Restart: displacement for restarting. (m)
        input_.restart_dt = this->declare_parameter<double>("restart_dt", 1.0);

        // Restart: displacement for restarting. (rad)
        input_.restart_dtheta = this->declare_parameter<double>("restart_dtheta", 0.1);

        // Max distance for staying in the same clustering
        input_.clustering_threshold = this->declare_parameter<double>("clustering_threshold", 0.25);

        // Number of neighbour rays used to estimate the orientation
        input_.orientation_neighbourhood = this->declare_parameter<int>("orientation_neighbourhood", 20);

        // If 0, it's vanilla ICP
        input_.use_point_to_line_distance = this->declare_parameter<int>("use_point_to_line_distance", 1);

        // Discard correspondences based on the angles
        input_.do_alpha_test = this->declare_parameter<int>("do_alpha_test", 0);

        // Discard correspondences based on the angles - threshold angle, in degrees
        input_.do_alpha_test_thresholdDeg = this->declare_parameter<double>("do_alpha_test_thresholdDeg", 20.0);

        // Percentage of correspondences to consider: if 0.9,
        // always discard the top 10% of correspondences with more error
        input_.outliers_maxPerc = this->declare_parameter<double>("outliers_maxPerc", 0.90);

        // Parameters describing a simple adaptive algorithm for discarding.
        //  1) Order the errors.
        //  2) Choose the percentile according to outliers_adaptive_order.
        //     (if it is 0.7, get the 70% percentile)
        //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
        //     with the value of the error at the chosen percentile.
        //  4) Discard correspondences over the threshold.
        //  This is useful to be conservative; yet remove the biggest errors.
        input_.outliers_adaptive_order = this->declare_parameter<double>("outliers_adaptive_order", 0.7);

        input_.outliers_adaptive_mult = this->declare_parameter<double>("outliers_adaptive_mult", 2.0);

        // If you already have a guess of the solution, you can compute the polar angle
        // of the points of one scan in the new position. If the polar angle is not a monotone
        // function of the readings index, it means that the surface is not visible in the
        // next position. If it is not visible, then we don't use it for matching.
        input_.do_visibility_test = this->declare_parameter<int>("do_visibility_test", 0);

        // no two points in laser_sens can have the same corr.
        input_.outliers_remove_doubles = this->declare_parameter<int>("outliers_remove_doubles", 1);

        // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
        // (G. Grisetti, C. Stachniss, W. Burgard and D. Fox. Aligning Maps of Unstructured Environments
        // using ICP. In Proceedings of the International Conference on Robotics and Automation (ICRA), 2007.)
        input_.do_compute_covariance = this->declare_parameter<int>("do_compute_covariance", 0);

        // Checks that find_correspondences_tricks gives the right answer
        input_.debug_verify_tricks = this->declare_parameter<int>("debug_verify_tricks", 0);

        // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
        // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
        input_.use_ml_weights = this->declare_parameter<int>("use_ml_weights", 0);

        // If 1, the field 'readings_sigma' in the second scan is used to weight the
        // correspondence by 1/sigma^2
        input_.use_sigma_weights = this->declare_parameter<int>("use_sigma_weights", 0);

    } // end of InitParams

void LaserOdomNode::createCache(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        a_cos_.clear();
        a_sin_.clear();
        RCLCPP_INFO(this->get_logger(),"scan range: %lu", scan_msg->ranges.size());

        for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
        {
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            a_cos_.push_back(cos(angle));
            a_sin_.push_back(sin(angle));
        }

        input_.min_reading = scan_msg->range_min;
        input_.max_reading = scan_msg->range_max;
    } // end of CreateCache

bool LaserOdomNode::getBaseToLaserTf(const std::string &frame_id)
    {
        rclcpp::Time time = this->now();
        geometry_msgs::msg::TransformStamped transformStamped;

        try
        {
            transformStamped = tf_buffer_->lookupTransform(base_frame_, frame_id, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return false;
        }

        tf2::Stamped<tf2::Transform> base_to_laser_;
        tf2::fromMsg(transformStamped, base_to_laser_);
        laser_to_base_ = base_to_laser_.inverse();
        return true;
    } // end of GetBaseToLaserTf
    
void LaserOdomNode::getPrediction(double &prediction_change_x, 
                       double &prediction_change_y,
                       double &prediction_change_angle, 
                       double dt)
    {
        prediction_change_x = latest_velocity_.linear.x < 1e-6 ? 0.0 : latest_velocity_.linear.x * dt;
        prediction_change_y = latest_velocity_.linear.y < 1e-6 ? 0.0 : latest_velocity_.linear.y * dt;
        prediction_change_angle = latest_velocity_.angular.z < 1e-6 ? 0.0 : latest_velocity_.linear.z* dt;

        if (prediction_change_angle >= M_PI)
            prediction_change_angle -= 2.0 * M_PI;
        else if (prediction_change_angle < -M_PI)
            prediction_change_angle += 2.0 * M_PI;
    }

void LaserOdomNode::createTfFromXYTheta(double x, double y, double theta, tf2::Transform &tf)
    {
        tf.setOrigin(tf2::Vector3(x, y, 0.0));
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta);
        tf.setRotation(q);
    }

bool LaserOdomNode::newKeyframeNeeded(const tf2::Transform &d){
        
        scan_count_++;

        if (fabs(tf2::getYaw(d.getRotation())) > kf_dist_angular_)
            return true;

        if (scan_count_ == kf_scan_count_)
        {
            scan_count_ = 0;
            return true;
        }
        
        double x = d.getOrigin().getX();
        double y = d.getOrigin().getY();
        if (x * x + y * y > kf_dist_linear_sq_)
            return true;

        return false;
    }

void LaserOdomNode::publishTFAndOdometry()
    {
        // 发布tf
        auto tf_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
        tf_msg->header.stamp = current_time_;
        tf_msg->header.frame_id = odom_frame_;
        tf_msg->child_frame_id = base_frame_;
        tf_msg->transform = tf2::toMsg(base_in_odom_);

        // 发布 odom 到 base_link 的 tf
        tf_broadcaster_->sendTransform(*tf_msg);

        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = current_time_;
        odom_msg->header.frame_id = odom_frame_;
        odom_msg->child_frame_id = base_frame_;

        // odom_msg->pose.pose.position.x = base_in_odom_.getOrigin().getX();
        // odom_msg->pose.pose.position.y = base_in_odom_.getOrigin().getY();
        // odom_msg->pose.pose.position.z = base_in_odom_.getOrigin().getZ();
        
        
        tf2::toMsg(base_in_odom_, odom_msg->pose.pose);

        double dt_pub = (current_time_ - last_icp_time_).nanoseconds()/1e+9;
        //std::cout << "发布时间t=" << dt_pub << std::endl;
        auto pose_difference = prev_base_in_odom_.inverse() * base_in_odom_;
        odom_msg->twist.twist.linear.x = pose_difference.getOrigin().getX() / dt_pub;
        odom_msg->twist.twist.linear.y = pose_difference.getOrigin().getY() / dt_pub;
        odom_msg->twist.twist.angular.z = tf2::getYaw(pose_difference.getRotation()) / dt_pub;
        //odom_msg->pose.pose.orientation = tf2::toMsg(base_in_odom_.getRotation());
        //odom_msg->twist.twist = latest_velocity_;

        // 发布 odomemtry 话题
        odom_publisher_->publish(*odom_msg);
        auto end = this->now();
        //RCLCPP_INFO(this->get_logger(), "publish time: %f", (end - start).seconds());
    }
    
void LaserOdomNode::getCornerPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
             std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
      std::vector<smoothness_t> scan_smoothness_(max_scan_count); // 存储每个点的曲率与索引
      float *scan_curvature_ = new float[max_scan_count];         // 存储每个点的曲率
      std::map<int, int> map_index;   // 有效点的索引 对应的 scan实际的索引
      int count = 0;                  // 有效点的索引
      float new_scan[max_scan_count]; // 存储scan数据的距离值
      // 通过ranges中数据的个数进行雷达数据的遍历
      int scan_count = scan_msg->ranges.size();
      // ROS_INFO_STREAM("scan_count: " << scan_count);
      // 去处inf或者nan点,保存有效点
      for (int i = 0; i < scan_count; i++)
      {
          if (!std::isfinite(scan_msg->ranges[i]))
          {
              // std::cout << " " << i << " " << scan_msg->ranges[i];
              continue;
          }
          // 这点在原始数据中的索引为i，在new_scan中的索引为count
          map_index[count] = i;
          // new_scan中保存了有效点的距离值
          new_scan[count] = scan_msg->ranges[i];
          count++;
      }
      // std::cout << "count: " << count << std::endl;
      // 计算曲率值, 通过当前点前后5个点距离值的偏差程度来代表曲率
      // 如果是球面, 则当前点周围的10个点的距离之和 减去 当前点距离的10倍 应该等于0
      for (int i = 5; i < count - 5; i++)
      {
          float diff_range = new_scan[i - 5] + new_scan[i - 4] +
                             new_scan[i - 3] + new_scan[i - 2] +
                             new_scan[i - 1] - new_scan[i] * 10 +
                             new_scan[i + 1] + new_scan[i + 2] +
                             new_scan[i + 3] + new_scan[i + 4] +
                             new_scan[i + 5];
          // diffX * diffX + diffY * diffY
          scan_curvature_[i] = diff_range * diff_range;
          scan_smoothness_[i].value = scan_curvature_[i];
          scan_smoothness_[i].index = i;
      }
      // declare shared pointer corner_scan
      auto corner_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
      corner_scan->header = scan_msg->header;
      corner_scan->angle_min = scan_msg->angle_min;
      corner_scan->angle_max = scan_msg->angle_max;
      corner_scan->angle_increment = scan_msg->angle_increment;
      corner_scan->range_min = scan_msg->range_min;
      corner_scan->range_max = scan_msg->range_max;
      // 对float[] 进行初始化
      corner_scan->ranges.resize(max_scan_count);
      // 进行角点的提取,将完整的scan分成6部分,每部分提取20个角点
      for (int j = 0; j < 6; j++)
      {
          int start_index = (0 * (6 - j) + count * j) / 6;
          int end_index = (0 * (5 - j) + count * (j + 1)) / 6 - 1;
          // std::cout << "start_index: " << start_index << " end_index: " << end_index << std::endl;
          if (start_index >= end_index)
              continue;
          // 将这段点云按照曲率从小到大进行排序
          std::sort(scan_smoothness_.begin() + start_index,
                    scan_smoothness_.begin() + end_index, by_value());
          int largestPickedNum = 0;
          // 最后的点 的曲率最大，如果满足条件，就是角点
          for (int k = end_index; k >= start_index; k--)
          {
              int index = scan_smoothness_[k].index;
              if (scan_smoothness_[k].value > edge_threshold_)
              {
                  // 每一段最多只取20个角点
                  largestPickedNum++;
                  if (largestPickedNum <= 20)
                  {
                      corner_scan->ranges[map_index[index]] = scan_msg->ranges[map_index[index]];
                  }
                  else
                  {
                      break;
                  }
              }
          }
      }

        // 将提取后的scan数据发布出去
        feature_scan_publisher_->publish(*corner_scan);

        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
        // std::cout<<"处理一次数据用时: "<< time_used.count() << " 秒。" << std::endl;

        // 将角点数据转换成点云数据发布出去
        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        projector_.projectLaser(*corner_scan, *cloud);
        point_cloud_publisher_->publish(*cloud);
    }

void LaserOdomNode::laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, LDP &ldp)
    {
        unsigned int n = scan_msg->ranges.size();

        ldp = ld_alloc_new(n);

        for (unsigned int i = 0; i < n; i++)
        {
            // calculate position in laser frame
            double r = scan_msg->ranges[i];

            if (r > scan_msg->range_min && r < scan_msg->range_max)
            {
                // fill in laser scan data

                ldp->valid[i] = 1;
                ldp->readings[i] = r;
            }
            else
            {
                ldp->valid[i] = 0;
                ldp->readings[i] = -1; // for invalid range
            }

            ldp->theta[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
            ldp->cluster[i] = -1;
        }

        ldp->min_theta = ldp->theta[0];
        ldp->max_theta = ldp->theta[n - 1];

        ldp->odometry[0] = 0.0;
        ldp->odometry[1] = 0.0;
        ldp->odometry[2] = 0.0;

        ldp->true_pose[0] = 0.0;
        ldp->true_pose[1] = 0.0;
        ldp->true_pose[2] = 0.0;
    } // end of LaserScanToLDP
    
void LaserOdomNode::scanMatchWithPLICP(LDP &curr_ldp_scan, const rclcpp::Time &time){
        // CSM is used in the following way:
        // The scans are always in the laser frame
        // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
        // The new scan (currLDPScan) has a pose equal to the movement
        // of the laser in the laser frame since the last scan
        // The computed correction is then propagated using the tf machinery

        prev_ldp_scan_->odometry[0] = 0.0;
        prev_ldp_scan_->odometry[1] = 0.0;
        prev_ldp_scan_->odometry[2] = 0.0;

        prev_ldp_scan_->estimate[0] = 0.0;
        prev_ldp_scan_->estimate[1] = 0.0;
        prev_ldp_scan_->estimate[2] = 0.0;

        prev_ldp_scan_->true_pose[0] = 0.0;
        prev_ldp_scan_->true_pose[1] = 0.0;
        prev_ldp_scan_->true_pose[2] = 0.0;

        input_.laser_ref = prev_ldp_scan_;
        input_.laser_sens = curr_ldp_scan;

        // 匀速模型，速度乘以时间，得到预测的odom坐标系下的位姿变换
        double dt = (time - last_icp_time_).seconds();
        double prediction_change_x, prediction_change_y, prediction_change_angle;
        LaserOdomNode::getPrediction(prediction_change_x, prediction_change_y, prediction_change_angle, dt);

        tf2::Transform prediction_change;
        LaserOdomNode::createTfFromXYTheta(prediction_change_x, prediction_change_y, prediction_change_angle, prediction_change);

        // account for the change since the last kf, in the fixed frame
        // 将odom坐标系下的坐标变换 转换成 base_in_odom_keyframe_坐标系下的坐标变换
        prediction_change = prediction_change * (base_in_odom_ * base_in_odom_keyframe_.inverse());

        // the predicted change of the laser's position, in the laser frame
        // 将base_link坐标系下的坐标变换 转换成 雷达坐标系下的坐标变换
        tf2::Transform prediction_change_lidar;
        prediction_change_lidar = laser_to_base_ * base_in_odom_.inverse() * prediction_change * base_in_odom_ * base_to_laser_;

        /* change
        input_.first_guess[0] = prediction_change_lidar.getOrigin().getX();
        input_.first_guess[1] = prediction_change_lidar.getOrigin().getY();
        input_.first_guess[2] = tf2::getYaw(prediction_change_lidar.getRotation());
        */

        input_.first_guess[0] = 0;
        input_.first_guess[1] = 0;
        input_.first_guess[2] = 0;

        // If they are non-Null, free covariance gsl matrices to avoid leaking memory
        if (output_.cov_x_m)
        {
            gsl_matrix_free(output_.cov_x_m);
            output_.cov_x_m = 0;
        }
        if (output_.dx_dy1_m)
        {
            gsl_matrix_free(output_.dx_dy1_m);
            output_.dx_dy1_m = 0;
        }
        if (output_.dx_dy2_m)
        {
            gsl_matrix_free(output_.dx_dy2_m);
            output_.dx_dy2_m = 0;
        }
        

        start_time_ = std::chrono::steady_clock::now();
        // 调用csm进行plicp计算
        sm_icp(&input_, &output_);

        end_time_ = std::chrono::steady_clock::now();
        time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
        //RCLCPP_INFO(this->get_logger(),"PLICP compute cost: %.5f sec", time_used_.count());

        tf2::Transform corr_ch;

        if (output_.valid)
        {
            // std::cout << "transfrom: (" << output_.x[0] << ", " << output_.x[1] << ", " 
            // << output_.x[2] * 180 / M_PI << ")" << std::endl;
            // 雷达坐标系下的坐标变换
            tf2::Transform corr_ch_l;
            LaserOdomNode::createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);



            // 将雷达坐标系下的坐标变换 转换成 base_link坐标系下的坐标变换
            corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;

            // 更新 base_link 在 odom 坐标系下 的坐标
            //base_in_odom_ = base_in_odom_keyframe_ * corr_ch;
            base_in_odom_ = base_in_odom_keyframe_ * corr_ch_l;

            // transform printer test
            //tf2::Quaternion test_q = base_in_odom_.getRotation();
            //std::cout << "Rotation Quaternion: x=" << test_q.x() << ", y=" << test_q.y() << ", z=" << test_q.z() << ", w=" << test_q.w() << std::endl;
            //tf2::Vector3 test_v = base_in_odom_.getOrigin();
            //std::cout << "Translation Vector: x=" << test_v.x() << ", y=" << test_v.y() << ", z=" << test_v.z() << std::endl;



            latest_velocity_.linear.x = corr_ch.getOrigin().getX() / dt;
            latest_velocity_.angular.z = tf2::getYaw(corr_ch.getRotation()) / dt;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "not Converged!");
        } // end of if (output_.valid)

        // test transform base_in_odom_ 0.5 0 0 0, 0 0 0

        // 发布tf与odom话题
        LaserOdomNode::publishTFAndOdometry();

        // 检查是否需要更新关键帧坐标
        if (LaserOdomNode::newKeyframeNeeded(corr_ch))
        {
            // 更新关键帧坐标
            ld_free(prev_ldp_scan_);
            prev_ldp_scan_ = curr_ldp_scan;
            base_in_odom_keyframe_ = base_in_odom_;
            // std::cout << "add new key frame!" << std::endl;
        }
        else
        {

            ld_free(curr_ldp_scan);

        }

        last_icp_time_ = time;
        prev_base_in_odom_ = base_in_odom_;
        } // end of ScanMatchWithPLICP

    // 回调函数
void LaserOdomNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {

        LaserOdomNode::getCornerPoints(scan_msg);
        tf2::Transform laser_in_tf_odom;
        
        //PLICP

        //first scan, initialize
        current_time_ = scan_msg->header.stamp;

        if (!initialized_)
        {

            LaserOdomNode::createCache(scan_msg);

            // cache the static tf from base to laser
            // if(!getBaseToLaserTf(scan_msg->header.frame_id)) distribustion
            if(!LaserOdomNode::getBaseToLaserTf(scan_msg->header.frame_id))
            {
                RCLCPP_ERROR(this->get_logger(), "Skipping scan");
                return;
            } // get base to laser tf true

            LaserOdomNode::laserScanToLDP(scan_msg, prev_ldp_scan_);
            last_icp_time_ = current_time_;
            prev_laser_in_tf_odom_ = laser_in_tf_odom;
            initialized_ = true;
        } // initialization finished

        // Step 1, data type convert
        start_time_ = std::chrono::steady_clock::now();

        LDP curr_ldp_scan;
        LaserOdomNode::laserScanToLDP(scan_msg, curr_ldp_scan);
        LaserOdomNode::scanMatchWithPLICP(curr_ldp_scan, current_time_);
        end_time_ = std::chrono::steady_clock::now();
        time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
        RCLCPP_INFO(this->get_logger(),"PLICP compute cost: %.5f sec", time_used_.count());
        // std::cout << "LaserScanToLDP time: " << time_used_.count() << " seconds." << std::endl;

        // Step 2, PLICP

      }; // end of scanCallback   

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  auto node = std::make_shared<LaserOdomNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
