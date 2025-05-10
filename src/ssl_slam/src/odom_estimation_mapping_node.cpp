// Author of SSL_SLAM2: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <memory>

// ros lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h> // new lib for Odom
#include <tf2/LinearMath/Transform.h>

// pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local lib
#include "lidar.h"
#include "odom_estimation_mapping_class.h"

class OdomEstimationMappingNode : public rclcpp::Node
{
public:
    OdomEstimationMappingNode() : Node("odom_estimation_mapping_node")
    {
        // Initialize parameters
        this->declare_parameter("scan_period", 0.1);
        this->declare_parameter("vertical_angle", 2.0);
        this->declare_parameter("max_dis", 60.0);
        this->declare_parameter("min_dis", 2.0);
        this->declare_parameter("scan_line", 64);
        this->declare_parameter("map_resolution", 0.05);
        this->declare_parameter("offset_x", 0.0);
        this->declare_parameter("offset_y", 0.0);
        this->declare_parameter("offset_yaw", 0.0);

        // Get parameters
        double scan_period = this->get_parameter("scan_period").as_double();
        double vertical_angle = this->get_parameter("vertical_angle").as_double();
        double max_dis = this->get_parameter("max_dis").as_double();
        double min_dis = this->get_parameter("min_dis").as_double();
        int scan_line = this->get_parameter("scan_line").as_int();
        double map_resolution = this->get_parameter("map_resolution").as_double();
        double offset_x = this->get_parameter("offset_x").as_double();
        double offset_y = this->get_parameter("offset_y").as_double();
        double offset_z = this->get_parameter("offset_yaw").as_double();

        // Set lidar parameters
        lidar_param_.setScanPeriod(scan_period);
        lidar_param_.setVerticalAngle(vertical_angle);
        lidar_param_.setLines(scan_line);
        lidar_param_.setMaxDistance(max_dis);
        lidar_param_.setMinDistance(min_dis);

        // Initialize odometry estimation
        odom_estimation_.init(lidar_param_, map_resolution);

        // Create subscribers
        edge_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_edge", 100,
            std::bind(&OdomEstimationMappingNode::edgeCloudCallback, this, std::placeholders::_1));

        surf_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_surf", 100,
            std::bind(&OdomEstimationMappingNode::surfCloudCallback, this, std::placeholders::_1));

        // Create publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
        edge_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/edge_map", 100);
        surf_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/surf_map", 100);

        // Create transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create timer for odometry estimation
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&OdomEstimationMappingNode::odomEstimationCallback, this));

        RCLCPP_INFO(this->get_logger(), "Odometry estimation node initialized");
    }

private:
    void edgeCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        edge_cloud_buf_.push(msg);
    }

    void surfCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        surf_cloud_buf_.push(msg);
    }

    void odomEstimationCallback()
    {
        if (edge_cloud_buf_.empty() || surf_cloud_buf_.empty()) // 两种点云数据都有了我才能进行下列的处理
            return;

        std::lock_guard<std::mutex> lock(mutex_);

        // Check time alignment
        if (surf_cloud_buf_.front()->header.stamp.sec <
            edge_cloud_buf_.front()->header.stamp.sec - 0.5 * lidar_param_.scan_period)
        {
            surf_cloud_buf_.pop();
            RCLCPP_INFO(this->get_logger(), "Time stamp unaligned with extra point cloud, please check your data");
            return;
        }

        if (edge_cloud_buf_.front()->header.stamp.sec <
            surf_cloud_buf_.front()->header.stamp.sec - 0.5 * lidar_param_.scan_period)
        {
            edge_cloud_buf_.pop();
            RCLCPP_INFO(this->get_logger(), "Time stamp unaligned with extra point cloud, please check your data");
            return;
        }

        // Convert ROS messages to PCL point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr surf_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        pcl::fromROSMsg(*edge_cloud_buf_.front(), *edge_cloud); // 前者转化为后者
        pcl::fromROSMsg(*surf_cloud_buf_.front(), *surf_cloud);

        rclcpp::Time pointcloud_time = edge_cloud_buf_.front()->header.stamp;
        edge_cloud_buf_.pop();
        surf_cloud_buf_.pop();

        // Initialize odometry if not done
        if (!is_odom_inited_)
        {
            odom_estimation_.initMapWithPoints(edge_cloud, surf_cloud);
            is_odom_inited_ = true;
            RCLCPP_INFO(this->get_logger(), "Odometry initialized");
        }
        else
        {
            auto start = std::chrono::system_clock::now();
            // 点云进行匹配，输出当前位姿
            odom_estimation_.updatePointsToMap(edge_cloud, surf_cloud);
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;

            total_frame_++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time_ += time_temp;

            if (total_frame_ % 1 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Average odom estimation time: %f ms", total_time_ / total_frame_);
            }
        }

        // Publish odometry
        // 1. 获取当前位姿:旋转矩阵+平移矩阵
        Eigen::Quaterniond q_current(odom_estimation_.odom.linear());
        Eigen::Vector3d t_current = odom_estimation_.odom.translation();

        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.frame_id = "map";      // 全局坐标系
        odom_msg->child_frame_id = "base_link"; // 机器人坐标系
        odom_msg->header.stamp = pointcloud_time;
        odom_msg->pose.pose.orientation.x = q_current.x(); // 姿态四元数
        odom_msg->pose.pose.orientation.y = q_current.y();
        odom_msg->pose.pose.orientation.z = q_current.z();
        odom_msg->pose.pose.orientation.w = q_current.w();
        odom_msg->pose.pose.position.x = t_current.x(); // 位置
        odom_msg->pose.pose.position.y = t_current.y();
        odom_msg->pose.pose.position.z = t_current.z();
        odom_pub_->publish(std::move(odom_msg));

        // Publish transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = pointcloud_time;
        transform.header.frame_id = "map"; // 表示一个从map到baselink的变换
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = t_current.x();
        transform.transform.translation.y = t_current.y();
        transform.transform.translation.z = t_current.z();
        transform.transform.rotation.x = q_current.x();
        transform.transform.rotation.y = q_current.y();
        transform.transform.rotation.z = q_current.z();
        transform.transform.rotation.w = q_current.w();
        tf_broadcaster_->sendTransform(transform);
    }

    // Member variables
    OdomEstimationClass odom_estimation_;
    lidar::Lidar lidar_param_;
    std::mutex mutex_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> edge_cloud_buf_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> surf_cloud_buf_;
    bool is_odom_inited_ = false;
    double total_time_ = 0;
    int total_frame_ = 0;

    // ROS2 specific members
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr edge_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr surf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_map_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomEstimationMappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}