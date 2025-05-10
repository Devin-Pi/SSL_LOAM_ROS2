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
#include <atomic>

// ros2 lib
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/transform_datatypes.h"

// pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local lib
#include "lidar.h"
#include "laser_processing_class.h"

class LaserProcessingNode : public rclcpp::Node
{
public:
    LaserProcessingNode() : Node("laser_processing_node")
    {
        // Initialize parameters
        this->declare_parameter("scan_period", 0.1);
        this->declare_parameter("vertical_angle", 2.0);
        this->declare_parameter("max_dis", 60.0);
        this->declare_parameter("min_dis", 2.0);
        this->declare_parameter("scan_line", 64);
        this->declare_parameter("skip_frames", 1);

        // Get parameters
        double scan_period = this->get_parameter("scan_period").as_double();
        double vertical_angle = this->get_parameter("vertical_angle").as_double();
        double max_dis = this->get_parameter("max_dis").as_double();
        double min_dis = this->get_parameter("min_dis").as_double();
        int scan_line = this->get_parameter("scan_line").as_int();
        skip_frames = this->get_parameter("skip_frames").as_int();

        // Initialize lidar parameters
        lidar_param.setScanPeriod(scan_period);
        lidar_param.setVerticalAngle(vertical_angle);
        lidar_param.setLines(scan_line);
        lidar_param.setMaxDistance(max_dis);
        lidar_param.setMinDistance(min_dis);

        laserProcessing.init(lidar_param);

        // Create subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 100,
            std::bind(&LaserProcessingNode::velodyneHandler, this, std::placeholders::_1));

        // Create publishers
        pub_laser_cloud_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points_filtered", 100);
        pub_edge_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_edge", 100);
        pub_surf_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf", 100);

        // Start processing thread
        processing_thread_ = std::thread(&LaserProcessingNode::laser_processing, this);
        RCLCPP_INFO(this->get_logger(), "Laser processing node started");
    }

    ~LaserProcessingNode()
    {
        // Signal thread to stop
        running_ = false;

        // Wait for thread to finish
        if (processing_thread_.joinable())
        {
            processing_thread_.join();
        }
    }

private:
    void velodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        std::lock_guard<std::mutex> lock(mutex_lock_);
        pointCloudBuf.push(laserCloudMsg);
    }

    void laser_processing()
    {
        while (rclcpp::ok() && running_)
        {
            if (!pointCloudBuf.empty())
            {
                // Read data
                std::lock_guard<std::mutex> lock(mutex_lock_);
                if (pointCloudBuf.empty())
                    continue; // Double check after lock

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
                rclcpp::Time pointcloud_time = pointCloudBuf.front()->header.stamp;
                pointCloudBuf.pop();

                frame_count_++;
                if (frame_count_ % skip_frames != 0)
                    continue;

                // Create edge and surf point clouds
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZRGB>());

                auto start = std::chrono::system_clock::now();
                laserProcessing.featureExtraction(pointcloud_in, pointcloud_edge, pointcloud_surf);
                auto end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame_++;

                float time_temp = elapsed_seconds.count() * 1000;
                total_time_ += time_temp;
                if (total_frame_ % 10 == 0)
                    RCLCPP_INFO(this->get_logger(), "average laser processing time %f ms", total_time_ / total_frame_);

                // Publish filtered cloud
                sensor_msgs::msg::PointCloud2 laserCloudFilteredMsg;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
                *pointcloud_filtered += *pointcloud_edge;
                *pointcloud_filtered += *pointcloud_surf;
                pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
                laserCloudFilteredMsg.header.stamp = pointcloud_time;
                laserCloudFilteredMsg.header.frame_id = "base_link";
                pub_laser_cloud_filtered_->publish(laserCloudFilteredMsg);

                // Publish edge points
                sensor_msgs::msg::PointCloud2 edgePointsMsg;
                pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
                edgePointsMsg.header.stamp = pointcloud_time;
                edgePointsMsg.header.frame_id = "base_link";
                pub_edge_points_->publish(edgePointsMsg);

                // Publish surf points
                sensor_msgs::msg::PointCloud2 surfPointsMsg;
                pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
                surfPointsMsg.header.stamp = pointcloud_time;
                surfPointsMsg.header.frame_id = "base_link";
                pub_surf_points_->publish(surfPointsMsg);
            }

            // Sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    // Global variables for laser processing
    std::mutex mutex_lock_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudBuf;
    double total_time_ = 0;
    int total_frame_ = 0;
    int frame_count_ = 0;
    int skip_frames = 1;
    LaserProcessingClass laserProcessing;
    lidar::Lidar lidar_param;
    std::atomic<bool> running_{true};

    // Global publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_filtered_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_edge_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surf_points_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    std::thread processing_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}