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

// ros lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "map_optimization.h"

using namespace std::chrono_literals;

class MapOptimizationNode : public rclcpp::Node
{
public:
    MapOptimizationNode() : Node("map_optimization_node")
    {
        // Initialize parameters
        this->declare_parameter("scan_period", 0.1);
        this->declare_parameter("vertical_angle", 2.0);
        this->declare_parameter("max_dis", 60.0);
        this->declare_parameter("min_dis", 2.0);
        this->declare_parameter("scan_line", 64);
        this->declare_parameter("map_resolution", 0.4);
        this->declare_parameter("map_path", "");
        this->declare_parameter("min_map_update_distance", 1.0);
        this->declare_parameter("min_map_update_angle", 30.0);
        this->declare_parameter("min_map_update_frame", 8);

        // Get parameters
        scan_period_ = this->get_parameter("scan_period").as_double();
        vertical_angle_ = this->get_parameter("vertical_angle").as_double();
        max_dis_ = this->get_parameter("max_dis").as_double();
        min_dis_ = this->get_parameter("min_dis").as_double();
        scan_line_ = this->get_parameter("scan_line").as_int();
        map_resolution_ = this->get_parameter("map_resolution").as_double();
        map_path_ = this->get_parameter("map_path").as_string();
        min_map_update_distance_ = this->get_parameter("min_map_update_distance").as_double();
        min_map_update_angle_ = this->get_parameter("min_map_update_angle").as_double();
        min_map_update_frame_ = this->get_parameter("min_map_update_frame").as_int();

        // Initialize map optimization
        mapOptimization_.init(map_resolution_);
        last_pose_.translation().x() = 100;

        // Create subscribers
        edge_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_edge", 100,
            std::bind(&MapOptimizationNode::edgeCallback, this, std::placeholders::_1));

        surf_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_surf", 100,
            std::bind(&MapOptimizationNode::surfCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 100,
            std::bind(&MapOptimizationNode::odomCallback, this, std::placeholders::_1));

        // Create publisher
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", 100);

        // Create service
        save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "save_map",
            std::bind(&MapOptimizationNode::saveMapCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Create timer for map optimization
        timer_ = this->create_wall_timer(
            2ms, std::bind(&MapOptimizationNode::mapOptimizationCallback, this));
    }

private:
    void edgeCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pointCloudEdgeBuf_.push(msg);
    }

    void surfCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pointCloudSurfBuf_.push(msg);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        odometryBuf_.push(msg);
    }

    bool saveMapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        mapOptimization_.optimizeGraph(stamp_, mapOptimization_.getFrameNum() - 1);
        stamp_ = mapOptimization_.getFrameNum();
        mapOptimization_.saveMap(map_path_);
        res->success = true;
        res->message = "write feature map to folder ssl_slam2/map ...";
        return true;
    }

    void mapOptimizationCallback()
    {
        if (!odometryBuf_.empty() && !pointCloudSurfBuf_.empty() && !pointCloudEdgeBuf_.empty())
        {
            std::lock_guard<std::mutex> lock(mutex_);

            // Check time alignment
            if (!odometryBuf_.empty() &&
                (odometryBuf_.front()->header.stamp.sec < pointCloudSurfBuf_.front()->header.stamp.sec - 0.5 * scan_period_ ||
                 odometryBuf_.front()->header.stamp.sec < pointCloudEdgeBuf_.front()->header.stamp.sec - 0.5 * scan_period_))
            {
                RCLCPP_WARN(this->get_logger(), "time stamp unaligned error and odom discarded, pls check your data --> map optimization");
                odometryBuf_.pop();
                return;
            }

            if (!pointCloudSurfBuf_.empty() &&
                (pointCloudSurfBuf_.front()->header.stamp.sec < odometryBuf_.front()->header.stamp.sec - 0.5 * scan_period_ ||
                 pointCloudSurfBuf_.front()->header.stamp.sec < pointCloudEdgeBuf_.front()->header.stamp.sec - 0.5 * scan_period_))
            {
                pointCloudSurfBuf_.pop();
                RCLCPP_INFO(this->get_logger(), "time stamp unaligned with extra point cloud, pls check your data --> map optimization");
                return;
            }

            if (!pointCloudEdgeBuf_.empty() &&
                (pointCloudEdgeBuf_.front()->header.stamp.sec < odometryBuf_.front()->header.stamp.sec - 0.5 * scan_period_ ||
                 pointCloudEdgeBuf_.front()->header.stamp.sec < pointCloudSurfBuf_.front()->header.stamp.sec - 0.5 * scan_period_))
            {
                pointCloudEdgeBuf_.pop();
                RCLCPP_INFO(this->get_logger(), "time stamp unaligned with extra point cloud, pls check your data --> map optimization");
                return;
            }

            // Process aligned data
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*pointCloudEdgeBuf_.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf_.front(), *pointcloud_surf_in);

            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(
                odometryBuf_.front()->pose.pose.orientation.w,
                odometryBuf_.front()->pose.pose.orientation.x,
                odometryBuf_.front()->pose.pose.orientation.y,
                odometryBuf_.front()->pose.pose.orientation.z));
            current_pose.pretranslate(Eigen::Vector3d(
                odometryBuf_.front()->pose.pose.position.x,
                odometryBuf_.front()->pose.pose.position.y,
                odometryBuf_.front()->pose.pose.position.z));

            auto pointcloud_time = odometryBuf_.front()->header.stamp;
            pointCloudEdgeBuf_.pop();
            pointCloudSurfBuf_.pop();
            odometryBuf_.pop();

            total_frame_++;
            if (total_frame_ % 10 == 0)
                RCLCPP_INFO(this->get_logger(), "total_frame %d", total_frame_);
            update_count_++;

            Eigen::Isometry3d delta_transform = last_pose_.inverse() * current_pose;
            double displacement = delta_transform.translation().squaredNorm();
            double angular_change = delta_transform.linear().eulerAngles(2, 1, 0)[0] * 180 / M_PI;
            if (angular_change > 90)
                angular_change = fabs(180 - angular_change);

            if (displacement > min_map_update_distance_ ||
                angular_change > min_map_update_angle_ ||
                update_count_ > min_map_update_frame_)
            {
                last_pose_ = current_pose;
                update_count_ = 0;
                mapOptimization_.addPoseToGraph(pointcloud_edge_in, pointcloud_surf_in, current_pose);
            }

            if (total_frame_ % 30 == 0)
            {
                sensor_msgs::msg::PointCloud2 PointsMsg;
                pcl::toROSMsg(*(mapOptimization_.edgeMap) + *(mapOptimization_.surfMap), PointsMsg);
                RCLCPP_INFO(this->get_logger(), "Edge Map size:%d, Surf Map Size:%d",
                            (mapOptimization_.edgeMap)->points.size(),
                            (mapOptimization_.surfMap)->points.size());
                PointsMsg.header.stamp = pointcloud_time;
                PointsMsg.header.frame_id = "map";
                map_pub_->publish(PointsMsg);
            }
        }
    }

    // Parameters
    double scan_period_;
    double vertical_angle_;
    double max_dis_;
    double min_dis_;
    int scan_line_;
    double map_resolution_;
    std::string map_path_;
    double min_map_update_distance_;
    double min_map_update_angle_;
    int min_map_update_frame_;

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr edge_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr surf_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data buffers
    std::mutex mutex_;
    std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudEdgeBuf_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudSurfBuf_;

    // Map optimization
    MapOptimizationClass mapOptimization_;
    Eigen::Isometry3d last_pose_ = Eigen::Isometry3d::Identity();
    int stamp_ = 0;
    int update_count_ = 0;
    int total_frame_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOptimizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}