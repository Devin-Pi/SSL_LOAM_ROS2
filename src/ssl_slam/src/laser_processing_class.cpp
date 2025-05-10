// Author of SSL_SLAM2: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "laser_processing_class.h"
#include <rclcpp/logging.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
void LaserProcessingClass::init(lidar::Lidar lidar_param_in)
{
    lidar_param = lidar_param_in;
}

void LaserProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_in,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_edge,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_surf)
{
    std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*pc_in,  indices); // ROS1
    pcl::removeNaNFromPointCloud(*pc_in, indices); // ROS2

    // Coordinate transform
    for (auto &point : pc_in->points)
    {
        double new_x = point.z;
        double new_y = -point.x;
        double new_z = -point.y;
        point.x = new_x;
        point.y = new_y;
        point.z = new_z;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> laserCloudScans;
    // Remove small cluster (< 30 consecutive points)
    double last_angle = atan2(pc_in->points[0].z, pc_in->points[0].y) * 180 / M_PI;
    int count = 0;
    int point_size = pc_in->points.size() - 1;
    int out_of_range_count = 0;

    for (size_t i = 0; i < pc_in->points.size(); i++)
    {
        int scanID = 0;
        double angle = atan2(pc_in->points[i].x, pc_in->points[i].z) * 180 / M_PI;
        count++;

        if (fabs(angle - last_angle) > 0.05)
        {
            if (count > 30)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
                for (int k = 0; k < count; k++)
                {
                    int index = i - count + k + 1;
                    double distance = sqrt(
                        pc_in->points[index].x * pc_in->points[index].x +
                        pc_in->points[index].y * pc_in->points[index].y +
                        pc_in->points[index].z * pc_in->points[index].z);
                    if (distance > lidar_param.max_distance || distance < lidar_param.min_distance)
                    {
                        out_of_range_count++;
                        continue;
                    }
                    pc_temp->push_back(pc_in->points[index]);
                }
                if (!pc_temp->points.empty())
                    laserCloudScans.push_back(pc_temp);
            }
            count = 0;
            last_angle = angle;
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("laser_processing"),
                 "Out of range points: %d (%.2f%%)",
                 out_of_range_count,
                 100.0 * out_of_range_count / double(pc_in->points.size()));

    for (const auto &scan : laserCloudScans)
    {
        std::vector<Double2d> cloudCurvature;
        int total_points = scan->points.size() - 10;

        for (int j = 5; j < static_cast<int>(scan->points.size()) - 5; j++)
        {
            double angle_difference = fabs((atan2(scan->points[j - 5].y, scan->points[j - 5].z) -
                                            atan2(scan->points[j + 5].y, scan->points[j + 5].z)) *
                                           180 / M_PI);
            if (angle_difference > 5)
            {
                // Consider as a surf point
                pc_out_surf->push_back(scan->points[j]);
                continue;
            }

            double diffX = scan->points[j - 5].x + scan->points[j - 4].x + scan->points[j - 3].x +
                           scan->points[j - 2].x + scan->points[j - 1].x - 10 * scan->points[j].x +
                           scan->points[j + 1].x + scan->points[j + 2].x + scan->points[j + 3].x +
                           scan->points[j + 4].x + scan->points[j + 5].x;

            double diffY = scan->points[j - 5].y + scan->points[j - 4].y + scan->points[j - 3].y +
                           scan->points[j - 2].y + scan->points[j - 1].y - 10 * scan->points[j].y +
                           scan->points[j + 1].y + scan->points[j + 2].y + scan->points[j + 3].y +
                           scan->points[j + 4].y + scan->points[j + 5].y;

            double diffZ = scan->points[j - 5].z + scan->points[j - 4].z + scan->points[j - 3].z +
                           scan->points[j - 2].z + scan->points[j - 1].z - 10 * scan->points[j].z +
                           scan->points[j + 1].z + scan->points[j + 2].z + scan->points[j + 3].z +
                           scan->points[j + 4].z + scan->points[j + 5].z;

            cloudCurvature.emplace_back(j, diffX * diffX + diffY * diffY + diffZ * diffZ);
        }

        featureExtractionFromSector(scan, cloudCurvature, pc_out_edge, pc_out_surf);
    }
}

void LaserProcessingClass::featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_in,
                                                       std::vector<Double2d> &cloudCurvature,
                                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_edge,
                                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_surf)
{
    std::sort(cloudCurvature.begin(), cloudCurvature.end(),
              [](const Double2d &a, const Double2d &b)
              { return a.value < b.value; });

    int largestPickedNum = 0;
    std::vector<int> picked_points;
    int point_info_count = 0;

    for (int i = cloudCurvature.size() - 1; i >= 0; i--)
    {
        int ind = cloudCurvature[i].id;
        if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end())
        {
            if (cloudCurvature[i].value <= 0.1)
            {
                break;
            }

            largestPickedNum++;
            picked_points.push_back(ind);

            if (largestPickedNum <= 10)
            {
                pc_out_edge->push_back(pc_in->points[ind]);
                point_info_count++;
            }
            else
            {
                break;
            }

            // Check forward points
            for (int k = 1; k <= 5; k++)
            {
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                {
                    break;
                }
                picked_points.push_back(ind + k);
            }

            // Check backward points
            for (int k = -1; k >= -5; k--)
            {
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                {
                    break;
                }
                picked_points.push_back(ind + k);
            }
        }
    }

    // Add remaining points as surf points
    for (const auto &point : cloudCurvature)
    {
        if (std::find(picked_points.begin(), picked_points.end(), point.id) == picked_points.end())
        {
            pc_out_surf->push_back(pc_in->points[point.id]);
        }
    }
}

LaserProcessingClass::LaserProcessingClass() = default;

Double2d::Double2d(int id_in, double value_in) : id(id_in), value(value_in) {}

PointsInfo::PointsInfo(int layer_in, double time_in) : layer(layer_in), time(time_in) {}