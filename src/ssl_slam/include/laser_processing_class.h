// Author of SSL_SLAM2: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef LASER_PROCESSING_CLASS_H
#define LASER_PROCESSING_CLASS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "lidar.h"

struct Double2d
{
    int id;
    double value;
    Double2d(int id_in, double value_in);
};

struct PointsInfo
{
    int layer;
    double time;
    PointsInfo(int layer_in, double time_in);
};

class LaserProcessingClass
{
public:
    LaserProcessingClass();
    void init(lidar::Lidar lidar_param_in);
    void featureExtraction(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_in,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_edge,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_surf);
    void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_in,
                                     std::vector<Double2d> &cloudCurvature,
                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_edge,
                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_surf);

private:
    lidar::Lidar lidar_param;
};

#endif // LASER_PROCESSING_CLASS_H