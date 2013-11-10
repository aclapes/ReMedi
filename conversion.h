#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


void MatToPointCloud(cv::Mat&, pcl::PointCloud<pcl::PointXYZ>&);
void PointCloudToMat(pcl::PointCloud<pcl::PointXYZ>&, cv::Mat&);

