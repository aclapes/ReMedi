#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/io/io.h>


void MatToPointCloud(cv::Mat&, pcl::PointCloud<pcl::PointXYZ>&);
void PointCloudToMat(pcl::PointCloud<pcl::PointXYZ>&, cv::Mat&);
void passthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ min, pcl::PointXYZ max, 
	pcl::PointCloud<pcl::PointXYZ>& cloudF);
