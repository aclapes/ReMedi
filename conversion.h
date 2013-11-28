#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>


void MatToPointCloud(cv::Mat&, pcl::PointCloud<pcl::PointXYZ>&);

void MatToPointCloud(cv::Mat, cv::Mat, pcl::PointCloud<pcl::PointXYZ>&);

void PointCloudToMat(pcl::PointCloud<pcl::PointXYZ>&, cv::Mat&);

void MaskDensePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr, cv::Mat, pcl::PointCloud<pcl::PointXYZ>&);

void passthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ min, pcl::PointXYZ max, 
	pcl::PointCloud<pcl::PointXYZ>& cloudF);

void enclosure(cv::Mat src, cv::Mat& dst, int size);

void biggestEuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float clusterTol, 
	pcl::PointCloud<pcl::PointXYZ>& cluster);

float euclideanDistance(Eigen::Vector4f, Eigen::Vector4f);