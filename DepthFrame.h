#pragma once

#include "Frame.h"

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// OpenCV <-> PCL
#include "conversion.h"

class DepthFrame : public Frame
{
public:
	// Constructors
	DepthFrame(void);
	DepthFrame(cv::Mat);
	DepthFrame(const DepthFrame&);
	~DepthFrame(void);

	// Operators
	DepthFrame& operator=(const DepthFrame& other);

	// Methods
	bool isValid();
	cv::Mat getDepthMap(cv::Mat&);
	cv::Mat getUserFreeDepthMap(cv::Mat&);
	pcl::PointCloud<pcl::PointXYZ>& getPointCloud();
	pcl::PointCloud<pcl::PointXYZ>& getUserFreePointCloud();

private:
	// m_Mat from parent is raw (uint16 from which last 3 pixels contain player idx value)
	cv::Mat m_projDepthMat; // Here is the actual depth distance in mm (projective coordinates)
	cv::Mat m_UIDMat;		// Here the player indices

};

