#pragma once

#include "Frame.h"

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// OpenCV <-> PCL
#include "conversion.h"

#include "ColorFrame.h"

class DepthFrame : public Frame
{
public:
	// Constructors
	DepthFrame(void);
	DepthFrame(cv::Mat);
	DepthFrame(cv::Mat, cv::Mat);
	DepthFrame(const DepthFrame&);
	~DepthFrame(void);

	// Operators
	DepthFrame& operator=(const DepthFrame& other);

	// Methods
	bool isValid();
	cv::Mat getMask();
	void setMask(cv::Mat);
	cv::Mat getDepthMap(cv::Mat&);
	cv::Mat getDepthMap();
	cv::Mat getUserFreeDepthMap(cv::Mat&);
	void getPointCloud(pcl::PointCloud<pcl::PointXYZ>&);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud();
    void getColoredPointCloud(ColorFrame cframe, pcl::PointCloud<pcl::PointXYZRGB>& cloud);
	void getForegroundPointCloud(pcl::PointCloud<pcl::PointXYZ>&);
    void getForegroundPointCloud(cv::Mat mask, pcl::PointCloud<pcl::PointXYZ>&, bool combined = true);
	void getUserFreePointCloud(pcl::PointCloud<pcl::PointXYZ>&);
	void getForegroundUserFreePointCloud(pcl::PointCloud<pcl::PointXYZ>&);

    void show(std::string wndName);
    
private:
	// m_Mat from parent is raw (uint16 from which last 3 pixels contain player idx value)
	cv::Mat m_Mask;
	cv::Mat m_projDepthMat; // Here is the actual depth distance in mm (projective coordinates)
	cv::Mat m_UIDMat;		// Here the player indices

};

