#pragma once

#include "InteractiveRegisterer.h"
#include "DepthFrame.h"

#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/correspondence.h>
#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/common/common.h>

#include <opencv2/opencv.hpp>

static const float colors[][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
    {1, 0, 1},
    {1, 1, 0},
    {0, 1, 1},
    {.5, 0, 0},
    {0, .5, 0},
    {0, 0, .5},
    {1, 0, .5},
    {1, .5, 0},
    {1, .5, .5},
    {0, 1, .5},
    {.5, 1, 0},
    {.5, 0, .5},
    {0, .5, .5}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class InteractiveRegisterer
{
public:    
	InteractiveRegisterer();

	void setNumPoints(int);

    void translate(const pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void translate(const pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Vector4f, pcl::PointCloud<pcl::PointXYZ>::Ptr);
    
    void keyboardCallback(const pcl::visualization::KeyboardEvent&, void*);
    void mouseCallback(const pcl::visualization::MouseEvent&, void*);
    void ppCallback(const pcl::visualization::PointPickingEvent&, void*);
    
    void setDefaultCamera(pcl::visualization::PCLVisualizer::Ptr, int);
    
    void stop(pcl::visualization::PCLVisualizer&);

    void find_transformation (const pcl::PointCloud<pcl::PointXYZ>::Ptr, const pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Matrix4f&);
    void align (const pcl::PointCloud<pcl::PointXYZ>::Ptr, const pcl::PointCloud<pcl::PointXYZ>::Ptr, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);

    void computeTransformation(DepthFrame&, DepthFrame&);

	bool loadTransformation(const char*);
	void saveTransformation(const char*);

	void getRegisteredClouds(DepthFrame, DepthFrame, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr,
		bool userPoints = true);

	void visualizeRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void visualizeRegistration(pcl::visualization::PCLVisualizer::Ptr, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
    
    void interact();
    
private:
    pcl::visualization::PCLVisualizer::Ptr cloud_viewer_;
    int viewport_left_, viewport_right_;
        
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_, cloud_right_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr lefties_, righties_;
    std::vector<int> lefties_idx_, righties_idx_;

    bool reallocate_points_;
    int num_points_;
    
    bool must_translate_, must_align_;
    
    pcl::CorrespondencesPtr corresps_;

	Eigen::Vector4f m_tLeft, m_tRight;
	Eigen::Matrix4f m_Transformation;
};