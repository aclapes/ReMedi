#pragma once

#include "InteractiveRegisterer.h"
#include "DepthFrame.h"
#include "ColorFrame.h"

#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
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

using namespace std;

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
    
    InteractiveRegisterer(const InteractiveRegisterer& other);

	void setNumPoints(int);

    void translate(const pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>&);
    void translate(const pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Vector4f, pcl::PointCloud<pcl::PointXYZ>&);
    
    void keyboardCallback(const pcl::visualization::KeyboardEvent&, void*);
    void mouseCallback(const pcl::visualization::MouseEvent&, void*);
    void ppCallback(const pcl::visualization::PointPickingEvent&, void*);
    
    void setDefaultCamera(pcl::visualization::PCLVisualizer::Ptr, int);
    
    void setInputFrames(ColorFrame, ColorFrame, DepthFrame, DepthFrame);
    
    void interact();
    
    void stop(pcl::visualization::PCLVisualizer&);

//    void align(DepthFrame&, DepthFrame&);

    void computeTransformation();
    
	bool loadTransformation(string filePath);
	void saveTransformation(string filePath);
    
	void registration(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&);
	void registration(DepthFrame, DepthFrame,
		pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&,
		bool backgroundPoints = true, bool userPoints = true);

    pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> getRegisteredClouds();
    
	void visualizeRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void visualizeRegistration(pcl::visualization::PCLVisualizer::Ptr, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void visualizeRegistration(DepthFrame, DepthFrame);
    
    pcl::PointXYZ getLeftRefPoint();
    pcl::PointXYZ getRightRefPoint();
    
private:
    void find_transformation (const pcl::PointCloud<pcl::PointXYZ>::Ptr, const pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Matrix4f&);
//    void align (const pcl::PointCloud<pcl::PointXYZ>::Ptr, const pcl::PointCloud<pcl::PointXYZ>::Ptr,
//                pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr,
//                pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
    
    // Members
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    int viewport_left_, viewport_right_;
    
    ColorFrame m_ColorFrameA, m_ColorFrameB;
    DepthFrame m_DepthFrameA, m_DepthFrameB;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_, cloud_right_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_left_, aligned_cloud_right_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr lefties_, righties_;
    vector<int> lefties_idx_, righties_idx_;

    bool reallocate_points_;
    int num_points_;
    
    bool must_translate_, must_align_;
    
    pcl::CorrespondencesPtr corresps_;

	Eigen::Vector4f m_tLeft, m_tRight;
	Eigen::Matrix4f m_Transformation;
};