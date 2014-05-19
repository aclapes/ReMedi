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
	typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
    
public:    
	InteractiveRegisterer();
    
    InteractiveRegisterer(const InteractiveRegisterer& other);

	void setNumPoints(int);

    void translate(const PointCloudPtr, PointT, PointCloud&);
    void translate(const PointCloudPtr, Eigen::Vector4f, PointCloud&);
    
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
    
    void registration(DepthFrame, DepthFrame,
                      PointCloud&, PointCloud&,
                      bool backgroundPoints = true, bool userPoints = true);
    
	void registration(PointCloudPtr, PointCloudPtr, PointCloud&, PointCloud&);
    void deregistration(PointCloudPtr pRegCloudA, PointCloudPtr pRegCloudB, PointCloud& cloudA, PointCloud& cloudB);
    void deregistration(vector<PointCloudPtr> pRegCloudsA, vector<PointCloudPtr> pRegCloudsB, vector<PointCloudPtr>& pCloudsA, vector<PointCloudPtr>& pCloudsB);

    pair<PointCloudPtr,PointCloudPtr> getRegisteredClouds();
    
	void visualizeRegistration(PointCloudPtr, PointCloudPtr);
	void visualizeRegistration(pcl::visualization::PCLVisualizer::Ptr, 
		PointCloudPtr, PointCloudPtr);
	void visualizeRegistration(DepthFrame, DepthFrame);
    
    PointT getLeftRefPoint();
    PointT getRightRefPoint();
    
    typedef boost::shared_ptr<InteractiveRegisterer> Ptr;
    
private:
    void find_transformation (const PointCloudPtr, const PointCloudPtr, Eigen::Matrix4f&);
//    void align (const PointCloudPtr, const PointCloudPtr,
//                PointCloudPtr, PointCloudPtr,
//                PointCloudPtr, PointCloudPtr);
    
    // Members
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    int viewport_left_, viewport_right_;
    
    ColorFrame m_ColorFrameA, m_ColorFrameB;
    DepthFrame m_DepthFrameA, m_DepthFrameB;
    
    PointCloudPtr cloud_left_, cloud_right_;
    PointCloudPtr aligned_cloud_left_, aligned_cloud_right_;
    
    PointCloudPtr lefties_, righties_;
    vector<int> lefties_idx_, righties_idx_;

    bool reallocate_points_;
    int num_points_;
    
    bool must_translate_, must_align_;
    
    pcl::CorrespondencesPtr corresps_;

	Eigen::Vector4f m_tLeft, m_tRight;
	Eigen::Matrix4f m_Transformation, m_InverseTransformation;
};