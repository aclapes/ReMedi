#pragma once

#include "DepthFrame.h"
#include "Table.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>

#include <pcl/visualization/pcl_visualizer.h>

class TableModeler
{
public:
	TableModeler();
	~TableModeler();

	void setInputClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB);

	void setLeafSize(float);
	void setNormalRadius(float);
	void setSACIters(int);
	void setSACDistThresh(float);
	void setYOffset(float);

	void model(pcl::PointCloud<pcl::PointXYZ>&);
	void model(pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&);

	void segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>&);
	void segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&);

private:

	void estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>&,
		pcl::PointXYZ& min, pcl::PointXYZ& max, Eigen::Affine3f& transformation);
	bool isTowardsLookingDirectionPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane);

	void segment(pcl::PointCloud<pcl::PointXYZ>::Ptr, 
		pcl::PointXYZ, pcl::PointXYZ, float offset, Eigen::Affine3f, pcl::PointCloud<pcl::PointXYZ>&);

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCloudA, m_pCloudB;
	pcl::visualization::PCLVisualizer::Ptr m_pViz;
	float m_LeafSize;
	float m_NormalRadius;
	float m_SACIters;
	float m_SACDistThresh;

	pcl::PointXYZ m_MinA, m_MaxA, m_MinB, m_MaxB;
	float m_OffsetA, m_OffsetB; // sum to the axis corresponding to plane's normal (probably the y dimension)
	Eigen::Affine3f m_ytonA, m_ytonB; // y axis to n plane normal vector transformtion
};