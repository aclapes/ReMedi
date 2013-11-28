#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "conversion.h"

#include <vector>

#include "Cloudject.hpp"

class CloudjectDetector
{
	typedef pcl::PointXYZ PointT;
	typedef Cloudject<PointT, pcl::FPFHSignature33> Cloudject;

public:
	CloudjectDetector(void);
	~CloudjectDetector(void);
	
	void setInputClouds( pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr );

	void setLeafSize(float);
	void setMaxCorrespondenceDistance(float);

	void detect(std::vector<Cloudject>&);

private:
	// Methods
	void computeCentroids(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, 
		std::vector<Eigen::Vector4f>& centroids);

	void extractClustersFromView(pcl::PointCloud<pcl::PointXYZ>::Ptr,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

	void findCorrespondences(
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersB,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsA,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsB,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversA,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversB);

	void findCorrespondences2(
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersB,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsA,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsB,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversA,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversB);

	float clustersBoxDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	
	float centroidsDistance(Eigen::Vector4f, Eigen::Vector4f);
	float clustersCentroidsDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	
	void variate(
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& P, 
		int n, 
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& C,
		std::vector<
			std::pair<
				std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,
				std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
			>
		>& V);

	void variations(
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& P, 
		int n,
		// vector< pair<vector<cloudptr>,vector<cloudptr>> >
		std::vector<
			std::pair<
				std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,
				std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
			>
		>& V);

	float correspondenceDistance(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&);

	//
	// Members
	//

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCloudA, m_pCloudB;

	float m_LeafSize;
	float m_MaxCorrespondenceDistance;

	std::vector<Cloudject> m_Cloudjects;
};

