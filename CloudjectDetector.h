#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>

#include "Cloudject.hpp"

class CloudjectDetector
{
	typedef Cloudject<pcl::PointXYZ, pcl::FPFHSignature33> Cloudject;

public:
	CloudjectDetector(void);
	~CloudjectDetector(void);

	void detect( pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr,
		std::vector<Cloudject>& );

private:
	// Methods
	void extractClustersFromView(pcl::PointCloud<pcl::PointXYZ>::Ptr, 
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

	void findCorrespondences(
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersB,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsA,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsB,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversA,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversB);

	float clustersBoxDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
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

	// Members

	std::vector<Cloudject> m_Cloudjects;
};

