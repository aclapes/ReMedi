#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Cloudject.hpp"
#include "CloudjectDetector.h"
#include "CloudjectFPFHRecognizer.h"

class Monitorizer
{
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;
	typedef Cloudject<PointT, pcl::FPFHSignature33> Cloudject;

public:
	Monitorizer(float leafSize, float posCorrespThres);
	~Monitorizer(void);

	void monitor(PointCloudPtr cloudA, PointCloudPtr cloudB);
	void handleCloudjectDrops();
	//void handleCloudjectPicks(std::vector<Cloudject>& cloudjects);

	void visualizeCloudjects(pcl::visualization::PCLVisualizer::Ptr pViz);

private:
	void appeared(std::vector<Cloudject> detecteds, std::vector<Cloudject>& appeareds);
	bool compareEquals(Cloudject, Cloudject);

	void drop(std::vector<Cloudject> cloudjects);
	//void pick(std::vector<Cloudject> cloudjects);

	// Members

	PointCloudPtr m_CloudA;
	PointCloudPtr m_CloudB;

	CloudjectDetector			m_cjDetector;
	CloudjectFPFHRecognizer		m_cjRecognizer;

	std::vector<Cloudject> m_cloudjects; // yet present

	float m_LeafSize;
	float m_PosCorrespThres; // dist thres to state wheter two cloudjects are the same based on pos criterion
};

