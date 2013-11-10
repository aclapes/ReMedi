#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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
	Monitorizer(float posCorrespThres);
	~Monitorizer(void);

	void monitor(PointCloudPtr cloudA, PointCloudPtr cloudB);
	void handleCloudjectDrops(std::vector<Cloudject>& cloudjects);
	//void handleCloudjectPicks(std::vector<Cloudject>& cloudjects);

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

	float m_posCorrespThres; // dist thres to state wheter two cloudjects are the same based on pos criterion
};

