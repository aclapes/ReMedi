#pragma once

#include "Cloudject.hpp"

#include <pcl/features/fpfh.h>

class CloudjectFPFHRecognizer
{
	typedef LFCloudject<pcl::PointXYZ, pcl::FPFHSignature33> Cloudject;
public:
	CloudjectFPFHRecognizer(void);
	CloudjectFPFHRecognizer(float normalRadius, float fpfhRadius);
	~CloudjectFPFHRecognizer(void);

	void describe(Cloudject&, pcl::PointCloud<pcl::FPFHSignature33>::Ptr);

	void recognize(Cloudject& cloudject);
	void recognize(std::vector<Cloudject>& cloudject);
	
private:
	void describeView(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::FPFHSignature33>::Ptr);


	// FPFH description parameters
	float m_NormalRadius;
	float m_FpfhRadius;

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> m_Estimator;
};

