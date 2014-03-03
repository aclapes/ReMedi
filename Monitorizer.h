#pragma once

#include <opencv2/opencv.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "InteractiveRegisterer.h"
#include "TableModeler.h"
#include "DepthFrame.h"

#include "Cloudject.hpp"
#include "CloudjectDetector.h"
#include "CloudjectFPFHRecognizer.h"

#include "MonitorizerParams.hpp"

#include "conversion.h"


class Monitorizer
{
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;
	typedef Cloudject<PointT, pcl::FPFHSignature33> Cloudject;

public:
	Monitorizer(InteractiveRegisterer*, TableModeler*);
	Monitorizer(InteractiveRegisterer*, TableModeler*, MonitorizerParams);
	~Monitorizer(void);

	void setParams(MonitorizerParams);

	void monitor(DepthFrame, DepthFrame);

	void updateHistory(DepthFrame, DepthFrame);
	bool isHistoryComplete();

	void bufferDepthFrame(std::vector<DepthFrame>&, DepthFrame);
	bool isBufferFilled(std::vector<DepthFrame>);

	void handleCloudjectDrops();
	//void handleCloudjectPicks(std::vector<Cloudject>& cloudjects);

	void visualizeCloudjects(pcl::visualization::PCLVisualizer::Ptr pViz);

private:
	void segmentMotion(cv::Mat&, cv::Mat&, float); // threshold and motion mask
	void segmentMotionInView(std::vector<DepthFrame>, float, cv::Mat&);

	// Temporal coherence per pixel segmentation
	void segmentPerPixelStatics(cv::Mat&, cv::Mat&, float);
	void segmentPerPixelStaticsInView(std::vector<DepthFrame>, float, cv::Mat&);

	// Temporal coherence per cluster centroid position constance

	void segmentStatics( std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, 
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, float );
	void segmentStaticsInView( std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, std::vector<std::vector<Eigen::Vector4f>>&, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, float );


	void detectCloudjects(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr,
		 std::vector<Cloudject>&);

	void extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr,
		 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, float leafSize);

	void extractClustersFromView(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
		float leafSize, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

	void appeared(std::vector<Cloudject> detecteds, std::vector<Cloudject>& appeareds);
	bool compareEquals(Cloudject, Cloudject);

	void drop(std::vector<Cloudject> cloudjects);
	//void pick(std::vector<Cloudject> cloudjects);

	// Members
	MonitorizerParams m_Params;

	InteractiveRegisterer* m_pIR;
	TableModeler* m_pTableModeler;

	std::vector<DepthFrame> m_DepthStreamBufferA;
	std::vector<DepthFrame> m_DepthStreamBufferB;

	pcl::visualization::PCLVisualizer::Ptr m_pViz;
	int m_SceneVp;
	int m_ObjectsAVp, m_ObjectsBVp;

	DepthFrame m_dFrameA;
	DepthFrame m_dFrameB;

	PointCloudPtr m_CloudA;
	PointCloudPtr m_CloudB;

	CloudjectDetector			m_cjDetector;
	CloudjectFPFHRecognizer		m_cjRecognizer;

	std::vector<Cloudject> m_cloudjects; // yet present

	float m_LeafSize;
	float m_PosCorrespThresh; // dist thres to state wheter two cloudjects are the same based on pos criterion

	std::vector<std::vector<Eigen::Vector4f> > m_centroidsA;
	std::vector<std::vector<Eigen::Vector4f> > m_centroidsB;
};

