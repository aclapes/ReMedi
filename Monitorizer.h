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

#include "MotionSegmentator.h"

#include "MonitorizerParams.hpp"

#include "conversion.h"

using namespace std;

class Monitorizer
{
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;
	typedef LFCloudject<PointT, pcl::FPFHSignature33> Cloudject;

public:
	Monitorizer(InteractiveRegisterer ir, TableModeler tm, CloudjectDetector cd);
	~Monitorizer(void);

	void setParams(MonitorizerParams);
    void setLeafSize(float leafSize);
    void setClusteringToleranceFactor(int factor);
    void setMinimumClusterSize(int npoints);
    void setMaximumClusterSize(int npoints);

	void monitor(DepthFrame, DepthFrame);

	void updateFrameHistory(DepthFrame, DepthFrame);
	bool isHistoryComplete();

	void bufferDepthFrame(std::vector<DepthFrame>&, DepthFrame);
	bool isBufferFilled(std::vector<DepthFrame>);

	void handleCloudjectDrops();
	//void handleCloudjectPicks(std::vector<Cloudject>& cloudjects);

	void visualizeCloudjects(pcl::visualization::PCLVisualizer::Ptr pViz);

private:
	void segmentMotion(float, cv::Mat&, cv::Mat&); // threshold and motion mask
	void segmentMotionInView(std::vector<DepthFrame>, float, cv::Mat&);

	// Temporal coherence per pixel segmentation
//	void segmentPerPixelStatics(cv::Mat&, cv::Mat&, float);
//	void segmentPerPixelStaticsInView(std::vector<DepthFrame>, float, cv::Mat&);

	// Temporal coherence per cluster centroid position constance

//	void segmentStatics( std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, 
//		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, float );
//	void segmentStaticsInView( std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, std::vector<std::vector<Eigen::Vector4f> >&, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, float );

	void detectCloudjects(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr,
		 std::vector<Cloudject>&, float leafSize = .005f);

	void extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr,
		 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, float leafSize = .005f);

	void extractClustersFromView(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters, float leafSize = .005f);

	void appeared(std::vector<Cloudject> detecteds, std::vector<Cloudject>& appeareds);
	bool compareEquals(Cloudject, Cloudject);

	void drop(std::vector<Cloudject> cloudjects);
	//void pick(std::vector<Cloudject> cloudjects);
    
    void updateCentroids(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, std::vector<Eigen::Vector4f>& centroids);
    
    void classifyActorsAndInteractors(vector<PointCloudPtr> tableTopRegionClusters,
                                      vector<PointCloudPtr> interactionRegionClusters,
                                      vector<PointCloudPtr>& actorClusters,
                                      vector<PointCloudPtr>& interactorClusters,
                                      float leafSize = .005f);
    
	// Members
	MonitorizerParams m_Params;

	InteractiveRegisterer m_ir;
	TableModeler m_tm;

	pcl::visualization::PCLVisualizer::Ptr m_pViz;
	int m_SceneVp;
	int m_ObjectsVpA, m_ObjectsVpB;

	DepthFrame m_dFrameA;
	DepthFrame m_dFrameB;

	PointCloudPtr m_CloudA;
	PointCloudPtr m_CloudB;

	CloudjectDetector m_CloudjectDetector;
	MotionSegmentator m_MotionSegmentator;

	std::vector<Cloudject> m_cloudjects; // yet present

	float m_LeafSize;
    int m_ClusterTolFactor;
	float m_PosCorrespThresh; // dist thres to state wheter two cloudjects are the same based on pos criterion
    int m_MinClusterSize, m_MaxClusterSize;
    
	std::vector<std::vector<Eigen::Vector4f> > m_centroidsA;
	std::vector<std::vector<Eigen::Vector4f> > m_centroidsB;
    
    float m_TextSize;
    vector<string> m_TextsVpA, m_TextsVpB;
};

