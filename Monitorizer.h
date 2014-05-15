#pragma once

#include <opencv2/opencv.hpp>

#include <boost/shared_ptr.hpp>

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
#include "DetectionOutput.h"

using namespace std;

class Monitorizer
{
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;
	typedef LFCloudject<PointT, pcl::FPFHSignature33> Cloudject;
    typedef boost::shared_ptr<Cloudject> CloudjectPtr;

public:
	Monitorizer(InteractiveRegisterer ir, TableModeler tm, CloudjectDetector cd);
	Monitorizer(const Monitorizer& rhs);
	~Monitorizer(void);
    Monitorizer& operator=(const Monitorizer& rhs);
    void clear();
    
	void setParams(MonitorizerParams);
    void setLeafSize(float leafSize);
    void setClusteringToleranceFactor(int factor);

	void monitor(DepthFrame, DepthFrame);
    void display();

	void updateFrameHistory(DepthFrame, DepthFrame);
	bool isHistoryComplete();

	void bufferDepthFrame(std::vector<DepthFrame>&, DepthFrame);
	bool isBufferFilled(std::vector<DepthFrame>);

	void visualizeCloudjects(pcl::visualization::PCLVisualizer::Ptr pViz);
    
    DetectionOutput getObjectDetectionOutput();

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

	void extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr,
		 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, float leafSize = .005f);

	void extractClustersFromView(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters, float leafSize = .005f);
    
    void classifyTabletop(vector<PointCloudPtr> tableTopRegionClusters,
                          vector<PointCloudPtr> interactionRegionClusters,
                          vector<PointCloudPtr>& actorClusters,
                          vector<PointCloudPtr>& interactorClusters,
                          float leafSize = .005f);
    bool isInteractive(PointCloudPtr tabletopRegionCluster,
                       vector<PointCloudPtr> interactionRegionClusters,
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
//	MotionSegmentator m_MotionSegmentator;

	std::vector<Cloudject> m_cloudjects; // yet present

	float m_LeafSize;
    int m_ClusterTolFactor;
	float m_PosCorrespThresh; // dist thres to state wheter two cloudjects are the same based on pos criterion
    
	std::vector<std::vector<Eigen::Vector4f> > m_centroidsA;
	std::vector<std::vector<Eigen::Vector4f> > m_centroidsB;
    
    float m_TextSize;
    vector<string> m_TextsVpA, m_TextsVpB;
    
    PointCloudPtr m_pInteractionCloudA, m_pInteractionCloudB;
    vector<PointCloudPtr> m_InteractorClustersA, m_InteractorClustersB; // tabletop outliers
    
    vector<CloudjectPtr> m_PresentCloudjects, m_AppearedCloudjects, m_DisappearedCloudjects;
};

