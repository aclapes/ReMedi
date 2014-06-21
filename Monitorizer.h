#pragma once

#include <opencv2/opencv.hpp>

#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "BackgroundSubtractor.h"
#include "InteractiveRegisterer.h"
#include "TableModeler.h"
#include "DepthFrame.h"

#include "Cloudject.hpp"
#include "CloudjectDetector.h"

#include "MotionSegmentator.h"

#include "MonitorizerParams.hpp"

#include "conversion.h"
#include "Sequence.h"
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
    Monitorizer();
	Monitorizer(BackgroundSubtractor::Ptr pBS, InteractiveRegisterer::Ptr pIR,
                TableModeler::Ptr pTM, CloudjectDetector::Ptr pCD);
	Monitorizer(const Monitorizer& rhs);
	~Monitorizer(void);
    Monitorizer& operator=(const Monitorizer& rhs);
    void clear();
    
    void setInputSequence(Sequence::Ptr pSequence);
    
    void setBackgroundSubtractor(BackgroundSubtractor::Ptr bs);
    void setRegisterer(InteractiveRegisterer::Ptr ir);
    void setTableModeler(TableModeler::Ptr tm);
    void setCloudjectDetector(CloudjectDetector::Ptr cd);
    
    void setParams(MonitorizerParams params);
    void setLeafSize(float leafSize);
    void setClusteringToleranceFactor(int factor);
    void setVisualization(bool visualize = false);
    
	void monitor(DetectionOutput& output);
    void process(DepthFrame dFrameA, DepthFrame dFrameB);

	void updateFrameHistory(DepthFrame, DepthFrame);
	bool isHistoryComplete();

	void bufferDepthFrame(std::vector<DepthFrame>&, DepthFrame);
	bool isBufferFilled(std::vector<DepthFrame>);
    
    DetectionOutput getObjectDetectionOutput();
    
    typedef boost::shared_ptr<Monitorizer> Ptr;
    
private:
	void segmentMotion(float, cv::Mat&, cv::Mat&); // threshold and motion mask
	void segmentMotionInView(std::vector<DepthFrame>, float, cv::Mat&);
    
    void visualize();

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
    Sequence::Ptr m_pSeq;
	MonitorizerParams m_Params;

    BackgroundSubtractor::Ptr m_pBS;
    InteractiveRegisterer::Ptr m_pIR;
    TableModeler::Ptr m_pTM;
    CloudjectDetector::Ptr m_pCD;
    //	MotionSegmentator m_MotionSegmentator;
    
//    pcl::visualization::PCLVisualizer::Ptr m_pViz;
    bool m_bVisualize;

	int m_SceneVp;
	int m_ObjectsVpA, m_ObjectsVpB;

	DepthFrame m_dFrameA;
	DepthFrame m_dFrameB;

	PointCloudPtr m_CloudA;
	PointCloudPtr m_CloudB;

	std::vector<Cloudject> m_cloudjects; // yet present

	float m_LeafSize;
    int m_ClusterTolFactor;
	float m_PosCorrespThresh; // dist thres to state wheter two cloudjects are the same based on pos criterion
    
	vector<vector<Eigen::Vector4f> > m_centroidsA;
	vector<vector<Eigen::Vector4f> > m_centroidsB;
    
    float m_TextSize;
    vector<string> m_TextsVpA, m_TextsVpB;
    
    PointCloudPtr m_pInteractionCloudA, m_pInteractionCloudB;
    vector<PointCloudPtr> m_InteractorClustersA, m_InteractorClustersB; // tabletop outliers
    
    vector<Cloudject> m_PresentCloudjects, m_AppearedCloudjects, m_DisappearedCloudjects;
    
    DetectionOutput m_DetectionOutput;
};

