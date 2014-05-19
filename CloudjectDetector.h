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

#include <boost/shared_ptr.hpp>

#include "conversion.h"

#include <vector>

#include "Cloudject.hpp"
#include "CloudjectModel.hpp"
#include "DetectionOutput.h"

using namespace std;

class CloudjectDetector
{
	typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
	typedef LFCloudject<pcl::PointXYZ, pcl::FPFHSignature33> Cloudject;
    typedef boost::shared_ptr<Cloudject> CloudjectPtr;
    typedef LFCloudjectModel<pcl::PointXYZ, pcl::FPFHSignature33> CloudjectModel;
    typedef boost::shared_ptr<CloudjectModel> CloudjectModelPtr;
    
public:
	CloudjectDetector(void);
    CloudjectDetector(const CloudjectDetector& rhs);
	~CloudjectDetector(void);
    void clear();
	
    CloudjectDetector& operator=(const CloudjectDetector& rhs);
    
	void setInputClouds(PointCloudPtr pCloudA,
                        PointCloudPtr pCloudB,
                        PointCloudPtr pTableTopCloudA,
                        PointCloudPtr pTableTopCloudB);
    
    void setInputClusters(vector<PointCloudPtr> clustersA, vector<PointCloudPtr> clustersB);
    
    void loadCloudjectModels(string dir);
    void setCloudjectModels(vector<CloudjectModelPtr> models);
    int getNumCloudjectModels();
	
    void setModelLeafSize(float);
    void setLeafSize(float);
	void setMaxCorrespondenceDistance(float);
    void setTemporalWindow(int tmpWnd);
    void setNormalRadius(float radius);
    void setFpfhRadius(float radius);
    void setScoreThreshold(float score);

	void detect();
    void getPresentCloudjects(vector<CloudjectPtr>& cloudjects);
    void getAppearedCloudjects(vector<CloudjectPtr>& cloudjects);
    void getDisappearedCloudjects(vector<CloudjectPtr>& cloudjects);
    
    DetectionOutput getDetectionOutput();
    
    typedef boost::shared_ptr<CloudjectDetector> Ptr;
    
private:
	// Methods
    void findInliers(vector<PointCloudPtr>, vector<PointCloudPtr>, vector<PointCloudPtr>&);
	void computeCentroids(vector<PointCloudPtr> clusters, 
		vector<Eigen::Vector4f>& centroids);

	void extractClustersFromView(PointCloudPtr,
		vector<PointCloudPtr>& clusters);
    
	void findCorrespondences(
		vector<PointCloudPtr> clustersA,
		vector<PointCloudPtr> clustersB,
		vector<PointCloudPtr>& correspsA,
		vector<PointCloudPtr>& correspsB,
		vector<PointCloudPtr>& leftoversA,
		vector<PointCloudPtr>& leftoversB);

	void makeInterviewCorrespondences(
		vector<PointCloudPtr> clustersA,
		vector<PointCloudPtr> clustersB,
		vector<PointCloudPtr>& correspsA,
		vector<PointCloudPtr>& correspsB,
		vector<PointCloudPtr>& leftoversA,
		vector<PointCloudPtr>& leftoversB);

    void makeSpatiotemporalCorrespondences(vector<CloudjectPtr> cloudjects, vector<CloudjectPtr>& appeared, vector<CloudjectPtr>& disappeared, vector< vector<CloudjectPtr> >& cloudjectsHistory);
    
	float clustersBoxDistance(PointCloudPtr, PointCloudPtr);
	
	float centroidsDistance(Eigen::Vector4f, Eigen::Vector4f);
	float clustersCentroidsDistance(PointCloudPtr, PointCloudPtr);
	
	void variate(
		vector<PointCloudPtr>& P, 
		int n, 
		vector<PointCloudPtr>& C,
		vector<
			pair<
				vector<PointCloudPtr>,
				vector<PointCloudPtr>
			>
		>& V);

	void variations(
		vector<PointCloudPtr>& P, 
		int n,
		// vector< pair<vector<cloudptr>,vector<cloudptr>> >
		vector<
			pair<
				vector<PointCloudPtr>,
				vector<PointCloudPtr>
			>
		>& V);

	float correspondenceDistance(vector<PointCloudPtr>&,
		vector<PointCloudPtr>&, vector<PointCloudPtr>&);

    
    float matchClusters(PointCloudPtr pSrcCloud, PointCloudPtr pTgtCloud);
    void matchClustersFromView(vector<PointCloudPtr> src, vector<PointCloudPtr> tgt, vector<PointCloudPtr>& matches);
    
    
    bool match(Cloudject src, Cloudject tgt);
    
    void recognize(vector< vector<CloudjectPtr> >& history);
    
    void greedyAssign(vector< vector<double> > scores, vector<int>& assignations, vector<double>& assigned_scores);

    void combAssign(vector< vector<double> > scores, vector<int>& assignations, vector<double>& assigned_scores);
    void recursive_assignation(vector< vector< pair<int,double> > > ordered,
                               int idx, bool recursive,
                               vector<int>& assignations, vector<double>& assignscores);
	//
	// Members
	//

    PointCloudPtr m_pCloudA, m_pCloudB; // foreground
	PointCloudPtr m_pTableTopCloudA, m_pTableTopCloudB;

    vector<PointCloudPtr> m_ClustersA, m_ClustersB;
    
    vector<CloudjectModelPtr> m_CloudjectModels;
    
    float m_ModelLeafSize;
	float m_LeafSize;
	float m_MaxCorrespondenceDistance;
    int m_TmpWnd;
    float m_NormalRadius;
    float m_FpfhRadius;
    float m_ScoreThreshold;
    
	vector< vector<CloudjectPtr> > m_CloudjectsHistory; // history
    
    vector<CloudjectPtr> m_AppearedCloudjects; // appered in frame
    //vector<CloudjectPtr> m_PresentCloudjects; // present in frame
    vector<CloudjectPtr> m_DisappearedCloudjects; // disappeared in frame
    
    DetectionOutput m_DetectionOutput;
};

