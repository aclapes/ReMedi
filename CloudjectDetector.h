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
#include "CloudjectModel.hpp"

using namespace std;

class CloudjectDetector
{
	typedef pcl::PointXYZ PointT;
	typedef LFCloudject<pcl::PointXYZ, pcl::FPFHSignature33> Cloudject;
    typedef LFCloudjectModel<pcl::PointXYZ, pcl::FPFHSignature33> CloudjectModel;

public:
	CloudjectDetector(void);
    CloudjectDetector(const CloudjectDetector& rhs);
	~CloudjectDetector(void);
	
    CloudjectDetector& operator=(const CloudjectDetector& rhs);
    
	void setInputClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr pTableTopCloudA,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr pTableTopCloudB);
    
    void setInputClusters(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersB);
    
    void loadCloudjectModels(string dir, int nModels, int nModelViews);
    void setCloudjectModels(vector< LFCloudjectModel<PointT,pcl::FPFHSignature33> > models);
    int getNumCloudjectModels();
	
    void setLeafSize(float);
	void setMaxCorrespondenceDistance(float);
    void setTemporalWindow(int tmpWnd);
    void setNormalRadius(float radius);
    void setFpfhRadius(float radius);
    void setScoreThreshold(float score);

	void detect(vector<Cloudject>&);
    
private:
	// Methods
    void findInliers(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&);
	void computeCentroids(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, 
		vector<Eigen::Vector4f>& centroids);

	void extractClustersFromView(pcl::PointCloud<pcl::PointXYZ>::Ptr,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);
    
	void findCorrespondences(
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersB,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsA,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsB,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversA,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversB);

	void makeInterviewCorrespondences(
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersB,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsA,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsB,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversA,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversB);

    void makeSpatiotemporalCorrespondences(vector<Cloudject>& cloudjects,
                                           vector< vector<Cloudject> >& cloudjectsHistory);
    
	float clustersBoxDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	
	float centroidsDistance(Eigen::Vector4f, Eigen::Vector4f);
	float clustersCentroidsDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	
	void variate(
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& P, 
		int n, 
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& C,
		vector<
			pair<
				vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,
				vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
			>
		>& V);

	void variations(
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& P, 
		int n,
		// vector< pair<vector<cloudptr>,vector<cloudptr>> >
		vector<
			pair<
				vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,
				vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
			>
		>& V);

	float correspondenceDistance(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&,
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&);

    
    float matchClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pSrcCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pTgtCloud);
    void matchClustersFromView(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> src, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tgt, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& matches);
    
    
    bool match(Cloudject src, Cloudject tgt);
    
    void recognize(vector< vector<Cloudject> >& history);
    
    void assign(vector< vector<double> > scores, vector<int>& assignations, vector<double>& assigned_scores);
    void recursive_assignation(vector< vector< pair<int,double> > > ordered,
                               int idx, bool recursive,
                               vector<int>& assignations, vector<double>& assignscores);
	//
	// Members
	//

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCloudA, m_pCloudB; // foreground
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pTableTopCloudA, m_pTableTopCloudB;

    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_ClustersA, m_ClustersB;
    
    vector<CloudjectModel> m_CloudjectModels;
    
	float m_LeafSize;
	float m_MaxCorrespondenceDistance;
    int m_TmpWnd;
    float m_NormalRadius;
    float m_FpfhRadius;
    float m_ScoreThreshold;
    
    int m_NumOfDetections;

	vector< vector<Cloudject> > m_Cloudjects; // history
};

