#include "Monitorizer.h"


Monitorizer::Monitorizer(InteractiveRegisterer ir, TableModeler tm)
	: m_ir(ir), m_tm(tm), m_pViz(new pcl::visualization::PCLVisualizer("Monitorizer"))
{
    m_CloudjectDetector.setLeafSize(0.01); // default
}


Monitorizer::Monitorizer(InteractiveRegisterer ir, TableModeler tm, MonitorizerParams params)
	: m_ir(ir), m_Params(params), m_tm(tm), m_pViz(new pcl::visualization::PCLVisualizer("Monitorizer"))
{
    m_CloudjectDetector.setLeafSize(params.leafSize);
    
	m_pViz->createViewPort(0, 0, 0.5, 1, m_SceneVp);
	m_pViz->createViewPort(0.5, 0, 1, 0.5, m_ObjectsAVp);
	m_pViz->createViewPort(0.5, 0.5, 1, 1, m_ObjectsBVp);

	m_pViz->addCoordinateSystem(1.0, "SceneVp", m_SceneVp);
	m_pViz->addCoordinateSystem(1.0, "ObjectsA", m_ObjectsAVp);
	m_pViz->addCoordinateSystem(1.0, "ObjectsB", m_ObjectsBVp);
}


Monitorizer::~Monitorizer(void)
{
}


void Monitorizer::setParams(MonitorizerParams params)
{
	m_Params = params;
}


void Monitorizer::monitor(DepthFrame dFrameA, DepthFrame dFrameB)
{
	// frames (dense depth images) -> registered point clouds
    
	PointCloudPtr pRCloudA (new PointCloud); // foreground (background subtracted)
	PointCloudPtr pRCloudB (new PointCloud);
    
	m_ir.registration(dFrameA, dFrameB, *pRCloudA, *pRCloudB, false, false);

	// Segment table top

	PointCloudPtr pTableTopCloudA (new PointCloud);
	PointCloudPtr pTableTopCloudB (new PointCloud);
    PointCloudPtr pInteractionCloudA (new PointCloud);
	PointCloudPtr pInteractionCloudB (new PointCloud);

	m_tm.segmentTableTop(pRCloudA, pRCloudB,
                         *pTableTopCloudA, *pTableTopCloudB);
    
    m_tm.segmentInteractionRegion(pRCloudA, pRCloudB,
                                  *pInteractionCloudA, *pInteractionCloudB);

	// Detect motion
    
//    PointCloudPtr pMotionCloudA (new PointCloud);
//	PointCloudPtr pMotionCloudB (new PointCloud);
//    
//    m_motionSegmentator.setInputFrames(dFrameA, dFrameB);
//    m_motionSegmentator.setMotionThreshold(m_Params.motionThresh);
//    //m_motionSegmentator.setNegativeMotion(false);
//    m_motionSegmentator.segment(*pMotionCloudA, *pMotionCloudB);
//    
//    PointCloudPtr pMotionRCloudA (new PointCloud);
//	PointCloudPtr pMotionRCloudB (new PointCloud);
//    
//	m_ir.registration(pMotionCloudA, pMotionCloudB, *pMotionRCloudA, *pMotionRCloudB);
    
    // Clusterize the blobs and create cloudjects

//	vector<Cloudject> cloudjects;
//
//	m_CloudjectDetector.setInputClouds(pInteractionCloudA, pInteractionCloudB, pTableTopCloudA, pTableTopCloudB);
//	m_CloudjectDetector.detect(cloudjects);



	//pcl::PointCloud<pcl::PointXYZ>::Ptr ontoMotionCloudA (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ontoMotionCloudB (new pcl::PointCloud<pcl::PointXYZ>);

	//m_tm.segmentObjectsOnTop(motionCloudA, motionCloudB, *ontoMotionCloudA, *ontoMotionCloudB);

	// Detect interaction

	// ...
	 
	// Visualization

	m_pViz->removeAllPointClouds();

	m_pViz->addPointCloud (pRCloudA, "cloud left", m_SceneVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud left");
    m_pViz->addPointCloud (pRCloudB, "cloud right", m_SceneVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud right");
    
    m_pViz->addPointCloud (pInteractionCloudA, "interaction left", m_ObjectsAVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0, "interaction left");
    m_pViz->addPointCloud (pInteractionCloudB, "interaction right", m_ObjectsBVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 0, "interaction right");
    
    m_pViz->addPointCloud (pTableTopCloudA, "tabletop left", m_ObjectsAVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "tabletop left");
    m_pViz->addPointCloud (pTableTopCloudB, "tabletop right", m_ObjectsBVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "tabletop right");
    
//    m_pViz->addPointCloud (pMotionRCloudA, "motion left", m_ObjectsAVp);
//    m_pViz->addPointCloud (pMotionRCloudB, "motion right", m_ObjectsBVp);

//    for (int i = 0; i < cloudjects.size(); i++)
//    {
//        std::stringstream ssA;
//        ssA << "cloudject_viewA_" << i;
//        m_pViz->addPointCloud (cloudjects[i].getViewA(), ssA.str(), m_ObjectsAVp);
//        m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.3, 0.3, ssA.str());
//        
//        std::stringstream ssB;
//        ssB << "cloudject_viewB_" << i;
//        m_pViz->addPointCloud (cloudjects[i].getViewB(), ssB.str(), m_ObjectsBVp);
//        m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 1, 0.3, ssB.str());
//    }

	m_pViz->spinOnce(10);
}


//void Monitorizer::segmentMotion(float thresh, cv::Mat& motionA, cv::Mat& motionB)
//{
//	segmentMotionInView(m_DepthStreamBufferA, thresh, motionA);
//	segmentMotionInView(m_DepthStreamBufferB, thresh, motionB);
//}
//
//
//void Monitorizer::segmentPerPixelStatics(cv::Mat& staticsA, cv::Mat& staticsB, float thresh)
//{
//	segmentPerPixelStaticsInView(m_DepthStreamBufferA, thresh, staticsA);
//	segmentPerPixelStaticsInView(m_DepthStreamBufferB, thresh, staticsB);
//}
//
//
//void Monitorizer::segmentPerPixelStaticsInView(vector<DepthFrame> buffer, float thresh, cv::Mat& statics)
//{
//	cv::Mat gNonBackgroundMask (480, 640, CV_8UC1);
//	cv::Mat gDepthMask (480, 640, CV_32FC1);
//
//	gNonBackgroundMask.setTo(255);
//	gDepthMask.setTo(255);
//
//
//	cv::namedWindow("1");
//	cv::namedWindow("2");
//	for (int k = buffer.size() - 1; k > 0; k--)
//	{
//		DepthFrame& currDepthFrame = buffer[k];
//			
//		cv::bitwise_and(currDepthFrame.getMask(), gNonBackgroundMask, gNonBackgroundMask);
//
//		//cv::imshow("1", gNonBackgroundMask);
//
//		DepthFrame& prevDepthFrame = buffer[k-1];
//		cv::Mat depthDiff, depthMask;
//
//		cv::absdiff(currDepthFrame.getDepthMap(), prevDepthFrame.getDepthMap(), depthDiff);
//		depthDiff.convertTo(depthDiff, CV_32FC1);
//		cv::threshold(depthDiff, depthMask, thresh, 255, cv::THRESH_BINARY_INV); // mask the nearly constant depth pixels
//		cv::bitwise_and(depthMask, gDepthMask, gDepthMask);
//
//		//cv::imshow("2", gDepthMask);
//
//		//cv::waitKey();
//	}
//
//	gDepthMask.convertTo(gDepthMask, CV_8UC1);
//	cv::bitwise_and(gNonBackgroundMask, gDepthMask, statics);
//}


void Monitorizer::updateCentroids(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, vector<Eigen::Vector4f>& centroids)
{
//	vector<Eigen::Vector4f> centroidsTmp; // Centroids in this frame
//	for (int i = 0; i < clusters.size(); i++)
//	{
//		Eigen::Vector4f c;
//		pcl::compute3DCentroid(*(clusters[i]), c);
//		centroidsTmp.push_back(c);
//	}
//
//	centroids.push_back(centroidsTmp);
//	if (centroids.size() > m_Params.tmpCoherence + 1)
//	{
//		centroids.erase(centroids.begin());
//	}
}


//void Monitorizer::segmentStatics(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> candidatesA, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> candidatesB,
//	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& staticsA, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& staticsB, float thresh)
//{
//	segmentStaticsInView(candidatesA, m_centroidsA, staticsA, thresh);
//	segmentStaticsInView(candidatesB, m_centroidsB, staticsB, thresh);
//}
//
//
//void Monitorizer::segmentStaticsInView( 
//	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> candidates,
//	vector<vector<Eigen::Vector4f> >& centroids,
//	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& statics, float thresh)
//{
//	for (int i = 0; i < centroids[centroids.size()-1].size(); i++)
//	{
//		Eigen::Vector4f c = centroids[centroids.size()-1][i];
//
//		bool isPresent = true;
//		for (int k = centroids.size()-2; k <= 0 && isPresent; k--)
//		{
//			float minDist = numeric_limits<float>::infinity();
//
//			for (int j = 0; j < centroids[k].size(); j++)
//			{
//				float dist = euclideanDistance(c, centroids[k][j]);
//				if (dist <= minDist) 
//					minDist = dist;
//			}
//
//			if (minDist > thresh)
//				isPresent = false;
//		}
//
//		if (isPresent) 
//			statics.push_back(candidates[i]);
//	}
//}


void Monitorizer::handleCloudjectDrops()
{
//	// Detect present objects
//	vector<Cloudject> detectedCjs;
//	m_CloudjectDetector.detect(m_CloudA, m_CloudB, m_LeafSize, detectedCjs);
//	cout << "detected: " << detectedCjs.size() << endl;
//	// Check for new appearitions
//	vector<Cloudject> newCjs;
//	appeared(detectedCjs, newCjs);
//
//	// Recognize the new appeared objects
//	m_cjRecognizer.recognize(newCjs);
//
//	// 
//	//drop(appearedCjs);
}


void Monitorizer::drop(vector<Cloudject> cloudjects)
{
	for (int i = 0; i < cloudjects.size(); i++)
	{
		m_cloudjects.push_back(cloudjects[i]);
	}
}


//void Monitorizer::handleCloudjectPicks(vector<Cloudject>& cloudjects)
//{
//
//}


void Monitorizer::appeared(vector<Cloudject> detecteds, vector<Cloudject>& news)
{
	for (int i = 0; i < detecteds.size(); i++)
	{
		bool isNew = true;
		for (int j = 0; j < m_cloudjects.size() && isNew; j++)
		{
			if ( compareEquals(detecteds[i], m_cloudjects[j]) ) 
			{
				isNew = false;
			}
		}

		if (isNew)
		{
			news.push_back(detecteds[i]);
		}
	}
}


bool Monitorizer::compareEquals(Cloudject a, Cloudject b)
{
	PointT posA, posB;
	posA = a.getPosA();
	posB = b.getPosB();

	float dist = sqrt(powf(posA.x-posB.x,2)+powf(posA.y-posB.y,2)+powf(posA.z-posB.z,2));

	return dist < m_Params.posCorrespThresh;
}


void Monitorizer::visualizeCloudjects(pcl::visualization::PCLVisualizer::Ptr pViz)
{
	for (int i = 0; i < m_cloudjects.size(); i++)
	{
		stringstream ss;
		ss << m_cloudjects[i].getID();
		pViz->removePointCloud((ss.str() + "A").c_str());
		pViz->removePointCloud((ss.str() + "B").c_str());

		pViz->addPointCloud (m_cloudjects[i].getViewA(), (ss.str() + "A").c_str());
		pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, (ss.str() + "A").c_str());
		pViz->addPointCloud (m_cloudjects[i].getViewB(), (ss.str() + "B").c_str());
		pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, (ss.str() + "B").c_str());
	}
}


void Monitorizer::detectCloudjects(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB, vector<Cloudject>& cloudjects, float leafSize)
{
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA, clustersB;

	extractClustersFromView(pCloudA, leafSize, clustersA);
	extractClustersFromView(pCloudB, leafSize, clustersB);
}


void Monitorizer::extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustersA, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustersB, float leafSize)
{
	extractClustersFromView(pCloudA, leafSize, clustersA);
	extractClustersFromView(pCloudB, leafSize, clustersB);
}


void Monitorizer::extractClustersFromView(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
	float leafSize, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters )
{
	pcl::PointCloud<PointT>::Ptr pCloudR (new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr pCloudF (new pcl::PointCloud<PointT>());

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (pCloud);
	sor.setMeanK (10);
	sor.setStddevMulThresh (1.0);
	sor.filter (*pCloudR);

	pcl::ApproximateVoxelGrid<PointT> avg;
	avg.setInputCloud(pCloudR);
	avg.setLeafSize(leafSize, leafSize, leafSize);
	avg.filter(*pCloudF);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (pCloudF);

	vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (8 * leafSize); // cm
	ec.setMinClusterSize (200);
	ec.setMaxClusterSize (700);
	ec.setSearchMethod (tree);
	ec.setInputCloud (pCloudF);
	ec.extract (clusterIndices);

	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersF; // clusters filtered

	pcl::PassThrough<PointT> pt;
	PointT min, max;

	int j = 0;
	for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pClusterF (new pcl::PointCloud<pcl::PointXYZ>);
		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			pClusterF->points.push_back (pCloudF->points[*pit]); //*
		pClusterF->width = pClusterF->points.size();
		pClusterF->height = 1;
		pClusterF->is_dense = false;

		clusters.push_back(pClusterF);

		j++;
	}
}
