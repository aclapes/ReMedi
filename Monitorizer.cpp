#include "Monitorizer.h"

Monitorizer::Monitorizer(InteractiveRegisterer ir, TableModeler tm, CloudjectDetector cd)
: m_ir(ir), m_tm(tm), m_CloudjectDetector(cd),
m_pInteractionCloudA(new PointCloud), m_pInteractionCloudB(new PointCloud),
m_pViz(new pcl::visualization::PCLVisualizer("Monitorizer"))
{
    m_CloudjectDetector.setLeafSize(0.005);
    m_CloudjectDetector.setTemporalWindow(3);
    m_CloudjectDetector.setMaxCorrespondenceDistance(0.07);
    
//    m_SceneVp = 1;
//    m_ObjectsVpA = 2;
//    m_ObjectsVpB = 3;
    
//	m_pViz->createViewPort(0, 0, 0.5, 1, m_SceneVp);
//	m_pViz->createViewPort(0.5, 0.5, 1, 1, m_ObjectsVpA);
//	m_pViz->createViewPort(0.5, 0, 1, 0.5, m_ObjectsVpB);

    m_pViz->setPosition(0,0);
    m_pViz->setSize(1280,480);
    
    m_pViz->createViewPort(0.0, 0.0, 0.5, 1.0, m_ObjectsVpA);
    m_pViz->createViewPort(0.5, 0.0, 1.0, 1.0, m_ObjectsVpB);

//	m_pViz->addCoordinateSystem(0.5, "SceneVp", m_SceneVp);
	m_pViz->addCoordinateSystem(0.5, "ObjectsA", m_ObjectsVpA);
	m_pViz->addCoordinateSystem(0.5, "ObjectsB", m_ObjectsVpB);
    
    m_pViz->setCameraPosition(0, 0, 3, 0, 0, 0);
    
    m_TextSize = 0.05;
}

Monitorizer::Monitorizer(const Monitorizer& rhs)
{
    *this = rhs;
}

Monitorizer::~Monitorizer(void)
{
}

Monitorizer& Monitorizer::operator=(const Monitorizer& rhs)
{
    if (this != &rhs)
    {
        m_Params = rhs.m_Params;
        
        m_ir = rhs.m_ir;
        m_tm = rhs.m_tm;
        
        m_pViz = rhs.m_pViz;
        m_SceneVp = rhs.m_SceneVp;
        m_ObjectsVpA = rhs.m_ObjectsVpA;
        m_ObjectsVpB = rhs.m_ObjectsVpB;
        
        m_dFrameA = rhs.m_dFrameA;
        m_dFrameB = rhs.m_dFrameB;
        
        m_CloudA = rhs.m_CloudA;
        m_CloudB = rhs.m_CloudB;
        
        m_CloudjectDetector = rhs.m_CloudjectDetector;
        m_MotionSegmentator = rhs.m_MotionSegmentator;
        
        m_cloudjects = rhs.m_cloudjects; // yet present
        
        m_LeafSize = rhs.m_LeafSize;
        m_ClusterTolFactor = rhs.m_ClusterTolFactor;
        m_PosCorrespThresh = rhs.m_PosCorrespThresh;
        
        m_TextSize = rhs.m_TextSize;
        m_TextsVpA = rhs.m_TextsVpA;
        m_TextsVpB = rhs.m_TextsVpB;
    }
    
    return *this;
}

void Monitorizer::clear()
{
    m_CloudjectDetector.clear();
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

    PointCloudPtr pRDownCloudA (new PointCloud);
	PointCloudPtr pRDownCloudB (new PointCloud);
    
    pcl::VoxelGrid<PointT> vox;
    vox.setLeafSize(m_LeafSize, m_LeafSize, m_LeafSize);
    vox.setInputCloud(pRCloudA);
    vox.filter(*pRDownCloudA);
    vox.setInputCloud(pRCloudB);
    vox.filter(*pRDownCloudB);
    
	PointCloudPtr pTableTopCloudA (new PointCloud);
	PointCloudPtr pTableTopCloudB (new PointCloud);
	m_tm.segmentTableTop(pRDownCloudA, pRDownCloudB,
                         *pTableTopCloudA, *pTableTopCloudB);
    
    m_pInteractionCloudA->clear();
    m_pInteractionCloudB->clear();
    m_tm.segmentInteractionRegion(pRDownCloudA, pRDownCloudB,
                                  *m_pInteractionCloudA, *m_pInteractionCloudB);
    
    vector<PointCloudPtr> tabletopClustersA, tabletopClustersB; // tabletop inliers
    vector<PointCloudPtr> interactionClustersA, interactionClustersB; // tabletop outliers
    
    extractClustersFromView(pTableTopCloudA, tabletopClustersA, m_LeafSize);
    extractClustersFromView(pTableTopCloudB, tabletopClustersB, m_LeafSize);
    extractClustersFromView(m_pInteractionCloudA, interactionClustersA, m_LeafSize);
    extractClustersFromView(m_pInteractionCloudB, interactionClustersB, m_LeafSize);
    
    vector<PointCloudPtr> actorClustersA, actorClustersB; // tabletop inliers
    classifyTabletop(tabletopClustersA, interactionClustersA,
                     actorClustersA, m_InteractorClustersA,
                     1.5 * m_LeafSize);
    classifyTabletop(tabletopClustersB, interactionClustersB,
                     actorClustersB, m_InteractorClustersB,
                     1.5 * m_LeafSize);
    
    //
    // Cloudjects part
    // ---------------
    //
    // Lot of stuff in here:
    // - creating cloudjects from clusters in two views
    // - spatiotemporal coherence
    // - recognition
    // - detect apparitions and disapparitions
    //
    
    m_CloudjectDetector.setInputClusters(actorClustersA, actorClustersB);
    m_CloudjectDetector.detect();

    m_CloudjectDetector.getPresentCloudjects(m_PresentCloudjects);
    m_CloudjectDetector.getAppearedCloudjects(m_AppearedCloudjects);
    m_CloudjectDetector.getDisappearedCloudjects(m_DisappearedCloudjects);
    
    for (int i = 0; i < m_DisappearedCloudjects.size(); i++)
    {
        bool grabInA = isInteractive(m_DisappearedCloudjects[i]->getViewA(), m_InteractorClustersA, 1.5 * m_LeafSize);
        bool grabInB = isInteractive(m_DisappearedCloudjects[i]->getViewB(), m_InteractorClustersB, 1.5 * m_LeafSize);
        
        if (grabInA || grabInB)
            cout << m_DisappearedCloudjects[i]->getName() << endl;
    }
}

void Monitorizer::display()
{
    // Visualization
    
	m_pViz->removeAllPointClouds();
    m_pViz->removeAllShapes();
    
    //	m_pViz->addPointCloud (pRCloudA, "cloud left", m_SceneVp);
    //    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "cloud left");
    //    m_pViz->addPointCloud (pRCloudB, "cloud right", m_SceneVp);
    //    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "cloud right");
    
    stringstream ss;
    m_pViz->addPointCloud (m_pInteractionCloudA, "interaction left", m_ObjectsVpA);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "interaction left");
    for (int i = 0; i < m_InteractorClustersA.size(); i++)
    {
        ss.str("");
        ss << "interactors left " << i;
        m_pViz->addPointCloud (m_InteractorClustersA[i], ss.str(), m_ObjectsVpA);
        m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.35, 1, ss.str());
    }
    
    
    m_pViz->addPointCloud (m_pInteractionCloudB, "interaction right", m_ObjectsVpB);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "interaction left");
    for (int i = 0; i < m_InteractorClustersB.size(); i++)
    {
        ss.str("");
        ss << "interactors righties " << i;
        m_pViz->addPointCloud (m_InteractorClustersB[i], ss.str(), m_ObjectsVpB);
        m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.35, 1, ss.str());
    }
    
    for (int i = 0; i < m_TextsVpA.size(); i++)
        m_pViz->removeText3D("a" + m_TextsVpA[i], m_ObjectsVpA + 1);
    
    for (int i = 0; i < m_TextsVpB.size(); i++)
        m_pViz->removeText3D("b" + m_TextsVpB[i], m_ObjectsVpB + 1);
    
    
    for (int i = 0; i < m_PresentCloudjects.size(); i++)
    {
        size_t pid = ((size_t) &(*m_PresentCloudjects[i]));// pointer-based id
        
        if (!m_PresentCloudjects[i]->getViewA()->empty())
        {
            m_pViz->addPointCloud (m_PresentCloudjects[i]->getViewA(), "a" + to_string(pid), m_ObjectsVpA);
            m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.1, 0.1, "a" + to_string(pid));
            
            //            if (cloudjects[i].getID() >= 0)
            //            {
            m_pViz->addText3D(m_PresentCloudjects[i]->getName(), m_PresentCloudjects[i]->getPosA(), m_TextSize, 1.0, 1.0, 1.0, "a" + to_string(pid), m_ObjectsVpA + 1);
            m_TextsVpA.push_back( to_string(pid) );
            //            }
        }
        
        if (!m_PresentCloudjects[i]->getViewB()->empty())
        {
            m_pViz->addPointCloud (m_PresentCloudjects[i]->getViewB(), "b" + to_string(pid), m_ObjectsVpB);
            m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.1, 0.1, "b" + to_string(pid));
            
            //            if (cloudjects[i].getID() >= 0)
            //            {
            m_pViz->addText3D(m_PresentCloudjects[i]->getName(), m_PresentCloudjects[i]->getPosB(), m_TextSize, 1.0, 1.0, 1.0, "b" + to_string(pid), m_ObjectsVpB + 1);
            m_TextsVpB.push_back( "b" + to_string(pid) );
            //            }
        }
    }
    
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


void Monitorizer::extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustersA, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustersB, float leafSize)
{
	extractClustersFromView(pCloudA, clustersA, leafSize);
	extractClustersFromView(pCloudB, clustersB, leafSize);
}


void Monitorizer::extractClustersFromView(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters, float leafSize)
{
	pcl::PointCloud<PointT>::Ptr pCloudR (new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr pCloudF (new pcl::PointCloud<PointT>());

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (pCloud);
	sor.setMeanK (10);
	sor.setStddevMulThresh (1.0);
	sor.filter (*pCloudR);

    if (leafSize > 0.f)
    {
        pcl::ApproximateVoxelGrid<PointT> avg;
        avg.setInputCloud(pCloudR);
        avg.setLeafSize(leafSize, leafSize, leafSize);
        avg.filter(*pCloudF);
    }
    else
    {
        pcl::copyPointCloud(*pCloudR, *pCloudF);
    }

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (pCloudF);

	vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    float tol = (leafSize > 0.f) ? m_ClusterTolFactor * leafSize : m_ClusterTolFactor * 0.005;
	ec.setClusterTolerance (tol); // cm
	ec.setMinClusterSize (0.1/leafSize);
	ec.setMaxClusterSize (1000/leafSize);
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

void Monitorizer::classifyTabletop(vector<PointCloudPtr> tabletopRegionClusters,
                                   vector<PointCloudPtr> interactionRegionClusters,
                                   vector<PointCloudPtr>& actorClusters,
                                   vector<PointCloudPtr>& interactorClusters,
                                   float leafSize)
{
    actorClusters.clear();
    interactorClusters.clear();
    
    // Check for each cluster that lies on the table
    for (int i = 0; i < tabletopRegionClusters.size(); i++)
    {
        bool interactive = isInteractive(tabletopRegionClusters[i],
                                         interactionRegionClusters,
                                         leafSize);
        if (!interactive)
            actorClusters.push_back(tabletopRegionClusters[i]);
        else
            interactorClusters.push_back(tabletopRegionClusters[i]);
    }
}

bool Monitorizer::isInteractive(PointCloudPtr tabletopRegionCluster,
                                vector<PointCloudPtr> interactionRegionClusters,
                                float leafSize)
{
    bool interactive = false;
    
    // if some of its points
    for (int k_i = 0; k_i < tabletopRegionCluster->points.size() && !interactive; k_i++)
    {
        PointT p = tabletopRegionCluster->points[k_i];
        
        // is very stick together with
        for (int j = 0; j < interactionRegionClusters.size() && !interactive; j++)
        {
            // some point of the interaction region.
            for (int k_j = 0; k_j < interactionRegionClusters[j]->points.size() && !interactive; k_j++)
            {
                PointT q = interactionRegionClusters[j]->points[k_j];
                
                // If they are practically contiguous, they must had been part of the same cloud
                float pqDist = sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2) + pow(p.z - q.z, 2));
                interactive = (pqDist <= 2 * leafSize); // if tabletop cluster is stick to any cluster
                // in the interaction region, we say it is interactive
                // and become an interactor
            }
        }
    }
    
    return interactive;
}

void Monitorizer::setLeafSize(float leafSize)
{
    m_LeafSize = leafSize;
}

void Monitorizer::setClusteringToleranceFactor(int factor)
{
    m_ClusterTolFactor = factor;
}

void Monitorizer::getDetectionOutput()
{
    
}
