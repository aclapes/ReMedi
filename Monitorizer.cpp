#include "Monitorizer.h"

Monitorizer::Monitorizer(InteractiveRegisterer ir, TableModeler tm, CloudjectDetector cd)
	: m_ir(ir), m_tm(tm), m_CloudjectDetector(cd), m_pViz(new pcl::visualization::PCLVisualizer("Monitorizer"))
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
    
    m_TextSize = 0.1;
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
    PointCloudPtr pInteractionCloudA (new PointCloud);
	PointCloudPtr pInteractionCloudB (new PointCloud);

	m_tm.segmentTableTop(pRDownCloudA, pRDownCloudB,
                         *pTableTopCloudA, *pTableTopCloudB);
    
    m_tm.segmentInteractionRegion(pRDownCloudA, pRDownCloudB,
                                  *pInteractionCloudA, *pInteractionCloudB);
    
    vector<PointCloudPtr> tabletopClustersA, tabletopClustersB; // tabletop inliers
    vector<PointCloudPtr> interactionClustersA, interactionClustersB; // tabletop outliers
    
    extractClustersFromView(pTableTopCloudA, tabletopClustersA, m_LeafSize);
    extractClustersFromView(pTableTopCloudB, tabletopClustersB, m_LeafSize);
    extractClustersFromView(pInteractionCloudA, interactionClustersA, m_LeafSize);
    extractClustersFromView(pInteractionCloudB, interactionClustersB, m_LeafSize);
    
    vector<PointCloudPtr> actorClustersA, actorClustersB; // tabletop inliers
    vector<PointCloudPtr> interactorClustersA, interactorClustersB; // tabletop outliers
    
    classifyActorsAndInteractors(tabletopClustersA, interactionClustersA,
                                 actorClustersA, interactorClustersA,
                                 1.5 * m_LeafSize);
    classifyActorsAndInteractors(tabletopClustersB, interactionClustersB,
                                 actorClustersB, interactorClustersB,
                                 1.5 * m_LeafSize);
    
    vector<Cloudject> cloudjects;
    m_CloudjectDetector.setInputClusters(actorClustersA, actorClustersB);
    m_CloudjectDetector.detect(cloudjects);
    
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
    m_pViz->removeAllShapes();

//	m_pViz->addPointCloud (pRCloudA, "cloud left", m_SceneVp);
//    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "cloud left");
//    m_pViz->addPointCloud (pRCloudB, "cloud right", m_SceneVp);
//    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "cloud right");
    
    stringstream ss;
    m_pViz->addPointCloud (pInteractionCloudA, "interaction left", m_ObjectsVpA);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "interaction left");
    for (int i = 0; i < interactorClustersA.size(); i++)
    {
        ss.str("");
        ss << "interactors left " << i;
        m_pViz->addPointCloud (interactorClustersA[i], ss.str(), m_ObjectsVpA);
        m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.35, 1, ss.str());
    }
    
    
    m_pViz->addPointCloud (pInteractionCloudB, "interaction right", m_ObjectsVpB);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "interaction left");
    for (int i = 0; i < interactorClustersB.size(); i++)
    {
        ss.str("");
        ss << "interactors righties " << i;
        m_pViz->addPointCloud (interactorClustersB[i], ss.str(), m_ObjectsVpB);
        m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.35, 1, ss.str());
    }
    
    for (int i = 0; i < m_TextsVpA.size(); i++)
        m_pViz->removeText3D("a" + m_TextsVpA[i], m_ObjectsVpA + 1);
    
    for (int i = 0; i < m_TextsVpB.size(); i++)
        m_pViz->removeText3D("b" + m_TextsVpB[i], m_ObjectsVpB + 1);
    
    
    for (int i = 0; i < cloudjects.size(); i++)
    {
        size_t pid = (size_t) &(cloudjects[i]);// pointer-based id
        
        if (!cloudjects[i].getViewA()->empty())
        {
            m_pViz->addPointCloud (cloudjects[i].getViewA(), "a" + to_string(pid), m_ObjectsVpA);
            m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.1, 0.1, "a" + to_string(pid));
            
//            if (cloudjects[i].getID() >= 0)
//            {
                m_pViz->addText3D(to_string(cloudjects[i].getID()), cloudjects[i].getPosA(), m_TextSize, 1.0, 1.0, 1.0, "a" + to_string(pid), m_ObjectsVpA + 1);
                m_TextsVpA.push_back( to_string(pid) );
//            }
        }
        
        if (!cloudjects[i].getViewB()->empty())
        {
            m_pViz->addPointCloud (cloudjects[i].getViewB(), "b" + to_string(pid), m_ObjectsVpB);
            m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.1, 0.1, "b" + to_string(pid));
            
//            if (cloudjects[i].getID() >= 0)
//            {
                m_pViz->addText3D(to_string(cloudjects[i].getID()), cloudjects[i].getPosB(), m_TextSize, 1.0, 1.0, 1.0, "b" + to_string(pid), m_ObjectsVpB + 1);
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

	extractClustersFromView(pCloudA, clustersA, leafSize);
	extractClustersFromView(pCloudB, clustersB, leafSize);
}


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

void Monitorizer::classifyActorsAndInteractors(vector<PointCloudPtr> tabletopRegionClusters,
                                               vector<PointCloudPtr> interactionRegionClusters,
                                               vector<PointCloudPtr>& actorClusters,
                                               vector<PointCloudPtr>& interactorClusters,
                                               float leafSize)
{
    // Check for each cluster that lies on the table
    for (int i = 0; i < tabletopRegionClusters.size(); i++)
    {
        bool interactive = false;
        // if some of its points
        for (int k_i = 0; k_i < tabletopRegionClusters[i]->points.size() && !interactive; k_i++)
        {
            PointT p = tabletopRegionClusters[i]->points[k_i];
            
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
        
        if (!interactive)
            actorClusters.push_back(tabletopRegionClusters[i]);
        else
            interactorClusters.push_back(tabletopRegionClusters[i]);
    }
}

void Monitorizer::setLeafSize(float leafSize)
{
    m_LeafSize = leafSize;
}

void Monitorizer::setClusteringToleranceFactor(int factor)
{
    m_ClusterTolFactor = factor;
}

void Monitorizer::setMinimumClusterSize(int npoints)
{
    m_MinClusterSize = npoints;
}

void Monitorizer::setMaximumClusterSize(int npoints)
{
    m_MaxClusterSize = npoints;
}
