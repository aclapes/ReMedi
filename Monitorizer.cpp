#include "Monitorizer.h"


Monitorizer::Monitorizer(InteractiveRegisterer* pIR, TableModeler* pTM) 
	: m_pIR(pIR), m_pTableModeler(pTM), m_pViz(new pcl::visualization::PCLVisualizer("Monitorizer"))
{
}


Monitorizer::Monitorizer(InteractiveRegisterer* pIR, TableModeler* pTM, MonitorizerParams params)
	: m_pIR(pIR),  m_pTableModeler(pTM), m_Params(params), m_pViz(new pcl::visualization::PCLVisualizer("Monitorizer"))
{
	m_pViz->createViewPort(0, 0, 0.5, 1, m_SceneVp);
	m_pViz->createViewPort(0.5, 0, 1, 0.5, m_ObjectsAVp);
	m_pViz->createViewPort(0.5, 0.5, 1, 1, m_ObjectsBVp);

	m_pViz->addCoordinateSystem(1.0, m_SceneVp);
	m_pViz->addCoordinateSystem(1.0, m_ObjectsAVp);
	m_pViz->addCoordinateSystem(1.0, m_ObjectsBVp);
}


Monitorizer::~Monitorizer(void)
{
}


void Monitorizer::monitor(DepthFrame dFrameA, DepthFrame dFrameB)
{
	// Update history, for temporal coherence's sake

	updateHistory(dFrameA, dFrameB);

	if (!isHistoryComplete())
		return;

	// From dense depth images to registered point clouds

	pcl::PointCloud<pcl::PointXYZ>::Ptr pRCloudA (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pRCloudB (new pcl::PointCloud<pcl::PointXYZ>());

	pcl::PointCloud<pcl::PointXYZ>::Ptr pFgRCloudA (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pFgRCloudB (new pcl::PointCloud<pcl::PointXYZ>());

	m_pIR->getRegisteredClouds(dFrameA, dFrameB, *pRCloudA, *pRCloudB, true, false);
	m_pIR->getRegisteredClouds(dFrameA, dFrameB, *pFgRCloudA, *pFgRCloudB, false, false);

	// Segment objects on top of the modeled table

	pcl::PointCloud<pcl::PointXYZ>::Ptr pRObjsCloudA (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pRObjsCloudB (new pcl::PointCloud<pcl::PointXYZ>());

	m_pTableModeler->segmentTableTop(pFgRCloudA, pFgRCloudB, *pRObjsCloudA, *pRObjsCloudB);

	// Way 1
	//cv::Mat staticsMaskA, staticsMaskB;
	//segmentPerPixelStatics(staticsMaskA, staticsMaskA, m_Params.motionThresh);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr staticsCloudA (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr staticsCloudB (new pcl::PointCloud<pcl::PointXYZ>);

	//MaskDensePointCloud(pFgRCloudA, staticsMaskA, *staticsCloudA);
	//MaskDensePointCloud(pFgRCloudB, staticsMaskB, *staticsCloudB);

	// Way 2
	

	std::vector<Cloudject> cloudjects;

	CloudjectDetector cjDetector;
	cjDetector.setInputClouds(pRObjsCloudA, pRObjsCloudB);
	cjDetector.detect(cloudjects);


	// Detect motion

	cv::Mat motionMaskA, motionMaskB;
	segmentMotion(motionMaskA, motionMaskB, m_Params.motionThresh);

	pcl::PointCloud<pcl::PointXYZ>::Ptr motionCloudA (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr motionCloudB (new pcl::PointCloud<pcl::PointXYZ>);

	MaskDensePointCloud(pFgRCloudA, motionMaskA, *motionCloudA);
	MaskDensePointCloud(pFgRCloudB, motionMaskB, *motionCloudB);

	//m_pIR->getRegisteredClouds(motionA, motionB, *motionCloudA, *motionCloudB);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr ontoMotionCloudA (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ontoMotionCloudB (new pcl::PointCloud<pcl::PointXYZ>);

	//m_pTableModeler->segmentObjectsOnTop(motionCloudA, motionCloudB, *ontoMotionCloudA, *ontoMotionCloudB);

	// Detect interaction

	// ...
	 
	// Visualization

	m_pViz->removeAllPointClouds();

	m_pViz->addPointCloud (pRCloudA, "cloud left", m_SceneVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.4, 0, 0, "cloud left");
    m_pViz->addPointCloud (pRCloudB, "cloud right", m_SceneVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.4, 0, "cloud right");

	/*m_pViz->addPointCloud (staticsCloudA, "objects left", m_ObjectsAVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.4, 0, 0, "objects left");*/
	m_pViz->addPointCloud (motionCloudA, "motion left", m_ObjectsAVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.3, 0.3, "motion left");

   /* m_pViz->addPointCloud (staticsCloudB, "objects right", m_ObjectsBVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.4, 0, "objects right");*/
    m_pViz->addPointCloud (motionCloudB, "motion right", m_ObjectsBVp);
    m_pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 1, 0.3, "motion right");

	m_pViz->spinOnce(10);
}


void Monitorizer::monitor(PointCloudPtr cloudA, PointCloudPtr cloudB)
{
	m_CloudA = cloudA;
	m_CloudB = cloudB;
}


void Monitorizer::updateHistory(DepthFrame dFrameA, DepthFrame dFrameB)
{
	bufferDepthFrame(m_DepthStreamBufferA, dFrameA);
	bufferDepthFrame(m_DepthStreamBufferB, dFrameB);
}


bool Monitorizer::isHistoryComplete()
{
	return isBufferFilled(m_DepthStreamBufferA) && isBufferFilled(m_DepthStreamBufferB);
}


void Monitorizer::bufferDepthFrame(std::vector<DepthFrame>& buffer, DepthFrame dFrame)
{
	buffer.push_back(dFrame);

	if (buffer.size() > m_Params.tmpCoherence + 1)
	{
		buffer.erase(buffer.begin());
	}
}


bool Monitorizer::isBufferFilled(std::vector<DepthFrame> buffer)
{
	return buffer.size() == m_Params.tmpCoherence + 1;
}


void Monitorizer::setParams(MonitorizerParams params)
{
	m_Params = params;
}


void Monitorizer::segmentMotion( cv::Mat& motionA, cv::Mat& motionB, float thresh)
{
	segmentMotionInView(m_DepthStreamBufferA, thresh, motionA);
	segmentMotionInView(m_DepthStreamBufferB, thresh, motionB);
}


void Monitorizer::segmentMotionInView(std::vector<DepthFrame> buffer, float thresh, cv::Mat& motion)
{
	DepthFrame& currDepthFrame = buffer[buffer.size() - 1];
	DepthFrame& prevDepthFrame = buffer[buffer.size() - 2];

	cv::Mat mask_t, mask_tm;
	currDepthFrame.getMask().convertTo(mask_t, CV_32FC1);
	prevDepthFrame.getMask().convertTo(mask_tm, CV_32FC1);

	cv::Mat masksDiff = currDepthFrame.getMask() - prevDepthFrame.getMask();
	cv::threshold(masksDiff, masksDiff, 0, 255, cv::THRESH_BINARY);

	masksDiff.convertTo(masksDiff, CV_8UC1);

	cv::Mat depthsDiff;
	cv::subtract(currDepthFrame.getDepthMap(), prevDepthFrame.getDepthMap(), depthsDiff, masksDiff, CV_32FC1);

	cv::threshold(cv::abs(depthsDiff), motion, thresh, 255, cv::THRESH_BINARY);
	
	enclosure(motion, motion, 2);
}


void Monitorizer::segmentPerPixelStatics(cv::Mat& staticsA, cv::Mat& staticsB, float thresh)
{
	segmentPerPixelStaticsInView(m_DepthStreamBufferA, thresh, staticsA);
	segmentPerPixelStaticsInView(m_DepthStreamBufferB, thresh, staticsB);
}


void Monitorizer::segmentPerPixelStaticsInView(std::vector<DepthFrame> buffer, float thresh, cv::Mat& statics)
{
	cv::Mat gNonBackgroundMask (480, 640, CV_8UC1);
	cv::Mat gDepthMask (480, 640, CV_32FC1);

	gNonBackgroundMask.setTo(255);
	gDepthMask.setTo(255);


	cv::namedWindow("1");
	cv::namedWindow("2");
	for (int k = buffer.size() - 1; k > 0; k--)
	{
		DepthFrame& currDepthFrame = buffer[k];
			
		cv::bitwise_and(currDepthFrame.getMask(), gNonBackgroundMask, gNonBackgroundMask);

		//cv::imshow("1", gNonBackgroundMask);

		DepthFrame& prevDepthFrame = buffer[k-1];
		cv::Mat depthDiff, depthMask;

		cv::absdiff(currDepthFrame.getDepthMap(), prevDepthFrame.getDepthMap(), depthDiff);
		depthDiff.convertTo(depthDiff, CV_32FC1);
		cv::threshold(depthDiff, depthMask, thresh, 255, cv::THRESH_BINARY_INV); // mask the nearly constant depth pixels
		cv::bitwise_and(depthMask, gDepthMask, gDepthMask);

		//cv::imshow("2", gDepthMask);

		//cv::waitKey();
	}

	gDepthMask.convertTo(gDepthMask, CV_8UC1);
	cv::bitwise_and(gNonBackgroundMask, gDepthMask, statics);
}





void Monitorizer::updateCentroids(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, std::vector<Eigen::Vector4f>& centroids)
{
	std::vector<Eigen::Vector4f> centroidsTmp; // Centroids in this frame
	for (int i = 0; i < clusters.size(); i++)
	{
		Eigen::Vector4f c;
		pcl::compute3DCentroid(*(clusters[i]), c);
		centroidsTmp.push_back(c);
	}

	centroids.push_back(centroidsTmp);
	if (centroids.size() > m_Params.tmpCoherence + 1)
	{
		centroids.erase(centroids.begin());
	}
}


void Monitorizer::segmentStatics(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> candidatesA, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> candidatesB,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& staticsA, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& staticsB, float thresh)
{
	segmentStaticsInView(candidatesA, m_centroidsA, staticsA, thresh);
	segmentStaticsInView(candidatesB, m_centroidsB, staticsB, thresh);
}


void Monitorizer::segmentStaticsInView( 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> candidates,
	std::vector<std::vector<Eigen::Vector4f>>& centroids, 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& statics, float thresh)
{
	for (int i = 0; i < centroids[centroids.size()-1].size(); i++)
	{
		Eigen::Vector4f c = centroids[centroids.size()-1][i];

		bool isPresent = true;
		for (int k = centroids.size()-2; k <= 0 && isPresent; k--)
		{
			float minDist = std::numeric_limits<float>::infinity();

			for (int j = 0; j < centroids[k].size(); j++)
			{
				float dist = euclideanDistance(c, centroids[k][j]);
				if (dist <= minDist) 
					minDist = dist;
			}

			if (minDist > thresh)
				isPresent = false;
		}

		if (isPresent) 
			statics.push_back(candidates[i]);
	}
}


void Monitorizer::handleCloudjectDrops()
{
	// Detect present objects
	std::vector<Cloudject> detectedCjs;
	m_cjDetector.detect(m_CloudA, m_CloudB, m_LeafSize, detectedCjs);
	std::cout << "detected: " << detectedCjs.size() << std::endl;
	// Check for new appearitions
	std::vector<Cloudject> newCjs;
	appeared(detectedCjs, newCjs);

	// Recognize the new appeared objects
	m_cjRecognizer.recognize(newCjs);

	// 
	//drop(appearedCjs);
}


void Monitorizer::drop(std::vector<Cloudject> cloudjects)
{
	for (int i = 0; i < cloudjects.size(); i++)
	{
		m_cloudjects.push_back(cloudjects[i]);
	}
}


//void Monitorizer::handleCloudjectPicks(std::vector<Cloudject>& cloudjects)
//{
//
//}


void Monitorizer::appeared(std::vector<Cloudject> detecteds, std::vector<Cloudject>& news)
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
	posA = a.getPos();
	posB = b.getPos();

	float dist = sqrt(powf(posA.x-posB.x,2)+powf(posA.y-posB.y,2)+powf(posA.z-posB.z,2));

	return dist < m_Params.posCorrespThresh;
}


void Monitorizer::visualizeCloudjects(pcl::visualization::PCLVisualizer::Ptr pViz)
{
	for (int i = 0; i < m_cloudjects.size(); i++)
	{
		std::stringstream ss;
		ss << m_cloudjects[i].getID();
		pViz->removePointCloud((ss.str() + "A").c_str());
		pViz->removePointCloud((ss.str() + "B").c_str());

		pViz->addPointCloud (m_cloudjects[i].getViewA(), (ss.str() + "A").c_str());
		pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, (ss.str() + "A").c_str());
		pViz->addPointCloud (m_cloudjects[i].getViewB(), (ss.str() + "B").c_str());
		pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, (ss.str() + "B").c_str());
	}
}


void Monitorizer::detectCloudjects(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB,
	std::vector<Cloudject>& cloudjects, float leafSize)
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA, clustersB;

	extractClustersFromView(pCloudA, leafSize, clustersA);
	extractClustersFromView(pCloudB, leafSize, clustersB);


}


void Monitorizer::extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustersA, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustersB, float leafSize)
{
	extractClustersFromView(pCloudA, leafSize, clustersA);
	extractClustersFromView(pCloudB, leafSize, clustersB);
}


void Monitorizer::extractClustersFromView(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
	float leafSize, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters )
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

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (8 * leafSize); // cm
	ec.setMinClusterSize (200);
	ec.setMaxClusterSize (700);
	ec.setSearchMethod (tree);
	ec.setInputCloud (pCloudF);
	ec.extract (clusterIndices);

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersF; // clusters filtered

	pcl::PassThrough<PointT> pt;
	PointT min, max;

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pClusterF (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			pClusterF->points.push_back (pCloudF->points[*pit]); //*
		pClusterF->width = pClusterF->points.size();
		pClusterF->height = 1;
		pClusterF->is_dense = false;

		clusters.push_back(pClusterF);

		j++;
	}
}
