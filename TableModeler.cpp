#include "TableModeler.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>


TableModeler::TableModeler() : m_pViz(new pcl::visualization::PCLVisualizer())
{
	m_pViz->addCoordinateSystem();
}


TableModeler::~TableModeler()
{

}


void TableModeler::setInputClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB)
{
	m_pCloudA = pCloudA;
	m_pCloudB = pCloudB;
}


void TableModeler::setLeafSize(float leafSize)
{
	m_LeafSize = leafSize;
}


void TableModeler::setNormalRadius(float normalRadius)
{
	m_NormalRadius = normalRadius;
}


void TableModeler::setSACIters(int iters)
{
	m_SACIters = iters;
}


void TableModeler::setSACDistThresh(float distThresh)
{
	m_SACDistThresh = distThresh;
}


void TableModeler::setYOffset(float yOffset)
{
	m_OffsetA = yOffset;
	m_OffsetB = yOffset;
}


void TableModeler::model(pcl::PointCloud<pcl::PointXYZ>& plane)
{
	estimate(m_pCloudA, plane, m_MinA, m_MaxA, m_ytonA);
}


void TableModeler::model(pcl::PointCloud<pcl::PointXYZ>& planeA, pcl::PointCloud<pcl::PointXYZ>& planeB)
{
	std::cout << "Estimating plane in A ..." << std::endl;
	estimate(m_pCloudA, planeA, m_MinA, m_MaxA, m_ytonA);
	std::cout << "Esitmating plane in B ..." << std::endl;
	estimate(m_pCloudB, planeB, m_MinB, m_MaxB, m_ytonB);
}


void TableModeler::estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, pcl::PointCloud<pcl::PointXYZ>& plane,
		pcl::PointXYZ& min, pcl::PointXYZ& max, Eigen::Affine3f& yton)
{
	// downsampling

	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudF (new pcl::PointCloud<pcl::PointXYZ>);
    
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (pCloud);
	sor.setLeafSize (m_LeafSize, m_LeafSize, m_LeafSize);
	sor.filter(*pCloudF);
	
	// normal estimation

	pcl::PointCloud<pcl::Normal>::Ptr pNormalsF (new pcl::PointCloud<pcl::Normal>);
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(pCloudF);
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
 
    ne.setRadiusSearch (m_NormalRadius); // neighbors in a sphere of radius X meters
    
    ne.compute (*pNormalsF);

	// model estimation

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sac;
	sac.setOptimizeCoefficients (true); // optional
    sac.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    sac.setMethodType (pcl::SAC_RANSAC);

    sac.setMaxIterations (m_SACIters);
    sac.setDistanceThreshold (m_SACDistThresh);

    // create filtering object
    
    pcl::ExtractIndices<pcl::PointXYZ> pointExtractor;
    pcl::ExtractIndices<pcl::Normal>   normalExtractor;

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane (new pcl::PointCloud<pcl::PointXYZ>);


	// (define some auxiliary cloud variables)

    pcl::PointCloud<pcl::PointXYZ>::Ptr pAuxCloudF (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr   pAuxNormalsF (new pcl::PointCloud<pcl::Normal>);

	bool found = false; // table plane was found
    int nr_points = (int) pCloudF->points.size();
	while (!found && pCloudF->points.size() > 0.3 * nr_points) // Do just one iteration!
    {
        // Segment the largest planar component from the remaining cloud
        
        sac.setInputCloud(pCloudF);
        sac.setInputNormals(pNormalsF);
        sac.segment(*inliers, *coefficients);
          
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            exit(-1); // panic
        }

        // Extract the inliers (points in the plane)
        
        pointExtractor.setInputCloud (pCloudF);
        normalExtractor.setInputCloud(pNormalsF);
        
        pointExtractor.setIndices (inliers);
        normalExtractor.setIndices (inliers);
        
        pointExtractor.setNegative (false);
        pointExtractor.filter (*pPlane);
          
        std::cerr << "PointCloud representing the planar component: " << pPlane->width * pPlane->height << " data points." << std::endl;
		
		pointExtractor.setNegative (true); // we keep what is not in the plane
        normalExtractor.setNegative(true); // and the associated normals
        
        pointExtractor.filter (*pAuxCloudF);
        normalExtractor.filter (*pAuxNormalsF);
        
        // update variables
        pCloudF.swap (pAuxCloudF);
        pNormalsF.swap (pAuxNormalsF);

		// Check

		if ( found = isTowardsLookingDirectionPlane(pPlane) )
		{
			// Compute a transformation in which a bounding box in a convenient base

			Eigen::Vector3f origin (0, 0, coefficients->values[3]/coefficients->values[2]);
			Eigen::Vector3f n (coefficients->values[0], coefficients->values[1], coefficients->values[2]);
			//n = -n; // it is upside down
			Eigen::Vector3f u ( 1, 1, (- n.x() - n.y() ) / n.z() );
			Eigen::Vector3f v = n.cross(u);

			pcl::getTransformationFromTwoUnitVectorsAndOrigin(n.normalized(), v.normalized(), -origin, yton);

			pcl::PointCloud<pcl::PointXYZ>::Ptr pPlaneT (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::transformPointCloud(*pPlane, *pPlaneT, yton);

			pcl::PointCloud<pcl::PointXYZ>::Ptr pPlaneTF (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr pPlaneF (new pcl::PointCloud<pcl::PointXYZ>);
			 
			// Create the filtering object
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud (pPlaneT);
			sor.setMeanK (100);
			sor.setStddevMulThresh (1);
			sor.filter (*pPlaneTF);

			pcl::PointCloud<pcl::PointXYZ>::Ptr pBiggestClusterT (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr pBiggestCluster (new pcl::PointCloud<pcl::PointXYZ>);
			biggestEuclideanCluster(pPlaneTF, 2.5 * m_LeafSize, *pBiggestClusterT);

			pcl::getMinMax3D(*pBiggestClusterT, min, max);

			pcl::transformPointCloud(*pBiggestClusterT, *pBiggestCluster, yton.inverse());

			// Visualization

	        m_pViz->addPointCloud(pCloudF, "filtered");
			m_pViz->addPointCloud(pBiggestCluster, "biggestCluster");
	        m_pViz->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.41, 0.1, 1.0, "biggestCluster");  
	          
			m_pViz->spin();

	        m_pViz->removeAllPointClouds();

			
			pcl::copyPointCloud(*pPlane, plane);

			return;
		}
	}
}


bool TableModeler::isTowardsLookingDirectionPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane)
{
	for (int i = 0; i < pPlane->points.size(); i++)
	{
		// Is there any point in the camera viewpoint direction (or close to)?
		// That is having x and y close to 0
		const pcl::PointXYZ & p =  pPlane->points[i];
		if (abs(p.x) < 10*m_LeafSize && abs(p.y) < 10*m_LeafSize && abs(p.z) < 10*m_LeafSize)
		{
			return true;
		}
	}

	return false;
}


void TableModeler::segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, 
	pcl::PointCloud<pcl::PointXYZ>& cloudObjs)
{
	segment(pCloud, m_MinA, m_MaxA, m_OffsetA, m_ytonA, cloudObjs);
}


void TableModeler::segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB, 
	pcl::PointCloud<pcl::PointXYZ>& cloudObjsA, pcl::PointCloud<pcl::PointXYZ>& cloudObjsB)
{
	segment(pCloudA, m_MinA, m_MaxA, m_OffsetA, m_ytonA, cloudObjsA);
	segment(pCloudB, m_MinB, m_MaxB, m_OffsetB, m_ytonB, cloudObjsB);
}


void TableModeler::segment(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, pcl::PointXYZ min, pcl::PointXYZ max, 
	float offset, Eigen::Affine3f yton, pcl::PointCloud<pcl::PointXYZ>& cloudObjs)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudAux (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudAuxF (new pcl::PointCloud<pcl::PointXYZ>());

	// Transformation
	pcl::transformPointCloud(*pCloud, *pCloudAux, yton);
	
	//pcl::visualization::PCLVisualizer viz ("hola");
	//viz.addCoordinateSystem();

	//viz.addPointCloud (pCloudAux, "cloud left");
 //   viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0, "cloud left");
 //   viz.addPointCloud (pCloudAuxF, "cloud right");
 //   viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 0, "cloud right");

	//viz.spin();


	// Passthrough

	pcl::PassThrough<pcl::PointXYZ> pt;

	pt.setInputCloud(pCloudAux);
	pt.setFilterFieldName("x");
	pt.setFilterLimits(min.x, max.x);
	pt.filter(*pCloudAuxF);

	pCloudAuxF.swap(pCloudAux);

	pt.setInputCloud(pCloudAux);
	pt.setFilterFieldName("y");
	pt.setFilterLimits(max.y, max.y + offset);
	pt.filter(*pCloudAuxF);

	pCloudAuxF.swap(pCloudAux);

	pt.setInputCloud(pCloudAux);
	pt.setFilterFieldName("z");
	pt.setFilterLimits(min.z, max.z);
	pt.filter(*pCloudAuxF);

	// De-transformation

	pcl::transformPointCloud(*pCloudAuxF, cloudObjs, yton.inverse());
}
