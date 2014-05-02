#include "TableModeler.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>


TableModeler::TableModeler()
: m_pViz(new pcl::visualization::PCLVisualizer()), m_bLimitsNegative(false)
{
	m_pViz->addCoordinateSystem();
}

TableModeler::TableModeler(const TableModeler& other)
{
    m_pCloudA = other.m_pCloudA;
    m_pCloudB = other.m_pCloudB;
    
	m_LeafSize = other.m_LeafSize;
	m_NormalRadius = other.m_NormalRadius;
	m_SACIters = other.m_SACIters;
	m_SACDistThresh = other.m_SACDistThresh;
    
	m_MinA = other.m_MinA;
    m_MaxA = other.m_MaxA;
    m_MinB = other.m_MinB;
    m_MaxB = other.m_MaxB;
    
	m_OffsetA = other.m_OffsetA;
    m_OffsetB = other.m_OffsetB;
    
    m_ytonA = other.m_ytonA;
    m_ytonB = other.m_ytonB;
    
    m_ytonAInv = other.m_ytonAInv;
    m_ytonBInv = other.m_ytonBInv;
    
    m_bLimitsNegative = other.m_bLimitsNegative;
    m_InteractionBorder = other.m_InteractionBorder;
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

void TableModeler::setInteractionBorder(float border)
{
	m_InteractionBorder = border;
}


void TableModeler::model()
{
    std::cout << "Estimating plane in A ..." << std::endl;
	estimate(m_pCloudA, m_MinA, m_MaxA, m_ytonA);
    m_ytonAInv = m_ytonA.inverse();
    
    std::cout << "Esitmating plane in B ..." << std::endl;
	estimate(m_pCloudB, m_MinB, m_MaxB, m_ytonB);
    m_ytonBInv = m_ytonB.inverse();

//    m_pViz->addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(&planeA), "planeA");
//    m_pViz->addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(&planeB), "planeB");
//    m_pViz->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.6, 0.2, 0.2, "biggestCluster");
//    m_pViz->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.6, 0.2, "biggestCluster");
//    
//    m_pViz->spin();
//    
//    m_pViz->removeAllPointClouds();
}

void TableModeler::getPointsDimensionCI(pcl::PointCloud<pcl::PointXYZ>& plane, int dim, float alpha, float& min, float& max)
{
    cv::Mat values (plane.size(), 1, cv::DataType<float>::type);
    for (int i = 0; i < plane.size(); i++)
    {
        if (dim == 1)
            values.at<float>(i,0) = plane.points[i].x;
        else if (dim == 2)
            values.at<float>(i,0) = plane.points[i].y;
        else
            values.at<float>(i,0) = plane.points[i].z;
    }
    
    cv::Scalar mean, stddev;
    cv::meanStdDev(values, mean, stddev);
    
    float pval;
    if (alpha == 0.80) pval = 1.28;
    else if (alpha == 0.85) pval = 1.44;
    else if (alpha == 0.90) pval = 1.65;
    else if (alpha == 0.95) pval = 1.96;
    else if (alpha == 0.99) pval = 2.57;
    else pval = 1.96;
    
    float confidence = pval * (stddev.val[0] / sqrt(values.rows));
    min = mean.val[0] - confidence;
    max = mean.val[0] + confidence;
}

void TableModeler::estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
		pcl::PointXYZ& min, pcl::PointXYZ& max, Eigen::Affine3f& yton)
{
	// downsampling

	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudF (new pcl::PointCloud<pcl::PointXYZ>);
    
    
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr pBiggestCluster (new pcl::PointCloud<pcl::PointXYZ>);

		if ( (found = isPlaneIncludingOrigin(pPlane)) )
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
//			pcl::PointCloud<pcl::PointXYZ>::Ptr pBiggestCluster (new pcl::PointCloud<pcl::PointXYZ>);
            pBiggestCluster = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

			biggestEuclideanCluster(pPlaneTF, 2.5 * m_LeafSize, *pBiggestClusterT);

			pcl::getMinMax3D(*pBiggestClusterT, min, max);
            getPointsDimensionCI(*pBiggestClusterT, 2, 0.8, min.y, max.y);

			pcl::transformPointCloud(*pBiggestClusterT, *pBiggestCluster, yton.inverse());
            pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudT (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*pCloud, *pCloudT, yton);
            //pcl::copyPointCloud(*pPlane, plane);
            
            // DEBUG
            // Visualization
            
            pcl::visualization::PCLVisualizer pViz(pCloud->header.frame_id);
            pViz.addCoordinateSystem();
            //pViz.addPointCloud(pCloudF, "filtered");
            pViz.addPointCloud(pBiggestCluster, "biggestCluster");
            pViz.addPointCloud(pBiggestClusterT, "biggestClusterT");
            pViz.addPointCloud(pCloudT, "pCloudT");
            pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.61, 0.1, 1.0, "biggestCluster");
            pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.21, 0.1, 1.0, "biggestClusterT");
            pViz.addCube(min.x, max.x, min.y, max.y + m_OffsetA, min.z, max.z, 1, 0, 0, "cube");
            pViz.addCube(min.x - m_InteractionBorder, max.x + m_InteractionBorder,
                         min.y, max.y + 2.0,
                         min.z - m_InteractionBorder, max.z + m_InteractionBorder, 1, 1, 1, "cube2");
            
            int c = 0;
            while(c++ < 2.5)
            {
                pViz.spinOnce(1000);
            }

			return;
		}

	}
}


bool TableModeler::isPlaneIncludingOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane)
{
	for (int i = 0; i < pPlane->points.size(); i++)
	{
		// Is there any point in the camera viewpoint direction (or close to)?
		// That is having x and y close to 0
		const pcl::PointXYZ & p =  pPlane->points[i];
        float dist = sqrtf(powf(p.x,2)+powf(p.y,2)+powf(p.z,2));
		if ( dist > 0 && dist < 2 * m_LeafSize)
		{
			return true;
		}
	}
    
	return false;
}


void TableModeler::segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
                                   pcl::PointCloud<pcl::PointXYZ>& cloudObjs)
{
	segmentTableTop(pCloud, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjs);
}


void TableModeler::segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB,
                                   pcl::PointCloud<pcl::PointXYZ>& cloudObjsA, pcl::PointCloud<pcl::PointXYZ>& cloudObjsB)
{
	segmentTableTop(pCloudA, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjsA);
	segmentTableTop(pCloudB, m_MinB, m_MaxB, m_OffsetB, m_ytonB, m_ytonBInv, cloudObjsB);
}


void TableModeler::segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, pcl::PointXYZ min, pcl::PointXYZ max,
                                   float offset, Eigen::Affine3f yton, Eigen::Affine3f ytonInv, pcl::PointCloud<pcl::PointXYZ>& cloudObjs)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudAux (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudAuxF (new pcl::PointCloud<pcl::PointXYZ>());
    
	// Transformation
	pcl::transformPointCloud(*pCloud, *pCloudAux, yton);
	
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr inTableTopRegionCondition (new pcl::ConditionAnd<pcl::PointXYZ> ());
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, min.x)));
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, max.x)));
    
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, max.y)));
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, max.y + offset)));
    
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, min.z)));
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, max.z)));
    
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem (inTableTopRegionCondition);
    
    condrem.setInputCloud (pCloudAux);
    condrem.setKeepOrganized(true);
    condrem.filter (*pCloudAuxF);
    
	// De-transformation
    
	pcl::transformPointCloud(*pCloudAuxF, cloudObjs, ytonInv);
}

void TableModeler::segmentInteractionRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
                                            pcl::PointCloud<pcl::PointXYZ>& cloudObjs)
{
	segmentInteractionRegion(pCloud, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjs);
}


void TableModeler::segmentInteractionRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB,
                                            pcl::PointCloud<pcl::PointXYZ>& cloudObjsA,
                                            pcl::PointCloud<pcl::PointXYZ>& cloudObjsB)
{
	segmentInteractionRegion(pCloudA, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjsA);
	segmentInteractionRegion(pCloudB, m_MinB, m_MaxB, m_OffsetB, m_ytonB, m_ytonBInv, cloudObjsB);
}


void TableModeler::segmentInteractionRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
                                            pcl::PointXYZ min, pcl::PointXYZ max,
                                            float offset, Eigen::Affine3f yton, Eigen::Affine3f ytonInv,
                                            pcl::PointCloud<pcl::PointXYZ>& cloudObjs)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudAux (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudAuxF (new pcl::PointCloud<pcl::PointXYZ>());

	// Transformation
	pcl::transformPointCloud(*pCloud, *pCloudAux, yton);
    
//    pcl::visualization::PCLVisualizer viz ("hola");
//    viz.addCoordinateSystem();
//    int c = 0;
//    
//    viz.addCube(min.x, max.x, min.y, max.y + m_OffsetA, min.z, max.z, 1, 1, 1, "cube");
//    viz.addCube(min.x - m_InteractionBorder, max.x + m_InteractionBorder,
//                 max.y, max.y + 1.5,
//                 min.z - m_InteractionBorder, max.z + m_InteractionBorder, 1, 0, 0, "cube2");
    
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr inInteractionRegionRangeCondition (new pcl::ConditionAnd<pcl::PointXYZ> ());
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, min.x - m_InteractionBorder)));
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, max.x + m_InteractionBorder)));
    
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, max.y)));
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, max.y + offset + m_InteractionBorder)));
    
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, min.z - m_InteractionBorder)));
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, max.z + m_InteractionBorder)));
    
    condrem.setCondition(inInteractionRegionRangeCondition);
    
    condrem.setInputCloud (pCloudAux);
    condrem.setKeepOrganized(true);
    condrem.filter (*pCloudAuxF);
    
    
    pCloudAuxF.swap(pCloudAux);
    
    pcl::ConditionOr<pcl::PointXYZ>::Ptr outTableTopRangeCondition (new pcl::ConditionOr<pcl::PointXYZ> ());
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, min.x)));
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, max.x)));
    
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, max.y)));
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, max.y + offset)));
    
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, min.z)));
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, max.z)));
    
    condrem.setCondition(outTableTopRangeCondition);
    condrem.setInputCloud (pCloudAux);
    condrem.setKeepOrganized(true);
    condrem.filter (*pCloudAuxF);
    
	// De-transformation
    
//    cout << "out tabletop filtered" << endl;
//    viz.removeAllPointClouds();
//    viz.addPointCloud (pCloudAuxF, "cloud left");
//    c = 0;
//    while(c++ < 10)
//    {
//        viz.spinOnce(1000);
//    }
    
	pcl::transformPointCloud(*pCloudAuxF, cloudObjs, ytonInv);
}

