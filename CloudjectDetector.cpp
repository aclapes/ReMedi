#include "CloudjectDetector.h"
#include <pcl/visualization/pcl_visualizer.h>

CloudjectDetector::CloudjectDetector(void)
{
}


CloudjectDetector::~CloudjectDetector(void)
{
}


void CloudjectDetector::setInputClouds( pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB )
{
	m_pCloudA = pCloudA;
	m_pCloudB = pCloudB;
}


void CloudjectDetector::setLeafSize(float leafSize)
{
	m_LeafSize = leafSize;
}


void CloudjectDetector::setMaxCorrespondenceDistance(float maxCorrespDist)
{
	m_MaxCorrespondenceDistance = maxCorrespDist;
}


void CloudjectDetector::extractClustersFromView(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters )
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
	ec.setClusterTolerance (8 * m_LeafSize); // cm
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

		//std::cout << "PointCloud representing the Cluster: " << cluster->points.size () << " data points." << std::endl;
		


		//pcl::getMinMax3D(*pClusterF, min, max);

		//pcl::PointCloud<pcl::PointXYZ>::Ptr pCluster (new pcl::PointCloud<pcl::PointXYZ>);
		//passthroughFilter(pCloud, min, max, *pCluster);

		clusters.push_back(pClusterF);

		j++;
	}

	//if (clusters.size() > 0)
	//{
	//	pcl::visualization::PCLVisualizer::Ptr pViz (new pcl::visualization::PCLVisualizer());
	//	pViz->addCoordinateSystem();
	//	std::cout << "extracted clusters" << std::endl;
	//	srand(0);
	//	for (int i = 0; i < clusters.size(); i++)
	//	{		
	//		std::stringstream ss;
	//		ss << i;
	//		pViz->addPointCloud(clusters[i], ss.str());
	//		float r = ((float) (rand() % 255)) / (2 * 255.0) + .5;
	//		float g = ((float) (rand() % 255)) / (2 * 255.0) + .5;
	//		float b = ((float) (rand() % 255)) / (2 * 255.0) + .5;
	//		 pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, ss.str());
	//		std::cout << clusters[i]->points.size() << std::endl;
	//	}

	//	while(!pViz->wasStopped())
	//		pViz->spin();
	//}


}

float CloudjectDetector::clustersBoxDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr clusterA, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr clusterB)
{
	pcl::PointXYZ minA, minB, maxA, maxB;
	float distMins, distMaxs;

	pcl::getMinMax3D(*clusterA, minA, maxA);
	pcl::getMinMax3D(*clusterB, minB, maxB);

	distMins = sqrtf(powf(minA.x - minB.x, 2) + powf(minA.y - minB.y, 2) + powf(minA.z - minB.z, 2));
	distMaxs = sqrtf(powf(maxA.x - maxB.x, 2) + powf(maxA.y - maxB.y, 2) + powf(maxA.z - maxB.z, 2));

	return (distMins + distMaxs) / 2;
}


float CloudjectDetector::clustersCentroidsDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr clusterA, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr clusterB)
{
	Eigen::Vector4f centroidA, centroidB;

	pcl::compute3DCentroid(*clusterA, centroidA);
	pcl::compute3DCentroid(*clusterB, centroidB);

	return sqrtf( powf(centroidA.x() - centroidB.x(), 2) 
				+ powf(centroidA.y() - centroidB.y(), 2) 
				+ powf(centroidA.z() - centroidB.z(), 2));
}


float CloudjectDetector::centroidsDistance(Eigen::Vector4f centroid1, Eigen::Vector4f centroid2)
{
	return sqrtf( powf(centroid1.x() - centroid2.x(), 2) 
				+ powf(centroid1.y() - centroid2.y(), 2) 
				+ powf(centroid1.z() - centroid2.z(), 2));
}


void CloudjectDetector::computeCentroids(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, 
	std::vector<Eigen::Vector4f>& centroids)
{
	for (int i = 0; i < clusters.size(); i++)
	{
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*(clusters[i]), centroid);
		centroids.push_back(centroid);
	}
}


float CloudjectDetector::correspondenceDistance(
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustersA,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustersB, 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& rejecteds)
{
	if (clustersA.size() != clustersB.size())
		return std::numeric_limits<float>::infinity(); // gross error check

	float dist, sumOfDists = 0;

	for (int i = 0; i < clustersA.size(); i++)
	{
		dist = clustersCentroidsDistance(clustersA[i], clustersB[i]);
		if (dist < 0.075) // 7.5 cm
		{
			sumOfDists += dist;
		}
		else
		{
			rejecteds.push_back(clustersA[i]);
		}
	}

	return sumOfDists;
}


void CloudjectDetector::variate(
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& P, 
	int n, 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& C,
	std::vector<
		std::pair<
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
		>
	>& V )
{
	if (n == 0)
	{
		std::pair<
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > p;
		p.first = C;
		p.second = P;
		V.push_back(p); // store choices
	}
	else
	{
		for (int i = 0; i < P.size(); i++)
		{
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tmpP = P;
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tmpC = C;

			tmpC.push_back(tmpP[i]); // add to choices
			tmpP.erase(tmpP.begin()+i); // remove from possible future choices

			variate(tmpP, n-1, tmpC, V);
		}
	}
}


void CloudjectDetector::variations(
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& P, 
	int n, 
	std::vector<
		std::pair<
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
		>
	>& V
)
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> C;
	C.resize(0);

	variate(P, n, C, V);
}


void CloudjectDetector::findCorrespondences(
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA, 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersB, 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsA,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsB,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversA,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversB)
{
	int sA = clustersA.size();
	int sB = clustersB.size();

	// Each cluster in the smaller list could have a correspondence in the larger one. non-sense in the other way
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters; // pending to match (smaller list)
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> candidates; // to match with (larger list)

	if (sA <= sB)
	{
		clusters = clustersA;
		candidates = clustersB;
	}
	else
	{
		clusters = clustersB;
		candidates = clustersA;
	}

	std::vector<std::pair<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > > V;
	variations(candidates, clusters.size(), V);

	float dist, minDist = std::numeric_limits<float>::infinity();
	int bestCorrespondence;

	for (int i = 0; i < V.size(); i++)
	{
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> rejecteds_i;
		dist = correspondenceDistance(clusters, V[i].first, rejecteds_i);
		if (dist < minDist)
		{
			minDist = dist;
			bestCorrespondence = i;
			if (sA <= sB) leftoversA = rejecteds_i;
			else leftoversB = rejecteds_i;
		}
	}
	
	if (sA <= sB)
	{
		correspsA = clustersA;
		correspsB = V[bestCorrespondence].first;
		leftoversB = V[bestCorrespondence].second;
	}
	else
	{
		correspsA = V[bestCorrespondence].first;
		leftoversA = V[bestCorrespondence].second;
		correspsB = clustersB;
	}
}


void CloudjectDetector::findCorrespondences2(
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA, 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersB, 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsA,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsB,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversA,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversB)
{

	// Variables initialization

	// Each cluster in the smaller list could have a correspondence in the larger one. non-sense in the other way
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters; // pending to match (smaller list)
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> candidates; // to match with (larger list)

	if (clustersA.size() <= clustersB.size())
	{
		clusters = clustersA;
		candidates = clustersB;
	}
	else
	{
		clusters = clustersB;
		candidates = clustersA;
	}


	std::vector<Eigen::Vector4f> centroids, centroidsCandidates;

	computeCentroids(clusters, centroids);
	computeCentroids(candidates, centroidsCandidates);

	std::vector<int> correspondences, correspondencesCandidates;
	std::vector<float> minimumDistances;

	std::fill_n(correspondences, centroids.size(), -1);
	std::fill_n(correspondencesCandidates, centroidsCandidates.size(), -1);
	std::fill_n(minimumDistances, centroids.size(), std::numeric_limits<float>::infinity());


	// Make the correspondences

	float dist;
	for (int i = 0; i < centroids.size(); i++)
	{
		for (int j = 0; j < centroidsCandidates.size(); j++)
		{
			dist = centroidsDistance(centroids[i], centroidsCandidates[j]);
			if (dist < m_MaxCorrespondenceDistance && dist < minimumDistances[i])
			{
				correspondences[i] = j; 
				minimumDistances[i] = dist;
				
				correspondencesCandidates[j] = i;
				// Deal with having the cluster assigned to a candidate (i in the list of candidates)
				bool found = false;
				for (int jj = 0; jj < j && !found; jj++)
					if (found = (correspondencesCandidates[jj] == i))
						correspondencesCandidates[jj] = -1;
				
				// Aaaaand also with having the candidate assigned already to other cluster (j in the list of clusters)
				int ii;
				found = false;
				for (ii = 0; ii < i && !found; ii++)
					found = (correspondences[ii] == correspondences[i]);
				
				// If the case is found (very rare)...
				if (found)
				{
					if (dist < minimumDistances[ii]) // the new corresponde wins
					{
						correspondences[ii] = -1; // unassign the previous
					}
					else
					{
						correspondences[i] = -1;
						correspondencesCandidates[j] = ii;
					}
				}
			}
		}
	}


	// Index the clusters

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> corresps, correspsCandidates, leftovers, leftoversCandidates;

	for (int i = 0; i < clusters.size(); i++)
	{
		if (correspondences[i] < 0)
			leftovers.push_back( clusters[i] );
		else
		{
			corresps.push_back( clusters[i] );
			correspsCandidates.push_back( candidates[correspondences[i]] );
		}
	}

	for (int i = 0; i < candidates.size(); i++)
		if (correspondencesCandidates[i] < 0)
			leftoversCandidates.push_back( candidates[i] );
	

	if (clustersA.size() <= clustersB.size())
	{
		correspsA = corresps;
		correspsB = correspsCandidates;
		leftoversA = leftovers;
		leftoversB = leftoversCandidates;
	}
	else
	{
		correspsA = correspsCandidates;
		correspsB = corresps;
		leftoversA = leftoversCandidates;
		leftoversB = leftovers;
	}

	std::cout << correspsA.size() << " " << correspsB.size() << " " << leftoversA.size() << " " << leftoversB.size() << std::endl;
}


void CloudjectDetector::detect( std::vector<Cloudject>& cloudjects )
{
	// Extract clusters from each view separately
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA, clustersB;

	extractClustersFromView(m_pCloudA, clustersA);
	extractClustersFromView(m_pCloudB, clustersB);

	// Merge clusters extracted separately
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> correspsA, correspsB, leftoversA, leftoversB; // clouds just seen in one image

	findCorrespondences2(clustersA, clustersB, correspsA, correspsB, leftoversA, leftoversB);

	// Create cloudejcts from detected clusters in views' pointclouds

	for (int i = 0; i < correspsA.size(); i++) // correspsA.size() == correspsB.size() !!
		m_Cloudjects.push_back(Cloudject(correspsA[i], correspsB[i]));

	for (int i = 0; i < leftoversA.size(); i++)
		m_Cloudjects.push_back(Cloudject(leftoversA[i]));

	for (int i = 0; i < leftoversB.size(); i++)
		m_Cloudjects.push_back(Cloudject(leftoversB[i]));

}