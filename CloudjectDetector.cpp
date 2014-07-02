#include "CloudjectDetector.h"
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#include "Remedi.h"

class Remedi;

CloudjectDetector::CloudjectDetector()
: m_NormalRadius(0.025), m_FpfhRadius(0.05), m_ScoreThreshold(0.5)
{
}

CloudjectDetector::CloudjectDetector(const CloudjectDetector& rhs)
{
    *this = rhs;
}

CloudjectDetector::~CloudjectDetector()
{
}

CloudjectDetector& CloudjectDetector::operator=(const CloudjectDetector& rhs)
{
    if (this != &rhs)
    {
        m_pCloudA = rhs.m_pCloudA;
        m_pCloudB = rhs.m_pCloudB;
        
        m_pTableTopCloudA = rhs.m_pTableTopCloudA;
        m_pTableTopCloudB = rhs.m_pTableTopCloudB;
        
        m_ClustersA = rhs.m_ClustersA;
        m_ClustersB = rhs.m_ClustersB;
        
        m_LeafSize = rhs.m_LeafSize;
        m_MaxCorrespondenceDistance = rhs.m_MaxCorrespondenceDistance;
        m_TmpWnd = rhs.m_TmpWnd;
        m_NormalRadius = rhs.m_NormalRadius;
        m_FpfhRadius = rhs.m_FpfhRadius;
        
        m_CloudjectModels = rhs.m_CloudjectModels;
        
        m_CloudjectsHistory = rhs.m_CloudjectsHistory;
        
        m_AppearedCloudjects = rhs.m_AppearedCloudjects;
        m_DisappearedCloudjects = rhs.m_DisappearedCloudjects;
    }
    
    return *this;
}

void CloudjectDetector::clear()
{
    m_CloudjectsHistory.clear();
    m_AppearedCloudjects.clear();
    m_DisappearedCloudjects.clear();
}

void CloudjectDetector::loadCloudjectModels(string dir)
{
    pcl::PCDReader reader;
    
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
	sor.setLeafSize (m_LeafSize, m_LeafSize, m_LeafSize);

    
    const char* path = dir.c_str();
	if( boost::filesystem::exists( path ) )
	{
		boost::filesystem::directory_iterator end;
		boost::filesystem::directory_iterator iter(path);
		for( ; iter != end ; ++iter )
		{
			if ( !is_directory( *iter ) && (iter->path().extension().string().compare(".pcd") == 0) )
            {
                string filename = iter->path().filename().string();
                
                PointCloudPtr pObject (new PointCloud);
                reader.read(path + filename, *pObject);
                
                vector<string> details;
                boost::split(details, filename, boost::is_any_of("_"));
                int id = stoi(details[0]);
                string name = details[1];
                int nview = stoi(details[2]);
                
                if (nview == 0)
                {
                    //cout << "Created model " << name << endl;
                    
                    CloudjectModel cloudjectModel (id, name, m_ModelLeafSize, 1);
                    m_CloudjectModels.push_back(cloudjectModel);
                }
                
                //cout << "Adding view to " << name << " ..." << endl;
                
                m_CloudjectModels.back().addView(pObject);
            }
		}
	}

    for (int i = 0; i < m_CloudjectModels.size(); i++)
    {
        //cout << "Describing model " << m_CloudjectModels[i].getName() << " ..." << endl;
        m_CloudjectModels[i].describe(m_NormalRadius, m_FpfhRadius);
    }
    
    cout << "Cloudject models created" << endl;
}

void CloudjectDetector::setInputClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr pTableTopCloudA,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr pTableTopCloudB)
{
	m_pCloudA = pCloudA;
    m_pCloudB = pCloudB;
    m_pTableTopCloudA = pTableTopCloudA;
	m_pTableTopCloudB = pTableTopCloudB;
}

void CloudjectDetector::setInputClusters(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersB)
{
	m_ClustersA = clustersA;
	m_ClustersB = clustersB;
}

void CloudjectDetector::setCloudjectModels(vector<CloudjectModel> models)
{
    m_CloudjectModels = models;
}

int CloudjectDetector::getNumCloudjectModels()
{
    return m_CloudjectModels.size();
}

void CloudjectDetector::setModelLeafSize(float leafSize)
{
	m_ModelLeafSize = leafSize;
}

void CloudjectDetector::setLeafSize(float leafSize)
{
	m_LeafSize = leafSize;
}


void CloudjectDetector::setMaxCorrespondenceDistance(float maxCorrespDist)
{
	m_MaxCorrespondenceDistance = maxCorrespDist;
}

void CloudjectDetector::setTemporalWindow(int tmpWnd)
{
    m_TmpWnd = tmpWnd;
}

void CloudjectDetector::setNormalRadius(float radius)
{
    m_NormalRadius = radius;
}

void CloudjectDetector::setFpfhRadius(float radius)
{
    m_FpfhRadius = radius;
}

void CloudjectDetector::setScoreThreshold(float threshold)
{
    m_ScoreThreshold = threshold;
}

void CloudjectDetector::extractClustersFromView(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters )
{
	pcl::PointCloud<PointT>::Ptr pCloudR (new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr pCloudF (new pcl::PointCloud<PointT>());

    pcl::ApproximateVoxelGrid<PointT> avg;
	avg.setInputCloud(pCloud);
	avg.setLeafSize(m_LeafSize, m_LeafSize, m_LeafSize);
	avg.filter(*pCloudF);
    
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//	sor.setInputCloud (pCloudR);
//	sor.setMeanK (10);
//	sor.setStddevMulThresh (1.0);
//	sor.filter (*pCloudF);



	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (pCloudF);

	vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance  ( 8 * m_LeafSize ); // cm
	ec.setMinClusterSize ( 0.25 / m_LeafSize );
	ec.setMaxClusterSize ( 1000 / m_LeafSize );
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


void CloudjectDetector::computeCentroids(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, 
	vector<Eigen::Vector4f>& centroids)
{
	for (int i = 0; i < clusters.size(); i++)
	{
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*(clusters[i]), centroid);
		centroids.push_back(centroid);
	}
}


float CloudjectDetector::correspondenceDistance(
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustersA,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustersB, 
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& rejecteds)
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
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& P, 
	int n, 
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& C,
	vector<
		std::pair<
			vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,
			vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
		>
	>& V )
{
	if (n == 0)
	{
		std::pair<
			vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,
			vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > p;
		p.first = C;
		p.second = P;
		V.push_back(p); // store choices
	}
	else
	{
		for (int i = 0; i < P.size(); i++)
		{
			vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tmpP = P;
			vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tmpC = C;

			tmpC.push_back(tmpP[i]); // add to choices
			tmpP.erase(tmpP.begin()+i); // remove from possible future choices

			variate(tmpP, n-1, tmpC, V);
		}
	}
}


void CloudjectDetector::variations(
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& P, 
	int n, 
	vector<
		std::pair<
			vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,
			vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
		>
	>& V
)
{
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> C;
	C.resize(0);

	variate(P, n, C, V);
}


void CloudjectDetector::findCorrespondences(
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA, 
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersB, 
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsA,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsB,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversA,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversB)
{
	int sA = clustersA.size();
	int sB = clustersB.size();

	// Each cluster in the smaller list could have a correspondence in the larger one. non-sense in the other way
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters; // pending to match (smaller list)
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> candidates; // to match with (larger list)

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

	vector<std::pair<vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>,vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > > V;
	variations(candidates, clusters.size(), V);

	float dist, minDist = std::numeric_limits<float>::infinity();
	int bestCorrespondence;

	for (int i = 0; i < V.size(); i++)
	{
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> rejecteds_i;
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


void CloudjectDetector::makeInterviewCorrespondences (
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersB,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsA,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& correspsB,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversA,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& leftoversB )
{

	// Variables initialization

	// Each cluster in the smaller list could have a correspondence in the larger one. non-sense in the other way
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters; // pending to match (smaller list)
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> candidates; // to match with (larger list)

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


	vector<Eigen::Vector4f> centroids, centroidsCandidates;

	computeCentroids(clusters, centroids);
	computeCentroids(candidates, centroidsCandidates);

	vector<int> correspondences (centroids.size(), -1);
    vector<int> correspondencesCandidates (centroidsCandidates.size(), -1);
	vector<float> minimumDistances (centroids.size(), std::numeric_limits<float>::infinity());
//	std::fill_n(correspondences, centroids.size(), -1);
//	std::fill_n(correspondencesCandidates, centroidsCandidates.size(), -1);
//	std::fill_n(minimumDistances, centroids.size(), std::numeric_limits<float>::infinity());


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
					if ((found = (correspondencesCandidates[jj] == i)))
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

	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> corresps, correspsCandidates, leftovers, leftoversCandidates;

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

//    // DEBUG
//	std::cout << correspsA.size() << " " << correspsB.size() << " " << leftoversA.size() << " " << leftoversB.size() << std::endl;
}


float CloudjectDetector::matchClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pSrcCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pTgtCloud)
{
    float minDist = numeric_limits<float>::max();
    for (int i = 0; i < pSrcCloud->size(); i++)
    {
        for (int j = 0; j < pTgtCloud->size(); j++)
        {
            pcl::PointXYZ& p = pSrcCloud->points[i];
            pcl::PointXYZ& q = pTgtCloud->points[j];

            float dist = sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2) + pow(p.z - q.z, 2));

            if (dist < minDist)
            {
                minDist = dist;
            }
        }

    }
    return minDist;
}


void CloudjectDetector::matchClustersFromView(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> src, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tgt, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& matches)
{
    for (int i = 0; i < src.size(); i++)
        for (int j = 0; j < tgt.size(); j++)
        {
            float ratio = ((float) src[i]->size()) / tgt[j]->size();
            if (ratio > 0.5) // an object with more than a half of its points outside the table surface should fall, and hence it is the subject *** RE-COMMENT ***
            {
                float dist = matchClusters(src[i], tgt[j]);
                std::cout << i << "(" << src[i]->size() << "), " << j << "(" << tgt[j]->size() << "): " <<  dist << std::endl;
            
                if (dist < 2 * m_LeafSize)
                {
                    matches.push_back(src[i]);
                    break;
                }
            }
        }
}

void CloudjectDetector::detect()
{
//	// Extract clusters from each view separately
//	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersA, clustersB,
//                                                     auxTableTopClustersA, auxTableTopClustersB,
//                                                     tableTopClustersA, tableTopClustersB;
//
//    extractClustersFromView(m_pCloudA, clustersA);
//	extractClustersFromView(m_pCloudB, clustersB);
//    
//	extractClustersFromView(m_pTableTopCloudA, auxTableTopClustersA);
//	extractClustersFromView(m_pTableTopCloudB, auxTableTopClustersB);
//    
//    matchClustersFromView(auxTableTopClustersA, clustersA, tableTopClustersA);
//    matchClustersFromView(auxTableTopClustersB, clustersB, tableTopClustersB);

	// Merge clusters extracted separately
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> correspsA, correspsB, leftoversA, leftoversB; // clouds just seen in one image

    // rename to: findViewsCorrespondences(...)
	makeInterviewCorrespondences(m_ClustersA, m_ClustersB, correspsA, correspsB, leftoversA, leftoversB);

    assert (correspsA.size() == correspsB.size());
    
	// Create cloudjects from detected clusters in views' pointclouds
    vector<Cloudject> cloudjects;
    
	for (int i = 0; i < correspsA.size(); i++)
		cloudjects.push_back( Cloudject(correspsA[i], correspsB[i]) );

	for (int i = 0; i < leftoversA.size(); i++)
		cloudjects.push_back( Cloudject(leftoversA[i], MASTER_VIEWPOINT) );

	for (int i = 0; i < leftoversB.size(); i++)
		cloudjects.push_back( Cloudject(leftoversB[i], SLAVE_VIEWPOINT) );

    makeSpatiotemporalCorrespondences(cloudjects, m_AppearedCloudjects, m_DisappearedCloudjects, m_CloudjectsHistory);
    
    recognize(m_CloudjectsHistory);
    

//    // Handle predictions/outputs
//    
//    vector<vector<vector<pcl::PointXYZ> > > positions(2);
//    for (int i = 0; i < positions.size(); i++)
//        positions[i].resize(m_CloudjectModels.size());
//    
//    for (int i = 0; i < m_CloudjectsHistory.size(); i++)
//    {
//        Cloudject pCloudject = m_CloudjectsHistory[i][0];
//
//        // Method call output (cloudjects)
//        cloudjects.push_back(pCloudject);
//        
//        // Detector output (xyz positions)
//        int oid  = pCloudject->getID();
//        if (oid >= 0)
//        {
//            int view = pCloudject->getViewpoint();
//            if (view != BINOCULAR_VIEWPOINT)
//            {
//                pcl::PointXYZ pos = (view == MASTER_VIEWPOINT) ? pCloudject->getPosA()
//                                                               : pCloudject->getPosB();
//                positions[view][oid].push_back(pos);
//            }
//            else
//            {
//                positions[MASTER_VIEWPOINT][oid].push_back(pCloudject->getPosA());
//                positions[SLAVE_VIEWPOINT][oid].push_back (pCloudject->getPosB());
//            }
//        }
//    }
//    
//    m_DetectionOutput.add(positions);
    
}

void CloudjectDetector::makeSpatiotemporalCorrespondences(vector<Cloudject> cloudjects, vector<Cloudject>& appeared, vector<Cloudject>& disappeared, vector< vector<Cloudject> >& cloudjectsHistory)
{
    // Check if, based in a certain criterion, the detected cloudject represents
    // an object that was already there.
    
    // If the cloudject represents an object that has been there during the last
    // K frames, K cloudject instances should be found in an arbitrary row of
    // cloudjectsHistory. Insert it at the beginning of the row and append the
    // row to the updated history.
    
    // If not, start a new row with this solely instance and append this
    // one-element row to the updated history.
    
    // Note the rows in cloudjectsHistory corresponding to a disappeared object
    // won't be appended in the updated history, since no cloudject exists to
    // trigger any of the two previous conditions.
    
    vector< vector<Cloudject> > cloudjectsHistoryTmp; // updated history (empty)
    
    appeared.clear();
    disappeared.clear();
    
    for (int i = 0; i < cloudjects.size(); i++) // detected cloudjects in frame
    {
        int minIdx; // most similar
        float minDist = std::numeric_limits<float>::max(); // distance to most similar
        
        // Was already there the object represented by the cloudject?
        for (int j = 0; j < cloudjectsHistory.size(); j++)
        {
            float dist1, dist2;
            cloudjects[i].distanceTo(cloudjectsHistory[j][0], &dist1, &dist2);
            
            if (dist1 >= 0 && dist1 < minDist)
            {
                minDist = dist1;
                minIdx = j;
            }
            if (dist2 >= 0 && dist2 < minDist)
            {
                minDist = dist2;
                minIdx = j;
            }
        }
        
        vector<Cloudject> cloudjectHistory; // history of a cloudject
        if (minDist <= m_MaxCorrespondenceDistance) // apply "being there" crit
        {
            cloudjectHistory = cloudjectsHistory[minIdx];
            cloudjectHistory.insert(cloudjectHistory.begin(), cloudjects[i]);
            if (cloudjectHistory.size() > m_TmpWnd) cloudjectHistory.pop_back();
        }
        else
        {
            cloudjectHistory.push_back(cloudjects[i]);
            
            appeared.push_back(cloudjects[i]); // was not already present
        }
        
        cloudjectsHistoryTmp.push_back(cloudjectHistory);
    }
    
    // Keep track of disappeared cloudjects in this frame
    for (int i = 0; i < cloudjectsHistory.size(); i++)
    {
        bool present = false;
        for (int j = 0; j < cloudjectsHistoryTmp.size() && !present; j++)
        {
            present = ( &(cloudjectsHistory[i][0]) == &(cloudjectsHistoryTmp[j][0]) );
        }
        if (!present) disappeared.push_back(cloudjectsHistory[i][0]);
    }
    
    cloudjectsHistory = cloudjectsHistoryTmp;
}


bool lt_cmp(const std::pair<int,double>& left, const std::pair<int,double>& right)
{
    return left.second < right.second;
}

bool gt_cmp(const std::pair<int,double>& left, const std::pair<int,double>& right)
{
    return left.second > right.second;
}

/**
 * A greedy assignatin given a matrix of scores.
 *
 * Given the scores' vectors for every element, assign the index of the
 * maximum score possible.
 * @param scores the matrix of scores (a row per element)
 * @param assignations the final assignations maximizing the scores
 * @param assigned_socres the final scores got in the assignations
 */
void CloudjectDetector::greedyAssign(vector< vector<double> > scores, vector<int>& assignations, vector<double>& assigned_scores)
{
    vector<int> assignations_tmp (scores.size(), 0);
    vector<double> assigned_scores_tmp (scores.size(), 0);
    
    for (int i = 0; i < scores.size(); i++)
    {
        for (int j = 0; j < scores[i].size(); j++)
        {
            if (scores[i][j] >= assigned_scores_tmp[j])
            {
                assignations_tmp[i] = j;
                assigned_scores_tmp[i] = scores[i][j];
            }
        }
    }
    
    assignations = assignations_tmp;
    assigned_scores = assigned_scores_tmp;
}

/**
 * A combinatorial optimization function given a matrix of scores.
 *
 * Given the scores' vectors for every element, assign the index of the
 * maximum score possible. If during the assignations it is found the
 * index was previously assigned, check the two scores and keep the one
 * with higher score and change the previously assigned (recursively).
 * @param scores the matrix of scores (a row per element)
 * @param assignations the final assignations maximizing the scores
 * @param assigned_socres the final scores got in the assignations
 */
void CloudjectDetector::combAssign(vector< vector<double> > scores, vector<int>& assignations, vector<double>& assigned_scores)
{
    // attach an integer index to the scores.
    // in the posterior ordering of the matrix's rows
    // the original positions can be tracked
    vector< vector< pair<int,double> > > indexed_scores;
    for (int i = 0; i < scores.size(); i++)
    {
        assert (scores[i].size() == scores[0].size());
        
        vector< pair<int,double> > row_indexed_scores (scores[i].size());
        for (int j = 0; j < scores[i].size(); j++)
            row_indexed_scores[j] = pair<int,double>(j,scores[i][j]);
        
        indexed_scores.push_back(row_indexed_scores);
    }
    
    // compute the ordered version (by rows) of the matrix of scores
    vector< vector< pair<int,double> > > ordered_indexed_scores;
    for (int i = 0; i < indexed_scores.size(); i++)
    {
        vector< pair<int,double> > row_ordered_indexed_scores = indexed_scores[i];
        std::sort(row_ordered_indexed_scores.begin(),
                  row_ordered_indexed_scores.end(),
                  gt_cmp);
        ordered_indexed_scores.push_back(row_ordered_indexed_scores);
    }
    
    // output variables with the results
    vector<int> assignationsTmp (indexed_scores.size(), -1);
    vector<double> assignscores (indexed_scores.size(), 0);
    
    // assign recursively
    // that is, if a new assignation has conflicts with a previous, try to
    // reassign the previous, which at the same time could cause new
    // conflicts
    for (int i = 0; i < indexed_scores.size(); i++)
    {
        recursive_assignation(ordered_indexed_scores, i, false,
                              assignationsTmp, assignscores);
    }
    
    assignations = assignationsTmp;
}

/**
 * Score-driven assignation of the idx-th element and recursive
 * re-assignation.
 *
 * Finds the best assignation possible to the element represented by the
 * idx-th rows of the ordered matrix. The best assignation possible is the
 * one maximizing the score. If the assignations causes a conflict, re-
 * assigns the previous assignation if necessary (achieved a lower score).
 * @param ordered a matrix of scores in which vectors are column-wise
 * sorted in descendent order
 * @param idx indicates the row of ordered dealt with
 * @param recursive indicates if the level of recursion is greater than 1.
 *   If not, there is no need to check assignations made for indices > idx.
 * @param assignations the assignations got after solving idx-th
 *   assignation.
 * @param assignscores the corresponding scores got in the assignations.
 */
void CloudjectDetector::recursive_assignation(vector< vector< pair<int,double> > > ordered,
                                              int idx, bool recursive,
                                              vector<int>& assignations, vector<double>& assignscores)
{
    for (int j = 0; j < ordered[idx].size(); j++)
    {
        assert (ordered[idx].size() == ordered[0].size());
        
        // Special case
        bool nextly_assigned = false;
        if (recursive) // level of recursion > 1, so a conflict is being dealt with
        {
            // check if the assignation was already made in greater indices (idx)
            for (int k = idx + 1; k < assignations.size() && !nextly_assigned; k++)
                nextly_assigned = (assignations[k] == ordered[idx][j].first);
        }
        
        bool previously_assigned = false;
        int past_assignation; // if conclict with past assignation, track it
        for (int k = idx - 1; k >= 0 && !previously_assigned; k--)
        {
            // check if the assignation was already made in lower indices (idx)
            previously_assigned = (ordered[idx][j].first == assignations[k]);
            if (previously_assigned) past_assignation = k;
        }
        
        if (!nextly_assigned)
        {
            if (!previously_assigned)
            {
                // No conflicts (straightforward case)
                if (ordered[idx][j].second > assignscores[idx])
                {
                    assignations[idx] = ordered[idx][j].first;
                    assignscores[idx] = ordered[idx][j].second;
                }
            }
            else
            {
                // Conflictive situation
                // If new assignation gets a greater score than already
                // assigned, re-assign that (recursively)
                if (ordered[idx][j].second > assignscores[idx]
                    && ordered[idx][j].second > assignscores[past_assignation])
                {
                    assignations[idx] = ordered[idx][j].first;
                    assignscores[idx] = ordered[idx][j].second;
                    
                    assignations[past_assignation] = -1;
                    assignscores[past_assignation] = 0;
                    recursive_assignation(ordered, past_assignation, true,
                                          assignations, assignscores);
                }
            }
        }
    }
}


void CloudjectDetector::recognize(vector< vector<Cloudject> >& history)
{
    vector< vector<double> > cloudjectScores (history.size());
    
    for (int i = 0; i < history.size(); i++)
    {
        history[i][0].describe(m_NormalRadius, m_FpfhRadius);
        for (int j = 0; j < m_CloudjectModels.size(); j++)
        {
            double score = m_CloudjectModels[j].match(history[i][0]);
            cloudjectScores[i].push_back(score);
//            cout << "\t";
            //cout << score << "\t";
        }
        //cout << endl;
    }
    //cout << endl;
    
    vector<int> assignations;
    vector<double> assignscores;
    greedyAssign(cloudjectScores, assignations, assignscores);
    
    for (int i = 0; i < history.size(); i++)
    {
        //cout << assignations[i] << " ";
        history[i][0].setID(assignations[i] + 1);
        if (assignations[i] >= 0)
            history[i][0].setName(m_CloudjectModels[assignations[i]].getName());
    }
}

void CloudjectDetector::getPresentCloudjects(vector<Cloudject>& cloudjects)
{
    cloudjects.clear();
    for (int i = 0; i < m_CloudjectsHistory.size(); i++)
        cloudjects.push_back(m_CloudjectsHistory[i][0]);
}

void CloudjectDetector::getAppearedCloudjects(vector<Cloudject>& cloudjects)
{
    cloudjects.clear();
    cloudjects = m_AppearedCloudjects;
}

void CloudjectDetector::getDisappearedCloudjects(vector<Cloudject>& cloudjects)
{
    cloudjects.clear();
    cloudjects = m_DisappearedCloudjects;
}