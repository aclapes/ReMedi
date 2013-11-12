#include "Monitorizer.h"


Monitorizer::Monitorizer(float leafSize, float posCorrespThres) 
	: m_LeafSize(leafSize), m_PosCorrespThres(posCorrespThres)
{
}


void Monitorizer::monitor(PointCloudPtr cloudA, PointCloudPtr cloudB)
{
	m_CloudA = cloudA;
	m_CloudB = cloudB;
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

	return dist < m_PosCorrespThres;
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


Monitorizer::~Monitorizer(void)
{
}
