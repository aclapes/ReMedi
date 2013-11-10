#include "Monitorizer.h"


Monitorizer::Monitorizer(float posCorrespThres) : m_posCorrespThres(posCorrespThres)
{
}


void Monitorizer::monitor(PointCloudPtr cloudA, PointCloudPtr cloudB)
{

}


void Monitorizer::handleCloudjectDrops(std::vector<Cloudject>& cloudjects)
{
	std::vector<Cloudject> detectedCjs;
	m_cjDetector.detect(m_CloudA, m_CloudB, detectedCjs);
	
	std::vector<Cloudject> appearedCjs;
	appeared(detectedCjs, appearedCjs);

	m_cjRecognizer.recognize(appearedCjs);

	drop(appearedCjs);
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


void Monitorizer::appeared(std::vector<Cloudject> detecteds, std::vector<Cloudject>& appeareds)
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
			appeareds.push_back(detecteds[i]);
		}
	}
}


bool Monitorizer::compareEquals(Cloudject a, Cloudject b)
{
	PointT posA, posB;
	posA = a.getPos();
	posB = b.getPos();

	float dist = sqrt(powf(posA.x-posB.x,2)+powf(posA.y-posB.y,2)+powf(posA.z-posB.z,2));

	return dist < m_posCorrespThres;
}



Monitorizer::~Monitorizer(void)
{
}
