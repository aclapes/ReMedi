#pragma once

#include "Cloudject.hpp"

#include <vector>

template<typename PointT, typename U>
class CloudjectModel
{
public:
	CloudjectModel(void) {}

	CloudjectModel(const char* name, const char* path, int nViewpoints = 4, float leafSize = 0.0)
	: m_Name(name), m_nViewpoints(nViewpoints), m_LeafSize
	{
		for (int i = 0; i < m_nViewpoints; i++)
		{
			std::stringstream ss;
			ss << i;
			addCloudject( Cloudject<PointT,U>(
				(std::string(path) + ss.str() + std::string("a.pcd")).c_str(),
				(std::string(path) + ss.str() + std::string("b.pcd")).c_str()), leafSize );
		}
}

	~CloudjectModel(void) {}

private:
	void addCloudject(Cloudject<PointT,U> c)
	{
		m_cloudjects.push_back(c);
	}

	std::vector<Cloudject<PointT,U>> m_cloudjects;
	const char* m_Name;
	int m_nViewpoints;

	float m_LeafSize; // in case of downsampling
};

