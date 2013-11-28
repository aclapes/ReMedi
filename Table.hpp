#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

class Table
{
public:

	Table(void) : m_pCloud( new pcl::PointCloud<pcl::PointXYZ> ) {}
	~Table(void) {}

	void getTableCloud(pcl::PointCloud<pcl::PointXYZ>& tableCloud, bool copy = false)
	{
		if (copy)
		{
			pcl::copyPointCloud(*m_pCloud, tableCloud);
		}
		else tableCloud = *m_pCloud;
	}

	void setTableCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
	{
		pcl::copyPointCloud(*pCloud, *m_pCloud);
	}

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCloud;
};