#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>

#include "Descriptor.hpp"

template<typename PointT, typename U>
class CloudjectBase
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
public:
	CloudjectBase(void) {}

	CloudjectBase(PointCloudPtr viewA, float leafSize = 0.0)
	{
		if (leafSize > 0.0)
		{
			m_ViewA = PointCloudPtr(new PointCloud);

			downsample(viewA, leafSize, m_ViewA);
		}
		else
		{
			m_ViewA = viewA;
		}

		// TODO: the position should be the mean of the the two centroids from the two views, not from only A
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*m_ViewA, centroid);
		m_Pos.x = centroid.x();
		m_Pos.y = centroid.y();
		m_Pos.z = centroid.z();
	}


	CloudjectBase(PointCloudPtr viewA, PointCloudPtr viewB, float leafSize = 0.0)
	{
		if (leafSize > 0.0)
		{
			m_ViewA = PointCloudPtr(new PointCloud);
			m_ViewB = PointCloudPtr(new PointCloud);

			downsample(viewA, leafSize, m_ViewA);
			downsample(viewB, leafSize, m_ViewB);
		}
		else
		{
			m_ViewA = viewA;
			m_ViewB = viewB;
		}

		// TODO: the position should be the mean of the the two centroids from the two views, not from only A
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*m_ViewA, centroid);
		m_Pos.x = centroid.x();
		m_Pos.y = centroid.y();
		m_Pos.z = centroid.z();
	}


	CloudjectBase(const char* viewPathA, const char* viewPathB, float leafSize = 0.0)
	{
		PointCloudPtr viewA (new PointCloud);
		PointCloudPtr viewB (new PointCloud);
	
		pcl::PCDReader reader;
		reader.read(viewPathA, *viewA);
		reader.read(viewPathB, *viewB);

		if (leafSize > 0.0)
		{
			m_ViewA = PointCloudPtr(new PointCloud);
			m_ViewB = PointCloudPtr(new PointCloud);

			downsample(viewA, leafSize, m_ViewA);
			downsample(viewB, leafSize, m_ViewB);
		}
		else
		{
			m_ViewA = viewA;
			m_ViewB = viewB;
		}
	}


	CloudjectBase(const CloudjectBase& cloudject)
	{
		m_ViewA = cloudject.m_ViewA;
		m_ViewB = cloudject.m_ViewB;
		m_Pos	= cloudject.m_Pos;
	}


	virtual ~CloudjectBase(void) {}


	PointT getPos()
	{
		return m_Pos;
	}


	PointCloudPtr getViewA()
	{
		return m_ViewA;
	}


	PointCloudPtr getViewB()
	{
		return m_ViewB;
	}


protected:
	// Methods

	void downsample(PointCloudPtr pCloud, float leafSize, PointCloudPtr pFilteredCloud)
	{
		pcl::ApproximateVoxelGrid<PointT> avg;
		avg.setInputCloud(pCloud);
		avg.setLeafSize(leafSize, leafSize, leafSize);
		avg.filter(*pFilteredCloud);
	}

	// Members

	PointCloudPtr m_ViewA, m_ViewB;
	PointT m_Pos;
};


template<typename PointT, typename U>
class Cloudject : public CloudjectBase<PointT,U>
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename pcl::PointCloud<PointT> PointCloudPtr;

public:
	Cloudject() : CloudjectBase<PointT,U>() {}
	Cloudject(PointCloudPtr viewA, float leafSize = 0.0) 
		: CloudjectBase<PointT,U>(viewA, leafSize) {}
	Cloudject(PointCloudPtr viewA, PointCloudPtr viewB, float leafSize = 0.0) 
		: CloudjectBase<PointT,U>(viewA, viewB, leafSize) {}
	Cloudject(const char* viewPathA, const char* viewPathB, float leafSize = 0.0) 
		: CloudjectBase<PointT,U>(viewPathA, viewPathB, leafSize) {}
	Cloudject(const Cloudject<PointT,U>& cloudject) 
		: CloudjectBase<PointT,U>(cloudject) {}

	virtual ~Cloudject() {}

	PointT getPos() { return CloudjectBase<PointT,U>::getPos(); }
	PointCloudPtr getViewA() { return CloudjectBase<PointT,U>::getViewA(); }
	PointCloudPtr getViewB() { return CloudjectBase<PointT,U>::getViewB(); }

private:
	void downsample(PointCloudPtr pCloud, float leafSize, PointCloudPtr pFilteredCloud)
	{
		CloudjectBase<PointT,U>::downsample(pCloud, leafSize, pFilteredCloud);
	}
};


template<>
class Cloudject<pcl::PointXYZ, pcl::FPFHSignature33> : public CloudjectBase<pcl::PointXYZ, pcl::FPFHSignature33>
{
public:
	Cloudject() 
		: CloudjectBase<pcl::PointXYZ,pcl::FPFHSignature33>() {}
	Cloudject(pcl::PointCloud<pcl::PointXYZ>::Ptr viewA, float leafSize = 0.0) 
		: CloudjectBase<pcl::PointXYZ,pcl::FPFHSignature33>(viewA, leafSize) {}
	Cloudject(pcl::PointCloud<pcl::PointXYZ>::Ptr viewA, pcl::PointCloud<pcl::PointXYZ>::Ptr viewB, float leafSize = 0.0) 
		: CloudjectBase<pcl::PointXYZ,pcl::FPFHSignature33>(viewA, viewB, leafSize) {}
	Cloudject(const char* viewPathA, const char* viewPathB, float leafSize = 0.0) 
		: CloudjectBase<pcl::PointXYZ,pcl::FPFHSignature33>(viewPathA, viewPathB, leafSize) {}
	Cloudject(const Cloudject<pcl::PointXYZ,pcl::FPFHSignature33>& cloudject) 
		: CloudjectBase<pcl::PointXYZ,pcl::FPFHSignature33>(cloudject) {}

	virtual ~Cloudject() {}

	pcl::PointXYZ getPos() { return CloudjectBase::getPos(); }
	pcl::PointCloud<pcl::PointXYZ>::Ptr getViewA() { return CloudjectBase::getViewA(); }
	pcl::PointCloud<pcl::PointXYZ>::Ptr getViewB() { return CloudjectBase::getViewB(); }

	void describe(pcl::PointCloud<pcl::FPFHSignature33>::Ptr descA, pcl::PointCloud<pcl::FPFHSignature33>::Ptr descB)
	{
		m_DescriptorA = descA;
		m_DescriptorB = descB;
	}

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr getDescriptionA()
	{
		return m_DescriptorA;
	}

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr getDescriptionB()
	{
		return m_DescriptorB;
	}


private:
	void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, float leafSize, pcl::PointCloud<pcl::PointXYZ>::Ptr pFilteredCloud)
	{
		CloudjectBase::downsample(pCloud, leafSize, pFilteredCloud);
	}

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr m_DescriptorA, m_DescriptorB;
};
