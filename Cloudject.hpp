#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/normal_3d.h>


enum Viewpoint { MASTER_VIEWPOINT = 0, SLAVE_VIEWPOINT = 1 };


template<typename PointT, typename SignatureT>
class CloudjectBase
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
    
public:
	CloudjectBase(void) { m_ID = -1; }

	CloudjectBase(PointCloudPtr view, int viewpoint, float leafSize = 0.0)
	{
		m_ID = -1;
        
        m_OriginalViewA = PointCloudPtr(new PointCloud());
        m_OriginalViewB = PointCloudPtr(new PointCloud());
        
        m_ViewA = PointCloudPtr(new PointCloud());
        m_ViewB = PointCloudPtr(new PointCloud());
        
        m_LeafSize = leafSize;
        
        if (viewpoint == MASTER_VIEWPOINT)
            init(view, *m_OriginalViewA, *m_ViewA, m_PosA, m_MedianDistA );
        else
            init(view, *m_OriginalViewB, *m_ViewB, m_PosB, m_MedianDistB );
	}


	CloudjectBase(PointCloudPtr viewA, PointCloudPtr viewB, float leafSize = 0.0)
	{
		m_ID = -1;
        m_LeafSize = leafSize;
        
        m_OriginalViewA = PointCloudPtr(new PointCloud());
        m_OriginalViewB = PointCloudPtr(new PointCloud());
        
        m_ViewA = PointCloudPtr(new PointCloud());
        m_ViewB = PointCloudPtr(new PointCloud());
        
        init(viewA, *m_OriginalViewA, *m_ViewA, m_PosA, m_MedianDistA );
        init(viewB, *m_OriginalViewB, *m_ViewB, m_PosB, m_MedianDistB );
	}
    

	CloudjectBase(const char* viewPath, int viewpoint, float leafSize = 0.0)
	{
		m_ID = -1;
        
        m_OriginalViewA = PointCloudPtr(new PointCloud());
        m_OriginalViewB = PointCloudPtr(new PointCloud());
        
        m_ViewA = PointCloudPtr(new PointCloud());
        m_ViewB = PointCloudPtr(new PointCloud());
        
        m_LeafSize = leafSize;
        
        if (viewpoint == MASTER_VIEWPOINT)
            init(viewPath, *m_OriginalViewA, *m_ViewA, m_PosA, m_MedianDistA );
        else
            init(viewPath, *m_OriginalViewB, *m_ViewB, m_PosB, m_MedianDistB );
	}
    
    
	CloudjectBase(const char* viewPathA, const char* viewPathB, float leafSize = 0.0)
	{
		m_ID = -1;
    
        m_LeafSize = leafSize;
        
        m_OriginalViewA = PointCloudPtr(new PointCloud());
        m_OriginalViewB = PointCloudPtr(new PointCloud());
        
        m_ViewA = PointCloudPtr(new PointCloud());
        m_ViewB = PointCloudPtr(new PointCloud());
        
        init(viewPathA, *m_OriginalViewA, *m_ViewA, m_PosA, m_MedianDistA );
        init(viewPathB, *m_OriginalViewB, *m_ViewB, m_PosB, m_MedianDistB );
	}


	CloudjectBase(const CloudjectBase& cloudject)
	{
		m_ID	= cloudject.m_ID;
		m_OriginalViewA = cloudject.m_OriginalViewA;
		m_OriginalViewB = cloudject.m_OriginalViewB;
		m_ViewA = cloudject.m_ViewA;
		m_ViewB = cloudject.m_ViewB;
		m_PosA	= cloudject.m_PosA;
		m_PosB	= cloudject.m_PosB;
		m_MedianDistA = cloudject.m_MedianDistA;
		m_MedianDistB = cloudject.m_MedianDistB;
	}


	virtual ~CloudjectBase(void) {}

    
    void init(PointCloudPtr view, PointCloud& originalView, PointCloud& transfView, PointT& pos, float& medianDist)
    {
        originalView = *view;
        
		if (m_LeafSize > 0.0)
			downsample(originalView, m_LeafSize, transfView);
		else
			pcl::copyPointCloud(originalView, transfView);
        
		// TODO: the position should be the mean of the the two centroids from the two views, not from only A
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(transfView, centroid);
		pos.x = centroid.x();
		pos.y = centroid.y();
		pos.z = centroid.z();
        
		medianDist = medianDistanceToCentroid(transfView, pos);
    }
    

    void init(const char* viewPath, PointCloud& originalView, PointCloud& transfView, PointT& pos, float& medianDist)
    {
		pcl::PCDReader reader;
		reader.read(viewPath, originalView);
        
		if (m_LeafSize > 0.0)
			downsample(originalView, m_LeafSize, transfView);
		else
            pcl::copyPointCloud(originalView, transfView);
        
		// Compute centroids' positions
        
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(transfView, centroid);
        
		pos.x = centroid.x();
		pos.y = centroid.y();
		pos.z = centroid.z();
        
		medianDist = medianDistanceToCentroid(transfView, pos);
    }
    
	
	int getID()
	{
		return m_ID;
	}


	void setID(int ID)
	{
		m_ID = ID;
	}


	PointT getPosA() const
	{
		return m_PosA;
	}


	PointT getPosB() const
	{
		return m_PosB;
	}


	int getNumOfPointsInOriginalViewA()
	{
		return m_OriginalViewA->points.size();
	}


	int getNumOfPointsInOriginalViewB()
	{
		return m_OriginalViewB->points.size();
	}


	int getNumOfPointsInViewA()
	{
		return m_ViewA->points.size();
	}


	int getNumOfPointsInViewB()
	{
		return m_ViewB->points.size();
	}


	float medianDistToCentroidInViewA()
	{
		return m_MedianDistA;
	}


	float medianDistToCentroidInViewB()
	{
		return m_MedianDistB;
	}


	PointCloudPtr getViewA() const
	{
		return m_ViewA;
	}


	PointCloudPtr getViewB() const
	{
		return m_ViewB;
	}


	float euclideanDistance(PointT p1, PointT p2)
	{
		return sqrt(powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2) + powf(p1.z - p2.z, 2));
	}


	float medianDistanceToCentroid(PointCloud& cloud, PointT centroid)
	{
		std::vector<float> distances;

		distances.push_back(euclideanDistance(centroid, cloud.points[0]));

		for (int i = 1; i < cloud.points.size(); i++)
		{
			float dist = euclideanDistance(centroid, cloud.points[i]);
			bool inserted = false;
			for (int j = 0; j < distances.size() && !inserted; j++)
			{
				if (dist < distances[j])
				{
					distances.insert(distances.begin() + j, dist);
					inserted = true;
				}
			}
		}

		return distances[(int)(distances.size() / 2)];
	}
    
    
    void distanceTo(const CloudjectBase& tgt, float* dist1, float* dist2)
    {
        *dist1 = -std::numeric_limits<float>::infinity();
        *dist2 = -std::numeric_limits<float>::infinity();
        
        if (!getViewA()->empty() && !getViewB()->empty()) // "this" is two-view
        {
            if (!tgt.getViewA()->empty() && !tgt.getViewB()->empty()) // "other" is two-view
            {
                *dist1 = euclideanDistance(getPosA(), tgt.getPosA());
                *dist2 = euclideanDistance(getPosB(), tgt.getPosB());
            }
            else
            {
                if (!tgt.getViewA()->empty()) // "other" is one-view (master)
                {
                    *dist1 = euclideanDistance(getPosA(), tgt.getPosA());
                    *dist2 = euclideanDistance(getPosB(), tgt.getPosA());
                }
                else // "other" is one-view (slave)
                {
                    *dist1 = euclideanDistance(getPosA(), tgt.getPosB());
                    *dist2 = euclideanDistance(getPosB(), tgt.getPosB());
                }
            }
        }
        else // "this" is one-view (master)
        {
            if (!getViewA()->empty()) // "this" is master's one-view
            {
                if (!tgt.getViewA()->empty() && !tgt.getViewB()->empty()) // "other" is two-view
                {
                    *dist1 = euclideanDistance(getPosA(), tgt.getPosA());
                    *dist2 = euclideanDistance(getPosA(), tgt.getPosB());
                }
                else // "other" is one-viewed
                {
                    if (!tgt.getViewA()->empty()) // "other" is one-view (master)
                        *dist1 = euclideanDistance(getPosA(), tgt.getPosA());
                    else // "other" is one-view (slave)
                        *dist2 = euclideanDistance(getPosA(), tgt.getPosB());
                }
            }
            else // "this" is one-view (slave)
            {
                if (!tgt.getViewA()->empty() && !tgt.getViewB()->empty()) // "other" is two-view
                {
                    *dist1 = euclideanDistance(getPosB(), tgt.getPosA());
                    *dist2 = euclideanDistance(getPosB(), tgt.getPosB());
                }
                else
                {
                    if (!tgt.getViewA()->empty()) // "other" is one-view (master)
                        *dist1 = euclideanDistance(getPosB(), tgt.getPosA());
                    else // "other" is one-view (slave)
                        *dist2 = euclideanDistance(getPosB(), tgt.getPosB());
                }
            }
        }
    }
    

protected:
	// Methods

	void downsample(PointCloud& cloud, float leafSize, PointCloud& filteredCloud)
	{
		pcl::ApproximateVoxelGrid<PointT> avg;
		avg.setInputCloud(PointCloudPtr(&cloud));
		avg.setLeafSize(leafSize, leafSize, leafSize);
		avg.filter(filteredCloud);
	}

	// Members

	int m_ID;
	int m_View;
	PointCloudPtr m_OriginalViewA, m_OriginalViewB;
    
    float m_LeafSize;
	PointCloudPtr m_ViewA, m_ViewB;
    
	PointT m_PosA, m_PosB;
	float m_MedianDistA, m_MedianDistB;
};


// Locally Featured (LF) Cloudject
template<typename PointT, typename SignatureT>
class LFCloudjectBase : public CloudjectBase<PointT,SignatureT>
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
	typedef typename pcl::PointCloud<SignatureT> Descriptor;
	typedef typename pcl::PointCloud<SignatureT>::Ptr DescriptorPtr;

public:
	LFCloudjectBase() : CloudjectBase<PointT,SignatureT>() {}

	LFCloudjectBase(PointCloudPtr view, int viewpoint, float leafSize = 0.0)
		: CloudjectBase<PointT,SignatureT>(view, viewpoint, leafSize) {}

	LFCloudjectBase(PointCloudPtr viewA, PointCloudPtr viewB, float leafSize = 0.0) 
		: CloudjectBase<PointT,SignatureT>(viewA, viewB, leafSize) {}

	LFCloudjectBase(const char* viewPath, int viewpoint, float leafSize = 0.0)
    : CloudjectBase<PointT,SignatureT>(viewPath, viewpoint, leafSize) {}
    
	LFCloudjectBase(const char* viewPathA, const char* viewPathB, float leafSize = 0.0) 
		: CloudjectBase<PointT,SignatureT>(viewPathA, viewPathB, leafSize) {}

	LFCloudjectBase(const LFCloudjectBase<PointT,SignatureT>& cloudject) 
		: CloudjectBase<PointT,SignatureT>(cloudject) 
	{
		m_DescriptorA = cloudject.m_DescriptorA;
		m_DescriptorB = cloudject.m_DescriptorB;
	}

	virtual ~LFCloudjectBase() {}


	int getID() { return CloudjectBase<PointT,SignatureT>::getID(); }
	void setID(int ID) { CloudjectBase<PointT,SignatureT>::setID(ID); }

	PointT getPosA() const { return CloudjectBase<PointT,SignatureT>::getPosA(); }
	PointT getPosB() const { return CloudjectBase<PointT,SignatureT>::getPosB(); }

	int getNumOfPointsInOriginalViewA() { return CloudjectBase<PointT,SignatureT>::getNumOfPointsInOriginalViewA(); }
	int getNumOfPointsInOriginalViewB() { return CloudjectBase<PointT,SignatureT>::getNumOfPointsInOriginalViewB(); }

	int getNumOfPointsInViewA() { return CloudjectBase<PointT,SignatureT>::getNumOfPointsInViewA(); }
	int getNumOfPointsInViewB() { return CloudjectBase<PointT,SignatureT>::getNumOfPointsInViewB(); }

	float medianDistToCentroidInViewA() { return CloudjectBase<PointT,SignatureT>::medianDistToCentroidInViewA(); }
	float medianDistToCentroidInViewB() { return CloudjectBase<PointT,SignatureT>::medianDistToCentroidInViewB(); }

	PointCloudPtr getViewA() const { return CloudjectBase<PointT,SignatureT>::getViewA(); }
	PointCloudPtr getViewB() const { return CloudjectBase<PointT,SignatureT>::getViewB(); }

	float euclideanDistance(PointT p1, PointT p2) { return CloudjectBase<PointT,SignatureT>::euclideanDistance(p1,p2); }
	float medianDistanceToCentroid(PointCloud& pCloud, PointT centroid)
	{ return CloudjectBase<PointT,SignatureT>::euclideanDistance(pCloud, centroid); }

    void distanceTo(const CloudjectBase<PointT,SignatureT>& tgt, float* dist1, float* dist2)
    { CloudjectBase<PointT,SignatureT>::distanceTo(tgt, dist1, dist2); }

	void describe(DescriptorPtr descA, DescriptorPtr descB)
	{
		m_DescriptorA = descA;
		m_DescriptorB = descB;
	}


	DescriptorPtr getDescriptionA()
	{ 
		return m_DescriptorA;
	}


	DescriptorPtr getDescriptionB()
	{ 
		return m_DescriptorB;
	}

protected:
	DescriptorPtr m_DescriptorA, m_DescriptorB;
};


// Locally Featured (LF) Cloudject
template<typename PointT, typename SignatureT>
class LFCloudject : public LFCloudjectBase<PointT,SignatureT> { };


template<typename PointT>
class LFCloudject<PointT, pcl::FPFHSignature33> : public LFCloudjectBase<PointT, pcl::FPFHSignature33>
{
    typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef pcl::search::KdTree<PointT> KdTree;
	typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;

public:
	LFCloudject() 
		: LFCloudjectBase<PointT,pcl::FPFHSignature33>() { }

	LFCloudject(PointCloudPtr view, int viewpoint, float leafSize = 0.0)
		: LFCloudjectBase<PointT,pcl::FPFHSignature33>(view, viewpoint, leafSize) { }

	LFCloudject(PointCloudPtr viewA, PointCloudPtr viewB, float leafSize = 0.0) 
		: LFCloudjectBase<PointT,pcl::FPFHSignature33>(viewA, viewB, leafSize) { }

	LFCloudject(const char* viewPath, int viewpoint, float leafSize = 0.0)
    : LFCloudjectBase<PointT,pcl::FPFHSignature33>(viewPath, viewpoint, leafSize) { }
    
	LFCloudject(const char* viewPathA, const char* viewPathB, float leafSize = 0.0) 
		: LFCloudjectBase<PointT,pcl::FPFHSignature33>(viewPathA, viewPathB, leafSize) { }

	LFCloudject(const LFCloudject<PointT,pcl::FPFHSignature33>& cloudject) 
		: LFCloudjectBase<PointT,pcl::FPFHSignature33>(cloudject) { }

	virtual ~LFCloudject() {}

	//
	// Methods
	// 
	
	int getID() { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::getID(); }
	void setID(int ID) { LFCloudjectBase<PointT,pcl::FPFHSignature33>::setID(ID); }

	PointT getPosA() const { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::getPosA(); }
	PointT getPosB() const { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::getPosB(); }

	int getNumOfPointsInOriginalViewA() { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::getNumOfPointsInOriginalViewA(); }
	int getNumOfPointsInOriginalViewB() { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::getNumOfPointsInOriginalViewB(); }
	int getNumOfPointsInViewA() { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::getNumOfPointsInViewA(); }
	int getNumOfPointsInViewB() { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::getNumOfPointsInViewB(); }

	float medianDistToCentroidInViewA() { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::medianDistToCentroidInViewA(); }
	float medianDistToCentroidInViewB() { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::medianDistToCentroidInViewB(); }

	PointCloudPtr getViewA() const { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::getViewA(); }
	PointCloudPtr getViewB() const { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::getViewB(); }

	float euclideanDistance(PointT p1, PointT p2) { return LFCloudjectBase<PointT,pcl::FPFHSignature33>::euclideanDistance(p1,p2); }
	float medianDistanceToCentroid(PointCloud& pCloud, PointT centroid)
	{ return LFCloudjectBase<PointT,pcl::FPFHSignature33>::euclideanDistance(pCloud, centroid); }

	void describe(pcl::PointCloud<pcl::FPFHSignature33>::Ptr descA, pcl::PointCloud<pcl::FPFHSignature33>::Ptr descB)
	{ LFCloudjectBase<PointT,pcl::FPFHSignature33>::describe(descA, descB); }

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr getDescriptionA()
	{ return LFCloudjectBase<PointT,pcl::FPFHSignature33>::getDescriptionA(); }
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr getDescriptionB()
	{ return LFCloudjectBase<PointT,pcl::FPFHSignature33>::getDescriptionB(); }

    void distanceTo(const LFCloudject<PointT,pcl::FPFHSignature33>& tgt, float* dist1, float* dist2)
    { LFCloudjectBase<PointT,pcl::FPFHSignature33>::distanceTo(tgt, dist1, dist2); }

	void describe(float normalRadius, float fpfhRadius)
	{
        if (!getViewA()->empty())
        {
            this->m_DescriptorA = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new  pcl::PointCloud<pcl::FPFHSignature33>);
            describeView(this->m_ViewA, normalRadius, fpfhRadius, *(this->m_DescriptorA));
        }
        
		if (!getViewB()->empty())
		{
			this->m_DescriptorB = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new  pcl::PointCloud<pcl::FPFHSignature33>);
			describeView(this->m_ViewB, normalRadius, fpfhRadius, *(this->m_DescriptorB));
		}
	}


	void describeView(PointCloudPtr pView, 
					  float normalRadius, float fpfhRadius,
					  pcl::PointCloud<pcl::FPFHSignature33>& descriptor)
	{
		//
		// Normals preprocess
		//

		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		ne.setInputCloud (pView);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		KdTreePtr tree (new KdTree());
		ne.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr pNormals (new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (normalRadius);

		// Compute the features
		ne.compute (*pNormals);	

		//
		// FPFH description extraction
		//

		pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
		fpfh.setInputCloud (pView);
		fpfh.setInputNormals (pNormals);
		// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		tree = KdTreePtr(new KdTree());
		fpfh.setSearchMethod (tree);

		// Output datasets
		// * initialize outside

		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		fpfh.setRadiusSearch (fpfhRadius);

		// Compute the features
		fpfh.compute (descriptor);
	}
    
private:
	void downsample(PointCloud& cloud, float leafSize, PointCloud& filteredCloud)
	{ LFCloudjectBase<PointT,pcl::FPFHSignature33>::downsample(cloud, leafSize, filteredCloud); }
};


template<typename PointT>
class LFCloudject<PointT, pcl::PFHRGBSignature250> : public LFCloudjectBase<PointT, pcl::PFHRGBSignature250>
{
    typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef pcl::search::KdTree<PointT> KdTree;
	typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;

public:
	LFCloudject() 
		: LFCloudjectBase<PointT,pcl::PFHRGBSignature250>() { }

	LFCloudject(PointCloudPtr view, int viewpoint, float leafSize = 0.0)
		: LFCloudjectBase<PointT,pcl::PFHRGBSignature250>(view, viewpoint, leafSize) { }

	LFCloudject(PointCloudPtr viewA, PointCloudPtr viewB, float leafSize = 0.0) 
		: LFCloudjectBase<PointT,pcl::PFHRGBSignature250>(viewA, viewB, leafSize) { }

	LFCloudject(const char* viewPath, int viewpoint, float leafSize = 0.0)
    : LFCloudjectBase<PointT,pcl::PFHRGBSignature250>(viewPath, viewpoint, leafSize) { }
    
	LFCloudject(const char* viewPathA, const char* viewPathB, float leafSize = 0.0) 
		: LFCloudjectBase<PointT,pcl::PFHRGBSignature250>(viewPathA, viewPathB, leafSize) { }

	LFCloudject(const LFCloudject<PointT,pcl::PFHRGBSignature250>& cloudject) 
		: LFCloudjectBase<PointT,pcl::PFHRGBSignature250>(cloudject) { }

	virtual ~LFCloudject() {}

	//
	// Methods
	// 
	
	int getID() { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::getID(); }
	void setID(int ID) { LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::setID(ID); }

	PointT getPosA() const { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::getPosA(); }
	PointT getPosB() const { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::getPosB(); }

	int getNumOfPointsInOriginalViewA() { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::getNumOfPointsInOriginalViewA(); }
	int getNumOfPointsInOriginalViewB() { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::getNumOfPointsInOriginalViewB(); }
	int getNumOfPointsInViewA() { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::getNumOfPointsInViewA(); }
	int getNumOfPointsInViewB() { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::getNumOfPointsInViewB(); }

	float medianDistToCentroidInViewA() { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::medianDistToCentroidInViewA(); }
	float medianDistToCentroidInViewB() { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::medianDistToCentroidInViewB(); }

	PointCloudPtr getViewA() const { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::getViewA(); }
	PointCloudPtr getViewB() const { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::getViewB(); }

	float euclideanDistance(PointT p1, PointT p2) { return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::euclideanDistance(p1,p2); }
	float medianDistanceToCentroid(PointCloud& pCloud, PointT centroid)
	{ return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::euclideanDistance(pCloud, centroid); }

	void describe(pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descA, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descB)
	{ LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::describe(descA, descB); }

	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr getDescriptionA()
	{ return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::getDescriptionA(); }
	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr getDescriptionB()
	{ return LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::getDescriptionB(); }

    void distanceTo(const LFCloudject<PointT,pcl::PFHRGBSignature250>& tgt, float* dist1, float* dist2)
    { LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::distanceTo(tgt, dist1, dist2); }

	void describe(float normalRadius, float pfhrgbRadius)
	{
        if (this->m_viewA->points.size() > 0)
        {
            this->m_DescriptorA = pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr (new  pcl::PointCloud<pcl::PFHRGBSignature250>);
            describeView(this->m_ViewA, normalRadius, pfhrgbRadius, *(this->m_DescriptorA));
        }
        
        if (this->m_viewB->points.size() > 0)
        {
			this->m_DescriptorB = pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr (new  pcl::PointCloud<pcl::PFHRGBSignature250>);
			describeView(this->m_ViewB, normalRadius, pfhrgbRadius, *(this->m_DescriptorB));
		}
	}


	void describeView(PointCloudPtr pView, 
					  float normalRadius, float pfhrgbRadius,
					  pcl::PointCloud<pcl::PFHRGBSignature250>& descriptor)
	{
		//
		// Normals preprocess
		//

		// Create the normal estimation class, and pass the input dataset to it
		typename pcl::NormalEstimation<PointT, pcl::Normal> ne;
		ne.setInputCloud (pView);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		KdTreePtr tree (new KdTree());
		ne.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr pNormals (new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (normalRadius);

		// Compute the features
		ne.compute (*pNormals);	

		//
		// FPFH description extraction
		//

		pcl::PFHRGBEstimation<PointT, pcl::Normal, pcl::PFHRGBSignature250> pfhrgb;
		pfhrgb.setInputCloud (pView);
		pfhrgb.setInputNormals (pNormals);
		// alternatively, if cloud is of tpe PointNormal, do pfhrgb.setInputNormals (cloud);

		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		tree = pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>);
		pfhrgb.setSearchMethod (tree);

		// Output datasets
		// * initialize outside

		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		pfhrgb.setRadiusSearch (pfhrgbRadius);

		// Compute the features
		pfhrgb.compute (descriptor);
	}

private:
	void downsample(PointCloud& cloud, float leafSize, PointCloud& filteredCloud)
	{ LFCloudjectBase<PointT,pcl::PFHRGBSignature250>::downsample(cloud, leafSize, filteredCloud); }
};