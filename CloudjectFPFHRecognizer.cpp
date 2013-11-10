#include "CloudjectFPFHRecognizer.h"

#include <pcl/features/normal_3d.h>

CloudjectFPFHRecognizer::CloudjectFPFHRecognizer()
{

}


CloudjectFPFHRecognizer::CloudjectFPFHRecognizer(float normalRadius, float fpfhRadius)
	: m_NormalRadius(normalRadius), m_FpfhRadius(fpfhRadius)
{

}


void CloudjectFPFHRecognizer::recognize(Cloudject& cloudject)
{
	
}


void CloudjectFPFHRecognizer::describeView(pcl::PointCloud<pcl::PointXYZ>::Ptr pView,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr signature)
{
	//
	// Normals preprocess
	//

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (pView);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr pNormals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (m_NormalRadius);

	// Compute the features
	ne.compute (*pNormals);	


	//
	// FPFH description extraction
	//

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (pView);
	fpfh.setInputNormals (pNormals);
	// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
	fpfh.setSearchMethod (tree);

	// Output datasets
	// * initialize outside

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch (m_FpfhRadius);

	// Compute the features
	fpfh.compute (*signature);
}


void CloudjectFPFHRecognizer::describe(Cloudject& cloudject, pcl::PointCloud<pcl::FPFHSignature33>::Ptr signature)
{
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr signatureA, signatureB;

	describeView(cloudject.getViewA(),signatureA);
	describeView(cloudject.getViewB(),signatureB);

	//cloudject.describe(*signatureA, *signatureB);
}

void CloudjectFPFHRecognizer::recognize(std::vector<Cloudject>& cloudjects)
{
	for (int i = 0; i < cloudjects.size(); i++)
	{
		recognize(cloudjects[i]);
	}
}

CloudjectFPFHRecognizer::~CloudjectFPFHRecognizer(void)
{
}