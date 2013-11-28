#include "conversion.h"

#include <math.h>


void MatToPointCloud(cv::Mat& mat, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	cloud.height = mat.rows;
    cloud.width =  mat.cols;
    cloud.resize(cloud.height * cloud.width);
    cloud.is_dense = true;
    
    // Easy to handle the conversion this way
    cv::Mat temp = cv::Mat(mat.size(), CV_32F);
    mat.convertTo(temp, CV_32F);
    
    float invfocal = 3.501e-3f; // Kinect inverse focal length. If depth map resolution of: 320 x 240
    if (mat.cols == 640) // else if 640 x 480
        invfocal /= 2.0;
    
	unsigned short z_us;
	float z;
	float rwx, rwy, rwz;
	pcl::PointXYZ p;

    for (unsigned int y = 0; y < cloud.height; y++) for (unsigned int x = 0; x < cloud.width; x++)
    {
		//z_us = mat.at<unsigned short>(y,x) >> 3;
		z = (float) mat.at<unsigned short>(y,x) /*z_us*/;
             
		rwx = (x - 320.0) * invfocal * z;
		rwy = (y - 240.0) * invfocal * z;
		rwz = z;
              
		p.x = rwx/1000.f; 
		p.y = rwy/1000.f;
		p.z = rwz/1000.f;
		cloud.at(x,y) = p;
    }
}



void MatToPointCloud(cv::Mat mat, cv::Mat mask, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	cloud.height = mat.rows;
    cloud.width =  mat.cols;
    cloud.resize(cloud.height * cloud.width);
    cloud.is_dense = true;
    
    // Easy to handle the conversion this way
    cv::Mat temp = cv::Mat(mat.size(), CV_32F);
    mat.convertTo(temp, CV_32F);
    
    float invfocal = 3.501e-3f; // Kinect inverse focal length. If depth map resolution of: 320 x 240
    if (mat.cols == 640) // else if 640 x 480
        invfocal /= 2.0;
    
	unsigned short z_us;
	float z;
	float rwx, rwy, rwz;
	pcl::PointXYZ p;

    for (unsigned int y = 0; y < cloud.height; y++) for (unsigned int x = 0; x < cloud.width; x++)
    {
		z = (mask.at<unsigned char>(y,x) == 255) ? ((float) mat.at<unsigned short>(y,x)) : 0;
             
		rwx = (x - 320.0) * invfocal * z;
		rwy = (y - 240.0) * invfocal * z;
		rwz = z;
              
		p.x = rwx/1000.f; 
		p.y = rwy/1000.f;
		p.z = rwz/1000.f;
		cloud.at(x,y) = p;
    }
}


void PointCloudToMat(pcl::PointCloud<pcl::PointXYZ>& cloud, cv::Mat& mat)
{
	mat = cv::Mat::zeros(cloud.height, cloud.width, CV_16UC1);
    
    float invfocal = 3.501e-3f; // Kinect inverse focal length. If depth map resolution of: 320 x 240
    if (mat.cols == 640) // else if 640 x 480
        invfocal /= 2.0;
    
    for (unsigned int i = 0; i < cloud.points.size(); i++)
    {
        float rwx = cloud.points[i].x;
        float rwy = cloud.points[i].y;
        float rwz = cloud.points[i].z;
            
        float x, y, z;
    
        if (rwz > 0)
        {
            z = rwz * 1000.f;
			x = std::floor( ((rwx ) / (invfocal * rwz )) + (cloud.width / 2.f) );
			y = std::floor( ((rwy ) / (invfocal * rwz )) + (cloud.height / 2.f) );

			if (x > 0 && x < cloud.width && y > 0 && y < cloud.height)
            {
                mat.at<unsigned short>(y,x) = z;
            }
        }
    }
}


void MaskDensePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudSrc, cv::Mat maskMat, 
	pcl::PointCloud<pcl::PointXYZ>& cloudTgt)
{
	for (unsigned int y = 0; y < pCloudSrc->height; y++) for (unsigned int x = 0; x < pCloudSrc->width; x++)
	{
		unsigned char maskValue;

		if (maskMat.type() == CV_8U || maskMat.type() == CV_8UC1)
			maskValue = maskMat.at<unsigned char>(y,x);
		else if (maskMat.type() == CV_32F || maskMat.type() == CV_32FC1)
			maskValue = static_cast<unsigned char>(maskMat.at<float>(y,x));

		if (maskValue > 0)
		{
			cloudTgt.points.push_back(pCloudSrc->points[y * pCloudSrc->width + x]);
		}
	}

	cloudTgt.width = cloudTgt.points.size();
	cloudTgt.height = 1;
	cloudTgt.is_dense = false;
}


void passthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, pcl::PointXYZ min, pcl::PointXYZ max, 
	pcl::PointCloud<pcl::PointXYZ>& cloudF)
{
	cloudF = *pCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudF (&cloudF);

	pcl::PassThrough<pcl::PointXYZ> pt;

	pt.setInputCloud(pCloudF);
	pt.setFilterFieldName("x");
	pt.setFilterLimits(min.x, max.x);
	pt.filter(*pCloudF);

	pt.setInputCloud(pCloudF);
	pt.setFilterFieldName("y");
	pt.setFilterLimits(min.y, max.y);
	pt.filter(*pCloudF);

	pt.setInputCloud(pCloudF);
	pt.setFilterFieldName("z");
	pt.setFilterLimits(min.z, max.z);
	pt.filter(*pCloudF);
}


void enclosure(cv::Mat src, cv::Mat& dst, int size)
{
	// Erode and dilate mask
	int erosion_size = size;
	int dilation_size = size;

	cv::Mat erode_element = cv::getStructuringElement( cv::MORPH_ERODE,
                                cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                cv::Point( erosion_size, erosion_size ) );
	cv::Mat dilate_element = cv::getStructuringElement( cv::MORPH_DILATE,
                                cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                cv::Point( dilation_size, dilation_size ) );

	cv::erode(src, src, erode_element);
	cv::dilate(src, dst, dilate_element);
}


void biggestEuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float clusterTol,
	pcl::PointCloud<pcl::PointXYZ>& cluster)
{
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (clusterTol); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (clusterIndices);

	// Supposedly, the biggest is the first in the list of extractions' indices
	std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();

	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		cluster.points.push_back (cloud->points[*pit]);

	cluster.width = cluster.points.size ();
	cluster.height = 1;
	cluster.is_dense = false;
}


float euclideanDistance(Eigen::Vector4f u, Eigen::Vector4f v)
{
	return std::sqrtf(std::powf(u.x() - v.x(), 2) + std::powf(u.y() - v.y(), 2) + std::powf(u.z() - v.z(), 2));
}