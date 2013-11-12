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