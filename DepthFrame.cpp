#include "DepthFrame.h"

DepthFrame::DepthFrame(void) : Frame()
{
}


DepthFrame::DepthFrame(cv::Mat mat) : Frame(mat)
{
	//// Projective depth
	//m_projDepthMat = m_Mat / 8;

	//// Player index map
	//m_UIDMat = m_Mat - (8 * m_projDepthMat);
	//m_UIDMat.convertTo(m_UIDMat, CV_8UC1);

	//mat.convertTo(mat, CV_16UC1);
	m_projDepthMat.create(mat.rows, mat.cols, CV_16UC1);
	m_UIDMat.create(mat.rows, mat.cols, CV_8UC1);

	const unsigned short playerMask = 0x0007; 

	for (int y = 0; y < mat.rows; y++) for (int x = 0; x < mat.cols; x++)
	{
		unsigned short d = mat.at<unsigned short>(y,x);
		m_projDepthMat.at<unsigned short>(y,x) = d >> 3;
		m_UIDMat.at<unsigned char>(y,x) = static_cast<unsigned char>(d & playerMask);
	}

	int erosionSize = 2;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                       cv::Size( 2*erosionSize + 1, 2*erosionSize+1 ),
                                       cv::Point( erosionSize, erosionSize ) );

	cv::dilate(m_UIDMat, m_UIDMat, element);
}


DepthFrame::DepthFrame(const DepthFrame& other) : Frame(other)
{
	other.m_projDepthMat.copyTo(m_projDepthMat);
	other.m_UIDMat.copyTo(m_UIDMat);
}


DepthFrame::~DepthFrame(void)
{
	m_projDepthMat.release();
	m_UIDMat.release();

	Frame::~Frame();
}


DepthFrame& DepthFrame::operator=(const DepthFrame& other)
{
	other.m_projDepthMat.copyTo(m_projDepthMat);
	other.m_UIDMat.copyTo(m_UIDMat);

	Frame::operator=(other);

	return *this;
}


bool DepthFrame::isValid()
{
	return Frame::isValid();
}


cv::Mat DepthFrame::getDepthMap(cv::Mat& depthMap)
{
	m_Mat.copyTo(depthMap);

	return m_Mat;
}


cv::Mat DepthFrame::getUserFreeDepthMap(cv::Mat& ufDepthMap)
{
	cv::Mat mask;
	cv::threshold(m_UIDMat, mask, 0, 255, CV_THRESH_BINARY_INV); // users' pixels to 0, non-users' ones to 255.

	m_projDepthMat.copyTo(ufDepthMap, mask);

	return ufDepthMap;
}


pcl::PointCloud<pcl::PointXYZ>& DepthFrame::getPointCloud()
{
	pcl::PointCloud<pcl::PointXYZ>* pCloud = new pcl::PointCloud<pcl::PointXYZ>();

	MatToPointCloud(m_projDepthMat, *pCloud);

	return *pCloud;
}


pcl::PointCloud<pcl::PointXYZ>& DepthFrame::getUserFreePointCloud()
{
	pcl::PointCloud<pcl::PointXYZ>* pCloud = new pcl::PointCloud<pcl::PointXYZ>();

	// Remove player from depth, setting user region depth values to 0 (like measurment errors)
	cv::Mat userFreeProjDepthMat;
	getUserFreeDepthMap(userFreeProjDepthMat);

	MatToPointCloud(userFreeProjDepthMat, *pCloud);

	return *pCloud;
}