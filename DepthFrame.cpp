#include "DepthFrame.h"

DepthFrame::DepthFrame(void) : Frame()
{
}


DepthFrame::DepthFrame(cv::Mat mat) : Frame(mat)
{
	// Projective depth
	m_projDepthMat = m_Mat / 8;

	// Player index map
	m_UIDMat = m_Mat - (8 * m_projDepthMat);
	m_UIDMat.convertTo(m_UIDMat, CV_8UC1);
}


DepthFrame::DepthFrame(cv::Mat fg, cv::Mat mask) : Frame(fg), m_Mask(mask)
{
	// Projective depth
	m_projDepthMat = m_Mat / 8;

	// Player index map
	m_UIDMat = m_Mat - (8 * m_projDepthMat);
	m_UIDMat.convertTo(m_UIDMat, CV_8UC1);
}


DepthFrame::DepthFrame(const DepthFrame& other) : Frame(other)
{
	other.m_Mask.copyTo(m_Mask);
	other.m_projDepthMat.copyTo(m_projDepthMat);
	other.m_UIDMat.copyTo(m_UIDMat);
}


DepthFrame::~DepthFrame(void)
{
	Frame::~Frame();
}


DepthFrame& DepthFrame::operator=(const DepthFrame& other)
{
	other.m_Mask.copyTo(m_Mask);
	other.m_projDepthMat.copyTo(m_projDepthMat);
	other.m_UIDMat.copyTo(m_UIDMat);

	Frame::operator=(other);

	return *this;
}

int DepthFrame::getResX()
{
    return Frame::getResX();
}

int DepthFrame::getResY()
{
    return Frame::getResY();
}

cv::Mat DepthFrame::getMask()
{
	return m_Mask;
}


void DepthFrame::setMask(cv::Mat mask)
{
	m_Mask = mask;
}


bool DepthFrame::isValid()
{
	return Frame::isValid();
}


cv::Mat DepthFrame::getDepthMap(cv::Mat& depthMap)
{
	m_projDepthMat.copyTo(depthMap);

	return m_projDepthMat;
}


cv::Mat DepthFrame::getDepthMap()
{
	return m_projDepthMat;
}


cv::Mat DepthFrame::getUserFreeDepthMap(cv::Mat& ufDepthMap)
{
	cv::Mat mask;
	cv::threshold(m_UIDMat, mask, 0, 255, CV_THRESH_BINARY_INV); // users' pixels to 0, non-users' ones to 255.

	m_projDepthMat.copyTo(ufDepthMap, mask);

	return ufDepthMap;
}


void DepthFrame::getPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	MatToPointCloud(m_projDepthMat, cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthFrame::getPointCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZ>());
    MatToPointCloud(m_projDepthMat, *pCloud);
	return pCloud;
}

void DepthFrame::getColoredPointCloud(ColorFrame cframe, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    cv::Mat colorMat = cframe.getMat();
	MatToColoredPointCloud(m_projDepthMat, colorMat, cloud);
}


void DepthFrame::getForegroundPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	cv::Mat fgProjDepthMat;
	m_projDepthMat.copyTo(fgProjDepthMat, m_Mask);

	MatToPointCloud(fgProjDepthMat, cloud);
}


void DepthFrame::getForegroundPointCloud(cv::Mat mask, pcl::PointCloud<pcl::PointXYZ>& cloud, bool combined)
{
	cv::Mat fgProjDepthMat;
    
    if (combined)
    {
        cv::Mat tmp, combinedMask;
        mask.convertTo(tmp, CV_8UC1);
        cv::bitwise_and(m_Mask, tmp, combinedMask);
        m_projDepthMat.copyTo(fgProjDepthMat, combinedMask);
    }
    else
    {
        m_projDepthMat.copyTo(fgProjDepthMat, m_Mask);
    }
    
	MatToPointCloud(fgProjDepthMat, cloud);
}


void DepthFrame::getUserFreePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	// Remove player from depth, setting user region depth values to 0 (like measurment errors)
	cv::Mat userFreeProjDepthMat;
	getUserFreeDepthMap(userFreeProjDepthMat);

	MatToPointCloud(userFreeProjDepthMat, cloud);
}


void DepthFrame::getForegroundUserFreePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	// Remove player from depth, setting user region depth values to 0 (like measurment errors)
	cv::Mat userFreeProjDepthMat;
	getUserFreeDepthMap(userFreeProjDepthMat);

	cv::Mat fgUserFreeProjDepthMat;
	userFreeProjDepthMat.copyTo(fgUserFreeProjDepthMat, m_Mask);

	MatToPointCloud(fgUserFreeProjDepthMat, cloud);
}

void DepthFrame::show(std::string wndName)
{
    cv::namedWindow(wndName);
    cv::imshow(wndName, m_projDepthMat);
    cv::waitKey();
}