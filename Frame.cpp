#include "Frame.h"


Frame::Frame(void)
{
}


Frame::Frame(cv::Mat mat)
{
	mat.copyTo(m_Mat);
}


Frame::~Frame(void)
{
	m_Mat.release();
}


Frame::Frame(const Frame& other)
{
	other.m_Mat.copyTo(m_Mat);
}


Frame& Frame::operator=(const Frame& other)
{
	other.m_Mat.copyTo(m_Mat);

	return *this;
}


bool Frame::isValid()
{
	return m_Mat.rows > 0 && m_Mat.cols > 0;
}


cv::Mat Frame::getMat()
{
	return m_Mat;
}


void Frame::setMat(cv::Mat mat)
{
	mat.copyTo(m_Mat);
}

void Frame::setMat(cv::Mat mat, cv::Mat mask)
{
	//m_Mat.release();
	//m_Mat.create(mat.rows, mat.cols, mat.type());
	mat.copyTo(m_Mat, mask);
}