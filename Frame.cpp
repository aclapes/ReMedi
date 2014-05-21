#include "Frame.h"


Frame::Frame(void)
{
}


Frame::Frame(cv::Mat mat)
{
	m_Mat = mat;
}


Frame::~Frame(void)
{
}


Frame::Frame(const Frame& other)
{
	*this = other;
}


Frame& Frame::operator=(const Frame& other)
{
	if (this != &other)
    {
        m_Mat = other.m_Mat;
    }

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

int Frame::getResX()
{
    return m_Mat.cols;
}

int Frame::getResY()
{
    return m_Mat.rows;
}