#include "ColorFrame.h"

ColorFrame::ColorFrame(void) : Frame()
{
}


ColorFrame::ColorFrame(cv::Mat mat) : Frame(mat)
{
}


ColorFrame::ColorFrame(const ColorFrame& other) : Frame(other)
{
}


ColorFrame::~ColorFrame(void)
{
	Frame::~Frame();
}


ColorFrame& ColorFrame::operator=(const ColorFrame& other)
{
	Frame::operator=(other);

	return *this;
}

bool ColorFrame::isValid()
{
	return Frame::isValid();
}

int ColorFrame::getResX()
{
    return Frame::getResX();
}

int ColorFrame::getResY()
{
    return Frame::getResY();
}

void ColorFrame::show(std::string wndName)
{
    cv::namedWindow(wndName);
    cv::imshow(wndName, m_Mat);
    cv::waitKey();
}