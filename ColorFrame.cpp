#include "ColorFrame.h"

ColorFrame::ColorFrame() : Frame()
{
}


ColorFrame::ColorFrame(cv::Mat mat) : Frame(mat)
{
}


ColorFrame::ColorFrame(const ColorFrame& rhs) : Frame(rhs)
{
    *this = rhs;
}


ColorFrame::~ColorFrame()
{
}


ColorFrame& ColorFrame::operator=(const ColorFrame& rhs)
{
    if (this != &rhs)
    {
        // -
    }

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