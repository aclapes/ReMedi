#pragma once

#include "DepthFrame.h"

#include <opencv2/opencv.hpp>

class BackgroundSubtractor
{
public:
	BackgroundSubtractor(int, int);
	~BackgroundSubtractor(void);

	void operator()(DepthFrame, DepthFrame&, float alpha = 0);

	bool isReady();

	void subtract(DepthFrame, DepthFrame&);

private:
	cv::BackgroundSubtractorMOG2*	m_pSubtractor;
	int								m_NFrames;

	Frame							m_BackgroundModel;
};

