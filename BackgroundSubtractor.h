#pragma once

#include "DepthFrame.h"
#include "Reader.h"

#include <opencv2/opencv.hpp>

class BackgroundSubtractor
{
public:
	BackgroundSubtractor(int, int);
    BackgroundSubtractor(const BackgroundSubtractor& rhs);
	~BackgroundSubtractor(void);
    
    BackgroundSubtractor& operator=(const BackgroundSubtractor& rhs);
	void operator()(DepthFrame, float alpha = 0);
	void operator()(DepthFrame, DepthFrame, float alpha = 0);

	bool isReady();

	void subtract(DepthFrame, DepthFrame&);
	void subtract(DepthFrame, DepthFrame, DepthFrame&, DepthFrame&);
    
    typedef boost::shared_ptr<BackgroundSubtractor> Ptr;

private:
	void operator()(cv::BackgroundSubtractorMOG2*, DepthFrame&, float alpha = 0);
	void subtract(cv::BackgroundSubtractorMOG2*, DepthFrame&, DepthFrame&);

	cv::BackgroundSubtractorMOG2*	m_pSubtractorA;
	cv::BackgroundSubtractorMOG2*	m_pSubtractorB;
	int								m_NFrames;
};

