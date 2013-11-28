#include "BackgroundSubtractor.h"

int type = CV_16UC1;


BackgroundSubtractor::BackgroundSubtractor(int nFrames, int nmixtures) : m_NFrames(nFrames)
{
	m_pSubtractorA = new cv::BackgroundSubtractorMOG2(m_NFrames, nmixtures, false);
	m_pSubtractorB = new cv::BackgroundSubtractorMOG2(m_NFrames, nmixtures, false);

	m_pSubtractorA->set("backgroundRatio", 0.999);
	m_pSubtractorB->set("backgroundRatio", 0.999);
}


BackgroundSubtractor::~BackgroundSubtractor(void)
{
}


bool BackgroundSubtractor::isReady()
{
	return m_NFrames == 0;
}


void BackgroundSubtractor::operator()(DepthFrame frame, float alpha)
{
	(*this)(m_pSubtractorA, frame, alpha);

	m_NFrames--;
}


void BackgroundSubtractor::operator()(DepthFrame frameA, DepthFrame frameB, float alpha)
{
	(*this)(m_pSubtractorA, frameA, alpha);
	(*this)(m_pSubtractorB, frameB, alpha);

	m_NFrames--;
}


void BackgroundSubtractor::operator()(cv::BackgroundSubtractorMOG2* pSubtractor, DepthFrame& frame, float alpha)
{
	cv::Mat frameMatF;
	frame.getDepthMap().convertTo(frameMatF, type);
	
	cv::Mat maskMat;
	(*pSubtractor)(frameMatF, maskMat, alpha);
}


void BackgroundSubtractor::subtract(DepthFrame frame, DepthFrame& foreground)
{
	subtract(m_pSubtractorA, frame, foreground);
}


void BackgroundSubtractor::subtract(DepthFrame frameA, DepthFrame frameB, DepthFrame& foregroundA, DepthFrame& foregroundB)
{
	subtract(m_pSubtractorA, frameA, foregroundA);
	subtract(m_pSubtractorB, frameB, foregroundB);
}


void BackgroundSubtractor::subtract(cv::BackgroundSubtractorMOG2* pSubtractor, DepthFrame& frame, DepthFrame& foreground)
{
	cv::Mat maskMat;
	cv::Mat frameMatF;
	frame.getDepthMap().convertTo(frameMatF, type);
	(*pSubtractor)(frameMatF, maskMat, 0); // learning rate = 0

	// Erode and dilate mask
	int erosion_size = 2;
	int dilation_size = 2;

	cv::Mat erode_element = cv::getStructuringElement( cv::MORPH_ERODE,
                                cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                cv::Point( erosion_size, erosion_size ) );
	cv::Mat dilate_element = cv::getStructuringElement( cv::MORPH_DILATE,
                                cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                cv::Point( dilation_size, dilation_size ) );

	cv::erode(maskMat, maskMat, erode_element);
	cv::dilate(maskMat, maskMat, dilate_element);

	foreground = DepthFrame(frame);
	foreground.setMask(maskMat);
	//foreground.setMat(frame.getMat(), maskMat);
}