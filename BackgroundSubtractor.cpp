#include "BackgroundSubtractor.h"

int type = CV_16UC1;


BackgroundSubtractor::BackgroundSubtractor(int nFrames, int nmixtures) : m_NFrames(nFrames)
{
	m_pSubtractor = new cv::BackgroundSubtractorMOG2(m_NFrames, nmixtures, false);
	//m_pSubtractor->initialize(cv::Size(640,480), type);

	m_pSubtractor->set("backgroundRatio", 0.999);
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
	if (!isReady())
	{
		cv::Mat frameMatF;
		frame.getMat().convertTo(frameMatF, type);
		
		cv::Mat maskMat;
		(*m_pSubtractor)(frameMatF, maskMat, alpha);

		m_NFrames--;

//        // debug
//        cv::imshow("BS", maskMat);
//        cv::waitKey(0.5);
	}

	if (isReady())
	{
		cv::Mat frameMatF;
		frame.getMat().convertTo(frameMatF, type);
		cv::Mat maskMat;
		(*m_pSubtractor)(frameMatF, maskMat, alpha);
     
		//// debug
  //      cv::namedWindow("BS");
  //      cv::imshow("BS", maskMat);
  //      cv::waitKey(100);
	}
}


void BackgroundSubtractor::subtract(DepthFrame frame, DepthFrame& foreground)
{
	cv::Mat maskMat;
	(*m_pSubtractor)(frame.getMat(), maskMat, 0); // learning rate = 0

	// Erode and dilate mask
	int erosion_size = 3;
	int dilation_size = 3;

	cv::Mat erode_element = cv::getStructuringElement( cv::MORPH_ERODE,
                                cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                cv::Point( erosion_size, erosion_size ) );
	cv::Mat dilate_element = cv::getStructuringElement( cv::MORPH_DILATE,
                                cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                cv::Point( dilation_size, dilation_size ) );

	cv::erode(maskMat, maskMat, erode_element);
	cv::dilate(maskMat, maskMat, dilate_element);

	cv::Mat fgMat;
	frame.getMat().copyTo(fgMat, maskMat);
	foreground = DepthFrame(fgMat);
	//foreground.setMat(frame.getMat(), maskMat);
}