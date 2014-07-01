#include "BackgroundSubtractor.h"

#include "DepthFrame.h"
#include "ColorFrame.h"

int type = CV_16UC1;

BackgroundSubtractor::BackgroundSubtractor()
    : m_NumOfMixtureComponents(4), m_LearningRate(0.02), m_BackgroundRatio(0.999)
{ }

BackgroundSubtractor::BackgroundSubtractor(const BackgroundSubtractor& rhs)
{
    *this = rhs;
}

BackgroundSubtractor::~BackgroundSubtractor()
{

}

BackgroundSubtractor& BackgroundSubtractor::operator=(const BackgroundSubtractor& rhs)
{
    if (this != &rhs)
    {
        m_pSeq = rhs.m_pSeq;
        m_subtractorA = rhs.m_subtractorA;
        m_subtractorB = rhs.m_subtractorB;
        m_NumOfMixtureComponents = rhs.m_NumOfMixtureComponents;
        m_LearningRate = rhs.m_LearningRate;
    }
    
    return *this;
}

void BackgroundSubtractor::setInputSequence(Sequence::Ptr pSeq)
{
    m_pSeq = pSeq;
}

void BackgroundSubtractor::setNumOfMixtureComponents(int k)
{
    m_NumOfMixtureComponents = k;
}

void BackgroundSubtractor::setLearningRate(float rate)
{
    m_LearningRate = rate;
}

void BackgroundSubtractor::setBackgroundRatio(float ratio)
{
    m_BackgroundRatio = ratio;
}

void BackgroundSubtractor::model()
{
    m_subtractorA = cv::BackgroundSubtractorMOG2(m_pSeq->getNumOfFrames()[0], m_NumOfMixtureComponents, false);
	m_subtractorB = cv::BackgroundSubtractorMOG2(m_pSeq->getNumOfFrames()[1], m_NumOfMixtureComponents, false);
    
	m_subtractorA.set("backgroundRatio", m_BackgroundRatio);
	m_subtractorB.set("backgroundRatio", m_BackgroundRatio);
    
    vector<DepthFrame> depthFrames = m_pSeq->nextDepthFrame();
    
    cv::Mat depthMap, depthMask;
    depthFrames[0].getDepthMap().convertTo(depthMap, type);
    m_subtractorA(depthMap, depthMask, 1);
    depthFrames[1].getDepthMap().convertTo(depthMap, type);
    m_subtractorB(depthMap, depthMask, 1);
    
	while (m_pSeq->hasNextDepthFrame())
	{
        depthFrames[0].getDepthMap().convertTo(depthMap, type);
        m_subtractorA(depthMap, depthMask, m_LearningRate);
        depthFrames[1].getDepthMap().convertTo(depthMap, type);
        m_subtractorB(depthMap, depthMask, m_LearningRate);
        
		depthFrames = m_pSeq->nextDepthFrame();
	}
}

void BackgroundSubtractor::subtract(DepthFrame frame, DepthFrame& foreground)
{
	subtract(m_subtractorA, frame, foreground);
}

void BackgroundSubtractor::subtract(DepthFrame frameA, DepthFrame frameB, DepthFrame& foregroundA, DepthFrame& foregroundB)
{
	subtract(m_subtractorA, frameA, foregroundA);
	subtract(m_subtractorB, frameB, foregroundB);
}

void BackgroundSubtractor::subtract(cv::BackgroundSubtractorMOG2& subtractor, DepthFrame& frame, DepthFrame& foreground)
{
	cv::Mat maskMat;
	cv::Mat frameMatF;
	frame.getDepthMap().convertTo(frameMatF, type);
	subtractor(frameMatF, maskMat, 0); // learning rate = 0

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