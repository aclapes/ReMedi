#pragma once

#include "DepthFrame.h"
#include "Reader.h"
#include "Sequence.h"

#include <opencv2/opencv.hpp>

class BackgroundSubtractor
{
public:
    BackgroundSubtractor();
    BackgroundSubtractor(const BackgroundSubtractor& rhs);
	~BackgroundSubtractor(void);
    
    BackgroundSubtractor& operator=(const BackgroundSubtractor& rhs);
	void operator()(DepthFrame, float alpha = 0);
	void operator()(DepthFrame, DepthFrame, float alpha = 0);
    
    void setInputSequence(Sequence::Ptr pSeq);
    void setNumOfMixtureComponents(int k);
    void setLearningRate(float rate);
    void setBackgroundRatio(float ratio);

    void model();
    
	void subtract(DepthFrame, DepthFrame&);
	void subtract(DepthFrame, DepthFrame, DepthFrame&, DepthFrame&);
        
    typedef boost::shared_ptr<BackgroundSubtractor> Ptr;

private:
	void operator()(cv::BackgroundSubtractorMOG2&, DepthFrame&, float alpha = 0);
	void subtract(cv::BackgroundSubtractorMOG2&, DepthFrame&, DepthFrame&);

    Sequence::Ptr m_pSeq;
    cv::BackgroundSubtractorMOG2	m_subtractorA;
	cv::BackgroundSubtractorMOG2	m_subtractorB;
    int m_NumOfMixtureComponents;
    float m_LearningRate;
    float m_BackgroundRatio;
};

