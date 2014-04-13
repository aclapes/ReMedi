//
//  MotionSegmentator.cpp
//  remedi
//
//  Created by Albert Clapés on 04/03/14.
//
//

#include "MotionSegmentator.h"


MotionSegmentator::MotionSegmentator() : m_bUseColor(false), m_bUseDepth(true)
{

}

void MotionSegmentator::setInputFrames(ColorFrame cfA, ColorFrame cfB)
{
    m_ColorPrevFrameA = m_ColorFrameA;
    m_ColorPrevFrameB = m_ColorFrameB;
    
    m_ColorFrameA = cfA;
    m_ColorFrameB = cfB;
    
    if (!m_ColorPrevFrameA.isValid() || !m_ColorPrevFrameB.isValid())
        return;
    
    cv::Mat colorMotionA, colorMotionB;
    
    segment2DMotion(m_ColorFrameA, m_ColorPrevFrameA, colorMotionA);
    segment2DMotion(m_ColorFrameB, m_ColorPrevFrameB, colorMotionB);
    
    bufferMotionFrame(m_ColorMotionStreamBufferA, colorMotionA);
    bufferMotionFrame(m_ColorMotionStreamBufferB, colorMotionB);
}

void MotionSegmentator::setInputFrames(DepthFrame dfA, DepthFrame dfB)
{
    m_DepthPrevFrameA = m_DepthFrameA;
    m_DepthPrevFrameB = m_DepthFrameB;
    
    m_DepthFrameA = dfA;
    m_DepthFrameB = dfB;
    
    if (!m_DepthPrevFrameA.isValid() || !m_DepthPrevFrameB.isValid())
        return;
    
    cv::Mat depthMotionA, depthMotionB;
    
    segment2DMotion(m_DepthFrameA, m_DepthPrevFrameA, depthMotionA);
    segment2DMotion(m_DepthFrameB, m_DepthPrevFrameB, depthMotionB);
    
    bufferMotionFrame(m_DepthMotionStreamBufferA, depthMotionA);
    bufferMotionFrame(m_DepthMotionStreamBufferB, depthMotionB);
}

void MotionSegmentator::setMotionThreshold(float motionThreshold)
{
    m_MotionThreshold = motionThreshold;
}

void MotionSegmentator::setNegativeMotion(bool flag)
{
    m_bNegativeMotion = flag;
}

void MotionSegmentator::bufferMotionFrame(vector<cv::Mat>& buffer, cv::Mat mf)
{
	buffer.push_back(mf);
    
	if (buffer.size() > m_TmpCoherence)
	{
		buffer.erase(buffer.begin());
	}
}


void MotionSegmentator::setUseColor(bool flag)
{
    m_bUseColor = flag;
}


void MotionSegmentator::setUseDepth(bool flag)
{
    m_bUseDepth = flag;
}


void MotionSegmentator::segment(PointCloud& motionCloudA, PointCloud& motionCloudB)
{
    if (m_bUseColor &&
        (m_ColorMotionStreamBufferA.size() < 2 || m_DepthMotionStreamBufferB.size() < 2))
        return;
    
    if (m_bUseDepth &&
        (m_DepthMotionStreamBufferA.size() < 2 || m_DepthMotionStreamBufferB.size() < 2))
        return;
    
    cv::Mat maskA = cv::Mat::ones(m_DepthMotionStreamBufferA.back().rows,
                                  m_DepthMotionStreamBufferA.back().cols,
                                  m_DepthMotionStreamBufferA.back().type());
    cv::Mat maskB = cv::Mat::ones(m_DepthMotionStreamBufferB.back().rows,
                                  m_DepthMotionStreamBufferB.back().cols,
                                  m_DepthMotionStreamBufferB.back().type());
    
    if (m_bUseColor)
    {
        cv::bitwise_and(m_ColorMotionStreamBufferA.back(), maskA, maskA);
        cv::bitwise_and(m_ColorMotionStreamBufferB.back(), maskB, maskB);
    }
    
    if (m_bUseDepth)
    {
        cv::bitwise_and(m_DepthMotionStreamBufferA.back(), maskA, maskA);
        cv::bitwise_and(m_DepthMotionStreamBufferB.back(), maskB, maskB);
    }
    
    m_DepthFrameA.getForegroundPointCloud(maskA, motionCloudA);
    m_DepthFrameB.getForegroundPointCloud(maskB, motionCloudB);
}


void MotionSegmentator::segment2DMotion(ColorFrame frame, ColorFrame pframe, cv::Mat& motion)
{    
    cv::Mat colorDiff;
    cv::subtract(frame.getMat(), pframe.getMat(),
                 colorDiff, cv::noArray(), CV_32FC1);
    
    cv::threshold(cv::abs(colorDiff), motion, m_ColorThreshold, 255, cv::THRESH_BINARY);
    
    enclosure(motion, motion, 2);
}

void MotionSegmentator::segment2DMotion(DepthFrame frame, DepthFrame pframe, cv::Mat& motion)
{
    cv::Mat masksDiff = frame.getMask() - pframe.getMask();
    cv::threshold(masksDiff, masksDiff, 0, 255, cv::THRESH_BINARY);
    
    masksDiff.convertTo(masksDiff, CV_8UC1);

    masksDiff.convertTo(masksDiff, CV_8UC1);
    
    cv::Mat depthsDiff;
    cv::subtract(frame.getDepthMap(), pframe.getDepthMap(),
                 depthsDiff, masksDiff, CV_32FC1);
    
    cv::threshold(cv::abs(depthsDiff), motion, m_MotionThreshold, 255, cv::THRESH_BINARY);
    
    motion.convertTo(motion,CV_8UC1);
    
    enclosure(motion, motion, 2);
    
    cv::namedWindow("hola");
    cv::imshow("hola", motion);
    cv::waitKey(1);

}