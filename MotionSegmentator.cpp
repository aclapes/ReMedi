//
//  MotionSegmentator.cpp
//  remedi
//
//  Created by Albert Clapés on 04/03/14.
//
//

#include "MotionSegmentator.h"





void MotionSegmentator::setInputFrames(ColorFrame cfA, ColorFrame cfB)
{
    bufferFrame(m_ColorStreamBufferA, cfA);
    bufferFrame(m_ColorStreamBufferB, cfB);
}

void MotionSegmentator::setInputFrames(DepthFrame dfA, DepthFrame dfB)
{
    bufferFrame(m_DepthStreamBufferA, dfA);
    bufferFrame(m_DepthStreamBufferB, dfB);
}


void MotionSegmentator::bufferFrame(vector<ColorFrame>& buffer, ColorFrame f)
{
	buffer.push_back(f);
    
	if (buffer.size() > m_TmpCoherence)
	{
		buffer.erase(buffer.begin());
	}
}

void MotionSegmentator::bufferFrame(vector<DepthFrame>& buffer, DepthFrame f)
{
	buffer.push_back(f);
    
	if (buffer.size() > m_TmpCoherence)
	{
		buffer.erase(buffer.begin());
	}
}


void MotionSegmentator::segment(PointCloud& motionCloudA, PointCloud& motionCloudB)
{
    cv::Mat motionA, motionB;
    
	segment2DMotion(m_DepthStreamBufferA, motionA);
    segment2DMotion(m_DepthStreamBufferB, motionB);
    
    m_DepthStreamBufferA.back().getForegroundPointCloud(motionA, motionCloudA);
    m_DepthStreamBufferB.back().getForegroundPointCloud(motionB, motionCloudB);
}


void MotionSegmentator::segment2DMotion(vector<DepthFrame> buffer, cv::Mat& motion)
{
    DepthFrame currDepthFrame = buffer[buffer.size() - 1];
    DepthFrame prevDepthFrame = buffer[buffer.size() - 2];
    
    cv::Mat masksDiff = currDepthFrame.getMask() - prevDepthFrame.getMask();
    cv::threshold(masksDiff, masksDiff, 0, 255, cv::THRESH_BINARY);
    
    masksDiff.convertTo(masksDiff, CV_8UC1);
    
    cv::Mat depthsDiff;
    cv::subtract(currDepthFrame.getDepthMap(), prevDepthFrame.getDepthMap(),
                 depthsDiff, masksDiff, CV_32FC1);
    
    cv::threshold(cv::abs(depthsDiff), motion, m_MotionThreshold, 255, cv::THRESH_BINARY);
    
    enclosure(motion, motion, 2);
}