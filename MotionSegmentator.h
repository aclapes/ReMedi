//
//  MotionSegmentator.h
//  remedi
//
//  Created by Albert Clapés on 04/03/14.
//
//

#ifndef __remedi__MotionSegmentator__
#define __remedi__MotionSegmentator__

#include <iostream>

#include "Frame.h"
#include "ColorFrame.h"
#include "DepthFrame.h"

using namespace std;

class MotionSegmentator
{
    typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;
    
public:
    MotionSegmentator();
    
    void setTemporalCoherence(int nframes);
    
    void setInputFrames(ColorFrame cfA, ColorFrame cfB);
    void setInputFrames(DepthFrame dfA, DepthFrame dfB);
    
    void setMotionThreshold(float motionThreshold);
    
    void setNegativeMotion(bool flag);
    
    void setUseColor(bool flag);
    void setUseDepth(bool flag);
    
    void segment(PointCloud& motionCloudA, PointCloud& motionCloudB);
    
private:
    void bufferMotionFrame(vector<cv::Mat>& buffer, cv::Mat mf);
    
    void segment2DMotion(ColorFrame pframe, ColorFrame frame, cv::Mat&);
    void segment2DMotion(DepthFrame pframe, DepthFrame frame, cv::Mat&);
    
    //
    // Members
    //
    
    // Current and previous color and depth frames
    ColorFrame m_ColorFrameA, m_ColorFrameB, m_ColorPrevFrameA, m_ColorPrevFrameB;
    DepthFrame m_DepthFrameA, m_DepthFrameB, m_DepthPrevFrameA, m_DepthPrevFrameB;
    
    // Buffers of the k previous motion frame (from color and depth frames)
    vector<cv::Mat> m_ColorMotionStreamBufferA, m_ColorMotionStreamBufferB;
    vector<cv::Mat> m_DepthMotionStreamBufferA, m_DepthMotionStreamBufferB;
    
    bool m_bNegativeMotion;
    bool m_bUseDepth, m_bUseColor;
    
    int m_TmpCoherence;
    float m_MotionThreshold;
    float m_ColorThreshold;
};

#endif /* defined(__remedi__MotionSegmentator__) */
