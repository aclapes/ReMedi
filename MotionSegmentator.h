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
    
    void setTemporalCoherence(int nframes);
    
    void setInputFrames(ColorFrame cfA, ColorFrame cfB);
    void setInputFrames(DepthFrame dfA, DepthFrame dfB);
    
    void segment(PointCloud& motionCloudA, PointCloud& motionCloudB);
    
private:
    void bufferFrame(vector<ColorFrame>& buffer, ColorFrame f);
    void bufferFrame(vector<DepthFrame>& buffer, DepthFrame f);
    
    void segment2DMotion(vector<ColorFrame>, cv::Mat&);
    void segment2DMotion(vector<DepthFrame>, cv::Mat&);
    
    //
    // Members
    //
    
    vector<ColorFrame> m_ColorStreamBufferA, m_ColorStreamBufferB;
    vector<DepthFrame> m_DepthStreamBufferA, m_DepthStreamBufferB;
    
    int m_TmpCoherence;
    float m_MotionThreshold;
};

#endif /* defined(__remedi__MotionSegmentator__) */
