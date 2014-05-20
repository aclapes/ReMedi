//
//  SupervisedObjectPicker.h
//  remedi
//
//  Created by Albert Clapés on 13/05/14.
//
//

#ifndef __remedi__SupervisedObjectPicker__
#define __remedi__SupervisedObjectPicker__

#include <iostream>

#include <opencv2/opencv.hpp>

#include "Sequence.h"
#include "ColorFrame.h"
#include "DetectionOutput.h"

class SupervisedObjectPicker
{
public:
    SupervisedObjectPicker(string objectsDir, int numOfObjects);
    
    void setSequence(Sequence::Ptr pSequence);
    
    void run();
    
private:
    // Mouse actions
    static void mouseCallback(int event, int x, int y, int flags, void* userdata);
    void modelHandler(int key);
    void nextPairedFrames(ColorFrame& color1, ColorFrame& color2,
                          DepthFrame& depth1, DepthFrame& depth2,
                          int step = 1);
    void prevPairedFrames(ColorFrame& color1, ColorFrame& color2,
                          DepthFrame& depth1, DepthFrame& depth2,
                          int step = 1);
    
    void mark(int wx, int wy);
    void mark(DetectionOutput dout);
    void remove(int wx, int wy);
    void draw(int wx, int wy);
    
    cv::Mat getConcatColor();
    cv::Mat getConcatDepth();
    
    int getResX();
    int getResY();
    
    Sequence::Ptr m_pSequence;
    
    string m_ObjectsDir;
    ColorFrame m_CurrentColorFrameA, m_CurrentColorFrameB;
    DepthFrame m_CurrentDepthFrameA, m_CurrentDepthFrameB;
    
    int m_NumOfViews;
    vector<int> m_NumOfFrames;
    int m_NumOfObjects;
    
    cv::Mat m_ConcatColor;
    cv::Mat m_ConcatDepth;
    int m_Delays;
    
    int m_ResX, m_ResY;
    
    int m_Object;
    vector<bool> m_PushedStates;
    vector<int>  m_PushedFrames;
    
    float m_Tol;
    int m_X, m_Y;
    
    vector< vector < vector<pcl::PointXYZ> > > m_ClickedPositions;
    vector< vector < vector< int > > > m_Presses;
    vector< vector< vector< vector<pcl::PointXYZ> > > > m_Positions;
    DetectionOutput m_DOutput;
};

#endif /* defined(__remedi__SupervisedObjectPicker__) */
