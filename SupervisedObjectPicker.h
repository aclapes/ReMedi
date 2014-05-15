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

#include "Reader.h"
#include "ColorFrame.h"
#include "DetectionOutput.h"

class SupervisedObjectPicker
{
public:
    SupervisedObjectPicker(string parentDir, int sid, int numOfViews, int numOfObjects);
    
    void run();
    
private:
    // Mouse actions
    static void mouseCallback(int event, int x, int y, int flags, void* userdata);
    void keyboardHandler(int key);
    
    void mark(int wx, int wy);
    void mark(DetectionOutput dout);
    void draw(int wx, int wy);
    
    cv::Mat getConcatColor();
    cv::Mat getConcatDepth();
    
    int getResX();
    int getResY();
    
    Reader m_Reader;
    int m_sid;
    
    string m_ParentDir;
    ColorFrame m_CurrentColorFrameA, m_CurrentColorFrameB;
    DepthFrame m_CurrentDepthFrameA, m_CurrentDepthFrameB;
    
    int m_NumOfViews;
    int m_NumOfFrames;
    int m_NumOfObjects;
    
    cv::Mat m_ConcatColor;
    cv::Mat m_ConcatDepth;
    
    int m_ResX, m_ResY;
    
    int m_Object;
    vector<bool> m_PushedStates;
    vector<int>  m_PushedFrames;
    
    vector< vector< vector< vector<cv::Point> > > > m_Positions;
    vector< vector < vector< cv::Point > > > m_ClickedPositions;
    vector< vector < vector< int > > > m_Presses;
    
    int m_X, m_Y;
    
    DetectionOutput m_DOutput;
};

#endif /* defined(__remedi__SupervisedObjectPicker__) */
