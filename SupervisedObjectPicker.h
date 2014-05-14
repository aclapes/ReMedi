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

class SupervisedObjectPicker
{
public:
    SupervisedObjectPicker(string parentDir, int sid, int numOfViews, int numOfObjects);
    
    void initializeAnnotations(int numOfObjects, int numOfFrames);
    
    // Mouse actions
    void mark(int x, int y);
    void draw(int x, int y);
    
    void read();
    void write();
    
    cv::Mat getConcatMat();
    int getResX();
    int getResY();
    void run();
    
private:
    static void mouseCallback(int event, int x, int y, int flags, void* userdata);
    void keyboardHandler(int key);
    
    Reader m_Reader;
    int m_sid;
    
    string m_ParentDir;
    ColorFrame m_CurrentColorFrameA, m_CurrentColorFrameB;
    
    int m_NumOfViews;
    int m_NumOfObjects;
    cv::Mat m_ConcatMat;
    
    int m_ResX, m_ResY;
    
    int m_Object;
    vector<bool> m_PushedStates;
    vector<int>  m_PushedFrames;
//    vector<cv::Mat> m_Annotations;
    
    vector< vector< vector< vector<cv::Point> > > > m_Positions;
    
    // vector view, vector object, vector point
    vector< vector < vector<cv::Point> > > m_PositionsTmp;
    vector< vector < vector<int> > > m_Presses;
    
    int m_X, m_Y;
};

#endif /* defined(__remedi__SupervisedObjectPicker__) */
