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
    
    void setViewsDisplay(int h, int w);
    void setSequence(Sequence::Ptr pSequence);

    void run();
    
private:
    
    // Methods
    
    static void mouseCallback(int event, int x, int y, int flags, void* userdata);
    void modelHandler(int key);
    
    void nextFrames(vector<cv::Mat>& colorFrames,
                    vector<cv::Mat>& depthFrames,
                    int step = 1);
    void prevFrames(vector<cv::Mat>& colorFrames,
                    vector<cv::Mat>& depthFrames,
                    int step = 1);
    
    void mark(int wx, int wy);
    void mark(DetectionOutput dout);
    void remove(int wx, int wy);
    void draw(int wx, int wy);
    
    int getResX();
    int getResY();
    
    void concatenateViews(vector<cv::Mat> views, int h, int w, cv::Mat& multiview);
    
    //
    // Atributes
    //
    
    string m_ObjectsDir;
    int m_NumOfObjects;
    
    Sequence::Ptr m_pSequence;
    int m_ResX, m_ResY;
    int m_NumOfViews;
    vector<int> m_NumOfFrames;
    vector<cv::Mat> m_CurrentColorFrames;
    vector<cv::Mat> m_CurrentDepthFrames;
    int m_Delays;
    
    // graphical interface
    int m_H, m_W;
    cv::Mat m_ColorViewsFrame;// m_ConcatColor;
    cv::Mat m_DepthViewsFrame;//m_ConcatDepth;

    int m_X, m_Y; // current mouse position
    int m_Object; // selected object to label
    float m_Tol;  // tolerance distance between mouse and already placed mark
    
    vector< vector < vector<pcl::PointXYZ> > > m_ClickedPositions;
    vector< vector < vector< int > > > m_Presses;
    vector< vector< vector< vector<pcl::PointXYZ> > > > m_Positions;
    
    // annotation
    
    DetectionOutput m_DOutput;
};

#endif /* defined(__remedi__SupervisedObjectPicker__) */
