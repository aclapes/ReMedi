//
//  Sequence.h
//  remedi
//
//  Created by Albert Clapés on 15/05/14.
//
//

#ifndef __remedi__Sequence__
#define __remedi__Sequence__

#include "ColorFrame.h"
#include "DepthFrame.h"

#include <iostream>
#include <vector>

using namespace std;

class Sequence
{
public:
    Sequence();
    Sequence(int numOfViews);
    Sequence(vector<vector<string> > colorFramesPaths, vector<vector<string> > depthFramesPaths);
    Sequence(vector<vector<ColorFrame> > colorStream, vector<vector<DepthFrame> > depthStream);
    Sequence(const Sequence& rhs);
    ~Sequence();
    Sequence& operator=(const Sequence& rhs);
    
    void setName(string name);
    string getName();
    
    void addColorFramePath(string colorFramePath, int view);
    void addColorFramePath(vector<string> colorFramePaths);
    void addDepthFramePath(string depthFramePath, int view);
    void addDepthFramePath(vector<string> depthFramesPaths);
    
    void addColorFrame(ColorFrame colorFrame, int view);
    void addColorFrame(vector<ColorFrame> colorFrames);
    void addDepthFrame(DepthFrame depthFrame, int view);
    void addDepthFrame(vector<DepthFrame> depthFrames);
    
    void addColorStream(vector<ColorFrame> stream);
    void addDepthStream(vector<DepthFrame> stream);
    
    void setColorStream(vector<ColorFrame> stream, int view);
    void setDepthStream(vector<DepthFrame> stream, int view);
    
    void setColorStreams(vector<vector<ColorFrame> > streams);
    void setDepthStreams(vector<vector<DepthFrame> > streams);

    void setColorFramesPaths(vector<vector<string> > paths);
    void setDepthFramesPaths(vector<vector<string> > paths);

    void setInteractionLabels(vector<unsigned short> labels);
    void setActionLabels(vector<unsigned short> labels);
    
    bool hasNextColorFrame(int step = 1);
    bool hasNextDepthFrame(int step = 1);
    bool hasPreviousColorFrame(int step = 1);
    bool hasPreviousDepthFrame(int step = 1);
    
    vector<ColorFrame> nextColorFrame(int step = 1); // multi-view
    vector<DepthFrame> nextDepthFrame(int step = 1);
    vector<ColorFrame> previousColorFrame(int step = 1); // multi-view
    vector<DepthFrame> previousDepthFrame(int step = 1);
    vector<ColorFrame> getColorFrame(int i); // multi-view
    vector<DepthFrame> getDepthFrame(int i);
    
    void nextColorFrame(vector<cv::Mat>& colorMat, int step = 1); // multi-view
    void nextDepthFrame(vector<cv::Mat>& depthMat, int step = 1);
    void previousColorFrame(vector<cv::Mat>& colorMat, int step = 1); // multi-view
    void previousDepthFrame(vector<cv::Mat>& depthMat, int step = 1);
    void getColorFrame(vector<cv::Mat>& colorMat, int i); // multi-view
    void getDepthFrame(vector<cv::Mat>& depthMat, int i);
    
    vector<int> getCurrentColorFramesID();
    vector<int> getCurrentDepthFramesID();
    vector<float> getColorProgress();
    vector<float> getDepthProgress();
    int getNumOfViews();
    vector<int> getNumOfFrames();
    vector<int> colorAt();
    vector<int> depthAt();
    
    void setDelays(vector<int> delays);
    
    typedef boost::shared_ptr<Sequence> Ptr;
    
private:
    void readColorFrame(vector<string> paths, int i, ColorFrame& frame);
    void readDepthFrame(vector<string> paths, int i, DepthFrame& frame);
    
    void readMat(vector<string> paths, int i, cv::Mat& mat);
    
    string m_Name;
    
    vector< vector<string> > m_ColorPaths;
    vector< vector<string> > m_DepthPaths;
    
    vector< vector<ColorFrame> > m_ColorStreams;
    vector< vector<DepthFrame> > m_DepthStreams;
    
    vector<int> m_ColorFrameCounter;
    vector<int> m_DepthFrameCounter;
    
    vector<unsigned short> m_InteractionLabels;
    vector<unsigned short> m_ActionLabels;
    
    vector<int> m_Delays;
};

#endif /* defined(__remedi__Sequence__) */
