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
    Sequence(vector<vector<ColorFrame> > colorStream, vector<vector<DepthFrame> > depthStream);
    Sequence(const Sequence& rhs);
    ~Sequence();
    Sequence& operator=(const Sequence& rhs);
    
    void setName(string name);
    string getName();
    
    void addColorFrame(ColorFrame colorFrame, int view);
    void addColorFrame(vector<ColorFrame> colorFrame);
    void addDepthFrame(DepthFrame depthFrame, int view);
    void addDepthFrame(vector<DepthFrame> colorFrame);
    
    void addColorStreamView(vector<ColorFrame> stream);
    void addDepthStreamView(vector<DepthFrame> stream);
    
    void setColorStreamView(vector<ColorFrame> stream, int view);
    void setDepthStreamView(vector<DepthFrame> stream, int view);

    void setColorStream(vector<vector<ColorFrame> > stream);
    void setDepthStream(vector<vector<DepthFrame> > stream);
    
    void setInteractionLabels(vector<unsigned char> labels);
    void setActionLabels(vector<unsigned char> labels);
    
    bool hasNextColorFrame(int step = 1);
    bool hasNextDepthFrame(int step = 1);
    vector<ColorFrame> nextColorFrame(int step = 1); // multi-view
    vector<DepthFrame> nextDepthFrame(int step = 1);
    
    bool hasPreviousColorFrame(int step = 1);
    bool hasPreviousDepthFrame(int step = 1);
    vector<ColorFrame> previousColorFrame(int step = 1); // multi-view
    vector<DepthFrame> previousDepthFrame(int step = 1);
    
    int getNumOfFrames();
    int colorAt();
    int depthAt();
    
    void setDelay(vector<int> delay);
    
    typedef boost::shared_ptr<Sequence> Ptr;
    
private:
    string m_Name;
    
    vector< vector<ColorFrame> > m_ColorStream;
    vector< vector<DepthFrame> > m_DepthStream;
    int m_ColorFrameCounter;
    int m_DepthFrameCounter;
    
    vector<unsigned char> m_InteractionLabels;
    vector<unsigned char> m_ActionLabels;
    
    vector<int> m_Delay;
};

#endif /* defined(__remedi__Sequence__) */
