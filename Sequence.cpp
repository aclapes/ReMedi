//
//  Sequence.cpp
//  remedi
//
//  Created by Albert Clapés on 15/05/14.
//
//

#include "Sequence.h"

Sequence::Sequence()
: m_ColorFrameCounter(-1), m_DepthFrameCounter(-1)
{
    
}

Sequence::Sequence(vector<vector<ColorFrame> > colorStream, vector<vector<DepthFrame> > depthStream)
: m_ColorStream(colorStream), m_DepthStream(depthStream), m_ColorFrameCounter(-1), m_DepthFrameCounter(-1)
{
    
}

Sequence::Sequence(const Sequence& rhs)
{
    *this = rhs;
}

Sequence::~Sequence()
{
    
}

Sequence& Sequence::operator=(const Sequence& rhs)
{
    if (this != &rhs)
    {
        m_ColorStream = rhs.m_ColorStream;
        m_DepthStream = rhs.m_DepthStream;
        m_ColorFrameCounter = rhs.m_ColorFrameCounter;
        m_DepthFrameCounter = rhs.m_DepthFrameCounter;
        m_InteractionLabels = rhs.m_InteractionLabels;
        m_ActionLabels = rhs.m_ActionLabels;
    }
    
    return *this;
}

void Sequence::setColorStream(vector<vector<ColorFrame> > stream)
{
    m_ColorStream = stream;
}

void Sequence::setDepthStream(vector<vector<DepthFrame> > stream)
{
    m_DepthStream = stream;
}

void Sequence::addColorFrame(ColorFrame colorFrame, int view)
{
    m_ColorStream[view].push_back(colorFrame);
}

void Sequence::addColorFrame(vector<ColorFrame> colorFrame)
{
    for (int i = 0; i < m_ColorStream.size(); i++)
        m_ColorStream[i].push_back(colorFrame[i]);
}

void Sequence::addDepthFrame(DepthFrame depthFrame, int view)
{
    m_DepthStream[view].push_back(depthFrame);
}

void Sequence::addDepthFrame(vector<DepthFrame> depthFrame)
{
    for (int i = 0; i < m_DepthStream.size(); i++)
        m_DepthStream[i].push_back(depthFrame[i]);
}

bool Sequence::hasNextColorFrame(int step)
{
    for (int i = 0; i < m_ColorStream.size(); i++)
        if (m_ColorFrameCounter + m_Delay[i] + step >= m_ColorStream[i].size() - 1)
            return false;
    
    return true;
}

bool Sequence::hasNextDepthFrame(int step)
{
    for (int i = 0; i < m_DepthStream.size(); i++)
        if (m_DepthFrameCounter + m_Delay[i] + step >= m_DepthStream[i].size() - 1)
            return false;
    
    return true;
}

vector<ColorFrame> Sequence::nextColorFrame(int step)
{
     m_ColorFrameCounter += step;
    
    vector<ColorFrame> frame;
    for (int i = 0; i < m_ColorStream.size(); i++)
        frame.push_back(m_ColorStream[i][m_ColorFrameCounter + m_Delay[i]]);

    return frame;
}

vector<DepthFrame> Sequence::nextDepthFrame(int step)
{
    m_DepthFrameCounter += step;
    
    vector<DepthFrame> frame;
    for (int i = 0; i < m_DepthStream.size(); i++)
        frame.push_back(m_DepthStream[i][m_DepthFrameCounter + m_Delay[i]]);
    
    return frame;
}

bool Sequence::hasPreviousColorFrame(int step)
{
    for (int i = 0; i < m_ColorStream.size(); i++)
        if (m_ColorFrameCounter + m_Delay[i] - step < 0)
            return false;
    
    return true;
}

bool Sequence::hasPreviousDepthFrame(int step)
{
    for (int i = 0; i < m_DepthStream.size(); i++)
        if (m_DepthFrameCounter + m_Delay[i] - step < 0)
            return false;
    
    return true;
}

vector<ColorFrame> Sequence::previousColorFrame(int step)
{
    m_ColorFrameCounter -= step;
    
    vector<ColorFrame> frame;
    for (int i = 0; i < m_ColorStream.size(); i++)
        frame.push_back(m_ColorStream[i][m_ColorFrameCounter + m_Delay[i]]);
    
    return frame;
}

vector<DepthFrame> Sequence::previousDepthFrame(int step)
{
    m_DepthFrameCounter -= step;
    
    vector<DepthFrame> frame;
    for (int i = 0; i < m_DepthStream.size(); i++)
        frame.push_back(m_DepthStream[i][m_DepthFrameCounter + m_Delay[i]]);
    
    return frame;
}

void sequence::setInteractionLabels(vector<unsigned char> labels)
{
    m_InteractionLabels = labels;
}

void sequence::setActionLabels(vector<unsigned char> labels)
{
    m_ActionLabels = labels;
}

int Sequence::getNumOfFrames()
{
    return m_DepthStream[0].size() - m_Delay[0];
}

void Sequence::setDelay(vector<int> delay)
{
    m_Delay = delay;
}