//
//  Sequence.cpp
//  remedi
//
//  Created by Albert Clapés on 15/05/14.
//
//

#include "Sequence.h"

Sequence::Sequence()
{
}

Sequence::Sequence(vector<vector<ColorFrame> > colorStream, vector<vector<DepthFrame> > depthStream)
: m_ColorStream(colorStream), m_DepthStream(depthStream)
{
    assert(m_ColorStream.size() == m_DepthStream.size());
    
    m_ColorFrameCounter.resize(m_ColorStream.size(), -1);
    m_DepthFrameCounter.resize(m_DepthStream.size(), -1);
    m_Delays.resize(m_ColorStream.size(), 0);
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
        m_Name = rhs.m_Name;
        m_ColorStream = rhs.m_ColorStream;
        m_DepthStream = rhs.m_DepthStream;
        m_ColorFrameCounter = rhs.m_ColorFrameCounter;
        m_DepthFrameCounter = rhs.m_DepthFrameCounter;
        m_InteractionLabels = rhs.m_InteractionLabels;
        m_ActionLabels = rhs.m_ActionLabels;
        m_Delays = rhs.m_Delays;
    }
    
    return *this;
}

string Sequence::getName()
{
    return m_Name;
}

void Sequence::setName(string name)
{
    m_Name = name;
}

void Sequence::setColorStream(vector<vector<ColorFrame> > stream)
{
    m_ColorStream = stream;
    
    m_ColorFrameCounter.resize(m_ColorStream.size(), -1);
    m_Delays.resize(m_ColorStream.size(), 0);
}

void Sequence::setDepthStream(vector<vector<DepthFrame> > stream)
{
    m_DepthStream = stream;
    
    m_DepthFrameCounter.resize(m_DepthStream.size(), -1);
    m_Delays.resize(m_DepthStream.size(), 0);
}

void Sequence::addColorStreamView(vector<ColorFrame> stream)
{
    m_ColorStream.push_back(stream);
    
    m_ColorFrameCounter.resize((m_ColorStream.size() >= m_DepthStream.size()) ? m_ColorStream.size() : m_DepthStream.size(), -1);
    m_Delays.resize((m_ColorStream.size() >= m_DepthStream.size() ) ? m_ColorStream.size() : m_DepthStream.size(), 0);
}

void Sequence::addDepthStreamView(vector<DepthFrame> stream)
{
    m_DepthStream.push_back(stream);
    
    m_DepthFrameCounter.resize((m_ColorStream.size() >= m_DepthStream.size()) ? m_ColorStream.size() : m_DepthStream.size(), -1);
    m_Delays.resize((m_ColorStream.size() >= m_DepthStream.size() ) ? m_ColorStream.size() : m_DepthStream.size(), 0);
}

void Sequence::setColorStreamView(vector<ColorFrame> stream, int view)
{
    m_ColorStream[view] = stream;
}

void Sequence::setDepthStreamView(vector<DepthFrame> stream, int view)
{
    m_DepthStream[view] = stream;
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
        if (m_ColorFrameCounter[i] + m_Delays[i] + step >= m_ColorStream[i].size() - 1)
            return false;
    
    return true;
}

bool Sequence::hasNextDepthFrame(int step)
{
    for (int i = 0; i < m_DepthStream.size(); i++)
        if (m_DepthFrameCounter[i] + m_Delays[i] + step >= m_DepthStream[i].size() - 1)
            return false;
    
    return true;
}

vector<ColorFrame> Sequence::nextColorFrame(int step)
{
    for (int i = 0; i < m_ColorStream.size(); i++)
        m_ColorFrameCounter[i] += step;
    
    vector<ColorFrame> frame;
    for (int i = 0; i < m_ColorStream.size(); i++)
        frame.push_back(m_ColorStream[i][m_ColorFrameCounter[i] + m_Delays[i]]);

    return frame;
}

vector<DepthFrame> Sequence::nextDepthFrame(int step)
{
    for (int i = 0; i < m_DepthStream.size(); i++)
        m_DepthFrameCounter[i] += step;
    
    vector<DepthFrame> frame;
    for (int i = 0; i < m_DepthStream.size(); i++)
        frame.push_back(m_DepthStream[i][m_DepthFrameCounter[i] + m_Delays[i]]);
    
    return frame;
}

bool Sequence::hasPreviousColorFrame(int step)
{
    for (int i = 0; i < m_ColorStream.size(); i++)
        if (m_ColorFrameCounter[i] + m_Delays[i] - step < 0)
            return false;
    
    return true;
}

bool Sequence::hasPreviousDepthFrame(int step)
{
    for (int i = 0; i < m_DepthStream.size(); i++)
        if (m_DepthFrameCounter[i] + m_Delays[i] - step < 0)
            return false;
    
    return true;
}

vector<ColorFrame> Sequence::previousColorFrame(int step)
{
    for (int i = 0; i < m_ColorStream.size(); i++)
        m_ColorFrameCounter[i] -= step;
    
    vector<ColorFrame> frame;
    for (int i = 0; i < m_ColorStream.size(); i++)
        frame.push_back(m_ColorStream[i][m_ColorFrameCounter[i] + m_Delays[i]]);
    
    return frame;
}

vector<DepthFrame> Sequence::previousDepthFrame(int step)
{
    for (int i = 0; i < m_DepthStream.size(); i++)
        m_DepthFrameCounter[i] -= step;
    
    vector<DepthFrame> frame;
    for (int i = 0; i < m_DepthStream.size(); i++)
        frame.push_back(m_DepthStream[i][m_DepthFrameCounter[i] + m_Delays[i]]);
    
    return frame;
}

void Sequence::setInteractionLabels(vector<unsigned char> labels)
{
    m_InteractionLabels = labels;
}

void Sequence::setActionLabels(vector<unsigned char> labels)
{
    m_ActionLabels = labels;
}

vector<ColorFrame> Sequence::getColorFrame(int i)
{
    vector<ColorFrame> frame;
    for (int v = 0; v < m_ColorStream.size(); v++)
        frame.push_back(m_ColorStream[v][i]);
    
    return frame;
}

vector<DepthFrame> Sequence::getDepthFrame(int i)
{
    vector<DepthFrame> frame;
    for (int v = 0; v < m_DepthStream.size(); v++)
        frame.push_back(m_DepthStream[v][i]);
    
    return frame;
}

int Sequence::getNumOfViews()
{
    return m_ColorStream.size();
}

vector<int> Sequence::getNumOfFrames()
{
    vector<int> numOfFrames(m_ColorStream.size());
    for (int i = 0; i < m_ColorStream.size(); i++)
        numOfFrames[i] = m_ColorStream[i].size();
    
    return numOfFrames;
}

vector<int> Sequence::colorAt()
{
    vector<int> colorFrameDelayedCounter(m_ColorStream.size());
    
    for (int i = 0; i < m_ColorStream.size(); i++)
        colorFrameDelayedCounter[i] = m_ColorFrameCounter[i] + m_Delays[i];
    
    return colorFrameDelayedCounter;
}

vector<int> Sequence::depthAt()
{
    vector<int> depthFrameDelayedCounter(m_DepthStream.size());
    for (int i = 0; i < m_DepthStream.size(); i++)
        depthFrameDelayedCounter[i] = m_DepthFrameCounter[i] + m_Delays[i];
    
    return depthFrameDelayedCounter;
}

void Sequence::setDelays(vector<int> delays)
{
    m_Delays = delays;
}