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

Sequence::Sequence(int numOfViews)
{
    m_Delays.resize(numOfViews, 0);
}


Sequence::Sequence(vector<vector<string> > colorFramesPaths, vector<vector<string> > depthFramesPaths)
: m_ColorPaths(colorFramesPaths), m_DepthPaths(depthFramesPaths)
{
    assert(m_ColorPaths.size() == m_DepthPaths.size());
    
    m_ColorFrameCounter.resize(m_ColorPaths.size(), -1);
    m_DepthFrameCounter.resize(m_DepthPaths.size(), -1);
    m_Delays.resize(m_ColorPaths.size(), 0);
}

Sequence::Sequence(vector<vector<ColorFrame> > colorStream, vector<vector<DepthFrame> > depthStream)
: m_ColorStreams(colorStream), m_DepthStreams(depthStream)
{
    assert(m_ColorStreams.size() == m_DepthStreams.size());
    
    m_ColorFrameCounter.resize(m_ColorStreams.size(), -1);
    m_DepthFrameCounter.resize(m_DepthStreams.size(), -1);
    m_Delays.resize(m_ColorStreams.size(), 0);
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
        m_ColorPaths = rhs.m_ColorPaths;
        m_DepthPaths = rhs.m_DepthPaths;
        m_ColorStreams = rhs.m_ColorStreams;
        m_DepthStreams = rhs.m_DepthStreams;
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

void Sequence::setColorFramesPaths(vector<vector<string> > paths)
{
    m_ColorPaths = paths;
    
    m_ColorFrameCounter.resize(m_ColorPaths.size(), -1);
    m_Delays.resize(m_ColorPaths.size(), 0);
}

void Sequence::setDepthFramesPaths(vector<vector<string> > paths)
{
    m_DepthPaths = paths;
    
    m_DepthFrameCounter.resize(m_DepthPaths.size(), -1);
    m_Delays.resize(m_DepthPaths.size(), 0);
}

void Sequence::setColorStreams(vector<vector<ColorFrame> > streams)
{
    m_ColorStreams = streams;
    
    m_ColorFrameCounter.resize(m_ColorStreams.size(), -1);
    m_Delays.resize(m_ColorStreams.size(), 0);
}

void Sequence::setDepthStreams(vector<vector<DepthFrame> > streams)
{
    m_DepthStreams = streams;
    
    m_DepthFrameCounter.resize(m_DepthStreams.size(), -1);
    m_Delays.resize(m_DepthStreams.size(), 0);
}

void Sequence::addColorStream(vector<ColorFrame> stream)
{
    m_ColorStreams.push_back(stream);
    
    m_ColorFrameCounter.push_back(-1);
    m_Delays.resize((m_ColorStreams.size() >= m_DepthStreams.size() ) ? m_ColorStreams.size() : m_DepthStreams.size(), 0);
}

void Sequence::addDepthStream(vector<DepthFrame> stream)
{
    m_DepthStreams.push_back(stream);
    
    m_DepthFrameCounter.push_back(-1);
    m_Delays.resize((m_ColorStreams.size() >= m_DepthStreams.size() ) ? m_ColorStreams.size() : m_DepthStreams.size(), 0);
}

void Sequence::setColorStream(vector<ColorFrame> stream, int view)
{
    while (view >= m_ColorStreams.size())
        m_ColorStreams.push_back(vector<ColorFrame>());
    
    m_ColorStreams[view] = stream;
}

void Sequence::setDepthStream(vector<DepthFrame> stream, int view)
{
    while (view >= m_DepthStreams.size())
        m_DepthStreams.push_back(vector<DepthFrame>());
    
    m_DepthStreams[view] = stream;
}

void Sequence::addColorFramePath(string colorFramePath, int view)
{
    while (view >= m_ColorPaths.size())
    {
        m_ColorPaths.push_back(vector<string>());
        m_ColorStreams.push_back(vector<ColorFrame>());
        m_ColorFrameCounter.push_back(-1);
        if (m_ColorPaths.size() > m_DepthPaths.size())
            m_Delays.push_back(0);
    }
    
    m_ColorPaths[view].push_back(colorFramePath);
}

void Sequence::addColorFramePath(vector<string> colorFramePaths)
{
    for (int i = 0; i < m_ColorPaths.size(); i++)
        m_ColorPaths[i].push_back(colorFramePaths[i]);
}

void Sequence::addDepthFramePath(string depthFramePath, int view)
{
    while (view >= m_DepthPaths.size())
    {
        m_DepthPaths.push_back(vector<string>());
        m_DepthStreams.push_back(vector<DepthFrame>());
        m_DepthFrameCounter.push_back(-1);
        if (m_DepthPaths.size() > m_ColorPaths.size())
            m_Delays.push_back(0);
    }
    
    m_DepthPaths[view].push_back(depthFramePath);
}

void Sequence::addDepthFramePath(vector<string> depthFramesPaths)
{
    for (int i = 0; i < m_DepthPaths.size(); i++)
        m_DepthPaths[i].push_back(depthFramesPaths[i]);
}

void Sequence::addColorFrame(ColorFrame colorFrame, int view)
{
    m_ColorStreams[view].push_back(colorFrame);
}

void Sequence::addColorFrame(vector<ColorFrame> colorFrame)
{
    for (int i = 0; i < m_ColorStreams.size(); i++)
        m_ColorStreams[i].push_back(colorFrame[i]);
}

void Sequence::addDepthFrame(DepthFrame depthFrame, int view)
{
    m_DepthStreams[view].push_back(depthFrame);
}

void Sequence::addDepthFrame(vector<DepthFrame> depthFrame)
{
    for (int i = 0; i < m_DepthStreams.size(); i++)
        m_DepthStreams[i].push_back(depthFrame[i]);
}

bool Sequence::hasNextColorFrame(int step)
{
    for (int i = 0; i < m_ColorPaths.size(); i++)
        if (m_ColorFrameCounter[i] + m_Delays[i] + step > m_ColorPaths[i].size() - 1)
            return false;
    
    return true;
}

bool Sequence::hasNextDepthFrame(int step)
{
    for (int i = 0; i < m_DepthPaths.size(); i++)
        if (m_DepthFrameCounter[i] + m_Delays[i] + step > m_DepthPaths[i].size() - 1)
            return false;
    
    return true;
}

bool Sequence::hasPreviousColorFrame(int step)
{
    for (int i = 0; i < m_ColorPaths.size(); i++)
        if (m_ColorFrameCounter[i] + m_Delays[i] - step < 0)
            return false;
    
    return true;
}

bool Sequence::hasPreviousDepthFrame(int step)
{
    for (int i = 0; i < m_DepthPaths.size(); i++)
        if (m_DepthFrameCounter[i] + m_Delays[i] - step < 0)
            return false;
    
    return true;
}

void Sequence::readColorFrame(vector<string> paths, int f, ColorFrame& frame)
{
    assert (f < paths.size());
    
	frame = ColorFrame( cv::imread(paths[f].c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
}

void Sequence::readDepthFrame(vector<string> paths, int f, DepthFrame& frame)
{
    assert (f < paths.size());
    
	frame = DepthFrame( cv::imread(paths[f].c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
}

void Sequence::readMat(vector<string> paths, int f, cv::Mat& frame)
{
    assert (f < paths.size());
    
	frame = cv::imread(paths[f].c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
}

vector<ColorFrame> Sequence::nextColorFrame(int step)
{
    for (int v = 0; v < m_ColorPaths.size(); v++)
        m_ColorFrameCounter[v] += step;
    
    vector<ColorFrame> frames;
    for (int v = 0; v < m_ColorPaths.size(); v++)
    {
        ColorFrame frame;
        int delay = (m_Delays.size()) > 0 ? m_Delays[v] : 0;
        if (m_ColorStreams[v].size() > 0)
            frame = m_ColorStreams[v][m_ColorFrameCounter[v] + delay]; // preallocation
        else
            readColorFrame(m_ColorPaths[v], m_ColorFrameCounter[v] + delay, frame);
        
        frames.push_back(frame);
    }

    return frames;
}

vector<DepthFrame> Sequence::nextDepthFrame(int step)
{
    for (int v = 0; v < m_DepthPaths.size(); v++)
        m_DepthFrameCounter[v] += step;
    
    vector<DepthFrame> frames;
    for (int v = 0; v < m_DepthPaths.size(); v++)
    {
        DepthFrame frame;
        int delay = (m_Delays.size()) > 0 ? m_Delays[v] : 0;
        if (m_DepthStreams[v].size() > 0)
            frame = m_DepthStreams[v][m_DepthFrameCounter[v] + delay]; // preallocation
        else
            readDepthFrame(m_DepthPaths[v], m_DepthFrameCounter[v] + delay, frame);
        
        frames.push_back(frame);
    }
    
    return frames;
}

vector<ColorFrame> Sequence::previousColorFrame(int step)
{
    for (int v = 0; v < m_ColorPaths.size(); v++)
        m_ColorFrameCounter[v] -= step;
    
    vector<ColorFrame> frames;
    for (int v = 0; v < m_ColorPaths.size(); v++)
    {
        ColorFrame frame;
        if (m_ColorStreams[v].size() > 0)
            frame = m_ColorStreams[v][m_ColorFrameCounter[v] + m_Delays[v]]; // preallocation
        else
            readColorFrame(m_ColorPaths[v], m_ColorFrameCounter[v] + m_Delays[v], frame);
        
        frames.push_back(frame);
    }
    
    return frames;
}

vector<DepthFrame> Sequence::previousDepthFrame(int step)
{
    for (int v = 0; v < m_DepthPaths.size(); v++)
        m_DepthFrameCounter[v] -= step;
    
    vector<DepthFrame> frames;
    for (int v = 0; v < m_DepthPaths.size(); v++)
    {
        DepthFrame frame;
        if (m_DepthStreams[v].size() > 0)
            frame = m_DepthStreams[v][m_DepthFrameCounter[v] + m_Delays[v]]; // preallocation
        else
            readDepthFrame(m_DepthPaths[v], m_DepthFrameCounter[v] + m_Delays[v], frame);
        
        frames.push_back(frame);
    }
    
    return frames;
}

vector<ColorFrame> Sequence::getColorFrame(int i)
{
    vector<ColorFrame> frames;
    for (int v = 0; v < m_ColorPaths.size(); v++)
    {
        ColorFrame frame;
        if (m_ColorStreams[v].size() > 0)
            frame = m_ColorStreams[v][i];
        else
            readColorFrame(m_ColorPaths[v], i, frame);
        
        frames.push_back(frame);
    }
    
    return frames;
}

vector<DepthFrame> Sequence::getDepthFrame(int i)
{
    vector<DepthFrame> frames;
    for (int v = 0; v < m_DepthPaths.size(); v++)
    {
        DepthFrame frame;
        if (m_DepthStreams[v].size() > 0)
            frame = m_DepthStreams[v][i];
        else
            readDepthFrame(m_DepthPaths[v], i, frame);
        
        frames.push_back(frame);
    }
    
    return frames;
}

void Sequence::nextColorFrame(vector<cv::Mat>& colorMats, int step)
{
    for (int v = 0; v < m_ColorStreams.size(); v++)
        m_ColorFrameCounter[v] += step;
    
    colorMats.clear();
    for (int v = 0; v < m_ColorStreams.size(); v++)
    {
        cv::Mat mat;
        if (m_ColorStreams[v].size() > 0)
            colorMats.push_back(m_ColorStreams[v][m_ColorFrameCounter[v] + m_Delays[v]].getMat());
        else
            readMat(m_ColorPaths[v], m_ColorFrameCounter[v] + m_Delays[v], mat);
            
        colorMats.push_back(mat);
    }
}

void Sequence::nextDepthFrame(vector<cv::Mat>& depthMats, int step)
{
    for (int v = 0; v < m_DepthStreams.size(); v++)
        m_DepthFrameCounter[v] += step;
    
    depthMats.clear();
    for (int v = 0; v < m_DepthStreams.size(); v++)
    {
        cv::Mat mat;
        if (m_DepthStreams[v].size() > 0)
            depthMats.push_back(m_DepthStreams[v][m_DepthFrameCounter[v] + m_Delays[v]].getMat());
        else
            readMat(m_DepthPaths[v], m_DepthFrameCounter[v] + m_Delays[v], mat);
        
        depthMats.push_back(mat);
    }
}

void Sequence::previousColorFrame(vector<cv::Mat>& colorMats, int step)
{
    for (int i = 0; i < m_ColorStreams.size(); i++)
        m_ColorFrameCounter[i] -= step;
    
    colorMats.clear();
    for (int v = 0; v < m_ColorStreams.size(); v++)
    {
        cv::Mat mat;
        if (m_ColorStreams[v].size() > 0)
            colorMats.push_back(m_ColorStreams[v][m_ColorFrameCounter[v] + m_Delays[v]].getMat());
        else
            readMat(m_ColorPaths[v], m_ColorFrameCounter[v] + m_Delays[v], mat);
        
        colorMats.push_back(mat);
    }
}

void Sequence::previousDepthFrame(vector<cv::Mat>& depthMats, int step)
{
    for (int i = 0; i < m_DepthStreams.size(); i++)
        m_DepthFrameCounter[i] -= step;

    depthMats.clear();
    for (int v = 0; v < m_DepthStreams.size(); v++)
    {
        cv::Mat mat;
        if (m_DepthStreams[v].size() > 0)
            depthMats.push_back(m_DepthStreams[v][m_DepthFrameCounter[v] + m_Delays[v]].getMat());
        else
            readMat(m_DepthPaths[v], m_DepthFrameCounter[v] + m_Delays[v], mat);
        
        depthMats.push_back(mat);
    }
}

void Sequence::getColorFrame(vector<cv::Mat>& colorMats, int i)
{
    colorMats.clear();
    for (int v = 0; v < m_ColorStreams.size(); v++)
    {
        cv::Mat mat;
        if (m_ColorStreams[v].size() > 0)
            colorMats.push_back(m_ColorStreams[v][i + m_Delays[v]].getMat());
        else
            readMat(m_ColorPaths[v], i + m_Delays[v], mat);
        
        colorMats.push_back(mat);
    }
}

void Sequence::getDepthFrame(vector<cv::Mat>& depthMats, int i)
{
    depthMats.clear();
    for (int v = 0; v < m_DepthStreams.size(); v++)
    {
        cv::Mat mat;
        if (m_DepthStreams[v].size() > 0)
            depthMats.push_back(m_DepthStreams[v][i + m_Delays[v]].getMat());
        else
            readMat(m_DepthPaths[v], i + m_Delays[v], mat);
        
        depthMats.push_back(mat);
    }
}

int Sequence::getNumOfViews()
{
    assert(m_ColorPaths.size() == m_DepthPaths.size());
    
    return m_ColorPaths.size();
}

vector<int> Sequence::getNumOfFrames()
{
    assert (m_ColorPaths.size() == m_DepthPaths.size());
    
    vector<int> numOfFrames;
    for (int i = 0; i < m_ColorPaths.size(); i++)
        numOfFrames.push_back(m_ColorPaths[i].size());
    
    return numOfFrames;
}

vector<int> Sequence::getCurrentColorFramesID()
{
    vector<int> counters;
    for (int i = 0; i < m_ColorPaths.size(); i++)
        counters.push_back(m_ColorFrameCounter[i] + m_Delays[i]);
    
    return counters;
}

vector<int> Sequence::getCurrentDepthFramesID()
{
    vector<int> counters;
    for (int i = 0; i < m_DepthPaths.size(); i++)
        counters.push_back(m_DepthFrameCounter[i] + m_Delays[i]);
    
    return counters;
}

vector<float> Sequence::getColorProgress()
{
    vector<float> progresses;
    for (int i = 0; i < m_ColorPaths.size(); i++)
        progresses.push_back(((float) (m_ColorFrameCounter[i] + m_Delays[i])) / m_ColorPaths[i].size());
    
    return progresses;
}

vector<float> Sequence::getDepthProgress()
{
    vector<float> progresses;
    for (int i = 0; i < m_DepthPaths.size(); i++)
        progresses.push_back(((float) (m_DepthFrameCounter[i] + m_Delays[i])) / m_DepthPaths[i].size());
    
    return progresses;
}

vector<int> Sequence::colorAt()
{
    vector<int> colorFrameDelayedCounter(m_ColorStreams.size());
    
    for (int i = 0; i < m_ColorStreams.size(); i++)
        colorFrameDelayedCounter[i] = m_ColorFrameCounter[i] + m_Delays[i];
    
    return colorFrameDelayedCounter;
}

vector<int> Sequence::depthAt()
{
    vector<int> depthFrameDelayedCounter(m_DepthStreams.size());
    for (int i = 0; i < m_DepthStreams.size(); i++)
        depthFrameDelayedCounter[i] = m_DepthFrameCounter[i] + m_Delays[i];
    
    return depthFrameDelayedCounter;
}

void Sequence::setDelays(vector<int> delays)
{
    m_Delays = delays;
}

void Sequence::setInteractionLabels(vector<unsigned short> labels)
{
    m_InteractionLabels = labels;
}

void Sequence::setActionLabels(vector<unsigned short> labels)
{
    m_ActionLabels = labels;
}