#include "Reader.h"

#include <sys/stat.h>
#include <string>
//#include <fstream>
#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#include "Remedi.h"

using namespace boost::filesystem;
using namespace std;

class Remedi; // solving cross reference

//typedef boost::shared_ptr<Sequence> SequencePtr;


Reader::Reader() {}

Reader::Reader(string sequencesPath,
               vector<string> colorDirs,
               vector<string> depthDirs)
: m_SequencesPath(sequencesPath),
m_ColorDirs(colorDirs), m_DepthDirs(depthDirs),
m_SequenceCounter(-1), m_LabelsPath(""),
m_bPreAllocation(false)
{
    assert(m_ColorDirs.size() == m_DepthDirs.size());
    
    
    loadDirectories(m_SequencesPath, m_SequencesDirs);
}

Reader::Reader(string sequencesPath,
               vector<string> colorDirs,
               vector<string> depthDirs,
               string labelsPath)
: m_SequencesPath(sequencesPath),
  m_ColorDirs(colorDirs), m_DepthDirs(depthDirs),
m_SequenceCounter(-1), m_LabelsPath(labelsPath),
m_bPreAllocation(false)
{
    loadDirectories(m_SequencesPath, m_SequencesDirs);
}

Reader::Reader(const Reader& rhs)
{
    *this = rhs;
}

Reader::~Reader(void)
{
}

void Reader::setData(string sequencesPath,
                     vector<string> colorDirs,
                     vector<string> depthDirs,
                     bool bPreAllocation)
{
    m_SequencesPath = sequencesPath;
    m_ColorDirs = colorDirs;
    m_DepthDirs = depthDirs;
    m_SequenceCounter = -1;
    m_LabelsPath = "";
    m_bPreAllocation = bPreAllocation;
    
    loadDirectories(m_SequencesPath, m_SequencesDirs);
}

void Reader::setData(string sequencesPath,
                     vector<string> colorDirs,
                     vector<string> depthDirs,
                     string labelsPath,
                     bool bPreAllocation)
{
    m_SequencesPath = sequencesPath;
    m_ColorDirs = colorDirs;
    m_DepthDirs = depthDirs;
    m_SequenceCounter = -1;
    m_LabelsPath = labelsPath;
    m_bPreAllocation = bPreAllocation;
    
    loadDirectories(m_SequencesPath, m_SequencesDirs);
}

Reader& Reader::operator=(const Reader& rhs)
{
    if (this != &rhs)
    {
        m_SequencesPath = rhs.m_SequencesPath;
        m_LabelsPath = rhs.m_LabelsPath;
        
        m_SequencesDirs = rhs.m_SequencesDirs;
        
        m_ColorDirs = rhs.m_ColorDirs;
        m_DepthDirs = rhs.m_DepthDirs;
        
        m_SequenceCounter = rhs.m_SequenceCounter;
        
        m_ColorFilesPaths = rhs.m_ColorFilesPaths;
        m_DepthFilesPaths = rhs.m_DepthFilesPaths;
        
        m_bPreAllocation = rhs.m_bPreAllocation;
        
//        m_ColorFrameCounter = rhs.m_ColorFrameCounter;
//        m_DepthFrameCounter = rhs.m_DepthFrameCounter;
        
//        m_ColorFilesPaths1 = rhs.m_ColorFilesPaths1;
//        m_ColorFilesPaths2 = rhs.m_ColorFilesPaths2;
//        m_DepthFilesPaths1 = rhs.m_DepthFilesPaths1;
//        m_DepthFilesPaths2 = rhs.m_DepthFilesPaths2;
        
//        m_InteractionLabels = rhs.m_InteractionLabels;
//        m_ActionLabels = rhs.m_ActionLabels;
    }
    
    return *this;
}

void Reader::setSequence(int i)
{
    m_SequenceCounter = i;
}

Sequence::Ptr Reader::getSequence(int s)
{
    Sequence::Ptr pSequence (new Sequence);
    
    if (hasSequence(s))
    {
        m_SequenceCounter = s;
        getSequence(m_SequenceCounter, *pSequence);
    }
    
    return pSequence;
}

void Reader::getSequence(int s, Sequence& sequence)
{
    sequence.setName(m_SequencesDirs[s]);
    
    // Read streams

    assert (m_ColorDirs.size() == m_DepthDirs.size());
    
    m_ColorFilesPaths.resize(m_ColorDirs.size());
    for (int v = 0; v < m_ColorDirs.size(); v++)
    {
        loadFilesPaths(m_SequencesPath + m_SequencesDirs[s] + "/Kinects/" + m_ColorDirs[v], "png", m_ColorFilesPaths[v]);
        
        for (int f = 0; f < m_ColorFilesPaths[v].size(); f++)
        {
            sequence.addColorFramePath(m_ColorFilesPaths[v][f], v);
            if (m_bPreAllocation)
            {
                ColorFrame frame;
                readColorFrame(m_ColorFilesPaths[v], f, frame);
                sequence.addColorFrame(frame, v);
            }
        }
    }
    
    m_DepthFilesPaths.resize(m_DepthDirs.size());
    for (int v = 0; v < m_DepthDirs.size(); v++)
    {
        loadFilesPaths(m_SequencesPath + m_SequencesDirs[s] + "/Kinects/" + m_DepthDirs[v], "png", m_DepthFilesPaths[v]);
        
        for (int f = 0; f < m_DepthFilesPaths[v].size(); f++)
        {
            sequence.addDepthFramePath(m_DepthFilesPaths[v][f], v);
            if (m_bPreAllocation)
            {
                DepthFrame frame;
                readDepthFrame(m_DepthFilesPaths[v], f, frame);
                sequence.addDepthFrame(frame, v);
            }
        }
    }
    
    sequence.setDelays(m_Delays);
    
    // Read labels
    
    vector<unsigned short> interactions;
    vector<unsigned short> actions;
    loadSequenceLabels(m_SequencesDirs[s], sequence.getNumOfFrames()[0],
                       interactions, actions);
    
    sequence.setInteractionLabels(interactions);
    sequence.setActionLabels(actions);
}

Sequence::Ptr Reader::getNextSequence()
{
    Sequence::Ptr pSequence (new Sequence);
    
    getNextSequence(*pSequence);
    
    return pSequence;
}

void Reader::getNextSequence(Sequence& sequence)
{
    getSequence(m_SequenceCounter++, sequence);
}

void Reader::setDelays(vector<int> delays)
{
    m_Delays = delays;
}

bool Reader::hasNextSequence()
{
    return ((m_SequenceCounter + 1) >= 0) && ((m_SequenceCounter + 1) < m_SequencesDirs.size());
}

bool Reader::hasSequence(int i)
{
    return i >= 0 && i < m_SequencesDirs.size();
}

void Reader::loadSequenceLabels(string sequenceDir, int length, vector<unsigned short>& interactions, vector<unsigned short>& actions)
{
    if (m_LabelsPath.compare("") == 0)
        return;

    std::string filename(m_LabelsPath + sequenceDir + ".csv");
    std::ifstream infile(filename);
    if (!infile.is_open())
    {
        cerr << filename << " doesn't exist." << endl;
        return;
    }
    
    vector<unsigned short> interactionsTmp;
    vector<unsigned short> actionsTmp;
    
    string cat, subcat;
    int vFrame, wFrameIni, wFrameEnd, val;
    while (infile >> cat >> subcat >> vFrame >> wFrameIni >> wFrameEnd >> val)
    {
        if (cat.compare("interaction") == 0)
        {
            unsigned short interacts = 0;
            
            if (subcat.compare("pillbox") == 0)
                interacts |= (unsigned short) pow(2, (int) Remedi::PILLBOX);
            else if (subcat.compare("dish") == 0)
                interacts |= (unsigned short) pow(2, (int) Remedi::DISH);
            else if (subcat.compare("book") == 0)
                interacts |= (unsigned short) pow(2, (int) Remedi::BOOK);
            else if (subcat.compare("glass") == 0)
                interacts |= (unsigned short) pow(2, (int) Remedi::GLASS);
            
            interactionsTmp.push_back(interacts);
        }
        else if (cat.compare("action") == 0)
        {
            unsigned short acts = 0;
            
            if (subcat.compare("takingpill") == 0)
                acts |= (unsigned short) pow(2, (int) Remedi::TAKINGPILL);
            else if (subcat.compare("eating") == 0)
                acts |= (unsigned short) pow(2, (int) Remedi::EATING);
            else if (subcat.compare("reading") == 0)
                acts |= (unsigned short) pow(2, (int) Remedi::READING);
            else if (subcat.compare("drinking") == 0)
                acts |= (unsigned short) pow(2, (int) Remedi::DRINKING);
            
            actionsTmp.push_back(acts);
        }
    }
    
    infile.close();
    
    interactions = interactionsTmp;
    actions = actionsTmp;
}

void Reader::loadFilesPaths(string dir, const char* filetype, vector<string>& filenames)
{
 	filenames.clear();
    
    string extension = "." + string(filetype);
    const char* path = dir.c_str();
	if( exists( path ) )
	{
		directory_iterator end;
		directory_iterator iter(path);
		for( ; iter != end ; ++iter )
		{
			if ( !is_directory( *iter ) && (iter->path().extension().string().compare(extension) == 0) )
            {
				filenames.push_back(iter->path().string());
            }
		}
	}
}

void Reader::loadDirectories(string parent, vector<string>& directories)
{
 	directories.clear();
    
    const char* path = parent.c_str();
	if( exists( path ) )
	{
		directory_iterator end;
		directory_iterator iter(path);
		for( ; iter != end ; ++iter )
		{
			if ( is_directory( *iter ) )
				directories.push_back(iter->path().filename().string());
		}
	}
}

bool Reader::readColorFrame(vector<string> filesPaths, int f, ColorFrame& cframe)
{
    assert (f < filesPaths.size());
    
	cframe = ColorFrame( cv::imread(filesPaths[f].c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
    
    return cframe.isValid();
}

bool Reader::readDepthFrame(vector<string> filesPaths, int f, DepthFrame& dframe)
{
    assert (f < filesPaths.size());
    
	dframe = DepthFrame( cv::imread(filesPaths[f].c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
    
    return dframe.isValid();
}