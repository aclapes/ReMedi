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

Reader::Reader() {}

Reader::Reader(string sequencesPath,
               string colorDir1, string colorDir2,
               string depthDir1, string depthDir2)
: m_SequencesPath(sequencesPath),
m_ColorDir1(colorDir1), m_ColorDir2(colorDir2),
m_DepthDir1(depthDir1), m_DepthDir2(depthDir2),
m_SequenceCounter(-1),
m_ColorFrameCounter(-1), m_DepthFrameCounter(-1), m_LabelsPath("")
{
    loadDirectories(m_SequencesPath, m_SequencesDirs);
}

Reader::Reader(string sequencesPath,
               string colorDir1, string colorDir2,
               string depthDir1, string depthDir2,
               string labelsPath)
: m_SequencesPath(sequencesPath),
  m_ColorDir1(colorDir1), m_ColorDir2(colorDir2),
  m_DepthDir1(depthDir1), m_DepthDir2(depthDir2),
  m_SequenceCounter(-1),
  m_ColorFrameCounter(-1), m_DepthFrameCounter(-1), m_LabelsPath(labelsPath)
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
                     string colorDir1, string colorDir2,
                     string depthDir1, string depthDir2)
{
    m_SequencesPath = sequencesPath;
    m_ColorDir1 = colorDir1;
    m_ColorDir2 = colorDir2;
    m_DepthDir1 = depthDir1;
    m_DepthDir2 = depthDir2;
    m_SequenceCounter = -1;
    m_ColorFrameCounter = -1;
    m_DepthFrameCounter = -1;
    m_LabelsPath = "";
    
    loadDirectories(m_SequencesPath, m_SequencesDirs);
}

void Reader::setData(string sequencesPath,
                     string colorDir1, string colorDir2,
                     string depthDir1, string depthDir2,
                     string labelsPath)
{
    m_SequencesPath = sequencesPath;
    m_ColorDir1 = colorDir1;
    m_ColorDir2 = colorDir2;
    m_DepthDir1 = depthDir1;
    m_DepthDir2 = depthDir2;
    m_SequenceCounter = -1;
    m_ColorFrameCounter = -1;
    m_DepthFrameCounter = -1;
    m_LabelsPath = labelsPath;
    
    loadDirectories(m_SequencesPath, m_SequencesDirs);
}

Reader& Reader::operator=(const Reader& rhs)
{
    if (this != &rhs)
    {
        m_SequencesPath = rhs.m_SequencesPath;
        m_LabelsPath = rhs.m_LabelsPath;
        
        m_SequencesDirs = rhs.m_SequencesDirs;
        
        m_ColorDir1 = rhs.m_ColorDir1;
        m_ColorDir2 = rhs.m_ColorDir2;
        m_DepthDir1 = rhs.m_DepthDir1;
        m_DepthDir2 = rhs.m_DepthDir2;
        
        m_SequenceCounter = rhs.m_SequenceCounter;
        m_ColorFrameCounter = rhs.m_ColorFrameCounter;
        m_DepthFrameCounter = rhs.m_DepthFrameCounter;
        
        m_ColorFilenames1 = rhs.m_ColorFilenames1;
        m_ColorFilenames2 = rhs.m_ColorFilenames2;
        m_DepthFilenames1 = rhs.m_DepthFilenames1;
        m_DepthFilenames2 = rhs.m_DepthFilenames2;
        
        m_InteractionLabels = rhs.m_InteractionLabels;
        m_ActionLabels = rhs.m_ActionLabels;
    }
    
    return *this;
}

void Reader::readSequenceFrames()
{
    // Color streams
    loadFilenames(m_SequencesPath + m_SequencesDirs[m_SequenceCounter] + "/Kinects/" + m_ColorDir1, "png", m_ColorFilenames1);
    loadFilenames(m_SequencesPath + m_SequencesDirs[m_SequenceCounter] + "/Kinects/" + m_ColorDir2, "png", m_ColorFilenames2);
    
    // Depth streams
    loadFilenames(m_SequencesPath + m_SequencesDirs[m_SequenceCounter] + "/Kinects/" + m_DepthDir1, "png", m_DepthFilenames1);
    loadFilenames(m_SequencesPath + m_SequencesDirs[m_SequenceCounter] + "/Kinects/" + m_DepthDir2, "png", m_DepthFilenames2);
    
    // assertions
    assert (m_ColorFilenames1.size() == m_DepthFilenames1.size());
    assert (m_ColorFilenames2.size() == m_DepthFilenames2.size());
    assert (abs((int) m_ColorFilenames1.size() - (int) m_ColorFilenames2.size()) <= 2);
    
    m_ColorFrameCounter = -1;
    m_DepthFrameCounter = -1;
}

void Reader::readSequenceLabels()
{
    if (m_LabelsPath.compare("") == 0)
    {
        return;
    }
    
    m_InteractionLabels = vector<unsigned char>(m_ColorFilenames1.size(), 0);
    m_ActionLabels = vector<unsigned char>(m_ColorFilenames1.size(), 0);

    std::ifstream infile(m_LabelsPath + m_SequencesDirs[m_SequenceCounter] + ".csv");
    if (!infile.is_open())
    {
        cerr << "ERROR: file doesn't exist." << endl;
        return;
    }
    
    string cat, subcat;
    int vFrame, wFrameIni, wFrameEnd, val;
    while (infile >> cat >> subcat >> vFrame >> wFrameIni >> wFrameEnd >> val)
    {
        if (cat.compare("interaction") == 0)
        {
            if (subcat.compare("pillbox") == 0)
                m_InteractionLabels[vFrame] |= (unsigned char) pow(2, (int) Remedi::PILLBOX);
            else if (subcat.compare("dish") == 0)
                m_InteractionLabels[vFrame] |= (unsigned char) pow(2, (int) Remedi::DISH);
            else if (subcat.compare("book") == 0)
                m_InteractionLabels[vFrame] |= (unsigned char) pow(2, (int) Remedi::BOOK);
            else if (subcat.compare("glass") == 0)
                m_InteractionLabels[vFrame] |= (unsigned char) pow(2, (int) Remedi::GLASS);
        }
        else if (cat.compare("action") == 0)
        {
            if (subcat.compare("takingpill") == 0)
                m_ActionLabels[vFrame] |= (unsigned char) pow(2, (int) Remedi::TAKINGPILL);
            else if (subcat.compare("eating") == 0)
                m_ActionLabels[vFrame] |= (unsigned char) pow(2, (int) Remedi::EATING);
            else if (subcat.compare("reading") == 0)
                m_ActionLabels[vFrame] |= (unsigned char) pow(2, (int) Remedi::READING);
            else if (subcat.compare("drinking") == 0)
                m_ActionLabels[vFrame] |= (unsigned char) pow(2, (int) Remedi::DRINKING);
        }
    }
}

bool Reader::setSequence(int i)
{
    if (i < 0 || i >= m_SequencesDirs.size())
        return false;
    
    m_SequenceCounter = i;
    
    readSequenceFrames();
    readSequenceLabels();
}

string Reader::getSequencePath()
{
    return m_SequencesPath + m_SequencesDirs[m_SequenceCounter] + "/";
}

string Reader::getSequenceDirName()
{
    return m_SequencesDirs[m_SequenceCounter];
}

bool Reader::nextSequence()
{
    if (m_SequenceCounter == m_SequencesDirs.size() - 1)
    {
        return false;
    }
    
    m_SequenceCounter++;
    
    readSequenceFrames();
    readSequenceLabels();
}

void Reader::loadFilenames(string dir, const char* filetype, vector<string>& filenames)
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
				filenames.push_back(iter->path().filename().string());
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

//bool Reader::readNextColorFrame(string sequencesPath, string colorDir, vector<string> filenames, ColorFrame& cframe)
//{
//	string filePath = sequencesPath + m_SequencesDirs[m_SequenceCounter] + "Kinects/" + colorDir + filenames[m_ColorFrameCounter];
//
//	cframe = ColorFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
//    
//    return cframe.isValid();
//}
//
//
//bool Reader::readNextDepthFrame(string sequencesPath, string depthDir, vector<string> filenames, DepthFrame& dframe)
//{
//	string filePath = sequencesPath + m_SequencesDirs[m_SequenceCounter] + "Kinects/" + depthDir + filenames[m_DepthFrameCounter];
//
//	dframe = DepthFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
//
//    return dframe.isValid();
//}


bool Reader::readColorFrame(string sequencesPath, string colorDir, vector<string> filenames, int i, ColorFrame& cframe)
{
    assert (i < filenames.size());
    
	string filePath = sequencesPath + m_SequencesDirs[m_SequenceCounter] + "/Kinects/" + colorDir + filenames[i];
    
	cframe = ColorFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );

    return cframe.isValid();
}


bool Reader::readDepthFrame(string sequencesPath, string depthDir, vector<string> filenames, int i, DepthFrame& dframe)
{
	assert (i < filenames.size());
    
	string filePath = sequencesPath + m_SequencesDirs[m_SequenceCounter] + "/Kinects/" + depthDir + filenames[i];
    
	dframe = DepthFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );

    return dframe.isValid();
}


bool Reader::readColorFrame(string sequencesPath, string colorDir, string filename, ColorFrame& cframe)
{
	string filePath = sequencesPath + m_SequencesDirs[m_SequenceCounter] + "/Kinects/" + colorDir + filename;
    
	cframe = ColorFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );

    return cframe.isValid();
}


bool Reader::readDepthFrame(string sequencesPath, string depthDir, string filename, DepthFrame& dframe)
{
	string filePath = sequencesPath + m_SequencesDirs[m_SequenceCounter] + "/Kinects/" + depthDir + filename;
    
	dframe = DepthFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );

    return dframe.isValid();
}

int Reader::getNumOfFrames()
{
    return m_ColorFilenames1.size(); // master stream and master modality
}

int Reader::getColorFrameCounter()
{
    return m_ColorFrameCounter;
}

int Reader::getDepthFrameCounter()
{
    return m_DepthFrameCounter;
}

bool Reader::nextColorFrame(string colorDir, vector<string> filenames, ColorFrame& frame, int step)
{
    if (m_ColorFrameCounter + step < filenames.size() - 1)
        m_ColorFrameCounter += step;

	return readColorFrame(m_SequencesPath, colorDir, filenames[m_ColorFrameCounter], frame);
}


bool Reader::nextDepthFrame(string colorDir, vector<string> filenames, DepthFrame& frame, int step)
{
    if (m_DepthFrameCounter + step < filenames.size() - 1)
        m_DepthFrameCounter += step;

	return readDepthFrame(m_SequencesPath, colorDir, filenames[m_DepthFrameCounter], frame);
}


bool Reader::getColorFrame(string colorDir, vector<string> filenames, int i, ColorFrame& frame)
{
    // TODO: range check
	return readColorFrame(m_SequencesPath, colorDir, filenames, i, frame);
}


bool Reader::getDepthFrame(string depthDir, vector<string> filenames, int i, DepthFrame& frame)
{
    // TODO: range check
	return readDepthFrame(m_SequencesPath, depthDir, filenames, i, frame);
}


bool Reader::nextColorPairedFrames(ColorFrame& frameA, ColorFrame& frameB, int step)
{
    if (m_ColorFrameCounter + step < m_ColorFilenames1.size() - 1)
        m_ColorFrameCounter += step;

	bool validA = readColorFrame(m_SequencesPath, m_ColorDir1, m_ColorFilenames1[m_ColorFrameCounter], frameA);
	bool validB = readColorFrame(m_SequencesPath, m_ColorDir2, m_ColorFilenames2[m_ColorFrameCounter], frameB);
    
	return validA && validB;
}


bool Reader::nextDepthPairedFrames(DepthFrame& frameA, DepthFrame& frameB, int step)
{
    if (m_DepthFrameCounter + step < m_DepthFilenames1.size() - 1)
        m_DepthFrameCounter += step;

	bool validA = readDepthFrame(m_SequencesPath, m_DepthDir1, m_DepthFilenames1[m_DepthFrameCounter], frameA);
	bool validB = readDepthFrame(m_SequencesPath, m_DepthDir2, m_DepthFilenames2[m_DepthFrameCounter], frameB);
	
	return validA && validB;
}

bool Reader::previousColorPairedFrames(ColorFrame& frameA, ColorFrame& frameB, int step)
{
    if (m_ColorFrameCounter >= step)
        m_ColorFrameCounter -= step;

	bool validA = readColorFrame(m_SequencesPath, m_ColorDir1, m_ColorFilenames1[m_ColorFrameCounter], frameA);
	bool validB = readColorFrame(m_SequencesPath, m_ColorDir2, m_ColorFilenames2[m_ColorFrameCounter], frameB);
    
	return validA && validB;
}


bool Reader::previousDepthPairedFrames(DepthFrame& frameA, DepthFrame& frameB, int step)
{
    if (m_DepthFrameCounter >= step)
        m_DepthFrameCounter -= step;

	bool validA = readDepthFrame(m_SequencesPath, m_DepthDir1, m_DepthFilenames1[m_DepthFrameCounter], frameA);
	bool validB = readDepthFrame(m_SequencesPath, m_DepthDir2, m_DepthFilenames2[m_DepthFrameCounter], frameB);
	
	return validA && validB;
}


bool Reader::getColorPairedFrames(int i, ColorFrame& frameA, ColorFrame& frameB)
{
	bool validA = getColorFrame(m_ColorDir1, m_ColorFilenames1, i, frameA);
	bool validB = getColorFrame(m_ColorDir2, m_ColorFilenames2, i, frameB);

	return validA && validB;
}


bool Reader::getDepthPairedFrames(int i, DepthFrame& frameA, DepthFrame& frameB)
{
	bool validA = getDepthFrame(m_DepthDir1, m_DepthFilenames1, i, frameA);
	bool validB = getDepthFrame(m_DepthDir2, m_DepthFilenames2, i, frameB);

	return validA && validB;
}