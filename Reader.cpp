#include "Reader.h"

Reader::Reader(std::string dataPath,
               std::string colorDir1, std::string colorDir2,
               std::string depthDir1, std::string depthDir2)
: m_DataPath(dataPath), m_colorDir1(colorDir1), m_colorDir2(colorDir2),
m_depthDir1(depthDir1), m_depthDir2(depthDir2), m_cCounter(0), m_dCounter(0)
{
}


Reader::~Reader(void)
{
}

void Reader::setSequence(std::string dir)
{
     m_SequenceDir = dir;
}

void Reader::readNextColorFrame(std::string dataPath, std::string colorDir, ColorFrame& cframe)
{
	std::stringstream ss;
	ss << m_cCounter;
	std::string filePath = dataPath + m_SequenceDir + "Kinects/" + colorDir + ss.str() + ".png";

	cframe = ColorFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
}


void Reader::readNextDepthFrame(std::string dataPath, std::string depthDir, DepthFrame& dframe)
{
	std::stringstream ss;
	ss << m_dCounter;
	std::string filePath = dataPath + m_SequenceDir + "Kinects/" + depthDir + ss.str() + ".png";

	dframe = DepthFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
}


void Reader::readColorFrame(std::string dataPath, std::string colorDir, int fID, ColorFrame& cframe)
{
	std::stringstream ss;
	ss << fID;
	std::string filePath = dataPath + m_SequenceDir + "Kinects/" + colorDir + ss.str() + ".png";

	cframe = ColorFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
}


void Reader::readDepthFrame(std::string dataPath, std::string depthDir, int fID, DepthFrame& dframe)
{
	std::stringstream ss;
	ss << fID;
	std::string filePath = dataPath + m_SequenceDir + "Kinects/" + depthDir + ss.str() + ".png";

	dframe = DepthFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
}


bool Reader::getNextColorFrame(std::string colorDir, ColorFrame& frame)
{
	readNextColorFrame(m_DataPath, colorDir, frame);

	if (frame.isValid())
	{
		m_cCounter++;
		return true;
	}
	else
	{
		return false;
	}
}


bool Reader::getNextDepthFrame(std::string colorDir, DepthFrame& frame)
{
	readNextDepthFrame(m_DataPath, colorDir, frame);

	if (frame.isValid())
	{
		m_cCounter++;
		return true;
	}
	else
	{
		return false;
	}
}


bool Reader::getColorFrame(std::string colorDir, int fID, ColorFrame& frame)
{
	readColorFrame(m_DataPath, colorDir, fID, frame);

	if (frame.isValid())
	{
		m_cCounter++;
		return true;
	}
	else
	{
		return false;
	}
}


bool Reader::getDepthFrame(std::string depthDir, int fID, DepthFrame& frame)
{
	readDepthFrame(m_DataPath, depthDir, fID, frame);

	if (frame.isValid())
	{
		m_cCounter++;
		return true;
	}
	else
	{
		return false;
	}
}


bool Reader::getNextColorPairedFrames(ColorFrame& frameA, ColorFrame& frameB)
{
	getNextColorFrame(m_colorDir1, frameA);
	getNextColorFrame(m_colorDir2, frameB);

	if (frameA.isValid() && frameB.isValid())
	{
		m_cCounter++;
		return true;
	}
	else
	{
		return false;
	}
}


bool Reader::getNextDepthPairedFrames(DepthFrame& frameA, DepthFrame& frameB)
{
	getNextDepthFrame(m_depthDir1, frameA);
	getNextDepthFrame(m_depthDir2, frameB);
	
	if (frameA.isValid() && frameB.isValid())
	{
		m_dCounter++;
		return true;
	}
	else
	{
		return false;
	}
}


bool Reader::getColorPairedFrames(int fID, ColorFrame& frameA, ColorFrame& frameB)
{
	getColorFrame(m_colorDir1, fID, frameA);
	getColorFrame(m_colorDir2, fID, frameB);

	if (frameA.isValid() && frameB.isValid()) 
		return true;
	else return false;
}


bool Reader::getDepthPairedFrames(int fID, DepthFrame& frameA, DepthFrame& frameB)
{
	getDepthFrame(m_depthDir1, fID, frameA);
	getDepthFrame(m_depthDir2, fID, frameB);

	if (frameA.isValid() && frameB.isValid()) 
		return true;
	else return false;
}