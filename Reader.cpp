#include "Reader.h"


Reader::Reader(const char* dataPathA, const char* dataPathB) 
	: m_DataPathA(dataPathA), m_DataPathB(dataPathB), m_cCounter(0), m_dCounter(0)
{
}


Reader::~Reader(void)
{
}


int Reader::getFrameID()
{
	return m_dCounter;
}


void Reader::readNextColorFrame(const char* dataPath, ColorFrame& frame)
{
	std::stringstream ss;
	ss << m_cCounter;
	std::string filePath = std::string(dataPath) + "Color/" + ss.str() + ".png";

	frame = ColorFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
}


void Reader::readNextDepthFrame(const char* dataPath, DepthFrame& frame)
{
	std::stringstream ss;
	ss << m_dCounter;
	std::string filePath = std::string(dataPath) + "Depth/" + ss.str() + ".png";

	frame = DepthFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
}


void Reader::readColorFrame(const char* dataPath, int fID, ColorFrame& frame)
{
	std::stringstream ss;
	ss << fID;
	std::string filePath = std::string(dataPath) + "Color/" + ss.str() + ".png";

	frame = ColorFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
}


void Reader::readDepthFrame(const char* dataPath, int fID, DepthFrame& frame)
{
	std::stringstream ss;
	ss << fID;
	std::string filePath = std::string(dataPath) + "Depth/" + ss.str() + ".png";

	frame = DepthFrame( cv::imread(filePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
}


bool Reader::getNextColorFrame(ColorFrame& frame)
{
	readNextColorFrame(m_DataPathA, frame);

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


bool Reader::getNextDepthFrame(DepthFrame& frame)
{
	readNextDepthFrame(m_DataPathA, frame);

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


bool Reader::getColorFrame(int fID, ColorFrame& frame)
{
	readColorFrame(m_DataPathA, fID, frame);

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


bool Reader::getDepthFrame(int fID, DepthFrame& frame)
{
	readDepthFrame(m_DataPathA, fID, frame);

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
	readNextColorFrame(m_DataPathA, frameA);
	readNextColorFrame(m_DataPathB, frameB);

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
	readNextDepthFrame(m_DataPathA, frameA);
	readNextDepthFrame(m_DataPathB, frameB);
	
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
	readColorFrame(m_DataPathA, fID, frameA);
	readColorFrame(m_DataPathB, fID, frameB);

	if (frameA.isValid() && frameB.isValid()) 
		return true;
	else return false;
}


bool Reader::getDepthPairedFrames(int fID, DepthFrame& frameA, DepthFrame& frameB)
{
	readDepthFrame(m_DataPathA, fID, frameA);
	readDepthFrame(m_DataPathB, fID, frameB);

	if (frameA.isValid() && frameB.isValid()) 
		return true;
	else return false;
}