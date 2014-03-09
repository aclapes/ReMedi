#pragma once

#include <opencv2/opencv.hpp>
#include "ColorFrame.h"
#include "DepthFrame.h"

class Reader
{
public:
	Reader(std::string parent,
           std::string colorDir1, std::string colorDir2,
           std::string depthDir1, std::string depthDir2);
	~Reader(void);

	// Public methods
    void setSequence(std::string dir);
    
	void loadColorStreams(std::string dataPath, std::string colorDir1, std::string colorDir2);
	void loadDepthStreams(std::string dataPath, std::string depthDir1, std::string depthDir2);

	bool getNextColorFrame(std::string colorDir, ColorFrame& cframe);
	bool getNextDepthFrame(std::string depthDir, DepthFrame& dframe);
	bool getColorFrame(std::string colorDir, int fID, ColorFrame& cframe);
	bool getDepthFrame(std::string depthDir, int fID, DepthFrame& dframe);

	bool getNextColorPairedFrames(ColorFrame& cframeA, ColorFrame& cframeB);
	bool getNextDepthPairedFrames(DepthFrame& dframeA, DepthFrame& dframeB);
	bool getColorPairedFrames(int fID, ColorFrame&, ColorFrame&);
	bool getDepthPairedFrames(int fID, DepthFrame&, DepthFrame&);

private:
	// Private methods
	void readNextColorFrame(std::string dataPath, std::string colorDir, ColorFrame& cframe);
	void readNextDepthFrame(std::string dataPath, std::string depthDir, DepthFrame& dframe);
	void readColorFrame(std::string dataPath, std::string colorDir, int fID, ColorFrame& cframe);
	void readDepthFrame(std::string dataPath, std::string depthDir, int fID, DepthFrame& dframe);

	bool isValid(ColorFrame cframe);
	bool isValid(DepthFrame dframe);

	// Private members
    std::string		m_DataPath;
    std::string     m_BackgroundDir, m_SequenceDir;
    std::string     m_colorDir1, m_colorDir2, m_depthDir1, m_depthDir2;

	int				m_cCounter, m_dCounter;
};

