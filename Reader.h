#pragma once

#include <opencv2/opencv.hpp>
#include "ColorFrame.h"
#include "DepthFrame.h"

class Reader
{
public:
	Reader(std::string, std::string);
	~Reader(void);

	// Public methods
	void loadColorStreams(std::string, std::string);
	void loadDepthStreams(std::string, std::string);

	bool getNextColorFrame(ColorFrame&);
	bool getNextDepthFrame(DepthFrame&);
	bool getColorFrame(int fID, ColorFrame&);
	bool getDepthFrame(int fID, DepthFrame&);

	bool getNextColorPairedFrames(ColorFrame&, ColorFrame&);
	bool getNextDepthPairedFrames(DepthFrame&, DepthFrame&);
	bool getColorPairedFrames(int fID, ColorFrame&, ColorFrame&);
	bool getDepthPairedFrames(int fID, DepthFrame&, DepthFrame&);

private:
	// Private methods
	void readNextColorFrame(std::string, ColorFrame&);
	void readNextDepthFrame(std::string, DepthFrame&);
	void readColorFrame(std::string, int fID, ColorFrame&);
	void readDepthFrame(std::string, int fID, DepthFrame&);

	bool isValid(ColorFrame);
	bool isValid(DepthFrame);

	// Private members
    std::string		m_DataPathA, m_DataPathB;

	int				m_cCounter, m_dCounter;
};

