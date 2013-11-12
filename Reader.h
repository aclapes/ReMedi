#pragma once

#include <opencv2/opencv.hpp>
#include "ColorFrame.h"
#include "DepthFrame.h"

class Reader
{
public:
	Reader(const char*, const char*);
	~Reader(void);

	// Public methods
	void loadColorStreams(const char*, const char*);
	void loadDepthStreams(const char*, const char*);

	void getFrameID();

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
	void readNextColorFrame(const char*, ColorFrame&);
	void readNextDepthFrame(const char*, DepthFrame&);
	void readColorFrame(const char*, int fID, ColorFrame&);
	void readDepthFrame(const char*, int fID, DepthFrame&);

	bool isValid(ColorFrame);
	bool isValid(DepthFrame);

	// Private members
	const char*		m_DataPathA;
	const char*		m_DataPathB;

	int				m_cCounter, m_dCounter;
};

