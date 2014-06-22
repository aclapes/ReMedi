#pragma once

#include "Frame.h"

// OpenCV
#include <opencv2/opencv.hpp>


class ColorFrame : public Frame
{
public:
	// Constructors
	ColorFrame();
	ColorFrame(cv::Mat mat);
	ColorFrame(const ColorFrame&);
	~ColorFrame();

	// Operators
	ColorFrame& operator=(const ColorFrame& rhs);

	// Methods
	bool isValid();
    int getResX();
    int getResY();
    
    void show(std::string wndName);
};

