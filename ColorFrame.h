#pragma once

#include "Frame.h"

// OpenCV
#include <opencv2/opencv.hpp>


class ColorFrame : public Frame
{
public:
	// Constructors
	ColorFrame(void);
	ColorFrame(cv::Mat mat);
	ColorFrame(const ColorFrame&);
	~ColorFrame(void);

	// Operators
	ColorFrame& operator=(const ColorFrame& other);

	// Methods
	bool isValid();
    
    void show(std::string wndName);
};

