#pragma once

#include <opencv2/opencv.hpp>

class Frame
{
public:
	// Constructors
	Frame(void);
	Frame(cv::Mat mat);
	Frame(const Frame&);
	~Frame(void);

	// Operators
	Frame& operator=(const Frame& other);

	// Methods
	bool isValid();
    int getResX();
    int getResY();
    
	cv::Mat getMat();
	void getMat(cv::Mat&);

	void setMat(cv::Mat);
	void setMat(cv::Mat, cv::Mat);
    
    virtual void show(std::string wndName) = 0;

protected:
	// Members
	cv::Mat m_Mat;
};

