#pragma once

#include <opencv2/opencv.hpp>
#include "ColorFrame.h"
#include "DepthFrame.h"

using namespace std;

class Reader
{
public:
    Reader();
    Reader(string sequencesPath,
           string colorDir1, string colorDir2,
           string depthDir1, string depthDir2);
	Reader(string sequencesPath,
           string colorDir1, string colorDir2,
           string depthDir1, string depthDir2,
           string labelsPath);
    Reader(const Reader& rhs);
	~Reader(void);
    
    void setData(string sequencesPath,
                 string colorDir1, string colorDir2,
                 string depthDir1, string depthDir2);
    void setData(string sequencesPath,
                 string colorDir1, string colorDir2,
                 string depthDir1, string depthDir2,
                 string labelsPath);

	// Public methods
    Reader& operator=(const Reader& rhs);
    
//	void loadColorStreams(string dataPath, string colorDir1, string colorDir2);
//	void loadDepthStreams(string dataPath, string depthDir1, string depthDir2);
    
    bool setSequence(int i);
    string getSequencePath();
    string getSequenceDirName();
    bool nextSequence();

    int getNumOfFrames();
    int getColorFrameCounter();
    int getDepthFrameCounter();
    
	bool nextColorFrame(string colorDir, vector<string> filenames, ColorFrame& cframe, int step = 1);
	bool nextDepthFrame(string depthDir, vector<string> filenames, DepthFrame& dframe, int step = 1);
    
	bool getColorFrame(string colorDir, vector<string> filenames, int i, ColorFrame& cframe);
	bool getDepthFrame(string depthDir, vector<string> filenames, int i, DepthFrame& dframe);

	bool nextColorPairedFrames(ColorFrame& cframeA, ColorFrame& cframeB, int step = 1);
	bool nextDepthPairedFrames(DepthFrame& dframeA, DepthFrame& dframeB, int step = 1);
    bool previousColorPairedFrames(ColorFrame& cframeA, ColorFrame& cframeB, int step = 1);
	bool previousDepthPairedFrames(DepthFrame& dframeA, DepthFrame& dframeB, int step = 1);
    
	bool getColorPairedFrames(int i, ColorFrame&, ColorFrame&);
	bool getDepthPairedFrames(int i, DepthFrame&, DepthFrame&);
    
private:
	// Private methods
    void readSequenceFrames();
    void readSequenceLabels();
    
    void loadFilenames(string dir, const char* filetype, vector<string>& filenames);
    void loadDirectories(string parent, vector<string>& directories);
    
//	bool readNextColorFrame(string dataPath, string colorDir, vector<string> filenames, ColorFrame& cframe);
//	bool readNextDepthFrame(string dataPath, string depthDir, vector<string> filenames, DepthFrame& dframe);
//    bool readPreviousColorFrame(string dataPath, string colorDir, vector<string> filenames, ColorFrame& cframe);
//	bool readPreviousDepthFrame(string dataPath, string depthDir, vector<string> filenames, DepthFrame& dframe);
	bool readColorFrame(string dataPath, string colorDir, vector<string> filenames, int i, ColorFrame& cframe);
	bool readDepthFrame(string dataPath, string depthDir, vector<string> filenames, int i, DepthFrame& dframe);
    bool readColorFrame(string dataPath, string colorDir, string filename, ColorFrame& cframe);
	bool readDepthFrame(string dataPath, string depthDir, string filename, DepthFrame& dframe);

	bool isValid(ColorFrame cframe);
	bool isValid(DepthFrame dframe);

	// Private members
    string m_SequencesPath;
    string m_LabelsPath;
    
    vector<string> m_SequencesDirs;

    string  m_ColorDir1, m_ColorDir2, m_DepthDir1, m_DepthDir2;
    vector<string> m_ColorFilenames1, m_ColorFilenames2, m_DepthFilenames1, m_DepthFilenames2;
    
    int m_SequenceCounter;
    int m_ColorFrameCounter, m_DepthFrameCounter;
    
    vector<unsigned char> m_InteractionLabels;
    vector<unsigned char> m_ActionLabels;
};

