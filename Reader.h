#pragma once

#include <opencv2/opencv.hpp>
#include "ColorFrame.h"
#include "DepthFrame.h"

#include "Sequence.h"

using namespace std;

class Reader
{
    //typedef boost::shared_ptr<Sequence> SequencePtr;
    
public:
    Reader();
    Reader(string sequencesPath,
           vector<string> colorDirs,
           vector<string> depthDirs);
	Reader(string sequencesPath,
           vector<string> colorDirs,
           vector<string> depthDirs,
           string labelsPath);
    Reader(const Reader& rhs);
	~Reader(void);
    
    void setData(string sequencesPath,
                 vector<string> colorDirs,
                 vector<string> depthDirs);
    void setData(string sequencesPath,
                 vector<string> colorDirs,
                 vector<string> depthDirs,
                 string labelsPath);

	// Public methods
    Reader& operator=(const Reader& rhs);
    
    bool hasNextSequence();
    bool hasSequence(int s);
    
    Sequence::Ptr getSequence(int i);
    void getSequence(int i, Sequence& sequence);
    Sequence::Ptr getNextSequence();
    void getNextSequence(Sequence& sequence);

    
//	void loadColorStreams(string dataPath, string colorDir1, string colorDir2);
//	void loadDepthStreams(string dataPath, string depthDir1, string depthDir2);
    
//    bool setSequence(int i);
//    string getSequencePath();
//    string getSequenceDirName();
//    bool nextSequence();
//
//    int getNumOfFrames();
//    int getColorFrameCounter();
//    int getDepthFrameCounter();
//    
//	bool nextColorFrame(string colorDir, vector<string> filenames, ColorFrame& cframe, int step = 1);
//	bool nextDepthFrame(string depthDir, vector<string> filenames, DepthFrame& dframe, int step = 1);
//    
//	bool getColorFrame(string colorDir, vector<string> filenames, int i, ColorFrame& cframe);
//	bool getDepthFrame(string depthDir, vector<string> filenames, int i, DepthFrame& dframe);
//
//	bool nextColorPairedFrames(ColorFrame& cframeA, ColorFrame& cframeB, int step = 1);
//	bool nextDepthPairedFrames(DepthFrame& dframeA, DepthFrame& dframeB, int step = 1);
//    bool previousColorPairedFrames(ColorFrame& cframeA, ColorFrame& cframeB, int step = 1);
//	bool previousDepthPairedFrames(DepthFrame& dframeA, DepthFrame& dframeB, int step = 1);
//    
//	bool getColorPairedFrames(int i, ColorFrame&, ColorFrame&);
//	bool getDepthPairedFrames(int i, DepthFrame&, DepthFrame&);
    
private:
	// Private methods
    void loadDirectories(string parent, vector<string>& directories);
    void loadFilenames(string dir, const char* filetype, vector<string>& filenames);
    void loadSequenceLabels(int s, vector<unsigned char>& interactions, vector<unsigned char>& actions);
    
//	bool readNextColorFrame(string dataPath, string colorDir, vector<string> filenames, ColorFrame& cframe);
//	bool readNextDepthFrame(string dataPath, string depthDir, vector<string> filenames, DepthFrame& dframe);
//    bool readPreviousColorFrame(string dataPath, string colorDir, vector<string> filenames, ColorFrame& cframe);
//	bool readPreviousDepthFrame(string dataPath, string depthDir, vector<string> filenames, DepthFrame& dframe);
	bool readColorFrame(string dataPath, string colorDir, vector<string> filenames, int i, ColorFrame& cframe);
	bool readDepthFrame(string dataPath, string depthDir, vector<string> filenames, int i, DepthFrame& dframe);
//    bool readColorFrame(string dataPath, string colorDir, string filename, ColorFrame& cframe);
//	bool readDepthFrame(string dataPath, string depthDir, string filename, DepthFrame& dframe);

	bool isValid(ColorFrame cframe);
	bool isValid(DepthFrame dframe);

	// Private members
    string m_SequencesPath;
    string m_LabelsPath;
    
    vector<string> m_SequencesDirs;
    int m_SequenceCounter;

    vector<string>  m_ColorDirs, m_DepthDirs;
    vector<vector<string> > m_ColorFilenames, m_DepthFilenames;
    
//    int m_ColorFrameCounter, m_DepthFrameCounter;
    
//    vector<unsigned char> m_InteractionLabels;
//    vector<unsigned char> m_ActionLabels;
};

