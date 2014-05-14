//
//  DetectionOutput.h
//  remedi
//
//  Created by Albert Clapés on 14/05/14.
//
//

#ifndef __remedi__DetectionOutput__
#define __remedi__DetectionOutput__

#include <iostream>
#include <vector>
#include <string>

using namespace std;

class DetectionOutput
{
public:
    DetectionOutput(int nviews, int nframes, int nobjects);
    DetectionOutput(vector< vector< vector< vector< pair<int,int> > > > > positions);
    DetectionOutput(const DetectionOutput& rhs);
    ~DetectionOutput();
    
    DetectionOutput& operator=(const DetectionOutput& rhs);
    
    void setPositions(vector< vector< vector< vector< pair<int,int> > > > > positions);
    void setNumOfViews(int n);
    void setNumOfFrames(int n);
    void setNumOfObjects(int n);
    void setTolerance(double tol);
    
    void add(int view, int frame, int object, pair<int,int> position);
    
    void read(string path);
    void write(string path);
    
private:
    float distance(pair<int,int> p1, pair<int,int> p2);


    // view, frame, model, model_instances_positions, (x,y) position
    vector< vector< vector< vector< pair<int,int> > > > > m_Positions;
    int m_NumOfViews;
    int m_NumOfFrames;
    int m_NumOfObjects;
    float m_Tol; // do not add a new detection if there is another very close
};

#endif /* defined(__remedi__DetectionOutput__) */
