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

#include <pcl/point_types.h>

using namespace std;

class DetectionOutput
{
public:
    DetectionOutput();
    DetectionOutput(int nviews, int nframes, int nobjects);
    DetectionOutput(vector< vector< vector< vector< pcl::PointXYZ > > > > positions);
    DetectionOutput(const DetectionOutput& rhs);
    ~DetectionOutput();
    
    DetectionOutput& operator=(const DetectionOutput& rhs);
    
    void setPositions(vector< vector< vector< vector< pcl::PointXYZ > > > > positions);
    int getNumOfViews();
    int getNumOfFrames();
    int getNumOfObjects();
//    void setNumOfViews(int n);
//    void setNumOfFrames(int n);
//    void setNumOfObjects(int n);
    void setTolerance(double tol);
    
    void add(int view, int frame, int object, pcl::PointXYZ position);
    void remove(int view, int frame, int object, pcl::PointXYZ position);
    void remove(int view, int frame, int object, int i);
    void get(int view, int frame, int object, vector<pcl::PointXYZ>& positions);
    
    void clear();
    void read(string path, string filename, string extension);
    void write(string path, string filename, string extension);
    
private:
    float distance(pcl::PointXYZ p1, pcl::PointXYZ p2);

    // view, frame, model, model_instances_positions, (x,y,z) "real world" position
    vector< vector< vector< vector< pcl::PointXYZ > > > > m_Positions;
    int m_NumOfViews;
    int m_NumOfFrames;
    int m_NumOfObjects;
    float m_Tol; // do not add a new detection if there is another very close
};

#endif /* defined(__remedi__DetectionOutput__) */
