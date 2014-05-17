//
//  DetectionOutput.cpp
//  remedi
//
//  Created by Albert Clapés on 14/05/14.
//
//

#include "DetectionOutput.h"

#include <math.h>
#include <assert.h>
#include <fstream>
#include <boost/algorithm/string.hpp>

DetectionOutput::DetectionOutput()
{
    
}

DetectionOutput::DetectionOutput(int nviews, vector<int> nframes, int nobjects, float tol)
: m_NumOfViews(nviews), m_NumOfFrames(nframes), m_NumOfObjects(nobjects), m_Tol(tol)
{
    m_Positions.resize(m_NumOfViews);
    
    for (int i = 0; i < m_NumOfViews; i++)
    {
        m_Positions[i].resize(m_NumOfFrames[i]);
        for (int j = 0; j < m_NumOfFrames[i]; j++)
        {
            m_Positions[i][j].resize(m_NumOfObjects);
        }
    }
}

DetectionOutput::DetectionOutput(vector< vector< vector< vector< pcl::PointXYZ > > > > positions)
{
    setPositions(positions);
}

DetectionOutput::DetectionOutput(const DetectionOutput& rhs)
{
    *this = rhs;
}

DetectionOutput::~DetectionOutput()
{
    
}

DetectionOutput& DetectionOutput::operator=(const DetectionOutput& rhs)
{
    if (this != &rhs)
    {
        m_Positions = rhs.m_Positions;
        if (m_Positions.size() > 0)
        {
            m_NumOfViews = m_Positions.size();
            for (int i = 0; i < m_NumOfViews; i++)
                m_NumOfFrames.push_back(m_Positions[i].size());
            m_NumOfObjects = m_Positions[0][0].size();
        }
        m_Tol = rhs.m_Tol;
    }
    
    return *this;
}

void DetectionOutput::setPositions(vector<vector<vector<vector<pcl::PointXYZ > > > > positions)
{
    m_Positions = positions;
    
    // Assertions
    for (int v = 0; v < m_Positions.size(); v++)
    {
        // assert num of frames
        assert( m_Positions[0].size() == m_Positions[v].size() );
        for (int f = 0; f < m_Positions[v][0].size(); f++)
            assert( m_Positions[0][0].size() == m_Positions[v][f].size() );
    }
    
    // Set important variables
    m_NumOfViews = m_Positions.size();
    for (int i = 0; i < m_NumOfViews; i++)
        m_NumOfFrames.push_back(m_Positions[i].size());
    m_NumOfObjects = m_Positions[0][0].size();
}

int DetectionOutput::getNumOfViews()
{
    return m_NumOfViews;
}

vector<int> DetectionOutput::getNumOfFrames()
{
    return m_NumOfFrames;
}

int DetectionOutput::getNumOfObjects()
{
    return m_NumOfObjects;
}

//void DetectionOutput::setNumOfViews(int n)
//{
//    m_NumOfViews = n;
//}
//
//void DetectionOutput::setNumOfFrames(int n)
//{
//    m_NumOfFrames = n;
//}
//
//void DetectionOutput::setNumOfObjects(int n)
//{
//    m_NumOfObjects = n;
//}

void DetectionOutput::setTolerance(float tol)
{
    m_Tol = tol;
}

float DetectionOutput::distance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    return sqrtf(powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2) + powf(p1.z - p2.z, 2));
}

void DetectionOutput::add(int view, int frame, int object, pcl::PointXYZ position)
{
    bool collision = false;
    for (int i = 0; i < m_Positions[view][frame][object].size() && !collision; i++)
        collision = distance(m_Positions[view][frame][object][i], position) < m_Tol;
        
    if (!collision)
        m_Positions[view][frame][object].push_back(position);
}

void DetectionOutput::remove(int view, int frame, int object, pcl::PointXYZ position)
{
    bool collision = false;
    int idx;
    for (int i = 0; i < m_Positions[view][frame][object].size() && !collision; i++)
    {
        collision = distance(m_Positions[view][frame][object][i], position) < m_Tol;
        if (collision) idx = i;
    }
    
    if (!collision)
        m_Positions[view][frame][object].erase(m_Positions[view][frame][object].begin() + idx);
}

void DetectionOutput::remove(int view, int frame, int object, int i)
{
    m_Positions[view][frame][object].erase(m_Positions[view][frame][object].begin() + i);
}

void DetectionOutput::get(int view, int frame, int object, vector<pcl::PointXYZ>& positions)
{
    positions = m_Positions[view][frame][object];
}

void DetectionOutput::clear()
{
    m_Positions.clear();
    
    m_NumOfViews = 0;
    m_NumOfFrames.clear();
    m_NumOfObjects = 0;
}

void DetectionOutput::write(string path, string filename, string extension)
{
    // Draw set points
    for (int v = 0; v < m_NumOfViews; v++)
    {
        ofstream outFile;
        string outputPath = path + filename + "_" + to_string(v) + "." + extension;
        outFile.open(outputPath, ios::out);
        
        // create two view files
        for (int f = 0; f < m_NumOfFrames[v]; f++)
        {
            for (int o = 0; o < m_NumOfObjects; o++)
            {
                outFile << o << ":";
                
                vector< pcl::PointXYZ > tmp = m_Positions[v][f][o];
                
                for (int p = 0; p < tmp.size(); p++)
                {
                    outFile << tmp[p].x << "," << tmp[p].y << "," << tmp[p].z << ";";
                    
                } outFile << "\t";
            } outFile << endl;
        }
        outFile.close();
    }
    
    cout << "DetectionOutput written successfully to " << path << "." << endl;
}

void DetectionOutput::read(string path, string filename, string extension)
{
    m_Positions.resize(m_NumOfViews);
    
    for (int i = 0; i < m_NumOfViews; i++)
    {
        m_Positions[i].resize(m_NumOfFrames[i]);
        for (int j = 0; j < m_NumOfFrames[i]; j++)
        {
            m_Positions[i][j].resize(m_NumOfObjects);
        }
    }
    
    // Draw set points
    for (int v = 0; v < m_NumOfViews; v++)
    {
        ifstream inFile;
        inFile.open(path + filename + "_" + to_string(v) + "." + extension, ios::in);

        string line;
        int f = 0; // number of lines, i.e. [f]rames
        while( getline(inFile, line) )
        {
            vector<string> objects_sublines; // ex: 1:202,22;104,123;
            boost::split(objects_sublines, line, boost::is_any_of("\t"));
            
            for (int o = 0; o < objects_sublines.size() - 1; o++)
            {
                vector<string> object_struct;
                boost::split(object_struct, objects_sublines[o], boost::is_any_of(":"));
                
                int oid = stoi(object_struct[0]); // object id
                if (object_struct[1].size() <= 2)
                    continue;
                
                vector<string> positions;
                boost::split(positions, object_struct[1], boost::is_any_of(";")); // object id's positions
                
                for (int p = 0; p < positions.size() - 1; p++)
                {
                    vector<string> coordinates;
                    boost::split(coordinates, positions[p], boost::is_any_of(","));
                    
                    string::size_type sz;
                    float x = stof(coordinates[0], &sz);
                    float y = stof(coordinates[1], &sz);
                    float z = stof(coordinates[2], &sz);
                    m_Positions[v][f][oid].push_back( pcl::PointXYZ(x,y,z) );
                }
            }
            f++;
        }
        
        inFile.close();
    }
    
    cout << "DetectionOutput read successfully from " << path << "." << endl;
}



