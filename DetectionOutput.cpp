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

DetectionOutput::DetectionOutput(int nviews, int nframes, int nobjects)
: m_NumOfViews(nviews), m_NumOfFrames(nframes), m_NumOfObjects(nobjects)
{
    
}

DetectionOutput::DetectionOutput(vector< vector< vector< vector< pair<int,int> > > > > positions)
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
        m_NumOfViews = m_Positions.size();
        m_NumOfFrames = m_Positions[0].size();
        m_NumOfObjects = m_Positions[0][0].size();
    }
    
    return *this;
}

void DetectionOutput::setPositions(vector<vector<vector<vector<pair<int, int> > > > > positions)
{
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
    m_NumOfFrames = m_Positions[0].size();
    m_NumOfObjects = m_Positions[0][0].size();
}

void DetectionOutput::setNumOfViews(int n)
{
    m_NumOfViews = n;
}

void DetectionOutput::setNumOfFrames(int n)
{
    m_NumOfFrames = n;
}

void DetectionOutput::setNumOfObjects(int n)
{
    m_NumOfObjects = n;
}

void DetectionOutput::setTolerance(double tol)
{
    m_Tol = tol;
}

float DetectionOutput::distance(pair<int,int> p1, pair<int,int> p2)
{
    return sqrtf(powf(p1.first - p2.first, 2) + powf(p1.second - p2.second, 2));
}

void DetectionOutput::add(int view, int frame, int object, pair<int,int> position)
{
    bool collision = false;
    for (int i = 0; i < m_Positions[view][frame][object].size() && !collision; i++)
        collision = distance(m_Positions[view][frame][object][i], position) < m_Tol;
        
    if (!collision)
        m_Positions[view][frame][object].push_back(position);
}

void DetectionOutput::write(string path)
{
    // Draw set points
    for (int v = 0; v < m_NumOfViews; v++)
    {
        ofstream outFile;
        outFile.open(path, ios::out);
        
        // create two view files
        for (int f = 0; f < m_NumOfFrames; f++)
        {
            for (int o = 0; o < m_NumOfObjects; o++)
            {
                outFile << o << ":";
                
                vector< pair<int,int> > tmp = m_Positions[v][f][o];
                
                for (int p = 0; p < tmp.size(); p++)
                {
                    outFile << tmp[p].first << "," << tmp[p].first << ";";
                    
                } outFile << "\t";
            } outFile << endl;
        }
        outFile.close();
    }
    
    cout << "DetectionOutput written successfully to " << path << "." << endl;
}

void DetectionOutput::read(string path)
{
    vector< vector< vector< vector< pair<int,int> > > > > positions;
    
    // Draw set points
    for (int v = 0; v < m_NumOfViews; v++)
    {
        ifstream inFile;
        inFile.open(path, ios::in);

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
                    
                    pair<int,int> position(stoi(coordinates[0]), stoi(coordinates[1]));
                    m_Positions[v][f][oid].push_back( position );
                }
            }
            f++;
        }
    }
    
    cout << "DetectionOutput read successfully from " << path << "." << endl;
}




