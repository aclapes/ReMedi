//
//  SupervisedObjectPicker.cpp
//  remedi
//
//  Created by Albert Clapés on 13/05/14.
//
//

#include "SupervisedObjectPicker.h"
#include "DetectionOutput.h"

#include <fstream>
#include <boost/algorithm/string.hpp>
//#include <boost/algorithm/string/split.hpp>
//#include <boost/algorithm/string/classification.hpp>

#include <pcl/point_types.h>

#include "conversion.h"

using namespace std;

int g_Colors[][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
    {1, 1, 0},
    {1, 0, 1},
    {0, 1, 1},
    {1, .5, 0},
    {1, 0, .5},
    {0, 1, .5},
    {0, .5, 1},
    {.5, 1, 0}
};

string g_ModelNames[] =
{
    "dish", "pillbox", "book", "tetrabrick", "glass"
};

SupervisedObjectPicker::SupervisedObjectPicker(string parentDir, int sid,
                                               int numOfViews, int numOfObjects)
: m_sid(sid), m_NumOfViews(numOfViews), m_NumOfObjects(numOfObjects), m_Object(0), m_Tol(0.05)
{
    m_ParentDir = parentDir;
    string sequencesPath = m_ParentDir + "Data/Sequences/";
    m_Reader.setData( sequencesPath, "Color1/", "Color2/", "Depth1/", "Depth2/" );
    m_Reader.setSequence(m_sid);
    m_NumOfFrames = m_Reader.getNumOfFrames();
    
    m_ResY = 480;
    m_ResX = 640;
    
    m_X = 0;
    m_Y = 0;
    
    m_Positions.resize(m_NumOfViews);
    m_ClickedPositions.resize(m_NumOfViews);
    m_Presses.resize(m_NumOfViews);
    
    for (int i = 0; i < m_NumOfViews; i++)
    {
        m_Positions[i].resize(m_NumOfFrames);
        for (int j = 0; j < m_NumOfFrames; j++)
            m_Positions[i][j].resize(m_NumOfObjects);
    }
    
    for (int i = 0; i < m_NumOfViews; i++)
    {
        m_ClickedPositions[i].resize(m_NumOfObjects);
        m_Presses[i].resize(m_NumOfObjects);
    }
    
    m_DOutput = DetectionOutput(m_NumOfViews, m_NumOfFrames, m_NumOfObjects, m_Tol);
}

cv::Mat SupervisedObjectPicker::getConcatColor()
{
    return m_ConcatColor;
}

cv::Mat SupervisedObjectPicker::getConcatDepth()
{
    return m_ConcatDepth;
}

int SupervisedObjectPicker::getResX()
{
    return m_ResX;
}

int SupervisedObjectPicker::getResY()
{
    return m_ResY;
}

void SupervisedObjectPicker::mark(int wx, int wy)
{
    // View coordinates in window's views
    int j = wx / getResX();
    int i = wy / getResY();
    // Mouse (x,y) coordinates in (i,j) view
    int x = wx % getResX();
    int y = wy % getResY();
    // View
    int ptr = i * m_NumOfViews + j;
    
    if (m_ConcatDepth.at<unsigned short>(wy,wx) == 0)
        return;
    
    int idx;
    bool found = false;
    for (int i = 0; i < m_ClickedPositions[ptr][m_Object].size() && !found; i++)
    {
        found = sqrtf( powf(x - m_ClickedPositions[ptr][m_Object][i].x, 2)
                      + powf(y - m_ClickedPositions[ptr][m_Object][i].y, 2) ) < 20;
        if (found) idx = i;
    }
    
    if (!found)
    {
        m_ClickedPositions[ptr][m_Object].push_back( pcl::PointXYZ(x, y, m_ConcatDepth.at<unsigned short>(wx,wy)) );
        m_Presses[ptr][m_Object].push_back(m_Reader.getColorFrameCounter());
    }
    else
    {
        int begin, end;
        if (m_Presses[ptr][m_Object][idx] <= m_Reader.getColorFrameCounter())
        {
            begin = m_Presses[ptr][m_Object][idx];
            end = m_Reader.getColorFrameCounter();
        }
        else
        {
            begin = m_Reader.getColorFrameCounter();
            end = m_Presses[ptr][m_Object][idx];
        }
        
        for (int f = begin; f <= end; f++)
        {
            pcl::PointXYZ pp = m_ClickedPositions[ptr][m_Object][idx];
            m_Positions[ptr][f][m_Object].push_back(pp);
            
            pcl::PointXYZ rwp;
            ProjectiveToRealworld(pp, getResX(), getResY(), rwp);
            m_DOutput.add(ptr, f, m_Object, rwp);
        }
        
        m_ClickedPositions[ptr][m_Object].erase(m_ClickedPositions[ptr][m_Object].begin() + idx);
        m_Presses[ptr][m_Object].erase(m_Presses[ptr][m_Object].begin() + idx);
    }
}

void SupervisedObjectPicker::remove(int wx, int wy)
{
    // View coordinates in window's views
    int j = wx / getResX();
    int i = wy / getResY();
    // Mouse (x,y) coordinates in (i,j) view
    int x = wx % getResX();
    int y = wy % getResY();
    
    int ptr = i * m_NumOfViews + j;
    bool found = false;
    pcl::PointXYZ point;
    for (int i = 0; i < m_Positions[ptr][m_Reader.getColorFrameCounter()][m_Object].size() && !found; i++)
    {
        found = sqrtf( powf(x - m_Positions[ptr][m_Reader.getColorFrameCounter()][m_Object][i].x, 2)
                      + powf(y - m_Positions[ptr][m_Reader.getColorFrameCounter()][m_Object][i].y, 2) ) < 20;
        if (found)
            point = m_Positions[ptr][m_Reader.getColorFrameCounter()][m_Object][i];
    }
    
    for (int f = 0; f < m_Reader.getNumOfFrames(); f++)
    {
        bool found = false;
        for (int i = 0; i < m_Positions[ptr][f][m_Object].size() && !found; i++)
        {
            found = m_Positions[ptr][f][m_Object][i].x == point.x && m_Positions[ptr][f][m_Object][i].y == point.y;
            if (found)
            {
//                    pcl::PointXYZ rwp;
//                    ProjectiveToRealworld(m_Positions[ptr][f][m_Object][i], getResX(), getResY(), rwp);
//                    m_DOutput.remove(ptr, f, m_Object, rwp);
                m_DOutput.remove(ptr, f, m_Object, i);
                m_Positions[ptr][f][m_Object].erase(m_Positions[ptr][f][m_Object].begin() + i);
            }
        }
    }
}


void SupervisedObjectPicker::mark(DetectionOutput dout)
{
    for (int v = 0; v < dout.getNumOfViews(); v++)
    {
        m_Positions[v].resize(dout.getNumOfFrames());
        for (int f = 0; f < dout.getNumOfFrames(); f++)
        {
            m_Positions[v][f].resize(dout.getNumOfObjects());
        }
    }
    
    // Draw set points
    for (int v = 0; v < dout.getNumOfViews(); v++)
        for (int f = 0; f < dout.getNumOfFrames(); f++)
            for (int o = 0; o < dout.getNumOfObjects(); o++)
            {
                vector<pcl::PointXYZ> points;
                dout.get(v, f, o, points);
                
                for (int p = 0; p < points.size(); p++)
                {
                    pcl::PointXYZ pp;
                    RealworldToProjective(points[p], getResX(), getResY(), pp);
                    m_Positions[v][f][o].push_back( pp );
                }
            }
}

void SupervisedObjectPicker::draw(int wx, int wy)
{
    // View coordinates in window's views
    int j = wx / getResX();
    int i = wy / getResY();
    // Mouse (x,y) coordinates in (i,j) view
    int x = wx % getResX();
    int y = wy % getResY();
    
    cv::Mat concatColorTmp = m_ConcatColor.clone();
    
    int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 0.75;
    int thickness = 1.5;
    
    
    // Draw set points
    int ptr = i * m_NumOfViews + j;
    
    for (int v = 0; v < m_NumOfViews; v++)
    {
        int vi = v / m_NumOfViews;
        int vj = v % m_NumOfViews;
        
        for (int o = 0; o < m_NumOfObjects; o++)
        {
            cv::Scalar color (255.0 * g_Colors[o][0], 255.0 * g_Colors[o][1], 255.0 * g_Colors[o][2]);
            vector<pcl::PointXYZ> tmp = m_Positions[v][m_Reader.getColorFrameCounter()][o];
            for (int p = 0; p < tmp.size(); p++)
            {
                int coordX = vj*getResX()+tmp[p].x;
                int coordY = vi*getResY()+tmp[p].y;
                
                cv::circle(concatColorTmp, cv::Point(coordX,coordY), 5, color, -1);
            }
        }
    }
    
    // Draw setting points
//    cv::Scalar color (255.0 * g_Colors[m_Object][0], 255.0 * g_Colors[m_Object][1], 255.0 * g_Colors[m_Object][2]);
//
//    for (int v = 0; v < m_NumOfViews; v++)
//    {
//        for (int l = 0; l < m_ClickedPositions[v][m_Object].size(); l++)
//        {
//            int vi = v / m_NumOfViews;
//            int vj = v % m_NumOfViews;
//            int coordX = vj*getResX()+m_ClickedPositions[v][m_Object][l].x;
//            int coordY = vi*getResY()+m_ClickedPositions[v][m_Object][l].y;
//            cv::circle(concatColorTmp, cv::Point(coordX,coordY), 5, color, 1);
//        }
//    }
    

    bool mouseNearToMark = false;
    
    for (int v = 0; v < m_NumOfViews; v++)
    {
        int vi = v / m_NumOfViews;
        int vj = v % m_NumOfViews;
        for (int o = 0; o < m_ClickedPositions[v].size(); o++)
        {
            cv::Scalar color (255.0 * g_Colors[o][0], 255.0 * g_Colors[o][1], 255.0 * g_Colors[o][2]);
            for (int p = 0; p < m_ClickedPositions[v][o].size(); p++)
            {
                int coordX = vj*getResX()+m_ClickedPositions[v][o][p].x;
                int coordY = vi*getResY()+m_ClickedPositions[v][o][p].y;
                
                pcl::PointXYZ projMouse(wx,wy, m_ConcatDepth.at<unsigned short>(wy,wx));
                pcl::PointXYZ projMark(coordX,coordY, m_ConcatDepth.at<unsigned short>(coordY,coordX));
                
                pcl::PointXYZ rwMouse, rwMark;
                ProjectiveToRealworld(projMouse, getResX(), getResY(), rwMouse);
                ProjectiveToRealworld(projMark, getResX(), getResY(), rwMark);
                float distance = sqrtf(  pow(rwMouse.x - rwMark.x, 2)
                                       + pow(rwMouse.y - rwMark.y, 2)
                                       + pow(rwMouse.z - rwMark.z, 2) );
                
                if ( o == m_Object && distance < m_Tol )
                {
                    cv::circle(concatColorTmp, cv::Point(coordX,coordY), 5, color, -1);
                    mouseNearToMark = true;
                }
                else
                {
                    cv::circle(concatColorTmp, cv::Point(coordX,coordY), 5, color, 2);
                }

                cv::putText(concatColorTmp, g_ModelNames[o], cv::Point(coordX + fontScale, coordY + fontScale), fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
            }
        }
    }
    
    if (m_ConcatDepth.at<unsigned short>(wy,wx) > 0)
    {
        // Draw mouse pointer
        
        cv::Scalar color (255.0 * g_Colors[m_Object][0], 255.0 * g_Colors[m_Object][1], 255.0 * g_Colors[m_Object][2]);
        cv::circle(concatColorTmp, cv::Point(wx,wy), 5, cv::Scalar(255,255,255), -1);
        cv::circle(concatColorTmp, cv::Point(wx,wy), 3, color, -1);
        
        if (!mouseNearToMark)
        {
            string text = to_string(m_Object);
            cv::Point textOrg(wx,wy);
            cv::putText(concatColorTmp, g_ModelNames[m_Object],
                        cv::Point(wx + fontScale, wy + fontScale),
                        fontFace, fontScale,
                        cv::Scalar::all(255), thickness, 8);
        }
    }
    
    // Draw number of frame

    string text = to_string(m_Reader.getColorFrameCounter());
    fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    fontScale = 1;
    thickness = 3;
    cv::putText(concatColorTmp, text, cv::Point(0, this->getResY() - 1), fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
    
    // Draw top rectangle
    int progressBarHeight = 7;
    cv::rectangle(concatColorTmp, cv::Point(0,0), cv::Point(m_ConcatColor.cols, progressBarHeight), cv::Scalar(0,0,0));
    float percentage = ((float) m_Reader.getColorFrameCounter()) / m_Reader.getNumOfFrames();
    cv::rectangle(concatColorTmp, cv::Point(0,0), cv::Point(m_ConcatColor.cols * percentage, progressBarHeight), cv::Scalar(50, 255, 50), -1);
    
    // Display all
    cv::imshow("Pick", concatColorTmp);
}

void SupervisedObjectPicker::mouseCallback(int event, int x, int y, int flags, void* userdata)
{
    SupervisedObjectPicker* _this = (SupervisedObjectPicker*) userdata;
    
    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        _this->mark(x,y);
        _this->draw(x,y);
        
//        cout << "Left button of the mouse is clicked - position (" << viewX << ", " << viewY << ") - frame (" << _this->m_Reader.getColorFrameCounter() << ")" << endl;
    }
    else if  ( event == cv::EVENT_RBUTTONDOWN )
    {
        _this->remove(x,y);
//        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        // DISABLED
        //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
//        cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
        
        _this->draw(x,y);
        
        _this->m_X = x;
        _this->m_Y = y;
    }
}

void SupervisedObjectPicker::keyboardHandler(int key)
{
    if (key >= '0' && key <= '9')
    {
        int object = key - '0';
        if (object >= 0 && object < m_NumOfObjects)
        {
            m_Object = object;
            
            cout << "Annoting label changed to " << m_Object << "." << endl;
        }
    }
}

//void SupervisedObjectPicker::write()
//{
//    // Draw set points
//    for (int v = 0; v < m_NumOfViews; v++)
//    {
//        ofstream outFile;
//        string outPath(m_ParentDir + "Data/ObjectLabels/" + m_Reader.getSequenceDirName() + "_v" + to_string(v) + ".csv");
//        outFile.open(outPath, ios::out);
//        
//        // create two view files
//        for (int f = 0; f < m_Reader.getNumOfFrames(); f++)
//        {
//            for (int o = 0; o < m_NumOfObjects; o++)
//            {
//                outFile << o << ":";
//                
//                vector<cv::Point> tmp = m_Positions[v][f][o];
//                
//                for (int p = 0; p < tmp.size(); p++)
//                {
//                    outFile << tmp[p].x << "," << tmp[p].y << ";";
//                    
//                } outFile << "\t";
//            } outFile << endl;
//        }
//        outFile.close();
//    }
//    
//    cout << "Saved" << endl;
//}
//
//void SupervisedObjectPicker::read()
//{
//    // Draw set points
//    for (int v = 0; v < m_NumOfViews; v++)
//    {
//        ifstream inFile;
//        string inPath(m_ParentDir + "Data/ObjectLabels/" + m_Reader.getSequenceDirName() + "_v" + to_string(v) + ".csv");
//        inFile.open(inPath, ios::in);
//        
//        // create two view files
//        for (int f = 0; f < m_Reader.getNumOfFrames(); f++)
//        {
//            string line;
//            getline(inFile, line);
//            
//            vector<string> objects_sublines; // ex: 1:202,22;104,123;
//            boost::split(objects_sublines, line, boost::is_any_of("\t"));
//
//            for (int o = 0; o < objects_sublines.size() - 1; o++)
//            {
//                vector<string> object_struct;
//                boost::split(object_struct, objects_sublines[o], boost::is_any_of(":"));
//
//                int oid = stoi(object_struct[0]); // object id
//                if (object_struct[1].size() <= 2)
//                    continue;
//                
//                vector<string> positions;
//                boost::split(positions, object_struct[1], boost::is_any_of(";")); // object id's positions
//                
//                for (int p = 0; p < positions.size() - 1; p++)
//                {
//                    vector<string> coordinates;
//                    boost::split(coordinates, positions[p], boost::is_any_of(","));
////
//                    m_Positions[v][f][oid].push_back(cv::Point(stoi(coordinates[0]), stoi(coordinates[1])));
//                }
//            }
//        }
//    }
//
//    cout << "Loaded" << endl;
//}

void SupervisedObjectPicker::run()
{
    cv::namedWindow("Pick");
    cv::setMouseCallback("Pick", SupervisedObjectPicker::mouseCallback, (void*) this);
    
    bool bSuccess = m_Reader.nextColorPairedFrames(m_CurrentColorFrameA, m_CurrentColorFrameB);
    m_Reader.nextDepthPairedFrames(m_CurrentDepthFrameA, m_CurrentDepthFrameB);
    int c = -1;
    bool quit = false;
    while (!quit)
    {
        switch (c)
        {
            case 'a':
                bSuccess = m_Reader.previousColorPairedFrames(m_CurrentColorFrameA, m_CurrentColorFrameB);
                break;
            case 'd':
                bSuccess = m_Reader.nextColorPairedFrames(m_CurrentColorFrameA, m_CurrentColorFrameB);
                break;
            case 'A':
                bSuccess = m_Reader.previousColorPairedFrames(m_CurrentColorFrameA, m_CurrentColorFrameB, 10);
                break;
            case 'D':
                bSuccess = m_Reader.nextColorPairedFrames(m_CurrentColorFrameA, m_CurrentColorFrameB, 10);
                break;
            case 'l':
                m_DOutput.read(m_ParentDir + "Data/ObjectLabels/", m_Reader.getSequenceDirName(), "csv");
                mark(m_DOutput);
                break;
            case 'k':
                m_DOutput.write(m_ParentDir + "Data/ObjectLabels/", m_Reader.getSequenceDirName(), "csv");
                break;
            case 27:
                exit(0);
                break;
            default:
                keyboardHandler(c);
                break;
        }
        
            if (bSuccess)
            {
                cv::hconcat(m_CurrentColorFrameA.getMat(), m_CurrentColorFrameB.getMat(), m_ConcatColor);
                cv::hconcat(m_CurrentDepthFrameA.getDepthMap(), m_CurrentDepthFrameB.getDepthMap(), m_ConcatDepth);
                draw(m_X, m_Y);
            }

            c = cv::waitKey();
    }
}