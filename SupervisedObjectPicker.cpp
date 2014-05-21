//
//  SupervisedObjectPicker.cpp
//  remedi
//
//  Created by Albert Clapés on 13/05/14.
//
//

#include "SupervisedObjectPicker.h"
#include "DetectionOutput.h"

#include <vector>
#include <string>
#include <fstream>
#include <boost/algorithm/string.hpp>
//#include <boost/algorithm/string/split.hpp>
//#include <boost/algorithm/string/classification.hpp>

#include <pcl/point_types.h>

#include "conversion.h"
#include <boost/assign/std/vector.hpp>

#include "Sequence.h"

using namespace boost::assign;
using namespace std;

// Object names
string g_ModelNames[] =
{
    "dish", "pillbox", "book", "tetrabrick", "glass"
};

// Marker colors (as many as objects at least)
int g_Colors[][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
    {1, 1, 0},
    {1, 0, 1}//,
//    {0, 1, 1},
//    {1, 1, 1},
//    {0, 0, 0},
//    {1, .5, 0},
//    {1, 0, .5},
//    {.5, 1, 0},
//    {0, 1, .5},
//    {.5, 0, 1},
//    {0, .5, 1},
//    {.5, 1, 0}
};


SupervisedObjectPicker::SupervisedObjectPicker(string objectsDir,
                                               int numOfObjects)
: m_ObjectsDir(objectsDir), m_NumOfObjects(numOfObjects),
 m_Object(0), m_Tol(0.05), m_X(0), m_Y(0)
{

}

void SupervisedObjectPicker::setViewsDisplay(int h, int w)
{
    m_H = h;
    m_W = w;
}

void SupervisedObjectPicker::setSequence(Sequence::Ptr pSequence)
{
    m_pSequence = pSequence;
    
    m_ResY = m_pSequence->getColorFrame(0)[0].getResY();
    m_ResX = m_pSequence->getColorFrame(0)[0].getResX();
    
    m_NumOfViews    = m_pSequence->getNumOfViews();
    m_NumOfFrames   = m_pSequence->getNumOfFrames();
    
    m_Positions.resize(m_NumOfViews);
    m_ClickedPositions.resize(m_NumOfViews);
    m_Presses.resize(m_NumOfViews);
    
    for (int i = 0; i < m_NumOfViews; i++)
    {
        m_Positions[i].resize(m_NumOfFrames[i]);
        for (int j = 0; j < m_NumOfFrames[i]; j++)
            m_Positions[i][j].resize(m_NumOfObjects);
    }
    
    for (int i = 0; i < m_NumOfViews; i++)
    {
        m_ClickedPositions[i].resize(m_NumOfObjects);
        m_Presses[i].resize(m_NumOfObjects);
    }
    
    m_DOutput = DetectionOutput(m_NumOfViews, m_NumOfFrames, m_NumOfObjects, m_Tol);
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
    // view coordinates in window's views
    int j = wx / getResX();
    int i = wy / getResY();
    // mouse (x,y) coordinates in (i,j) view
    int x = wx % getResX();
    int y = wy % getResY();
    // view
    int ptr = i * m_NumOfViews + j;
    
    // error in depth measurement
    unsigned short depth;
    if ( (depth = m_DepthViewsFrame.at<unsigned short>(wy,wx)) == 0)
        return;
    
    int idx;
    bool found = false;
    for (int i = 0; i < m_ClickedPositions[ptr][m_Object].size() && !found; i++)
    {
        found = sqrtf( powf(x - m_ClickedPositions[ptr][m_Object][i].x, 2)
                      + powf(y - m_ClickedPositions[ptr][m_Object][i].y, 2)
                      + powf(depth - m_ClickedPositions[ptr][m_Object][i].z, 2)) < 0.05;
        if (found) idx = i;
    }
    
    if (!found)
    {
        m_ClickedPositions[ptr][m_Object].push_back( pcl::PointXYZ(x, y, m_DepthViewsFrame.at<unsigned short>(wx,wy)) );
        m_Presses[ptr][m_Object].push_back(m_pSequence->colorAt()[ptr]);
    }
    else
    {
        int begin, end;
        if (m_Presses[ptr][m_Object][idx] <= m_pSequence->colorAt()[ptr])
        {
            begin = m_Presses[ptr][m_Object][idx];
            end = m_pSequence->colorAt()[ptr];
        }
        else
        {
            begin = m_pSequence->colorAt()[ptr];
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
    // view coordinates in window's views
    int j = wx / getResX();
    int i = wy / getResY();
    // mouse (x,y) coordinates in (i,j) view
    int x = wx % getResX();
    int y = wy % getResY();
    // view
    int ptr = i * m_NumOfViews + j;
    
    unsigned short depth = m_DepthViewsFrame.at<unsigned short>(wy,wx);
    
    bool found = false;
    pcl::PointXYZ point;
    for (int i = 0; i < m_Positions[ptr][m_pSequence->colorAt()[ptr]][m_Object].size() && !found; i++)
    {
        found = sqrtf( powf(x - m_Positions[ptr][m_pSequence->colorAt()[ptr]][m_Object][i].x, 2)
                      + powf(y - m_Positions[ptr][m_pSequence->colorAt()[ptr]][m_Object][i].y, 2)
                      + powf(depth - m_Positions[ptr][m_pSequence->colorAt()[ptr]][m_Object][i].z, 2)) < m_Tol;
        if (found)
            point = m_Positions[ptr][m_pSequence->colorAt()[ptr]][m_Object][i];
    }
    
    for (int f = 0; f < m_pSequence->getNumOfFrames()[ptr]; f++)
    {
        bool found = false;
        for (int i = 0; i < !found && m_Positions[ptr][f][m_Object].size(); i++)
        {
            found = m_Positions[ptr][f][m_Object][i].x == point.x && m_Positions[ptr][f][m_Object][i].y == point.y;
            if (found)
            {
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
        m_Positions[v].resize(dout.getNumOfFrames()[v]);
        for (int f = 0; f < dout.getNumOfFrames()[v]; f++)
        {
            m_Positions[v][f].resize(dout.getNumOfObjects());
        }
    }
    
    // Draw set points
    for (int v = 0; v < dout.getNumOfViews(); v++)
        for (int f = 0; f < dout.getNumOfFrames()[v]; f++)
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
    
    cv::Mat colorViewsFrame = m_ColorViewsFrame.clone();
    
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
            vector<pcl::PointXYZ> tmp = m_Positions[v][m_pSequence->colorAt()[v]][o];
            for (int p = 0; p < tmp.size(); p++)
            {
                int coordX = vj*getResX()+tmp[p].x;
                int coordY = vi*getResY()+tmp[p].y;
                
                cv::circle(colorViewsFrame, cv::Point(coordX,coordY), 5, color, -1);
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
//            cv::circle(colorViewsFrame, cv::Point(coordX,coordY), 5, color, 1);
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
                
                pcl::PointXYZ projMouse(wx,wy, m_DepthViewsFrame.at<unsigned short>(wy,wx));
                pcl::PointXYZ projMark(coordX,coordY, m_DepthViewsFrame.at<unsigned short>(coordY,coordX));
                
                pcl::PointXYZ rwMouse, rwMark;
                ProjectiveToRealworld(projMouse, getResX(), getResY(), rwMouse);
                ProjectiveToRealworld(projMark, getResX(), getResY(), rwMark);
                float distance = sqrtf(  pow(rwMouse.x - rwMark.x, 2)
                                       + pow(rwMouse.y - rwMark.y, 2)
                                       + pow(rwMouse.z - rwMark.z, 2) );
                
                if ( o == m_Object && distance < m_Tol )
                {
                    cv::circle(colorViewsFrame, cv::Point(coordX,coordY), 5, color, -1);
                    mouseNearToMark = true;
                }
                else
                {
                    cv::circle(colorViewsFrame, cv::Point(coordX,coordY), 5, color, 2);
                }

                cv::putText(colorViewsFrame, g_ModelNames[o], cv::Point(coordX + fontScale, coordY + fontScale), fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
            }
        }
    }
    
    if (m_DepthViewsFrame.at<unsigned short>(wy,wx) > 0)
    {
        // Draw mouse pointer
        
        cv::Scalar color (255.0 * g_Colors[m_Object][0], 255.0 * g_Colors[m_Object][1], 255.0 * g_Colors[m_Object][2]);
        cv::circle(colorViewsFrame, cv::Point(wx,wy), 5, cv::Scalar(255,255,255), -1);
        cv::circle(colorViewsFrame, cv::Point(wx,wy), 3, color, -1);
        
        if (!mouseNearToMark)
        {
            string text = to_string(m_Object);
            cv::Point textOrg(wx,wy);
            cv::putText(colorViewsFrame, g_ModelNames[m_Object],
                        cv::Point(wx + fontScale, wy + fontScale),
                        fontFace, fontScale,
                        cv::Scalar::all(255), thickness, 8);
        }
    }

    
    int progressBarHeight = 7;
    
    // Draw number of frame
    fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    fontScale = 1;
    thickness = 3;
    int baseline;
    cv::Size textSize;
    for (int i = 0; i < m_H; i++) for (int j = 0; j < m_W; j++)
    {
        string sid = to_string(m_pSequence->colorAt()[i * m_W + j]);
        textSize = cv::getTextSize(sid, fontFace, fontScale, 3, &baseline);
        cv::rectangle(colorViewsFrame,
                      cv::Point(j * getResX(), i * getResY() + progressBarHeight),
                      cv::Point(j * getResX() + textSize.width + 2*thickness, i * getResY() + progressBarHeight + textSize.height + 2*thickness),
                      cv::Scalar(0,0,0), CV_FILLED);
        cv::putText(colorViewsFrame,
                    sid,
                    cv::Point(j * getResX() + thickness, i * getResY() + progressBarHeight + textSize.height + thickness),
                    fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
    }
    
    // Draw top rectangles
    float percentage;
    
    for (int i = 0; i < m_H; i++) for (int j = 0; j < m_W; j++)
    {
        cv::rectangle(colorViewsFrame,
                      cv::Point(j * getResX(), i * getResY()),
                      cv::Point((j+1) * getResX() - 1, i * getResY() + progressBarHeight),
                      cv::Scalar(0,0,0));
        
        percentage = ((float) m_pSequence->colorAt()[i * getResX() + j]) / m_pSequence->getNumOfFrames()[i * getResX() + j];
        
        cv::rectangle(colorViewsFrame,
                      cv::Point(j * getResX(), i * getResY()),
                      cv::Point(j * getResX() - 1 + getResX() * percentage, i * getResY() + progressBarHeight),
                      cv::Scalar(50, 255, 50),
                      -1);
    }
    
    // Display all
    cv::imshow("Pick", colorViewsFrame);
}

void SupervisedObjectPicker::mouseCallback(int event, int x, int y, int flags, void* userdata)
{
    SupervisedObjectPicker* _this = (SupervisedObjectPicker*) userdata;
    
    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        _this->mark(x,y);
        _this->draw(x,y);
    }
    else if  ( event == cv::EVENT_RBUTTONDOWN )
    {
        _this->remove(x,y);
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        // DISABLED
    }
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
        _this->draw(x,y);
        
        _this->m_X = x;
        _this->m_Y = y;
    }
}

void SupervisedObjectPicker::modelHandler(int key)
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

void SupervisedObjectPicker::nextFrames(vector<cv::Mat>& colorFrames,
                                        vector<cv::Mat>& depthFrames,
                                        int step)
{
    if (m_pSequence->hasNextColorFrame(step))
        m_pSequence->nextColorFrame(colorFrames, step);
    
    if (m_pSequence->hasNextDepthFrame(step))
        m_pSequence->nextDepthFrame(depthFrames, step);
}

void SupervisedObjectPicker::prevFrames(vector<cv::Mat>& colorFrames,
                                        vector<cv::Mat>& depthFrames,
                                        int step)
{
    if (m_pSequence->hasPreviousColorFrame(step))
        m_pSequence->previousColorFrame(colorFrames, step);
    
    if (m_pSequence->hasPreviousDepthFrame(step))
        m_pSequence->previousDepthFrame(depthFrames, step);
}

void SupervisedObjectPicker::concatenateViews(vector<cv::Mat> views, int h, int w,
                                              cv::Mat& viewsFrame)
{
    assert (views.size() == h * w);
    
    viewsFrame.release();
    
    viewsFrame.create(0, views[0].cols * w, views[0].type());
    for (int i = 0; i < h; i++)
    {
        cv::Mat rowViewsMat = views[i * w + 0];
        for (int j = 1; j < w; j++)
            cv::hconcat(rowViewsMat, views[i * w + j], rowViewsMat);
        
        viewsFrame.push_back(rowViewsMat);
    }
}

void SupervisedObjectPicker::run()
{
    cv::namedWindow("Pick");
    cv::setMouseCallback("Pick", SupervisedObjectPicker::mouseCallback, (void*) this);
    
    nextFrames(m_CurrentColorFrames, m_CurrentDepthFrames);
    
    int c;
    bool quit = false;
    while (!quit)
    {
        switch (c)
        {
            case 'a':
                prevFrames(m_CurrentColorFrames, m_CurrentDepthFrames);
                break;
            case 'd':
                nextFrames(m_CurrentColorFrames, m_CurrentDepthFrames);
                break;
            case 'A':
                prevFrames(m_CurrentColorFrames, m_CurrentDepthFrames, 10);
                break;
            case 'D':
                nextFrames(m_CurrentColorFrames, m_CurrentDepthFrames, 10);
                break;
            case 'l':
                m_DOutput.read(m_ObjectsDir, m_pSequence->getName(), "csv");
                mark(m_DOutput);
                break;
            case 'k':
                m_DOutput.write(m_ObjectsDir, m_pSequence->getName(), "csv");
                break;
            case 27:
                exit(0);
                break;
            default:
                modelHandler(c);
                break;
        }
        
        concatenateViews(m_CurrentColorFrames, m_H, m_W, m_ColorViewsFrame);
        concatenateViews(m_CurrentDepthFrames, m_H, m_W, m_DepthViewsFrame);

        draw(m_X, m_Y);

        c = cv::waitKey();
    }
}