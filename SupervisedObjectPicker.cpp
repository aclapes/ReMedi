//
//  SupervisedObjectPicker.cpp
//  remedi
//
//  Created by Albert Clapés on 13/05/14.
//
//

#include "SupervisedObjectPicker.h"

#include <fstream>
#include <boost/algorithm/string.hpp>
//#include <boost/algorithm/string/split.hpp>
//#include <boost/algorithm/string/classification.hpp>

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
: m_NumOfViews(numOfViews), m_sid(sid), m_NumOfObjects(numOfObjects), m_Object(0)
{
    m_ParentDir = parentDir;
    
    m_ResY = 480;
    m_ResX = 640;
    
    m_X = 0;
    m_Y = 0;
    
    m_PushedStates.resize(m_NumOfViews, false);
    m_PushedFrames.resize(m_NumOfViews, -1);
//    m_Annotations.resize(m_NumOfViews);
    m_Positions.resize(m_NumOfViews);
    m_PositionsTmp.resize(m_NumOfViews);
    m_Presses.resize(m_NumOfViews);
}

cv::Mat SupervisedObjectPicker::getConcatMat()
{
    return m_ConcatMat;
}

int SupervisedObjectPicker::getResX()
{
    return m_ResX;
}

int SupervisedObjectPicker::getResY()
{
    return m_ResY;
}

void SupervisedObjectPicker::initializeAnnotations(int numOfObjects, int numOfFrames)
{
//    m_Annotations.resize(m_NumOfViews);
//    for (int i = 0; i < m_NumOfViews; i++)
//    {
//        m_Annotations[i].create(numOfFrames, numOfObjects, cv::DataType<unsigned char>::type);
//        m_Annotations[i].setTo(cv::Scalar(0));
//    }
    
    for (int i = 0; i < m_NumOfViews; i++)
    {
        m_Positions[i].resize(numOfFrames);
        for (int j = 0; j < numOfFrames; j++)
        {
            m_Positions[i][j].resize(numOfObjects);
        }
    }
    
    for (int i = 0; i < m_NumOfViews; i++)
    {
        m_PositionsTmp[i].resize(numOfObjects);
        m_Presses[i].resize(numOfObjects);
    }
}

void SupervisedObjectPicker::mark(int x, int y)
{
    int j = x / getResX();
    int i = y / getResY();
    int col = x % getResX();
    int row = y % getResY();
    
    int ptr = i * m_NumOfViews + j;

    
    vector<cv::Point>& tmp = m_PositionsTmp[ptr][m_Object];
    bool found = false;
    int idx;
    for (int i = 0; i < tmp.size() && !found; i++)
    {
        found = sqrt(pow(col - tmp[i].x, 2) + pow(row - tmp[i].y, 2)) < 20;
        if (found) idx = i;
    }
    
    if (!found)
    {
        m_PositionsTmp[ptr][m_Object].push_back(cv::Point(col,row));
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
            m_Positions[ptr][f][m_Object].push_back(m_PositionsTmp[ptr][m_Object][idx]);
        }
        
        m_PositionsTmp[ptr][m_Object].erase(m_PositionsTmp[ptr][m_Object].begin() + idx);
        m_Presses[ptr][m_Object].erase(m_Presses[ptr][m_Object].begin() + idx);
    }

//    if (m_PushedFrames[ptr] < 0)
//    {
//        m_PushedFrames[i * m_NumOfViews + j] = m_Reader.getColorFrameCounter();
//    }
//    else
//    {
//        int begin = m_PushedFrames[ptr];
//        int end = m_Reader.getColorFrameCounter();
//        
//        for (int i = begin; i <= end; i++)
//            m_Annotations[ptr].at<unsigned char>(i, m_Object) = 255;
//        
//        m_PushedFrames[ptr] = -1;
//    }
}


void SupervisedObjectPicker::draw(int x, int y)
{
    int j = x / getResX();
    int i = y / getResY();
    int col = x % getResX();
    int row = y % getResY();
    
    cv::Mat concatTmpMat = m_ConcatMat.clone();
    
    // Draw mouse-related
    
    cv::Scalar color (255.0 * g_Colors[m_Object][0], 255.0 * g_Colors[m_Object][1], 255.0 * g_Colors[m_Object][2]);
    cv::circle(concatTmpMat, cv::Point(x,y), 5, color, -1);
    
    string text = to_string(m_Object);
    int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 0.75;
    int thickness = 1.5;
    cv::Point textOrg(x,y);
    cv::putText(concatTmpMat, g_ModelNames[m_Object], cv::Point(x+fontScale,y+fontScale), fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
    
    int ptr = i * m_NumOfViews + j;
    
    // Draw set points
    for (int v = 0; v < m_NumOfViews; v++)
    {
        int vi = v / m_NumOfViews;
        int vj = v % m_NumOfViews;
        
        for (int o = 0; o < m_NumOfObjects; o++)
        {
            cv::Scalar color (255.0 * g_Colors[o][0], 255.0 * g_Colors[o][1], 255.0 * g_Colors[o][2]);
            vector<cv::Point> tmp = m_Positions[v][m_Reader.getColorFrameCounter()][o];
            for (int p = 0; p < tmp.size(); p++)
            {
                int coordX = vj*getResX()+tmp[p].x;
                int coordY = vi*getResY()+tmp[p].y;
                
                cv::circle(concatTmpMat, cv::Point(coordX,coordY), 5, color, -1);
            }
        }
    }
    
    // Draw setting points
//    cv::Scalar color (255.0 * g_Colors[m_Object][0], 255.0 * g_Colors[m_Object][1], 255.0 * g_Colors[m_Object][2]);
//
//    for (int v = 0; v < m_NumOfViews; v++)
//    {
//        for (int l = 0; l < m_PositionsTmp[v][m_Object].size(); l++)
//        {
//            int vi = v / m_NumOfViews;
//            int vj = v % m_NumOfViews;
//            int coordX = vj*getResX()+m_PositionsTmp[v][m_Object][l].x;
//            int coordY = vi*getResY()+m_PositionsTmp[v][m_Object][l].y;
//            cv::circle(concatTmpMat, cv::Point(coordX,coordY), 5, color, 1);
//        }
//    }
    
    for (int v = 0; v < m_NumOfViews; v++)
    {
        int vi = v / m_NumOfViews;
        int vj = v % m_NumOfViews;
        for (int o = 0; o < m_PositionsTmp[v].size(); o++)
        {
            cv::Scalar color (255.0 * g_Colors[o][0], 255.0 * g_Colors[o][1], 255.0 * g_Colors[o][2]);
            for (int p = 0; p < m_PositionsTmp[v][o].size(); p++)
            {
                int coordX = vj*getResX()+m_PositionsTmp[v][o][p].x;
                int coordY = vi*getResY()+m_PositionsTmp[v][o][p].y;
                cv::circle(concatTmpMat, cv::Point(coordX,coordY), 5, color, 1);
                cv::putText(concatTmpMat, g_ModelNames[o], cv::Point(coordX + fontScale, coordY + fontScale), fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
            }
        }
    }
    
    // Draw number of frame

    text = to_string(m_Reader.getColorFrameCounter());
    fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    fontScale = 1;
    thickness = 3;
    cv::putText(concatTmpMat, text, cv::Point(0, this->getResY() - 1), fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
    
    // Display all
    cv::imshow("Pick", concatTmpMat);
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
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
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
            
            for (int i = 0; i < m_PushedFrames.size(); i++)
                m_PushedFrames[i] = -1;
            
            cout << "Annoting label changed to " << m_Object << "." << endl;
        }
    }
}

void SupervisedObjectPicker::write()
{
    // Draw set points
    for (int v = 0; v < m_NumOfViews; v++)
    {
        ofstream outFile;
        string outPath(m_ParentDir + "Data/ObjectLabels/" + m_Reader.getSequenceDirName() + "_v" + to_string(v) + ".csv");
        outFile.open(outPath, ios::out);
        
        // create two view files
        for (int f = 0; f < m_Reader.getNumOfFrames(); f++)
        {
            for (int o = 0; o < m_NumOfObjects; o++)
            {
                outFile << o << ":";
                
                vector<cv::Point> tmp = m_Positions[v][f][o];
                
                for (int p = 0; p < tmp.size(); p++)
                {
                    outFile << tmp[p].x << "," << tmp[p].y << ";";
                    
                } outFile << "\t";
            } outFile << endl;
        }
        outFile.close();
    }
    
    cout << "Saved" << endl;
}

void SupervisedObjectPicker::read()
{
    // Draw set points
    for (int v = 0; v < m_NumOfViews; v++)
    {
        ifstream inFile;
        string inPath(m_ParentDir + "Data/ObjectLabels/" + m_Reader.getSequenceDirName() + "_v" + to_string(v) + ".csv");
        inFile.open(inPath, ios::in);
        
        // create two view files
        for (int f = 0; f < m_Reader.getNumOfFrames(); f++)
        {
            string line;
            getline(inFile, line);
            
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
//
                    m_Positions[v][f][oid].push_back(cv::Point(stoi(coordinates[0]), stoi(coordinates[1])));
                }
            }
        }
    }

    cout << "Loaded" << endl;
}

void SupervisedObjectPicker::run()
{
    cv::namedWindow("Pick");
    cv::setMouseCallback("Pick", SupervisedObjectPicker::mouseCallback, (void*) this);
    
    string sequencesPath = m_ParentDir + "Data/Sequences/";
    m_Reader.setData( sequencesPath, "Color1/", "Color2/", "Depth1/", "Depth2/" );
    
    m_Reader.setSequence(m_sid);
    initializeAnnotations(m_NumOfObjects, m_Reader.getNumOfFrames());
    
    bool bSuccess = m_Reader.nextColorPairedFrames(m_CurrentColorFrameA, m_CurrentColorFrameB);
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
                read();
                break;
            case 'k':
                write();
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
                cv::hconcat(m_CurrentColorFrameA.getMat(), m_CurrentColorFrameB.getMat(), m_ConcatMat);
                draw(m_X, m_Y);
            }

            c = cv::waitKey();
    }
}