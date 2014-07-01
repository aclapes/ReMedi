#pragma once

#include "Reader.h"
#include "InteractiveRegisterer.h"
#include "BackgroundSubtractor.h"
#include "TableModeler.h"
#include "CloudjectModel.hpp"

using namespace std;

class Remedi
{
    typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;
    
public:
	// Constructors
	Remedi();
	~Remedi();

	// Public methods
    void setInputDataPath(string dataPath);
    void setVisualization(bool enable = true);
    void setInteractiveRegistration(bool enable = true);
    void setInteractiveRegistrationParameters(int frameID, int numOfPoints);
    void setTableModeling(bool enable = true);
	void run();

    // Public enums
    
    enum Interaction { PILLBOX = 0, DISH = 1, BOOK = 2, GLASS = 3 };
    enum Action { TAKINGPILL = 0, EATING = 1, READING = 2, DRINKING = 3 };
    
private:
	void interactWithRegisterer(Sequence::Ptr pSequence, int fID, InteractiveRegisterer& registerer);
    void loadCloudjectModels(string dir, int nObjects, int nObjectViews,
                             float normalRadius, float fpfhRadius,
                             vector< LFCloudjectModel<pcl::PointXYZ,pcl::FPFHSignature33> >& cloudjectModels);
    
    // Attributes
    string m_DataPath;
    bool m_bVisualization;
    
    bool m_bRegistration;
    int m_RegistrationFrameID;
    int m_NumOfRegistrationPoints;
    bool m_bTableModeling;
};