#pragma once

#include "Reader.h"
#include "InteractiveRegisterer.h"
#include "BackgroundSubtractor.h"
#include "TableModeler.h"
#include "CloudjectModel.hpp"

using namespace std;

class Remedi
{
public:

	// Constructors
	Remedi(string parentDir);
	~Remedi();

	// Public methods
	void Run(bool display = false);

    // Public enums
    
    enum Interaction { PILLBOX = 0, DISH = 1, BOOK = 2, GLASS = 3 };
    enum Action { TAKINGPILL = 0, EATING = 1, READING = 2, DRINKING = 3 };
    
private:
	void modelBackground(Sequence::Ptr pSequence, BackgroundSubtractor& bs);
	void interactWithRegisterer(Sequence::Ptr pSequence, InteractiveRegisterer& registerer);
	void modelTablePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr pRegisteredCloudA,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr pRegisteredCloudB,
                          TableModeler& tableModeler);
    void loadCloudjectModels(string dir, int nObjects, int nObjectViews,
                             float normalRadius, float fpfhRadius,
                             vector< LFCloudjectModel<pcl::PointXYZ,pcl::FPFHSignature33> >& cloudjectModels);
    
    // Attributes
    string m_ParentDir;
};