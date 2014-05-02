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
	Remedi();
	~Remedi();

	// Public methods
	void Run();

private:
	void modelBackground(Reader reader, BackgroundSubtractor& bs);
	void interactWithRegisterer(Reader reader, InteractiveRegisterer& registerer);
	void modelTablePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr pRegisteredCloudA,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr pRegisteredCloudB,
                          TableModeler& tableModeler);
    void loadCloudjectModels(string dir, int nObjects, int nObjectViews,
                             float normalRadius, float fpfhRadius,
                             vector< LFCloudjectModel<pcl::PointXYZ,pcl::FPFHSignature33> >& cloudjectModels);
};