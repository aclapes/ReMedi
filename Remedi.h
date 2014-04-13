#pragma once

#include "Reader.h"
#include "InteractiveRegisterer.h"
#include "BackgroundSubtractor.h"
#include "TableModeler.h"

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
};