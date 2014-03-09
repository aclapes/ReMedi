#include "Remedi.h"

#include "Monitorizer.h"
#include "MonitorizerParams.hpp"
#include "DepthFrame.h"
#include "TableModeler.h"
#include "Table.hpp"
#include <pcl/visualization/pcl_visualizer.h>

#ifdef _WIN32
std::string g_parentDir = "../";
#elif __APPLE__
std::string g_parentDir = "../../";
#endif

Remedi::Remedi()
{
	
}

void Remedi::Run()
{
	/*
	 * Load data
	 */

	// Create a reader pointing the data streams

    std::string dataDir = g_parentDir + "Data/";
    std::string backgroundDir = dataDir + "00_bs/";
    
    Reader reader( g_parentDir, "Color1/", "Color2/", "Depth1/", "Depth2/" );

    
	/*
	 * REGISTRATION (paired frames)
	 */

	InteractiveRegisterer registerer;
	interactWithRegisterer(registerer, reader);

	/*
	 * PLANE SEGMENTATION
	 */

	TableModeler tableModeler;
	modelTables(tableModeler, registerer, reader);
	
	
	/*
	 * BACKGROUND SUBTRACTION
	 */

	int nFrames, nMixtures; // BS parameters
	cout << "(BS) Number of frames: ";
	cin >> nFrames;
	cout << "(BS) Number of mixtures: ";
	cin >> nMixtures;
	
	BackgroundSubtractor bgSubtractor(nFrames, nMixtures);
	waitForBackgroundSubtraction(reader, bgSubtractor);
	
	MonitorizerParams monitorParams;
	monitorParams.tmpCoherence = 2;
	monitorParams.motionThresh = 5;
	monitorParams.leafSize = 0.01;
	monitorParams.posCorrespThresh = 0.07; // 7 cm
	Monitorizer monitorizer (&registerer, &tableModeler, monitorParams);

	/*
	 * Loop
	 */

	DepthFrame dFrameA, dFrameB;
	bool bSuccess = reader.getNextDepthPairedFrames(dFrameA, dFrameB); // able to read, not finished
	while (bSuccess)
	{
		DepthFrame fgDFrameA, fgDFrameB;
		bgSubtractor.subtract(dFrameA, dFrameB, fgDFrameA, fgDFrameB);

		monitorizer.monitor(fgDFrameA, fgDFrameB);

		monitorizer.handleCloudjectDrops(/*drops*/);

		bSuccess = reader.getNextDepthPairedFrames(dFrameA, dFrameB);
	}
}


void Remedi::waitForBackgroundSubtraction(Reader& reader, BackgroundSubtractor& bs)
{
	DepthFrame dFrameA, dFrameB;
	reader.getNextDepthPairedFrames(dFrameA, dFrameB);

	bs(dFrameA, dFrameB, 1);

	while (!bs.isReady())
	{
		bs(dFrameA, dFrameB, 0.02);

		reader.getNextDepthPairedFrames(dFrameA, dFrameB);
	}
}


void Remedi::interactWithRegisterer(InteractiveRegisterer& registerer, Reader& reader)
{
    ColorFrame cFrameA, cFrameB;
	DepthFrame dFrameA, dFrameB;

	// Ask wheter re-select a set of points
	// or use a previously computed transformation
	char c;
	std::cout << "Re-use alineation [Y/n]?" << std::endl;
	cin >> c;
	if (c != 'n' && c != 'N') // re-use transformation
	{
		bool success = registerer.loadTransformation(g_parentDir + "transformation.yml");
        if (!success) std::cerr << "Could not read transformation" << std::endl;
        // DEBUG
		//reader.getDepthPairedFrames(2, dFrameA, dFrameB);
		//registerer.visualizeRegistration(dFrameA, dFrameB);
	}
	else // re-select points
	{
		// Select the paired frames two register
		int fID;
		std::cout << "-> Select a frame ID: ";
		cin >> fID;

        reader.getColorPairedFrames(fID, cFrameA, cFrameB);
		reader.getDepthPairedFrames(fID, dFrameA, dFrameB);
        
		// Select the number of points to compute a rigid transformation
		int nPoints;
		std::cout << "-> Select the number of points: ";
		cin >> nPoints;

		registerer.setNumPoints(nPoints);
		registerer.setManualCorrespondences(cFrameA, cFrameB, dFrameA, dFrameB);
		
        registerer.computeTransformation(); // find the transformation
        registerer.saveTransformation(g_parentDir + "transformation.yml");
	}
}


void Remedi::modelTables(TableModeler& tableModeler, InteractiveRegisterer& registerer, Reader& reader)
{
	DepthFrame dFrameA, dFrameB;
	reader.getDepthPairedFrames(2, dFrameA, dFrameB);

	pcl::PointCloud<pcl::PointXYZ>::Ptr rCloudA (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr rCloudB (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tableA (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tableB (new pcl::PointCloud<pcl::PointXYZ>);

	registerer.getRegisteredClouds(dFrameA, dFrameB, *rCloudA, *rCloudB, true, false);

	tableModeler.setInputClouds(rCloudA, rCloudB);
	tableModeler.setLeafSize(0.02); // 2cm
	tableModeler.setNormalRadius(0.05);
	tableModeler.setSACIters(200);
	tableModeler.setSACDistThresh(0.03);
	tableModeler.setYOffset(0.4);
    
	tableModeler.model(*tableA, *tableB);
}


Remedi::~Remedi()
{

}