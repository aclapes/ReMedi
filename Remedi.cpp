#include "Remedi.h"

#include "Monitorizer.h"
#include "MonitorizerParams.hpp"
#include "DepthFrame.h"
#include "TableModeler.h"
#include "Table.hpp"
#include "CloudjectModel.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/assign/std/vector.hpp>

using namespace boost::assign;

Remedi::Remedi(string parentDir)
{
	m_ParentDir = parentDir;
}

void Remedi::Run(bool display)
{
	/*
	 * Load data
	 */

	// Create a reader pointing the data streams

    string sequencesPath = m_ParentDir + "Data/Sequences/";
    vector<string> colorDirs = "Color1/", "Color2/";
    vector<string> depthDirs = "Depth1/", "Depth2/";
    string labelsPath = m_ParentDir + "Data/Labels/observer_1/csv/";

    Reader reader( sequencesPath, colorDirs, dephtDirs, labelsPath );
    
	/*
	 * REGISTRATION (paired frames)
	 */

	InteractiveRegisterer registerer;
	interactWithRegisterer(reader, registerer);

	/*
	 * PLANE SEGMENTATION
	 */

	TableModeler tableModeler;
	modelTablePlanes(registerer.getRegisteredClouds().first,
                      registerer.getRegisteredClouds().second,
                      tableModeler);
	
	/*
	 * BACKGROUND SUBTRACTION
	 */

	int nFrames = 400;
    int nMixtures = 5; // BS parameters
//	cout << "[BS] <<<< Number of frames: ";
//	cin >> nFrames;
//	cout << "[BS] <<<< Number of mixtures: ";
//	cin >> nMixtures;
	
	BackgroundSubtractor bgSubtractor(nFrames, nMixtures);
    modelBackground(reader, bgSubtractor);
    
    CloudjectDetector cloudjectDetector;
    cloudjectDetector.setModelLeafSize(0.015);
    cloudjectDetector.setNormalRadius(0.03);
    cloudjectDetector.setFpfhRadius(0.125);
    cloudjectDetector.setScoreThreshold(0.5);
    
    cloudjectDetector.loadCloudjectModels(m_ParentDir + "Data/CloudjectModels/");
    
//    MonitorizerParams monitorParams;
//	monitorParams.tmpCoherence = 2;
//	monitorParams.motionThresh = 5;
//    //	monitorParams.leafSize = 0.005;
//	monitorParams.posCorrespThresh = 0.1; // 7 cm
    
	Monitorizer monitorizer (registerer, tableModeler, cloudjectDetector);
    monitorizer.setLeafSize(0.0225);
    monitorizer.setClusteringToleranceFactor(3);

	/*
	 * Loop
	 */
    
    reader.nextSequence();
    
    monitorizer.clear();
    
	DepthFrame dFrameA, dFrameB;
	while ( reader.nextDepthPairedFrames(dFrameA, dFrameB) )
	{
		DepthFrame fgDFrameA, fgDFrameB;
		bgSubtractor.subtract(dFrameA, dFrameB, fgDFrameA, fgDFrameB);

        // Processing
		monitorizer.monitor(fgDFrameA, fgDFrameB);
        // Visualization
        if (display)
            monitorizer.display();
	}
    
    monitorizer.getObjectDetectionOutput();
    
}


void Remedi::modelBackground(Reader reader, BackgroundSubtractor& bs)
{
    DepthFrame dFrameA, dFrameB;
	reader.nextDepthPairedFrames(dFrameA, dFrameB);

	bs(dFrameA, dFrameB, 1);

	while (!bs.isReady())
	{
		bs(dFrameA, dFrameB, 0.02);

		reader.nextDepthPairedFrames(dFrameA, dFrameB);
	}
}


void Remedi::interactWithRegisterer(Reader reader, InteractiveRegisterer& registerer)
{
	// Ask wheter re-select a set of points
	// or use a previously computed transformation
//	char c;
//	cout << "[IR] <<<< Re-use alineation [Y/n]?" << endl;
//	cin >> c;
    char c = 'y'; // debug
    
	if (c != 'n' && c != 'N') // re-use transformation
	{
		registerer.loadTransformation(m_ParentDir);
        
        // DEBUG
        DepthFrame dFrameA, dFrameB;
		reader.getDepthPairedFrames(2, dFrameA, dFrameB);
		registerer.visualizeRegistration(dFrameA, dFrameB);
	}
	else // re-select points
	{
		// Select the paired frames two register
		int fID;
		cout << "-> Select a frame ID: ";
		cin >> fID;

        ColorFrame cFrameA, cFrameB;
        DepthFrame dFrameA, dFrameB;
        
        reader.getColorPairedFrames(fID, cFrameA, cFrameB);
		reader.getDepthPairedFrames(fID, dFrameA, dFrameB);
        
		// Select the number of points to compute a rigid transformation
		int nPoints;
        
		cout << "-> Select the number of points: ";
		cin >> nPoints;

        registerer.setInputFrames(cFrameA, cFrameB, dFrameA, dFrameB);
		registerer.setNumPoints(nPoints);
        
        registerer.interact(); // manual interaction
        registerer.computeTransformation(); // tranformation computation
        
        registerer.visualizeRegistration(dFrameA, dFrameB);
        
        pcl::PCDWriter writer;
        writer.write("registeredA.pcd", *registerer.getRegisteredClouds().first);
        writer.write("registeredB.pcd", *registerer.getRegisteredClouds().second);
        
        registerer.saveTransformation(m_ParentDir);
	}
}


void Remedi::modelTablePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr pRegisteredCloudA,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr pRegisteredCloudB,
                              TableModeler& tableModeler)
{
	tableModeler.setInputClouds(pRegisteredCloudA, pRegisteredCloudB);
	tableModeler.setLeafSize(0.02); // 2cm
	tableModeler.setNormalRadius(0.05);
	tableModeler.setSACIters(200);
	tableModeler.setSACDistThresh(0.03);
	tableModeler.setYOffset(0.4);
    tableModeler.setInteractionBorder(0.7);
    
	tableModeler.model();
}


Remedi::~Remedi()
{

}