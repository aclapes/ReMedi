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
    vector<string> colorDirs;
    colorDirs += "Color1/", "Color2/";
    vector<string> depthDirs;
    depthDirs += "Depth1/", "Depth2/";
    string labelsPath = m_ParentDir + "Data/Labels/observer_1/csv/";

    Reader reader( sequencesPath, colorDirs, depthDirs, labelsPath );
    
    Sequence::Ptr pSequence (new Sequence);
    reader.getSequence(0, *pSequence);
    
	/*
	 * REGISTRATION (paired frames)
	 */

    InteractiveRegisterer::Ptr pRegisterer (new InteractiveRegisterer);
	interactWithRegisterer(pSequence, *pRegisterer);

	/*
	 * PLANE SEGMENTATION
	 */

    TableModeler::Ptr pTableModeler (new TableModeler);
	modelTablePlanes(pRegisterer->getRegisteredClouds().first,
                     pRegisterer->getRegisteredClouds().second,
                     *pTableModeler);
	
	/*
	 * BACKGROUND SUBTRACTION
	 */

	int nFrames = 400;
    int nMixtures = 5; // BS parameters
//	cout << "[BS] <<<< Number of frames: ";
//	cin >> nFrames;
//	cout << "[BS] <<<< Number of mixtures: ";
//	cin >> nMixtures;
	
    BackgroundSubtractor::Ptr pBackgroundSubtractor(new BackgroundSubtractor(nFrames, nMixtures));
    modelBackground(pSequence, *pBackgroundSubtractor);
    
    CloudjectDetector::Ptr pCloudjectDetector (new CloudjectDetector);
    pCloudjectDetector->setModelLeafSize(0.015);
    pCloudjectDetector->setNormalRadius(0.03);
    pCloudjectDetector->setFpfhRadius(0.125);
    pCloudjectDetector->setScoreThreshold(0.5);
    pCloudjectDetector->setLeafSize(0.0225);
    pCloudjectDetector->setTemporalWindow(3);
    pCloudjectDetector->setMaxCorrespondenceDistance(0.07);
    pCloudjectDetector->loadCloudjectModels(m_ParentDir + "Data/CloudjectModels/");
    
//    MonitorizerParams monitorParams;
//	monitorParams.tmpCoherence = 2;
//	monitorParams.motionThresh = 5;
//    //	monitorParams.leafSize = 0.005;
//	monitorParams.posCorrespThresh = 0.1; // 7 cm
    
	Monitorizer monitorizer (pBackgroundSubtractor, pRegisterer, pTableModeler, pCloudjectDetector);
    monitorizer.setLeafSize(0.005);
    monitorizer.setClusteringToleranceFactor(3);
    monitorizer.setVisualization(display);

	/*
	 * Loop
	 */
    
    while (reader.hasNextSequence())
    {
        reader.getNextSequence(*pSequence);
        
        DetectionOutput output;
        monitorizer.setSequence(pSequence);
        monitorizer.monitor(output);
    }
    
//    monitorizer.clear();
//    
//	DepthFrame dFrameA,     ;
//	while ( reader.nextDepthPairedFrames(dFrameA, dFrameB) )
//	{
//		DepthFrame fgDFrameA, fgDFrameB;
//		bgSubtractor.subtract(dFrameA, dFrameB, fgDFrameA, fgDFrameB);
//
//        // Processing
//		monitorizer.monitor(fgDFrameA, fgDFrameB);
//        // Visualization
//        if (display)
//            monitorizer.display();
//	}
//    
//    monitorizer.getObjectDetectionOutput();
    
}


void Remedi::modelBackground(Sequence::Ptr pSequence, BackgroundSubtractor& bs)
{
    DepthFrame dFrameA, dFrameB;
    vector<DepthFrame> depthFrames = pSequence->nextDepthFrame();
    
	bs(depthFrames[0], depthFrames[1], 1);

	while (!bs.isReady())
	{
        bs(depthFrames[0], depthFrames[1], 0.02);

		depthFrames = pSequence->nextDepthFrame();
	}
}


void Remedi::interactWithRegisterer(Sequence::Ptr pSequence, InteractiveRegisterer& registerer)
{
	// Ask wheter re-select a set of points
	// or use a previously computed transformation
//	char c;
//	cout << "[IR] <<<< Re-use alineation [Y/n]?" << endl;
//	cin >> c;
    char c = 'y'; // debug
    
    vector<DepthFrame> depthFrames = pSequence->getDepthFrame(2);
	if (c != 'n' && c != 'N') // re-use transformation
	{
		registerer.loadTransformation(m_ParentDir);
        
        // DEBUG
		registerer.visualizeRegistration(depthFrames[0], depthFrames[1]);
	}
	else // re-select points
	{
		// Select the paired frames two register
		int fID;
		cout << "-> Select a frame ID: ";
		cin >> fID;
        
        vector<ColorFrame> colorFrames = pSequence->getColorFrame(fID);
        
		// Select the number of points to compute a rigid transformation
		int nPoints;
        
		cout << "-> Select the number of points: ";
		cin >> nPoints;

        registerer.setInputFrames(colorFrames[0], colorFrames[1], depthFrames[0], depthFrames[1]);
		registerer.setNumPoints(nPoints);
        
        registerer.interact(); // manual interaction
        registerer.computeTransformation(); // tranformation computation
        
        registerer.visualizeRegistration(depthFrames[0], depthFrames[1]);
        
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