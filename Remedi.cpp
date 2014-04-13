#include "Remedi.h"

#include "Monitorizer.h"
#include "MonitorizerParams.hpp"
#include "DepthFrame.h"
#include "TableModeler.h"
#include "Table.hpp"
#include <pcl/visualization/pcl_visualizer.h>

#ifdef _WIN32
string g_parentDir = "../";
#elif __APPLE__
string g_parentDir = "../../";
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

    string dataPath = g_parentDir + "Data/";
    Reader reader( dataPath, "Color1/", "Color2/", "Depth1/", "Depth2/" );
    reader.setInputStream("00_bs/"); // background frames

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

	int nFrames, nMixtures; // BS parameters
	cout << "[BS] <<<< Number of frames: ";
	cin >> nFrames;
	cout << "[BS] <<<< Number of mixtures: ";
	cin >> nMixtures;
	
	BackgroundSubtractor bgSubtractor(nFrames, nMixtures);
    modelBackground(reader, bgSubtractor);
	
	MonitorizerParams monitorParams;
	monitorParams.tmpCoherence = 2;
	monitorParams.motionThresh = 5;
	monitorParams.leafSize = 0.01;
	monitorParams.posCorrespThresh = 0.07; // 7 cm
    
	Monitorizer monitorizer (registerer, tableModeler, monitorParams);

	/*
	 * Loop
	 */
    
    reader.setInputStream("28_p14_2/"); // a sequence's frames
    
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


void Remedi::modelBackground(Reader reader, BackgroundSubtractor& bs)
{
    DepthFrame dFrameA, dFrameB;
	reader.getNextDepthPairedFrames(dFrameA, dFrameB);

	bs(dFrameA, dFrameB, 1);

	while (!bs.isReady())
	{
		bs(dFrameA, dFrameB, 0.02);
        
        DepthFrame fgDFrameA;
        bs.subtract(dFrameA, fgDFrameA);
        
        cv::Mat masked;
        cv::namedWindow("bs in A");
        fgDFrameA.getMat().copyTo(masked, fgDFrameA.getMask());
        cv::imshow("bs in A", fgDFrameA.getMask());
        cv::waitKey(100);

		reader.getNextDepthPairedFrames(dFrameA, dFrameB);
	}
    
    
}


void Remedi::interactWithRegisterer(Reader reader, InteractiveRegisterer& registerer)
{
	// Ask wheter re-select a set of points
	// or use a previously computed transformation
	char c;
	cout << "[IR] <<<< Re-use alineation [Y/n]?" << endl;
	cin >> c;
	if (c != 'n' && c != 'N') // re-use transformation
	{
		registerer.loadTransformation(g_parentDir);
        
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
        
        registerer.saveTransformation(g_parentDir);
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