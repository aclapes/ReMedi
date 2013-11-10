#include "Remedi.h"

Remedi::Remedi()
{
	
}

void Remedi::Run()
{
	/*
	 * Load data
	 */

	// Create a reader pointing the data streams
#ifdef _WIN32
	Reader reader("../Data/C240/","../Data/S240/");
#elif __APPLE__
	Reader reader("../../Data/C240/","../../Data/S240/");
#endif

	/*
	 * Registration (paired frames)
	 */

	InteractiveRegisterer registerer;
	
	// Ask wheter re-select a set of points
	// or use a previously computed transformation
	char c;
	std::cout << "Re-use alineation [Y/n]?" << std::endl;
	cin >> c;
	if (c != 'n' && c != 'N') // re-use transformation
	{
		registerer.loadTransformation("transformation.yml");
	}
	else // re-select points
	{
		DepthFrame dFrameA, dFrameB;
		
		// Select the paired frames two register
		int fID;
		std::cout << "-> Select a frame ID: ";
		cin >> fID;

		reader.getDepthPairedFrames(fID, dFrameA, dFrameB);
		
		// Select the number of points to compute a rigid transformation
		int nPoints;
		std::cout << "-> Select the number of points: ";
		cin >> nPoints;

		registerer.setNumPoints(nPoints);
		registerer.computeTransformation(dFrameA, dFrameB); // find the transformation

		registerer.saveTransformation("transformation.yml");
	}

	int nFrames, nMixtures; // BS parameters
	cout << "(BS) Number of frames: ";
	cin >> nFrames;
	cout << "(BS) Number of mixtures: ";
	cin >> nMixtures;
	
	BackgroundSubtractor bgSubtractorA(nFrames, nMixtures);
	BackgroundSubtractor bgSubtractorB(nFrames, nMixtures);
	DepthFrame dBackgroundA, dBackgroundB;

	pcl::visualization::PCLVisualizer::Ptr pViz (new pcl::visualization::PCLVisualizer);

	float posCorrespThresParam = 0.07; // 7 cm
	Monitorizer monitorizer (posCorrespThresParam);

	/*
	 * Loop
	 */

	/*cv::namedWindow("BS");*/

	DepthFrame dFrameA, dFrameB;
	bool bSuccess = reader.getNextDepthPairedFrames(dFrameA, dFrameB); // able to read, not finished

	bgSubtractorA(dFrameA, dBackgroundA, 1);
	bgSubtractorB(dFrameB, dBackgroundB, 1);

	while (bSuccess)
	{
		if ( !bgSubtractorA.isReady() && !bgSubtractorB.isReady() ) // Background subtraction
		{
			bgSubtractorA(dFrameA, dBackgroundA, 0.05);
			bgSubtractorB(dFrameB, dBackgroundB, 0.05);
		}
		else	// Proceed normally
		{
			DepthFrame fgDFrameA, fgDFrameB;
			bgSubtractorA.subtract(dFrameA, fgDFrameA);
			bgSubtractorB.subtract(dFrameB, fgDFrameB);

			pcl::PointCloud<pcl::PointXYZ>::Ptr regCloudA (new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr regCloudB (new pcl::PointCloud<pcl::PointXYZ>());
			registerer.getRegisteredClouds(fgDFrameA, fgDFrameB, regCloudA, regCloudB, false);

			registerer.visualizeRegistration(pViz, regCloudA, regCloudB, 1); // update in just 1ms

			monitorizer.monitor(regCloudA, regCloudB);

			//monitorizer.handleCloudjectDrops(...);
			//monitorizer.handleCloudjectPicks(...);

		}

		bSuccess = reader.getNextDepthPairedFrames(dFrameA, dFrameB);
	}

}

Remedi::~Remedi()
{

}