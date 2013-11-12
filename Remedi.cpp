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
	 * REGISTRATION (paired frames)
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

		registerer.interact();
		registerer.computeTransformation(dFrameA, dFrameB); // find the transformation

		registerer.saveTransformation("transformation.yml");
	}


	/*
	 * BACKGROUND SUBTRACTION
	 */

	int nFrames, nMixtures; // BS parameters
	cout << "(BS) Number of frames: ";
	cin >> nFrames;
	cout << "(BS) Number of mixtures: ";
	cin >> nMixtures;
	
	BackgroundSubtractor bgSubtractorA(nFrames, nMixtures);
	BackgroundSubtractor bgSubtractorB(nFrames, nMixtures);

	float leafSize = 0.01;
	float posCorrespThresParam = 0.07; // 7 cm
	Monitorizer monitorizer (leafSize, posCorrespThresParam);

	/*
	 * Loop
	 */

	pcl::visualization::PCLVisualizer::Ptr pViz;

	DepthFrame dFrameA, dFrameB;
	bool bSuccess = reader.getNextDepthPairedFrames(dFrameA, dFrameB); // able to read, not finished

	bgSubtractorA(dFrameA, 1);
	bgSubtractorB(dFrameB, 1);

	bool initViz = false;
	while (bSuccess)
	{
		if ( !bgSubtractorA.isReady() && !bgSubtractorB.isReady() ) // Background subtraction
		{
			bgSubtractorA(dFrameA, 0.02);
			bgSubtractorB(dFrameB, 0.02);
		}
		else if (!initViz)
		{
			 pViz = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer);
			 initViz = true;
		}
		else	// Proceed normally
		{
			DepthFrame fgDFrameA, fgDFrameB;
			bgSubtractorA.subtract(dFrameA, fgDFrameA);
			bgSubtractorB.subtract(dFrameB, fgDFrameB);

			pcl::PointCloud<pcl::PointXYZ>::Ptr regCloudA (new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr regCloudB (new pcl::PointCloud<pcl::PointXYZ>());
			registerer.getRegisteredClouds(fgDFrameA, fgDFrameB, regCloudA, regCloudB, false);

			registerer.visualizeRegistration(pViz, regCloudA, regCloudB); // update in just 1ms

			monitorizer.monitor(regCloudA, regCloudB);

			//sstd::vector<Cloudject<pcl::PointXYZ,pcl::FPFHSignature33> > drops;
			monitorizer.handleCloudjectDrops(/*drops*/);
			//monitorizer.handleCloudjectPicks(...);

			monitorizer.visualizeCloudjects(pViz);

			pViz->spinOnce(50);
		}

		bSuccess = reader.getNextDepthPairedFrames(dFrameA, dFrameB);
		std::cout << reader.getFrameID() << std::endl;
	}

}

Remedi::~Remedi()
{

}