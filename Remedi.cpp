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

Remedi::~Remedi()
{
    
}

void Remedi::interactWithRegisterer(Sequence::Ptr pSequence, int fID, InteractiveRegisterer& registerer)
{
    vector<DepthFrame> depthFrames = pSequence->getDepthFrame(fID);
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

void Remedi::setVisualization(bool enable)
{
    m_bVisualization = enable;
}

void Remedi::setInteractiveRegistration(int frame)
{
    m_RegistrationFrameID = frame;
}

void Remedi::run()
{
	/*
	 * LOAD DATA
	 */
    
	// Create a reader pointing the data streams
    
    string sequencesPath = m_ParentDir + "Data/Sequences/";
    vector<string> colorDirs;
    colorDirs += "Color1/", "Color2/";
    vector<string> depthDirs;
    depthDirs += "Depth1/", "Depth2/";
    string labelsPath = m_ParentDir + "Data/Labels/observer_1/csv/";
    
    Reader reader( sequencesPath, colorDirs, depthDirs, labelsPath );
    
    Sequence::Ptr pBackgroundSeq(new Sequence(2));
    reader.getSequence(0, *pBackgroundSeq);
    
    vector<int> delays;
    delays += 2,0;
    reader.setDelays(delays);
    
	/*
	 * REGISTRATION (paired frames)
	 */
    
    InteractiveRegisterer::Ptr pRegisterer (new InteractiveRegisterer);// (new InteractiveRegisterer);
    
    if (m_RegistrationFrameID < 0)
        pRegisterer->loadTransformation(m_ParentDir);
    else
        interactWithRegisterer(pBackgroundSeq, m_RegistrationFrameID, *pRegisterer);
    
	/*
	 * PLANE SEGMENTATION
	 */
    
    TableModeler::Ptr pTableModeler (new TableModeler);// (new TableModeler);
	modelTablePlanes(pRegisterer->getRegisteredClouds().first,
                     pRegisterer->getRegisteredClouds().second,
                     *pTableModeler);
	
	/*
	 * BACKGROUND SUBTRACTION
	 */
    
    BackgroundSubtractor::Ptr pBackgroundSubtractor (new BackgroundSubtractor);
    pBackgroundSubtractor->setInputSequence(pBackgroundSeq);
    pBackgroundSubtractor->setNumOfMixtureComponents(5);
    pBackgroundSubtractor->model();
    
    /*
	 * OBJECT DETECTION AND RECOGNITION
	 */
    
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
    
    Monitorizer::Ptr pMonitorizer (new Monitorizer);
    
    pMonitorizer->setBackgroundSubtractor(pBackgroundSubtractor);
    pMonitorizer->setRegisterer(pRegisterer);
    pMonitorizer->setTableModeler(pTableModeler);
    pMonitorizer->setCloudjectDetector(pCloudjectDetector);
    
    pMonitorizer->setLeafSize(0.005);
    pMonitorizer->setClusteringToleranceFactor(3);
    pMonitorizer->setVisualization(m_bVisualization);
    
	/*
	 * Loop
	 */
    
    reader.setSequence(1);
    
    Sequence::Ptr sequence = reader.getNextSequence();
    
//    DetectionOutput gt (m_ParentDir + "Data/ObjectLabels/", sequence.getName(), "csv");
    
//    DetectionOutput output;
    //pMonitorizer->setSequence(sequence);
    //pMonitorizer->monitor(output);
    
//    output.write("", pSequence->getName(), "yml");
//
//    int tp, fp, fn;
//    output.getResults(gt, tp, fp, fn);
//    cout << "tp: " << tp << "\t fp: " << fp << "\tfn: " << fn << endl;
    
}