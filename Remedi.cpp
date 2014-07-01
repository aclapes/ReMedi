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

#define DEFAULT_FRAME 2

Remedi::Remedi()
: m_RegistrationFrameID(2)
{
	
}

Remedi::~Remedi()
{
    
}

void Remedi::setInputDataPath(string dataPath)
{
    m_DataPath = dataPath;
}

void Remedi::setVisualization(bool enable)
{
    m_bVisualization = enable;
}

void Remedi::setInteractiveRegistration(bool enable)
{
    m_bRegistration = enable;
}

void Remedi::setInteractiveRegistrationParameters(int frame, int numOfPoints)
{
    m_RegistrationFrameID = frame;
    m_NumOfRegistrationPoints = numOfPoints;
}

void Remedi::setTableModeling(bool enable)
{
    m_bTableModeling = enable;
}

void Remedi::interactWithRegisterer(Sequence::Ptr pSequence, int fID, InteractiveRegisterer& registerer)
{
    vector<DepthFrame> depthFrames = pSequence->getDepthFrame(fID);
    vector<ColorFrame> colorFrames = pSequence->getColorFrame(fID);

    registerer.setInputFrames(colorFrames[0], colorFrames[1], depthFrames[0], depthFrames[1]);
    registerer.setNumPoints(m_NumOfRegistrationPoints);
    
    registerer.interact(); // manual interaction
    registerer.computeTransformation(); // tranformation computation
    
    registerer.visualizeRegistration(depthFrames[0], depthFrames[1]);
    
    pcl::PCDWriter writer;
    writer.write("registeredA.pcd", *registerer.getRegisteredClouds().first);
    writer.write("registeredB.pcd", *registerer.getRegisteredClouds().second);
    
    registerer.saveTransformation(m_DataPath + "transformation.yml");
}

void Remedi::run()
{
	/*
	 * LOAD DATA
	 */
    
	// Create a reader pointing the data streams
    
    string sequencesPath = m_DataPath + "Sequences/";
    vector<string> colorDirs;
    colorDirs += "Color1/", "Color2/";
    vector<string> depthDirs;
    depthDirs += "Depth1/", "Depth2/";
    string labelsPath = m_DataPath + "Labels/observer_1/csv/";
    
    Reader reader( sequencesPath, colorDirs, depthDirs, labelsPath );
    
    Sequence::Ptr pBackgroundSeq(new Sequence(2));
    reader.getSequence(0, *pBackgroundSeq);
    
    vector<int> delays;
    delays += 2,0;
    reader.setDelays(delays);
    
    vector<ColorFrame> colorFrames = pBackgroundSeq->getColorFrame(m_bRegistration ? m_RegistrationFrameID : DEFAULT_FRAME);
    vector<DepthFrame> depthFrames = pBackgroundSeq->getDepthFrame(m_bRegistration ? m_RegistrationFrameID : DEFAULT_FRAME);
    
	/*
	 * REGISTRATION (paired frames)
	 */
    
    InteractiveRegisterer::Ptr pRegisterer (new InteractiveRegisterer);
    
    if (m_RegistrationFrameID < 0)
        pRegisterer->loadTransformation(m_DataPath + "transformation.yml");
    else
    {
        pRegisterer->setInputFrames(colorFrames[0], colorFrames[1],
                                    depthFrames[0], depthFrames[1]);
        pRegisterer->setNumPoints(m_NumOfRegistrationPoints);
        pRegisterer->interact(); // manual interaction
        pRegisterer->computeTransformation(); // tranformation computation
        pRegisterer->saveTransformation(m_DataPath + "transformation.yml");
    }
    
	/*
	 * PLANE SEGMENTATION
	 */

    PointCloudPtr pCloudRegA (new PointCloud), pCloudRegB (new PointCloud);
    pRegisterer->registration(depthFrames[0], depthFrames[1], *pCloudRegA, *pCloudRegB);
    
    TableModeler::Ptr pTableModeler (new TableModeler);// (new TableModeler);
    pTableModeler->setInputClouds(pCloudRegA, pCloudRegB);
	pTableModeler->setLeafSize(0.02); // 2cm
	pTableModeler->setNormalRadius(0.05);
	pTableModeler->setSACIters(200);
	pTableModeler->setSACDistThresh(0.03);
	pTableModeler->setYOffset(0.4);
    pTableModeler->setInteractionBorder(0.7);
    pTableModeler->setConfidenceLevel(99);
    if (!m_bTableModeling)
    {
        pTableModeler->read("", "TableModeler", "dat");
        pTableModeler->model();
    }
    else
    {
        pTableModeler->model();
        pTableModeler->write("", "TableModeler", "dat");
    }
    
	/*
	 * BACKGROUND SUBTRACTION
	 */
    
    cv::theRNG().state = 74;
    BackgroundSubtractor::Ptr pBackgroundSubtractor (new BackgroundSubtractor);
    pBackgroundSubtractor->setInputSequence(pBackgroundSeq);
    pBackgroundSubtractor->setNumOfMixtureComponents(3);
    pBackgroundSubtractor->setLearningRate(0.005);
    pBackgroundSubtractor->setBackgroundRatio(0.9);
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
    pCloudjectDetector->loadCloudjectModels(m_DataPath + "CloudjectModels/");
    
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
    
    Sequence::Ptr pMonitorSeq = reader.getNextSequence();
    DetectionOutput output;
    
    pMonitorizer->setInputSequence(pMonitorSeq);
//    pMonitorizer->monitor(output);
//    output.write("", pMonitorSeq->getName(), "csv");
    
    output.read("", pMonitorSeq->getName(), "csv");
    DetectionOutput groundtruth (m_DataPath + "ObjectLabels/", pMonitorSeq->getName(), "csv");
    
    cout << output.getNumOfDetections() << "/" << groundtruth.getNumOfDetections() << endl;

    int segtp, segfn, segfp;
    output.getSegmentationResults(groundtruth, segtp, segfn, segfp);
    cout << "segmenentation,\t tp: " << segtp << "\t fn: " << segfn << "\t fp: " << segfp << endl;
    int rectp, recfn, recfp;
    output.getRecognitionResults(groundtruth, rectp, recfn, recfp);
    cout << "recognition,\t tp: " << rectp << "\t fn: " << recfn << "\t fp: " << recfp << endl;
}