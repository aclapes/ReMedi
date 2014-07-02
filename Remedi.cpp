#include "Remedi.h"

#include "Monitorizer.h"
#include "MonitorizerParams.hpp"
#include "DepthFrame.h"
#include "TableModeler.h"
#include "Table.hpp"
#include "CloudjectModel.hpp"
#include "StatTools.h"

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

//void Remedi::setValidationResultsPath(string path, string name, string extension)
//{
//    m_ValidationFilePath = path + path + "." + extension;
//}

void Remedi::setValidationParameters(vector<vector<float> > parameters)
{
    m_Parameters = parameters;
    expandParameters<float>(m_Parameters, m_ExpandedParameters);
    
    ofstream file;
    file.open("results.csv", ios::out);
    if (file.is_open())
    {
        for (int i = 0; i < m_Parameters.size(); i++)
            file << "v" << to_string(i) << "\t";
        file << "NGT\tNPR\t";
        file << "STP\tSFN\tSFP\tSPR\tSRE\tSF1\t";
        file << "RTP\tRFN\tRFP\tRPR\tRRE\tRF1";
        file << endl;
    }
    file.close();
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

void Remedi::run(vector<float> parameters)
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
    pBackgroundSubtractor->setNumOfMixtureComponents(parameters[0]);
    pBackgroundSubtractor->setLearningRate(parameters[1]);
    pBackgroundSubtractor->setBackgroundRatio(parameters[2]);
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
    pMonitorizer->monitor(output);
    output.write("", pMonitorSeq->getName(), "csv");
    
    output.read("", pMonitorSeq->getName(), "csv");
    DetectionOutput groundtruth (m_DataPath + "ObjectLabels/", pMonitorSeq->getName(), "csv");
    
    ofstream file;
    file.open("results.csv", ios::app);
    if (file.is_open())
    {
        file.precision(3);
        
        for (int i = 0; i < parameters.size(); i++)
            file << parameters[i] << "\t";
        
        file << groundtruth.getNumOfDetections() << "\t" << output.getNumOfDetections() << "\t";

        int TP, FN, FP;
        float precision, recall, f1;
        
        output.getSegmentationResults(groundtruth, TP, FN, FP);
        
        precision = ((float) TP) / (TP + FP);
        recall = ((float) TP) / (TP + FN);
        f1 = 2.f * ((precision * recall) / (precision + recall));
        file << TP << "\t" << FN << "\t" << FP << "\t" << precision << "\t" << recall << "\t" << f1 << "\t";
        
        output.getRecognitionResults(groundtruth, TP, FN, FP);
        
        precision = ((float) TP) / (TP + FP);
        recall = ((float) TP) / (TP + FN);
        f1 = 2.f * ((precision * recall) / (precision + recall));
        file << TP << "\t" << FN << "\t" << FP << "\t" << precision << "\t" << recall << "\t" << f1;
        file << endl;
        
        file.close();
    }
}

void Remedi::validate()
{
    for (int i = 0; i < m_ExpandedParameters.size(); i++)
    {
        run(m_ExpandedParameters[i]);
    }
}