#include "Remedi.h"

#include <pcl/console/parse.h>
#include <boost/assign/std/vector.hpp>
#include "StatTools.h"

using namespace boost::assign;

#ifdef _WIN32
string g_ParentDir = "../../";
#elif __APPLE__
string g_ParentDir = "../../../";
#endif

int main (int argc, char** argv)
{
    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    
    string dataPath;
    cout << "Set data relative path ";
    if ( pcl::console::parse (argc, argv, "-D", dataPath) < 0 )
        dataPath += g_ParentDir + "Data/";
    cout << "[" << dataPath << "]" << endl;
    
    bool bVisualization = false;
    cout << "Processing visualization ";
    if ( (bVisualization = (pcl::console::find_argument (argc, argv, "-v") >= 0)) )
        cout << "[YES]" << endl;
    else
        cout << "[NO]" << endl;
    
    bool bRegistration = false;
    int registrationFrameID = -1;
    int numOfPoints = 0;
    cout << "Views landmarks interactive selection for registration ";
    if ( (bRegistration = pcl::console::parse (argc, argv, "-r", registrationFrameID) >= 0) )
    {
        cout << "[YES]" << endl;
        if ( pcl::console::parse (argc, argv, "-p", numOfPoints) < 0 )
            numOfPoints = 3;
    }
    else
    {
        cout << "[NO]" << endl;
    }
    
    cout << "Table modeling ";
    bool bTableModeling = false;
    if ( (bTableModeling = (pcl::console::find_argument (argc, argv, "-t") >= 0)) )
        cout << "[YES]" << endl;
    else
        cout << "[NO]" << endl;
    
    Remedi app;
    app.setInputDataPath(dataPath);
    app.setVisualization(bVisualization);
    app.setInteractiveRegistration(bRegistration);
    app.setInteractiveRegistrationParameters(registrationFrameID, numOfPoints);
    app.setTableModeling(bTableModeling);
    
    //    app.run();
    
    vector<float> components;
    components += 2, 3, 5;
    vector<float> learningRates;
    learningRates += 0.001, 0.005, 0.01, 0.02;
    vector<float> backgroundRatios;
    backgroundRatios += 0.5, 0.9, 0.99, 0.999;
    
    vector<vector<float> > parameters;
    parameters += components, learningRates, backgroundRatios;
   
    app.setValidationParameters(parameters);
    app.validate();


	return 0;
}

