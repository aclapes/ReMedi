#include "Remedi.h"

#include <pcl/console/parse.h>
#include <boost/assign/std/vector.hpp>

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
    
    bool visualization = false;
    cout << "Processing visualization ";
    if ( (visualization = (pcl::console::find_argument (argc, argv, "-v") >= 0)) )
        cout << "[YES]" << endl;
    else
        cout << "[NO]" << endl;
    
    bool registration = false;
    int registrationFrameID = -1;
    int numOfPoints = 0;
    cout << "Views landmarks interactive selection for registration ";
    if ( (registration = pcl::console::parse (argc, argv, "-r", registrationFrameID) >= 0) )
    {
        cout << "[YES]" << endl;
        if ( pcl::console::parse (argc, argv, "-p", numOfPoints) < 0 )
            numOfPoints = 3;
    }
    else
    {
        cout << "[NO]" << endl;
    }
    
    Remedi app;
    app.setInputDataPath(dataPath);
    app.setVisualization(visualization);
    app.setInteractiveRegistration(registration);
    app.setInteractiveRegistrationParameters(registrationFrameID, numOfPoints);
    
	app.run();

	return 0;
}

