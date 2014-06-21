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
    
    bool visualization = false;
    cout << "Processing visualization ";
    if ( (visualization = (pcl::console::find_argument (argc, argv, "-v") >= 0)) )
        cout << "[YES]" << endl;
    else
        cout << "[NO]" << endl;
    
    int registrationFrameID = -1;
    cout << "Views landmarks interactive selection for registration ";
    if ( pcl::console::parse (argc, argv, "-r", registrationFrameID) >= 0 )
        cout << "[YES]" << endl;
    else
        cout << "[NO]" << endl;
    
    Remedi app(g_ParentDir);
    app.setVisualization(visualization);
    app.setInteractiveRegistration(registrationFrameID);
    
	app.run();

	return 0;
}

