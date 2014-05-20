#include "Remedi.h"
#include "SupervisedObjectPicker.h"

#include <pcl/console/parse.h>
#include <boost/assign/std/vector.hpp>

using namespace boost::assign;

#ifdef _WIN32
string g_parentDir = "../";
#elif __APPLE__
string g_parentDir = "../../";
#endif

int main (int argc, char** argv)
{
    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    
    int sid; // sequence autoincremental id
    if (pcl::console::parse (argc, argv, "-p", sid) >= 0)
    {
        string sequencesPath = g_parentDir + "Data/Sequences/";
        
        vector<string> colorDirs;
        colorDirs += "Color1/", "Color2/";
        vector<string > depthDirs;
        depthDirs += "Depth1/", "Depth2/";
        
        Reader reader (sequencesPath, colorDirs, depthDirs);
        vector<int> delays;
        delays += 2,0;
        reader.setDelays(delays);
        
        Sequence::Ptr seq = reader.getSequence(1);
        
        SupervisedObjectPicker pp(g_ParentDir + "Data/ObjectLabels/", 5);
        pp.setSequence(seq);
        pp.run();
        
        return 0;
    }
    
    bool display = false;
    if (pcl::console::find_argument (argc, argv, "-d") >= 0)
    {
        display = true;
        cout << "Display the processing of the sequences [YES]" << endl;
    }
    else
    {
        cout << "Display the processing of the sequences [NO]" << endl;
    }
    
    Remedi app(g_parentDir);
	app.Run(display);

	return 0;
}

