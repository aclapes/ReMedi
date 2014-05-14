#include "Remedi.h"
#include "SupervisedObjectPicker.h"

#include <pcl/console/parse.h>

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
        SupervisedObjectPicker pp(g_parentDir, sid, 2, 5);
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

