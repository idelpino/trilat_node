
#include <iostream>
#include <vector>

#include "rinex_reader_node.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rinex_reader_main");

	if( (argc < 3) || (argc > 4) )
	{
		cerr <<  "Usage:" << endl;
		cerr << "   " << argv[0]
			 << " <RINEX Obs file>  <RINEX Nav file>  [<RINEX Met file>]"
			 << endl;

		exit (-1);
	}





}
