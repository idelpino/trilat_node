
#include <iostream>

#include "rinex_reader_node.h"

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "rinex_reader_main");


	//TODO per ora i meteo file non li gestisco!!
	if( (argc < 2) || (argc > 3) )
	{
		cerr <<  "Usage:\n\t" << argv[0]
			 << " <RINEX Obs file>  <RINEX Nav file>" /* [<RINEX Met file>]"*/ << endl;

		exit (-1);
	}


	RinexReaderNode rinNode(argv[1], argv[2]);

	ros::Rate loopRate(0.5);

	while ( ros::ok() )
	{
		rinNode.processNextEpoch();
		if(rinNode.isSolutionValid())
		{
			rinNode.printEpochRecap();
			rinNode.publishMeasurements();
		}


		if(rinNode.isFileFinished()) {
			break;
		} else {
			loopRate.sleep();
		}
	}
	return 0;
}
