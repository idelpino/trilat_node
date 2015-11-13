
#include <iostream>

#include "orbit_prediction_node.h"

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "orbit_prediction_main");

	if( (argc < 2) || (argc > 3) )
	{
		cerr <<  "Usage:\n\t" << argv[0]
			 << " <RINEX Obs file>  <RINEX Nav file>" << endl;

		exit (-1);
	}

	OrbitPredictionNode op(argv[1], argv[2]);

	// update sats position every 'interval' seconds
	double interval = 1;

	ros::Rate loopRate(1.0/interval);


	while ( ros::ok() )
	{
		///prima di tutto leggi un'epoca
		/// poi interpola per un po di tempo quei dati
		/// ad ogni interpolazione pubblica i risultati, in modo che rviz visualizzi
		///

		// visualize a sphere that represent earth
		op.publishEarth();

		loopRate.sleep();
	}
	return 0;
}
