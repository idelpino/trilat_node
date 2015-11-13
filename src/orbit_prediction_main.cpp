
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

	ros::Rate loopRate(1000);

	int i = 0;
	bool fileNotFinished = true;

	while ( ros::ok() )
	{
		// visualize a sphere that represent earth
		op.publishEarth();

		if(fileNotFinished)
			cout << "\t";

		// at the first time compute position from obs file
		if(i%30  == 0 && fileNotFinished){
			fileNotFinished = op.processNextEpoch();

			i = 0;
		} else {
			op.computeSatsPositionAfter(i);
		}

		op.publishSatsPosition();


		///prima di tutto leggi un'epoca
		/// poi interpola per un po di tempo quei dati
		/// ad ogni interpolazione pubblica i risultati, in modo che rviz visualizzi
		///

		i++;
		loopRate.sleep();
	}
	return 0;
}
