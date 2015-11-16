//
// USAGE:
//
// rosrun trilateration orbit_prediction_node src/trilateration/data/bahr1620.04o src/trilateration/data/bahr1620.04n
//


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

		if(fileNotFinished)
			cout << "\t";

		// at the first time compute position from obs file
		if(i%30  == 0 && fileNotFinished){
			// visualize a sphere that represent earth
			op.publishEarth(); // TODO non so perche' non riesca a stamparlo solo una volta

			fileNotFinished = op.processNextEpoch();

			i = 0;
		} else {
			op.computeSatsPositionAfter(i);
		}

		op.publishSatsPositions();


		i++;
		loopRate.sleep();
	}
	return 0;
}
