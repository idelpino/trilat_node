

#include <vector>

#include "receiver_sim_node.h"

const Receiver DEF_REAL_RECEIVER = {Point<double>(), 100e-9};
const double DEF_STD_DEV = 1e-10;
const double SPEED_OF_LIGHT = 3e8; // m / s
const double PI = 3.14159265;

bool parseArgs(int argc, char** argv, Receiver &realReceiver, std::vector<SatelliteMeasurement> &satellites, double &std_dev);


//node main
int main(int argc, char **argv)
{
	//init ros
	ros::init(argc, argv, "receiver_sim_node");

	//parse arguments
	double std_dev = DEF_STD_DEV;
	Receiver realReceiver = DEF_REAL_RECEIVER;
	std::vector<SatelliteMeasurement> satellites;

	if(!parseArgs(argc, argv, realReceiver, satellites, std_dev))
	{
		std::cout << "Input is not valid\n";
		return -1;
	}

	// Receiver node
	ReceiverSimNode recNode;
	recNode.setRealRec(realReceiver);

	ros::Rate loopRate(20);

	int theta=0;
	double radius = 15;

	//node loop
	while ( ros::ok() )
	{
		//do things
		//recNode.move(0, 0, 0.4);
		recNode.moveTo(radius * cos(theta*PI/180), radius * sin(theta*PI/180), recNode.realRec.pos.getZ() + 0.01);
		theta++;
		recNode.simulateMeasurements(satellites, std_dev, SPEED_OF_LIGHT);
		recNode.publishMeasurements();
		recNode.publishRealReceiver();

		//execute pending callbacks
		//ros::spinOnce();

		//relax to fit output rate
		loopRate.sleep();
	}

	//exit program
	return 0;
}




//TODO boost gives a nice parser
//TODO metti tutte le funzioni parser in un file esterno se e' possibile (o in una classe)
bool parseArgs(int argc, char** argv, Receiver &realReceiver, std::vector<SatelliteMeasurement> &satellites, double &std_dev)
{
	bool receiver_setted = false;
	int n_satellites = 0;

	for (int i = 1; i < argc; ++i){
		if ((strcmp (argv[i], "--dev") == 0) || (strcmp (argv[i], "-d") == 0)){

			std_dev =  atof(argv[++i]);

		} else if (strcmp (argv[i], "--bias") == 0  || (strcmp (argv[i], "-b") == 0)){

			realReceiver.bias = atof(argv[++i]);

		} else if ((strcmp (argv[i], "--satellite") == 0) || (strcmp (argv[i], "-s") == 0)){

			double x = atof(argv[++i]);
			double y = atof(argv[++i]);
			double z = atof(argv[++i]);

			SatelliteMeasurement sm;
			sm.pos = Point<double>(x, y, z);
			sm.pseudorange = 0.0;

			satellites.push_back(sm);
			++n_satellites;

		} else if ((strcmp (argv[i], "--receiver") == 0) || (strcmp (argv[i], "-r") == 0)){

			double x = atof(argv[++i]);
			double y = atof(argv[++i]);
			double z = atof(argv[++i]);

			realReceiver.pos = Point<double>(x, y, z);
			receiver_setted = true;
		}
	}

	// TODO check if input is well formed

	return receiver_setted && (n_satellites >= 3);
}
