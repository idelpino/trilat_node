//ros node that trilaterate the position of the target and publish the result as visualization_msgs::Marker.

// USAGE:
//	roscore
//	rosrun rviz rviz
//	rosrun trilateration trilat_node -r 3 -3 2 -s 15 26 40 -s 1 -32 50 -s -60 -8 70 -s 24 73 56 -s -99 -66 708

//in RVIZ:
//	1) global opt -> fixed frame: my_frame
//	2) add -> marker
//	3) marker topic: trilat_node/visualization_marker



#include <vector>

//ros dependencies
#include "trilat_node.h"
#include "glog/logging.h"

//bool parseArgs(int argc, char** argv, Receiver &realReceiver, vector<Point<double> > &satellites, double &std_dev);


//node main
int main(int argc, char **argv)
{
	//init ros
	ros::init(argc, argv, "trilat_node");

	//init google logging
	google::InitGoogleLogging(argv[0]);

	TrilatNode trNode;

	ros::Rate loopRate(1);

	//node loop
	while ( ros::ok() )
	{

		//do things
		trNode.process();
		trNode.publishMarker();

		//execute pending callbacks
//		ros::spinOnce();

		//relax to fit output rate
		loopRate.sleep();
	}

	//exit program
	return 0;
}



/*
//TODO boost gives a nice parser
//TODO metti tutte le funzioni parser in un file esterno se e' possibile (o in una classe)
bool parseArgs(int argc, char** argv, Receiver &realReceiver, vector<Point<double>> &satellites, double &std_dev)
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

			satellites.push_back(Point<double>(x, y, z));
			++n_satellites;

		} else if ((strcmp (argv[i], "--receiver") == 0) || (strcmp (argv[i], "-r") == 0)){

			double x = atof(argv[++i]);
			double y = atof(argv[++i]);
			double z = atof(argv[++i]);

			realReceiver.coords = Point<double>(x, y, z);
			receiver_setted = true;
		}
	}

	// TODO check if input is well formed

	return receiver_setted && (n_satellites >= 3);
}*/
