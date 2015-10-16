
#include <vector>
#include "trilateration_node.h"


//node main
int main(int argc, char **argv)
{
	//init ros
	ros::init(argc, argv, "trilateration_node");

	// Trilateration node
	TrilaterationNode trNode;

	ros::spin();// TODO provv
/*
	ros::Rate loopRate(1);
	//node loop
	while ( ros::ok() )
	{
		//do things

		//execute pending callbacks
		ros::spinOnce();

		//relax to fit output rate
		loopRate.sleep();
	}*/

	//exit program
	return 0;
}
