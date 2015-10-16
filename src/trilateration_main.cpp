/*
 * USAGE
 *
 * -- To run the gps receiver simulator
 * rosrun trilateration receiver_sim_node  -r 3 -3 2 -s 15 26 40 -s 1 -32 50 -s - -8 70 -s 24 73 56 -s -99 -66 708
 *
 * -- To run the trilateration node
 * rosrun trilateration trilateration_node
 *
 * -- To visualize results
 * rosrun rviz rviz
 */
#include <vector>
#include "trilateration_node.h"


//node main
int main(int argc, char **argv)
{
	//init ros
	ros::init(argc, argv, "trilateration_node");

	// Trilateration node
	TrilaterationNode trNode;

	ros::spin();
	//exit program
	return 0;
}
