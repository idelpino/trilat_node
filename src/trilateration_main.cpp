/*
 * USAGE:
 *
 * -- To run the gps receiver simulator
 * rosrun trilateration receiver_sim_node -r 2 2 2 -s -25 36 140 -s 10 -32 150 -s -60 -8 170 -s 24 55 156 -s 99 16 188
 * rosrun trilateration receiver_sim_node -r 2 2 2 -s -2500 3600 14000 -s 1000 -3200 15000 -s -6000 -800 17000 -s 2400 5500 15600 -s 9900 1600 18800
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

	return 0;
}
