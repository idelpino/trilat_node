#ifndef TRILATERATION_NODE_H
#define TRILATERATION_NODE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "../include/trilateration/src/Trilateration.h"
#include "../include/trilateration/src/structs.h"


class TrilatNode
{
protected:

	// Trilateration object
	Trilateration tr;

	// ROS node handle
	ros::NodeHandle nh;

	// Publisher
	ros::Publisher markerPub;

public:
	//constructor
	TrilatNode();

	//destructor
	~TrilatNode();

	//execute point tracker
	void process();

	void publishMarker();

	visualization_msgs::Marker createMarker(Point<double> coords);

};

#endif // TRILATERATION_NODE_H
