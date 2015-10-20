#ifndef TRILATERATION_NODE_H
#define TRILATERATION_NODE_H

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include "trilateration/satMeasurement.h"
#include "trilateration/satMeasurementArray.h"

#include "../include/trilateration/src/trilateration.h"
#include "../include/trilateration/src/structs.h"

//TODO gestisci discorso velocita
const double SPEED_OF_LIGHT = 3e8; // m / s

class TrilaterationNode
{
protected:
	// Trilateration object
	Trilateration tr;

	// Result of the trilateration
	Receiver estReceiver;

	// Received measurements
	std::vector<SatelliteMeasurement> measurements;


	// ROS node handle
	ros::NodeHandle nh;

	// Subscriber (measurements)
	ros::Subscriber measurementSub;

	// Publisher (markers)
	ros::Publisher markerPub;

public:
	TrilaterationNode();
	~TrilaterationNode();

	void measurementsCallback(const trilateration::satMeasurementArray::ConstPtr &msg);

protected:
	void process();

	void publishEstReceiver();
	void publishSatellites();
};

#endif // TRILATERATION_NODE_H
