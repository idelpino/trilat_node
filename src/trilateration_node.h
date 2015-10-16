#ifndef TRILATERATION_NODE_H
#define TRILATERATION_NODE_H

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include "trilateration/satMeasurement.h"
#include "trilateration/satMeasurementArray.h"

#include "../include/trilateration/src/Trilateration.h"
#include "../include/trilateration/src/structs.h"


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

	// Subscriber for measurements
	ros::Subscriber measurementSub;

public:
	TrilaterationNode();
	~TrilaterationNode();

	void measurementsCallback(const trilateration::satMeasurementArray::ConstPtr &msg);


};

#endif // TRILATERATION_NODE_H
