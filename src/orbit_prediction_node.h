#ifndef ORBITPREDICTIONNODE_H
#define ORBITPREDICTIONNODE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
#include "trilateration/satMeasurement.h"
#include "trilateration/satMeasurementArray.h"

#include "../include/trilateration/src/trilateration.h"
#include "../include/trilateration/src/structs.h"

#include "../include/rinex_reader/src/rinex_reader.h"


class OrbitPredictionNode
{
public:
	OrbitPredictionNode(char *path_obs, char *path_nav);


protected:

	RinexReader rr;

	// ROS node handle
	ros::NodeHandle nh;

	// Publisher for measurements
	ros::Publisher measurementsPub;



};

#endif // ORBITPREDICTIONNODE_H
