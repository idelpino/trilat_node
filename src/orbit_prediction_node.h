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

	void processNextEpoch();

	// compute position of sats at time observation + offset
	void computeSatsPositionAfter(double offset);

	// publish sats list
	void publishSatsPosition();

	// publish a sphere that represents the earth
	void publishEarth();

	// to decide the scale of visualization
	void setScale(double value);



public:
	const double EARTH_RADIUS = 6371000; // meters
	const double METERS = 1;
	const double KILOMETERS = METERS/1000;
	const double MEGAMETERS = KILOMETERS/1000;

protected:

	RinexReader rr;

	std::vector<SatelliteMeasurement> sats;

	// ROS node handle
	ros::NodeHandle nh;

	// Publisher
	ros::Publisher markerPub;



	// provv: per gestire le dimensioni di stampa. forse e' meglio un enum
	double scale;
};

#endif // ORBITPREDICTIONNODE_H
