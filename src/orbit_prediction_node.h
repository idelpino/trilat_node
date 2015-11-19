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

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class OrbitPredictionNode
{
public:
	OrbitPredictionNode(char *path_obs, char *path_nav);

	bool processNextEpoch();

	// compute position of sats at time observation + offset
	void computeSatsPositionAfter(double offset);

	// publish sats list
	void publishSatsPositions();

	// publish a sphere that represents the earth
	void publishEarth();

	// to decide the scale of visualization
	void setScale(double value);

	void publishOdometry(int index); // TODO da togliere

protected:
	void publishSat(int index);
	void publishSatVelocity(int index);

	//TODO da togliere forse
	void publishXAxis(int index);


public:
	const double EARTH_RADIUS = 6371000; // meters
	const double METERS = 1;
	const double KILOMETERS = METERS/1000;
	const double MEGAMETERS = KILOMETERS/1000;

protected:
	RinexReader rr;

	ros::Time current_time;

	std::vector<SatelliteMeasurement> sats;
	std::vector<gpstk::Triple> vel;

	// ROS node handle
	ros::NodeHandle nh;

	// Publishers
	ros::Publisher markerPub;

	ros::Publisher odom_pub;
	tf::TransformBroadcaster odom_broadcaster;

	// provv: per gestire le dimensioni di stampa. forse e' meglio un enum
	double scale;
};

#endif // ORBITPREDICTIONNODE_H
