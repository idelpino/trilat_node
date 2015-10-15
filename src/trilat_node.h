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

	// Result of the trilateration
	Receiver estReceiver;

public:
	//constructor
	TrilatNode();

	//destructor
	~TrilatNode();

	//execute point tracker
	void process(const std::vector<SatelliteMeasurement> &measurements, const double speed);

	void publishSatellites(const std::vector<SatelliteMeasurement> &measurements);
	void publishSatellites(const std::vector<Point<double>> &sats);

	void publishRealReceiver(Receiver r);

	void publishEstReceiver();

private:
	void publishSatellite(const Point<double> &coords, int i);

};

#endif // TRILATERATION_NODE_H
