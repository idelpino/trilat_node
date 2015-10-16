#ifndef RECEIVERSIMNODE_H
#define RECEIVERSIMNODE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
#include "trilateration/satMeasurement.h"
#include "trilateration/satMeasurementArray.h"

/* TODO
 * devo farmi il mio messaggio particolare
 * vettore di measurement
 */

#include "../include/trilateration/src/Trilateration.h"
#include "../include/trilateration/src/structs.h"


class ReceiverSimNode
{
protected:
	// contains coords and bias of the true receiver
	Receiver realRec;
	// measure obtained from satellites (simulated)
	std::vector<SatelliteMeasurement> measurements;


	// ROS node handle
	ros::NodeHandle nh;

	// Publisher for measurements
	ros::Publisher measurementsPub;

	// Publisher for real position of the receiver
	ros::Publisher markerPub;


public:
	//constructor
	ReceiverSimNode(Receiver r = {Point<double>(0, 0, 0), 100e-9});

	//destructor
	~ReceiverSimNode();

	void move();

	void simulateMeasurements(const std::vector<Point<double>> satellites, const double std_dev, const double speed);

	void publishMeasurements();

	void publishRealReceiver();

	void setRealRec(const Receiver &value);
};

#endif // RECEIVERSIMNODE_H
