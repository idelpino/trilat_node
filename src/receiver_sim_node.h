#ifndef RECEIVERSIMNODE_H
#define RECEIVERSIMNODE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
/* TODO
 * devo farmi il mio messaggio particolare
 * vettore di measurement
 */

#include "../include/trilateration/src/Trilateration.h"
#include "../include/trilateration/src/structs.h"

//TODO sistema. sta variabile c'e' sia nel main che nella classe
const Receiver DEF_REAL_RECEIVER = {Point<double>(0, 0, 0), 100e-9};

class ReceiverSimNode
{
protected:
	Receiver realRec; // contains coords and true bias
	std::vector<SatelliteMeasurement> measurements;


	// ROS node handle
	ros::NodeHandle nh;

	// Publisher for measurements
	ros::Publisher measurementsPub;

	// Publisher for real position of the receiver
	ros::Publisher markerPub;


public:
	//constructor
	ReceiverSimNode(Receiver r = DEF_REAL_RECEIVER);

	//destructor
	~ReceiverSimNode();

	void move();

	void simulateMeasurements(const std::vector<Point<double>> satellites, const double std_dev, const double speed);

	void publishMeasurements();

	void publishRealReceiver();

	void setRealRec(const Receiver &value);
};

#endif // RECEIVERSIMNODE_H
