#include "trilateration_node.h"

void TrilaterationNode::measurementsCallback(const trilateration::satMeasurementArray::ConstPtr &msg)
{
	std::vector<SatelliteMeasurement> v;

	std::cout << "New message:\n";
	for (int i = 0; i < msg->measurements.size(); ++i) {
		const trilateration::satMeasurement data = msg->measurements[i];

		ROS_INFO_STREAM("x: " << data.x << "\ty: " << data.y << "\tz: " << data.z << "\tpseudorange: " << data.pseudorange);

		SatelliteMeasurement m = {
			Point<double>(data.x, data.y, data.z),
			data.pseudorange
		};

		v.push_back(m);

	}


	//TODO fai qualcosa

}

TrilaterationNode::TrilaterationNode() : nh(ros::this_node::getName())
{
	measurementSub = nh.subscribe("/gps_measurements", 1000, &TrilaterationNode::measurementsCallback, this);

}

TrilaterationNode::~TrilaterationNode() { }


