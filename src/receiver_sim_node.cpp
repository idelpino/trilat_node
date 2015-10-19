#include "receiver_sim_node.h"


ReceiverSimNode::ReceiverSimNode(Receiver r):
	nh(ros::this_node::getName()),
	realRec(r)		// Initialize receiver with default value
{
	// Initialize measurements publisher
	measurementsPub = nh.advertise<trilateration::satMeasurementArray>("/gps_measurements", 1000);

	// Initialize real position publisher
	markerPub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);




}

ReceiverSimNode::~ReceiverSimNode() { }

// move receiver of a delta
void ReceiverSimNode::move(double dx, double dy, double dz)
{
	realRec.coords = realRec.coords + Point<double>(dx, dy, dz);
	std::cout << "Real receiver moved to: " << realRec.toString() << "\n";

	/*TODO
	 * potrei fare che muovendosi trova nuovi satelliti e ne perde altri
	 *
	 * potrei fare un movimento piu' sensato
	 */
}

void ReceiverSimNode::moveTo(double x, double y, double z)
{
	realRec.coords = Point<double>(x, y, z);
	std::cout << "Real receiver moved to: " << realRec.toString() << "\n";
}

void ReceiverSimNode::simulateMeasurements(const std::vector<Point<double>> satellites, const double std_dev, const double speed)
{
	measurements = tr.simulateMeasurements(realRec, satellites, std_dev, speed);
}

void ReceiverSimNode::publishMeasurements()
{
	//std::cout << "Publishing measurements\n";

	trilateration::satMeasurement meas;
	trilateration::satMeasurementArray msg;

	for (int i = 0; i < measurements.size(); ++i) {
		meas.x = measurements.at(i).coords.getX();
		meas.y = measurements.at(i).coords.getY();
		meas.z = measurements.at(i).coords.getZ();
		meas.pseudorange = measurements.at(i).pseudorange;

		msg.measurements.push_back(meas);
	}

	measurementsPub.publish(msg);
}

void ReceiverSimNode::publishRealReceiver()
{
	//std::cout << "Publishing real receiver\n";

	visualization_msgs::Marker m;
	m.header.frame_id = "my_frame";
	m.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "receiver_real";
	m.id = 0;

	// Set the marker type.
	m.type = visualization_msgs::Marker::SPHERE;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	m.pose.position.x = realRec.coords.getX();
	m.pose.position.y = realRec.coords.getY();
	m.pose.position.z = realRec.coords.getZ();
	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	m.scale.x = 1.0;
	m.scale.y = 1.0;
	m.scale.z = 1.0;

	// Set the color -- be sure to set alpha to something non-zero!
	m.color.r = 0.0f;
	m.color.g = 1.0f;
	m.color.b = 1.0f;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();

	markerPub.publish(m);

}

void ReceiverSimNode::setRealRec(const Receiver &value)
{
	realRec = value;
}
