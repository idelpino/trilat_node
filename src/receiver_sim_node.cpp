#include "receiver_sim_node.h"


ReceiverSimNode::ReceiverSimNode(Receiver r):
	nh(ros::this_node::getName()),
	realRec(r)		// Initialize receiver with default value
{
	// Initialize measurements publisher
	// TODO for now i work with strings
	measurementsPub = nh.advertise<std_msgs::String>("gps_measurements", 1000);

}

ReceiverSimNode::~ReceiverSimNode() { }

void ReceiverSimNode::move()
{
	realRec.coords = realRec.coords + Point<double>(0, 0, 1);
	std::cout << "Real receiver moved to: " << realRec.toString() << "\n";
}

void ReceiverSimNode::simulateMeasurements(const std::vector<Point<double>> satellites, const double std_dev, const double speed)
{
	measurements = Trilateration::simulateMeasurements(realRec, satellites, std_dev, speed);
}

void ReceiverSimNode::publishMeasurements()
{
	std_msgs::String msg;

	std::stringstream ss;

	ss << "MEASUREMENTS:\n";
	for (int i = 0; i < measurements.size(); ++i) {
		ss << i << ")\t" << measurements.at(i).toString() << std::endl;
	}
	msg.data = ss.str();

	std::cout << "Publishing\n";
	measurementsPub.publish(msg);
}

void ReceiverSimNode::publishRealReceiver()
{
//	TODO
	std::cout << ">> publishRealReceiver() TODO\n";

/* TODO
 * aggiungi un nuovo publisher per questo topic
 * inizializzalo nel costruttore
 * includi msgs Marker
 *
 * creazione del marker
 * pubblicalo

	visualization_msgs::Marker m;
	m.header.frame_id = "my_frame";
	m.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "receiver";
	m.id = 1;

	// Set the marker type.
	m.type = visualization_msgs::Marker::SPHERE;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	m.pose.position.x = r.coords.getX();
	m.pose.position.y = r.coords.getY();
	m.pose.position.z = r.coords.getZ();
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

	markerPub.publish(m);*/



}

void ReceiverSimNode::setRealRec(const Receiver &value)
{
	realRec = value;
}
