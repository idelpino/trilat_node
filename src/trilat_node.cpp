#include "trilat_node.h"


TrilatNode::TrilatNode() : nh(ros::this_node::getName())
{
	// Initialize marker publisher
	markerPub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);
}

TrilatNode::~TrilatNode() {}


void TrilatNode::process(const std::vector<SatelliteMeasurement> &measurements, const double speed)
{
	estReceiver = tr.computePosition(measurements, speed);
	std::cout << "Estimated receiver:\t" << estReceiver.toString() << std::endl;

	// in the next simulation, the guess is the actual position
	tr.setInitialReceiverGuess(estReceiver);

}

void TrilatNode::publishSatellites(const std::vector<SatelliteMeasurement> &measurements)
{
	for (int i = 0; i < measurements.size(); ++i) {
		publishSatellite(measurements[i].coords, i);
	}
}

void TrilatNode::publishSatellites(const std::vector<Point<double> > &sats)
{
	for (int i = 0; i < sats.size(); ++i) {
		publishSatellite(sats[i], i);
	}
}

void TrilatNode::publishSatellite(const Point<double> &coords, int i)
{
	visualization_msgs::Marker m;
	m.header.frame_id = "my_frame";
	m.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "satellite";
	m.id = i;

	// Set the marker type.
	m.type = visualization_msgs::Marker::CUBE;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	m.pose.position.x = coords.getX();
	m.pose.position.y = coords.getY();
	m.pose.position.z = coords.getZ();
	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	m.scale.x = 1.0;
	m.scale.y = 1.0;
	m.scale.z = 1.0;

	// Set the color -- be sure to set alpha to something non-zero!
	m.color.r = 1.0f;
	m.color.g = 1.0f;
	m.color.b = 0.0f;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();

	markerPub.publish(m);
}

/* TODO
 * metti il marker come variabile privata della classe,
 * la inizializzi con il costruttore
 * quando ricalcoli, aggiorni solo le coordinate
 */
void TrilatNode::publishRealReceiver(Receiver r)
{
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

	markerPub.publish(m);
}

void TrilatNode::publishEstReceiver()
{
	visualization_msgs::Marker m;
	m.header.frame_id = "my_frame";
	m.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "receiver";
	m.id = 0;

	// Set the marker type.
	m.type = visualization_msgs::Marker::SPHERE;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	m.pose.position.x = estReceiver.coords.getX();
	m.pose.position.y = estReceiver.coords.getY();
	m.pose.position.z = estReceiver.coords.getZ();
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
	m.color.b = 0.0f;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();

	markerPub.publish(m);
}


