#include "trilat_node.h"


TrilatNode::TrilatNode() : nh(ros::this_node::getName())
{
	std::cout << "Costruttore ros node\n";
	count = 0;

	markerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);


}

TrilatNode::~TrilatNode() {}


void TrilatNode::process()
{
	std::cout << ++count << std::endl;
}


void TrilatNode::publishMarker()
{
	visualization_msgs::Marker mrk = createMarker(Point<double>(0, 0, count));

	markerPub.publish(mrk);
}








visualization_msgs::Marker TrilatNode::createMarker(Point<double> coords)
{
	visualization_msgs::Marker m;
	m.header.frame_id = "my_frame";
	m.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "basic_shapes";
	m.id = 0;

	// Set the marker type.
	m.type = visualization_msgs::Marker::SPHERE;

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
	m.color.r = 0.0f;
	m.color.g = 1.0f;
	m.color.b = 0.0f;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();
	return m;
}



