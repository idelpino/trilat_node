#include "orbit_prediction_node.h"

using namespace std;

OrbitPredictionNode::OrbitPredictionNode(char *path_obs, char *path_nav):
	nh(ros::this_node::getName()),
	rr(path_obs, path_nav)
{
	// Initialize measurements publisher
	markerPub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);


	scale = KILOMETERS;
}

void OrbitPredictionNode::processNextEpoch()
{
	rr.processNextEpoch();
}

void OrbitPredictionNode::computeSatsPositionAfter(double offset)
{
	rr.computeSatPosition(rr.getEpochTime() + offset);
	sats = rr.getMeasurements();
}

void OrbitPredictionNode::publishSatsPosition()
{
	//TODO
}

void OrbitPredictionNode::publishEarth()
{
	cout << "earth published" << endl;

	visualization_msgs::Marker m;
	m.header.frame_id = "my_frame";
	m.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "earth";
	m.id = 0;

	// Set the marker type.
	m.type = visualization_msgs::Marker::SPHERE;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	m.pose.position.x = 0;
	m.pose.position.y = 0;
	m.pose.position.z = 0;
	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	m.scale.x = EARTH_RADIUS * scale;
	m.scale.y = EARTH_RADIUS * scale;
	m.scale.z = EARTH_RADIUS * scale;

	// Set the color -- be sure to set alpha to something non-zero!
	m.color.r = 0.0f;
	m.color.g = 1.0f;
	m.color.b = 0.0f;
	m.color.a = 0.1;

	m.lifetime = ros::Duration();

	markerPub.publish(m);
}

void OrbitPredictionNode::setScale(double value)
{
	scale = value;
}

