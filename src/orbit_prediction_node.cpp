#include "orbit_prediction_node.h"

using namespace std;

OrbitPredictionNode::OrbitPredictionNode(char *path_obs, char *path_nav):
	nh(ros::this_node::getName()),
	rr(path_obs, path_nav)
{
	// Initialize measurements publisher
	markerPub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);
	odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

	scale = KILOMETERS;

	current_time = ros::Time::now(); // todo forse togli
}

///
/// \brief OrbitPredictionNode::processNextEpoch
/// \return true if the file IS NOT finished
///
bool OrbitPredictionNode::processNextEpoch()
{
	rr.processNextEpoch();
	sats = rr.getMeasurements();
	vel = rr.getSatVelocities();

	cout << RinexReader::timePretty(rr.getEpochTime()) << " <---" << endl;

	current_time = ros::Time::now();//dovrebbe essere = getEpochTime

	return ! rr.isFileFinished();
}

void OrbitPredictionNode::computeSatsPositionAfter(double offset)
{
	rr.updateMeasurementAtTime(rr.getEpochTime() + offset);
	current_time = ros::Time::now();//dovrebbe essere = getEpochTime + offset

	sats = rr.getMeasurements();
	vel = rr.getSatVelocities();

	cout << RinexReader::timePretty(rr.getEpochTime() + offset) << endl;
}

void OrbitPredictionNode::publishSatsPositions()
{
	for (int i = 0; i < sats.size(); ++i) {
		publishSat(i);
	}
}

void OrbitPredictionNode::publishEarth()
{
	visualization_msgs::Marker m;
	m.header.frame_id = "ecef";
	m.header.stamp = current_time;

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

void OrbitPredictionNode::publishSat(int index)
{
	visualization_msgs::Marker m;
	m.header.frame_id = "ecef";
	m.header.stamp = current_time;

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "sats";
	m.id = index;

	// Set the marker type.
	m.type = visualization_msgs::Marker::SPHERE;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	m.pose.position.x = sats[index].pos.getX() * scale;
	m.pose.position.y = sats[index].pos.getY() * scale;
	m.pose.position.z = sats[index].pos.getZ() * scale;
	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	m.scale.x = 1000;
	m.scale.y = 1000;
	m.scale.z = 1000;

	// Set the color -- be sure to set alpha to something non-zero!
	m.color.r = 1.0f;
	m.color.g = 0.0f;
	m.color.b = 0.0f;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();

	markerPub.publish(m);


	publishSatVelocity(index);
	publishXAxis(index);
	publishOdometry(index);

}

void OrbitPredictionNode::publishSatVelocity(int index)
{
	visualization_msgs::Marker m;
	m.header.frame_id = "ecef";
	m.header.stamp = current_time;

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "vel";
	m.id = index;

	// Set the marker type.
	m.type = visualization_msgs::Marker::ARROW;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
//	m.pose.position.x = sats[index].pos.getX() * scale;
//	m.pose.position.y = sats[index].pos.getY() * scale;
//	m.pose.position.z = sats[index].pos.getZ() * scale;

//	m.pose.orientation.x = (sats[index].pos.getX() + vel[index][0]) * scale;
//	m.pose.orientation.y = (sats[index].pos.getY() + vel[index][1]) * scale;
//	m.pose.orientation.z = (sats[index].pos.getZ() + vel[index][2]) * scale;
//	m.pose.orientation.w = 1.0;

	geometry_msgs::Point p1;
	p1.x = sats[index].pos.getX() * scale;
	p1.y = sats[index].pos.getY() * scale;
	p1.z = sats[index].pos.getZ() * scale;

	geometry_msgs::Point p2;
	p2.x = (sats[index].pos.getX() + vel[index][0]*500) * scale;
	p2.y = (sats[index].pos.getY() + vel[index][1]*500) * scale;
	p2.z = (sats[index].pos.getZ() + vel[index][2]*500) * scale;

	m.points.push_back(p1);
	m.points.push_back(p2);


	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	m.scale.x = 100000*scale;
	m.scale.y = 100000*scale;
	m.scale.z = 0;

	// Set the color -- be sure to set alpha to something non-zero!
	m.color.r = 1.0f;
	m.color.g = 1.0f;
	m.color.b = 0.0f;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();

	markerPub.publish(m);
}

void OrbitPredictionNode::publishOdometry(int index)
{
	///TODO
	/// in verita' dopo non vorro' un index come argomento,
	/// perche devo pubblicare i messaggi in un topic
	/// separato per ogni satellite

	///
	/// first, we'll publish the transform over tf
	///
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "ecef";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = sats[index].pos.getX() * scale;
	odom_trans.transform.translation.y = sats[index].pos.getY() * scale;
	odom_trans.transform.translation.z = sats[index].pos.getZ() * scale;
	odom_trans.transform.rotation.x = 1;
	odom_trans.transform.rotation.y = 0;
	odom_trans.transform.rotation.z = 0;
	odom_trans.transform.rotation.w = 0;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

	///
	/// publish the odometry message over ROS
	///
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "ecef";

	//set the position
	odom.pose.pose.position.x = sats[index].pos.getX() * scale;
	odom.pose.pose.position.y = sats[index].pos.getY() * scale;
	odom.pose.pose.position.z = sats[index].pos.getZ() * scale;

	//set the orientation
	odom.pose.pose.orientation.x = 1;
	odom.pose.pose.orientation.y = 0;
	odom.pose.pose.orientation.z = 0;
	odom.pose.pose.orientation.w = 0;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vel[index][0] * scale;
	odom.twist.twist.linear.y = vel[index][1] * scale;
	odom.twist.twist.linear.z = vel[index][2] * scale;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = 0;

	//publish the message
	odom_pub.publish(odom);
}


void OrbitPredictionNode::publishXAxis(int index)
{
	visualization_msgs::Marker m;
	m.header.frame_id = "ecef";
	m.header.stamp = ros::Time::now();// dovrebbe essere = current_time

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "quat(1,0,0,0)";
	m.id = index;

	// Set the marker type.
	m.type = visualization_msgs::Marker::ARROW;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	m.pose.position.x = sats[index].pos.getX() * scale;
	m.pose.position.y = sats[index].pos.getY() * scale;
	m.pose.position.z = sats[index].pos.getZ() * scale;

	//double norm = sqrt(vel[index][0]*vel[index][0] + vel[index][1]*vel[index][1] + vel[index][2]*vel[index][2]);


	m.pose.orientation.x = 1;//vel[index][0] / norm;
	m.pose.orientation.y = 0;//vel[index][1] / norm;
	m.pose.orientation.z = 0;//vel[index][2] / norm;
	m.pose.orientation.w = 0;

//	double roll = 0;
//	double pitch = atan( sqrt(vx*vx + vy*vy ) / vz );
//	double yaw = atan (vx/-vy);

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	m.scale.x = 2000 /* * scale*/;
	m.scale.y = 100 /* * scale*/;
	m.scale.z = 100 /* * scale*/;

	// Set the color -- be sure to set alpha to something non-zero!
	m.color.r = 0.0f;
	m.color.g = 0.0f;
	m.color.b = 1.0f;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();

	markerPub.publish(m);

}
