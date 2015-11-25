#include "orbit_prediction_node.h"

using namespace std;

///
/// \brief OrbitPredictionNode::OrbitPredictionNode
/// \param path_obs
/// \param path_nav
///
OrbitPredictionNode::OrbitPredictionNode(char *path_obs, char *path_nav):
	nh(ros::this_node::getName()),
	rr(path_obs, path_nav)
{
	// Initialize measurements publisher
	markerPub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 5000);
	odomAllPub = nh.advertise<nav_msgs::Odometry>("/odom_all", 50);

	scale = KILOMETERS;

	currentTime = ros::Time::now(); // todo forse togli
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

	cout << RinexReader::timePretty(rr.getEpochTime()) << " <--- from rinex file" << endl;

	currentTime = ros::Time::now();//todo dovrebbe essere = getEpochTime

	initOdomPublishers();

	return ! rr.isFileFinished();
}


void OrbitPredictionNode::computeSatsPositionAfter(double offset)
{
	rr.updateMeasurementAtTime(rr.getEpochTime() + offset);
	currentTime = ros::Time::now();//todo dovrebbe essere = getEpochTime + offset

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
	m.header.frame_id = WORLD_FRAME;
	m.header.stamp = currentTime;

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

///
/// \brief OrbitPredictionNode::setScale
/// \param value
///
void OrbitPredictionNode::setScale(double value)
{
	scale = value;
}

///
/// \brief OrbitPredictionNode::publishSat
/// \param index
///
void OrbitPredictionNode::publishSat(int index)
{
	visualization_msgs::Marker m;
	m.header.frame_id = WORLD_FRAME;
	m.header.stamp = currentTime;

	m.ns = "sats";
	m.id = index;

	m.type = visualization_msgs::Marker::CUBE;//SPHERE;

	m.action = visualization_msgs::Marker::ADD;

	// Pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	m.pose.position.x = sats[index].pos.getX() * scale;
	m.pose.position.y = sats[index].pos.getY() * scale;
	m.pose.position.z = sats[index].pos.getZ() * scale;
	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0; // todo dovrei mettere la stessa posa del suo frame

	m.scale.x =  m.scale.y = m.scale.z = 1000;

	m.color.r = 1.0f;
	m.color.g = 0.0f;
	m.color.b = 0.0f;
	m.color.a = 0.5;

	m.lifetime = ros::Duration();

	markerPub.publish(m);


	publishSatVelocityThroughEndings(index); //basato su 2 punti estremi
	publishSatVelocity(index);//basato su quaternione



	Eigen::Quaterniond rotation = rotateSatelliteFrame(index);


	publishOdometry(index, rotation);

}

///
/// \brief OrbitPredictionNode::publishSatVelocity
/// \param index
///
/// Print a marker representing the velocity vector using the 2 endings
///
void OrbitPredictionNode::publishSatVelocityThroughEndings(int index)
{
	visualization_msgs::Marker m;
	m.header.frame_id = WORLD_FRAME;
	m.header.stamp = currentTime;

	m.ns = "velocity_endings";
	m.id = index;

	m.type = visualization_msgs::Marker::ARROW;

	m.action = visualization_msgs::Marker::ADD;

	geometry_msgs::Point p1;
	p1.x = sats[index].pos.getX() * scale;
	p1.y = sats[index].pos.getY() * scale;
	p1.z = sats[index].pos.getZ() * scale;
	m.points.push_back(p1);

	geometry_msgs::Point p2;
	p2.x = (sats[index].pos.getX() + vel[index][0]*500) * scale;
	p2.y = (sats[index].pos.getY() + vel[index][1]*500) * scale;
	p2.z = (sats[index].pos.getZ() + vel[index][2]*500) * scale;
	m.points.push_back(p2);

	m.scale.x = 100000*scale;
	m.scale.y = 100000*scale;
	m.scale.z = 0;

	m.color.r = 1.0f;
	m.color.g = 1.0f;
	m.color.b = 0.0f;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();

	markerPub.publish(m);
}

///
/// \brief OrbitPredictionNode::publishSatVelocity
/// \param index
///
/// Print a marker representing the velocity vector through quaternion
///
void OrbitPredictionNode::publishSatVelocity(int index)
{
	visualization_msgs::Marker m;
	m.header.frame_id = getSatelliteFrame(index);
	m.header.stamp = currentTime;

	m.ns = "velocity";
	m.id = index;

	m.type = visualization_msgs::Marker::ARROW;

	m.action = visualization_msgs::Marker::ADD;

	// Pose of the marker.
	m.pose.position.x = 0;
	m.pose.position.y = 0;
	m.pose.position.z = 0;
	m.pose.orientation.x = 1;
	m.pose.orientation.y = 0;
	m.pose.orientation.z = 0;
	m.pose.orientation.w = 0;

	m.scale.x = 3000;//2000;
	m.scale.y = 200;//100;
	m.scale.z = 200;//100;

	m.color.r = 0.0f;
	m.color.g = 1.0f;
	m.color.b = 1.0f;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();

	markerPub.publish(m);
}

///
/// \brief OrbitPredictionNode::publishEarthVector
/// \param index
/// \param earth
///
void OrbitPredictionNode::publishEarthVector(int index, const Eigen::Vector3d &earth)
{
	visualization_msgs::Marker m;
	m.header.frame_id = getSatelliteFrame(index);
	m.header.stamp = currentTime;

	m.ns = "earthPointer";
	m.id = index;

	m.type = visualization_msgs::Marker::ARROW;

	m.action = visualization_msgs::Marker::ADD;

	// Find quaternion that point to earth
	Eigen::Quaterniond quatEarth;
	quatEarth.setFromTwoVectors(Eigen::Vector3d::UnitX(), earth);

	m.pose.position.x = 0;
	m.pose.position.y = 0;
	m.pose.position.z = 0;
	m.pose.orientation.x = quatEarth.x();
	m.pose.orientation.y = quatEarth.y();
	m.pose.orientation.z = quatEarth.z();
	m.pose.orientation.w = quatEarth.w();

	m.scale.x = 3000;
	m.scale.y = 200;
	m.scale.z = 200;

	m.color.r = 0.0f;
	m.color.g = 0.5f;
	m.color.b = 1.0f;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();

	markerPub.publish(m);


///TODO rimetti se vuoi stampare anche il cross product
//	earth = earth.cross(Eigen::Vector3d::UnitX());
//	quatY2Earth.setFromTwoVectors(Eigen::Vector3d::UnitX(), earth);
//	m.pose.orientation.x = quatY2Earth.x();
//	m.pose.orientation.y = quatY2Earth.y();
//	m.pose.orientation.z = quatY2Earth.z();
//	m.pose.orientation.w = quatY2Earth.w();
//	m.ns = "cross_productSERIO";

//	markerPub.publish(m);
}

/// TODO  ******* TODO ****** TODO ****** TODO ****** TODO ****** TODO **********
/// TODO  *																		*
/// TODO  *																		*
/// TODO  *	Prova a vedere se conviene NON routare i frame di ogni satellite	*
/// TODO  *	ma lasciarli fissi e solamente trovare i vettori terra e velocita'	*
/// TODO  *	In questo caso non servirebbe neanche fare conti complicati per		*
/// TODO  *	trovare il centro della terra. (ora serve fare quat->rotationMatrix	*																*
/// TODO  *	poi invertirla e moltiplicare										*
/// TODO  *																		*
/// TODO  *																		*
/// TODO  ***********************************************************************


///
/// \brief OrbitPredictionNode::rotateSatelliteFrame Rotate the satellite's frame
///		in order to allign x axes with satellite's velocity
/// \param index
/// \return quaternion representing rotation
///
Eigen::Quaterniond OrbitPredictionNode::rotateSatelliteFrame(int index)
{
	Eigen::Quaterniond rotation;

	Eigen::Vector3d satVelocity(vel[index][0] * scale, vel[index][1] * scale, vel[index][2] * scale);

	//quaternione che fa ruotare l'asse x in modo che combaci con il vettore velocita'
	rotation.setFromTwoVectors(Eigen::Vector3d::UnitX(), satVelocity);


	//TODO prova a concatenare un'altra rotazione in modo che x=velocita' e y=puntatore alla terra
	//	Eigen::Vector3d translation(sats[index].pos.getX() * scale, sats[index].pos.getY() * scale, sats[index].pos.getZ() * scale);
	//	Eigen::Vector3d earth = findWorldFromSatelliteSERIO(index, translation, rotationX2Direction);
	//	Eigen::Quaterniond quatY2Earth;
	//	quatY2Earth.setFromTwoVectors(Eigen::Vector3d::UnitY(), earth);
	//	rotationX2Direction = rotationX2Direction*quatY2Earth;



	geometry_msgs::Quaternion odom_quat;
	odom_quat.x = rotation.x();
	odom_quat.y = rotation.y();
	odom_quat.z = rotation.z();
	odom_quat.w = rotation.w();


	// Publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = currentTime;
	odom_trans.header.frame_id = WORLD_FRAME;				//frame padre
	odom_trans.child_frame_id = getSatelliteFrame(index);	//frame figlio (che sto creando ora)
	odom_trans.transform.translation.x = sats[index].pos.getX() * scale;//traslazione dell'origine
	odom_trans.transform.translation.y = sats[index].pos.getY() * scale;
	odom_trans.transform.translation.z = sats[index].pos.getZ() * scale;
	odom_trans.transform.rotation = odom_quat;							//rotazione

	//send the transform
	transBroadcaster.sendTransform(odom_trans);


	return rotation;
}

///
/// \brief OrbitPredictionNode::publishOdometry
/// \param index
/// \param rotation
///
void OrbitPredictionNode::publishOdometry(int index, const Eigen::Quaterniond &rotation)
{
	/// publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = WORLD_FRAME;

	//set the position
	odom.pose.pose.position.x = sats[index].pos.getX() * scale;
	odom.pose.pose.position.y = sats[index].pos.getY() * scale;
	odom.pose.pose.position.z = sats[index].pos.getZ() * scale;

	//set the orientation
	odom.pose.pose.orientation.x = rotation.x();
	odom.pose.pose.orientation.y = rotation.y();
	odom.pose.pose.orientation.z = rotation.z();
	odom.pose.pose.orientation.w = rotation.w();

	//set the velocity
	odom.child_frame_id = getSatelliteFrame(index);
	odom.twist.twist.linear.x = vel[index][0] * scale;
	odom.twist.twist.linear.y = vel[index][1] * scale;
	odom.twist.twist.linear.z = vel[index][2] * scale;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = 0;

	//publish the message
	odomPub[index].publish(odom);
	odomAllPub.publish(odom);

	//publish earth vector
	Eigen::Vector3d translation(sats[index].pos.getX() * scale, sats[index].pos.getY() * scale, sats[index].pos.getZ() * scale);
	Eigen::Vector3d earth = findEarthFromSatellite(index, translation, rotation);

	publishEarthVector(index, earth);
}




///
/// ??TODO fare una funzione che converta da mondo a frame e viceversa??
///


Eigen::Vector3d OrbitPredictionNode::findEarthFromSatellite(int index,
					const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation)
{
	Eigen::Vector3d pEarth(0, 0, 0); // punto che voglio trovare nelle altre coordinate
	Eigen::Vector3d pSat; // risultato

	pSat = rotation.toRotationMatrix().inverse() * (pEarth - translation);

	return pSat;
}


///
/// \brief OrbitPredictionNode::initOdomPublishers
/// TO BE CALLED EVERY TIME SATELLITES CHANGE!
///
/// TODO Comunque non mi piace molto come soluzione
/// perche'se perdo un satellite e il momento dopo ne aggiungo uno nuovo,
/// viene pubblicato nello stesso canale
/// potrei ad esempio concatenare il nome del satellite al posto che il numero
///
void OrbitPredictionNode::initOdomPublishers()
{
	odomPub.resize(sats.size());

	for (int i = 0; i < odomPub.size(); ++i) {

		std::stringstream ss;
		ss << "/odom" << i;

		odomPub[i] = nh.advertise<nav_msgs::Odometry>(ss.str(), 50);
	}
}

///
/// \brief OrbitPredictionNode::getSatelliteFrame
/// \param index
/// \return
///
std::string OrbitPredictionNode::getSatelliteFrame(int index)
{
	std::stringstream ss;
	ss << "sat_" << index;
	return ss.str();
}

