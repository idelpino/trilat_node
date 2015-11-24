#include "orbit_prediction_node.h"

using namespace std;

OrbitPredictionNode::OrbitPredictionNode(char *path_obs, char *path_nav):
	nh(ros::this_node::getName()),
	rr(path_obs, path_nav),
	transListener(ros::Duration(2))
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

	cout << RinexReader::timePretty(rr.getEpochTime()) << " <---" << endl;

	currentTime = ros::Time::now();//dovrebbe essere = getEpochTime



	initOdomPublishers();


	return ! rr.isFileFinished();
}

void OrbitPredictionNode::computeSatsPositionAfter(double offset)
{
	rr.updateMeasurementAtTime(rr.getEpochTime() + offset);
	currentTime = ros::Time::now();//dovrebbe essere = getEpochTime + offset

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

void OrbitPredictionNode::setScale(double value)
{
	scale = value;
}

void OrbitPredictionNode::publishSat(int index)
{
	visualization_msgs::Marker m;
	m.header.frame_id = WORLD_FRAME;
	m.header.stamp = currentTime;

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "sats";
	m.id = index;

	// Set the marker type.
	m.type = visualization_msgs::Marker::CUBE;//SPHERE;

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
	m.color.a = 0.5;

	m.lifetime = ros::Duration();

	markerPub.publish(m);


	publishSatVelocity(index); //basato su 2 punti estremi
	publishSatVelocity2(index);//basato su quaternione

	publishOdometry(index);

	//publishEarthVector(index);

}

///
/// \brief OrbitPredictionNode::publishSatVelocity
/// \param index
///
/// Print a marker representing the velocity vector
/// using the 2 endings
///
void OrbitPredictionNode::publishSatVelocity(int index)
{
	visualization_msgs::Marker m;
	m.header.frame_id = WORLD_FRAME;
	m.header.stamp = currentTime;

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "vel";
	m.id = index;

	// Set the marker type.
	m.type = visualization_msgs::Marker::ARROW;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m.action = visualization_msgs::Marker::ADD;

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

///
/// \brief OrbitPredictionNode::publishSatVelocity2
/// \param index
///
/// Print a marker representing the velocity vector
/// through quaternion
///
void OrbitPredictionNode::publishSatVelocity2(int index)
{
	visualization_msgs::Marker m;
	m.header.frame_id = getSatelliteName(index);
	m.header.stamp = currentTime;

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "vel_2";
	m.id = index;

	// Set the marker type.
	m.type = visualization_msgs::Marker::ARROW;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m.action = visualization_msgs::Marker::ADD;

	///
	/// Disegna freccia con position + orientation
	///
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	m.pose.position.x = 0;
	m.pose.position.y = 0;
	m.pose.position.z = 0;
	m.pose.orientation.x = 1;
	m.pose.orientation.y = 0;
	m.pose.orientation.z = 0;
	m.pose.orientation.w = 0;
	m.scale.x = 4000;//2000;
	m.scale.y = 300;//100;
	m.scale.z = 300;//100;

	// Set the color -- be sure to set alpha to something non-zero!
	m.color.r = 0.0f;
	m.color.g = 1.0f;
	m.color.b = 1.0f;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();

	markerPub.publish(m);
}



///
/// STA ROBA FA SCHIFO PERCHE' USA IL LISTENER
///

//void OrbitPredictionNode::publishEarthVector(int index)
//{
//	visualization_msgs::Marker m;
//	m.header.frame_id = getSatelliteName(index);
//	m.header.stamp = currentTime;

//	// Set the namespace and id for this marker.  This serves to create a unique ID
//	// Any marker sent with the same namespace and id will overwrite the old one
//	m.ns = "earthPointer";
//	m.id = index;

//	// Set the marker type.
//	m.type = visualization_msgs::Marker::ARROW;

//	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
//	m.action = visualization_msgs::Marker::ADD;


//	//* METODO 2 (TODO)

////	geometry_msgs::PointStamped originale, trasformato;
////	originale.point.x=0;
////	originale.point.y=0;
////	originale.point.z=0;

////	listener.transformPoint(getSatelliteName(index), originale, trasformato);

////	cout << "ASD: " << trasformato << endl;



//	////////////////////////////

//	Eigen::Vector3d earthVector = findWorldFromSatellite(index);

//	//cout << earthVector << endl;

//	Eigen::Quaterniond quatY2Earth;

//	quatY2Earth.setFromTwoVectors(Eigen::Vector3d::UnitX(), earthVector);


//	///
//	/// Disegna freccia con position + orientation
//	///
//	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
//	m.pose.position.x = 0;
//	m.pose.position.y = 0;
//	m.pose.position.z = 0;
//	m.pose.orientation.x = quatY2Earth.x();
//	m.pose.orientation.y = quatY2Earth.y();
//	m.pose.orientation.z = quatY2Earth.z();
//	m.pose.orientation.w = quatY2Earth.w();
//	m.scale.x = 4000;
//	m.scale.y = 300;
//	m.scale.z = 300;

//	// Set the color -- be sure to set alpha to something non-zero!
//	m.color.r = 1.0f;
//	m.color.g = 0.5f;
//	m.color.b = 0.5f;
//	m.color.a = 1.0;

//	m.lifetime = ros::Duration();

//	markerPub.publish(m);





//	earthVector = earthVector.cross(Eigen::Vector3d::UnitX());
//	quatY2Earth.setFromTwoVectors(Eigen::Vector3d::UnitX(), earthVector);
//	m.pose.orientation.x = quatY2Earth.x();
//	m.pose.orientation.y = quatY2Earth.y();
//	m.pose.orientation.z = quatY2Earth.z();
//	m.pose.orientation.w = quatY2Earth.w();
//	m.ns = "cross_product";

//	markerPub.publish(m);
//}

void OrbitPredictionNode::publishEarthVectorSERIO(int index, Eigen::Vector3d earth)
{
	visualization_msgs::Marker m;
	m.header.frame_id = getSatelliteName(index);
	m.header.stamp = currentTime;

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "earthPointerSERIO";
	m.id = index;

	// Set the marker type.
	m.type = visualization_msgs::Marker::ARROW;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m.action = visualization_msgs::Marker::ADD;


	////////////////////////////

	//Eigen::Vector3d earthVector = findWorldFromSatelliteSERIO(index, translation, rotation);

	Eigen::Quaterniond quatY2Earth;

	quatY2Earth.setFromTwoVectors(Eigen::Vector3d::UnitX(), earth);


	///
	/// Disegna freccia con position + orientation
	///
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	m.pose.position.x = 0;
	m.pose.position.y = 0;
	m.pose.position.z = 0;
	m.pose.orientation.x = quatY2Earth.x();
	m.pose.orientation.y = quatY2Earth.y();
	m.pose.orientation.z = quatY2Earth.z();
	m.pose.orientation.w = quatY2Earth.w();
	m.scale.x = 4000;
	m.scale.y = 300;
	m.scale.z = 300;

	// Set the color -- be sure to set alpha to something non-zero!
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



///
/// \brief OrbitPredictionNode::publishOdometry
/// \param index
///
/// Sta funzione ora non pubblica solamente l'odometria, ma ruota anche il
/// sistema di riferimento
/// sarebbe da separare in 2 funzioni
///  TODO
/// // TODO torna qui
void OrbitPredictionNode::publishOdometry(int index/*, Eigen::Vector3d &translation, Eigen::Quaterniond &rotation*/)
{
	//Eigen::Vector3d axesX(1, 0, 0);
	Eigen::Vector3d satVelocity(vel[index][0] * scale, vel[index][1] * scale, vel[index][2] * scale);
	Eigen::Quaterniond rotationX2Direction;


	//quaternione che fa ruotare l'asse x in modo che combaci con il vettore velocita'
	rotationX2Direction.setFromTwoVectors(Eigen::Vector3d::UnitX(), satVelocity);


	//////// TENTATIVO FALLITO
////	Eigen::Vector3d earthPos(-sats[index].pos.getX() * scale, -sats[index].pos.getY() * scale, -sats[index].pos.getZ() * scale);
////	Eigen::Quaterniond rotationY2Earth;
////	rotationY2Earth.setFromTwoVectors(Eigen::Vector3d::UnitY(), satVelocity);
////	rotationX2Direction = rotationX2Direction * rotationY2Earth;

	geometry_msgs::Quaternion odom_quat;
	odom_quat.x = rotationX2Direction.x();
	odom_quat.y = rotationX2Direction.y();
	odom_quat.z = rotationX2Direction.z();
	odom_quat.w = rotationX2Direction.w();


	///
	/// first, we'll publish the transform over tf
	///
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = currentTime;
	odom_trans.header.frame_id = WORLD_FRAME;				//frame padre
	odom_trans.child_frame_id = getSatelliteName(index);	//frame figlio (che sto creando ora)
	odom_trans.transform.translation.x = sats[index].pos.getX() * scale;//traslazione dell'origine
	odom_trans.transform.translation.y = sats[index].pos.getY() * scale;
	odom_trans.transform.translation.z = sats[index].pos.getZ() * scale;
	odom_trans.transform.rotation = odom_quat;							//rotazione

	//send the transform
	transBroadcaster.sendTransform(odom_trans);

	/*
	 * TODO: separa le 2 funzioni!!
	 *
	 * da qui in su e' la funzione che effettivamente ruota i frames
	 * dei satelliti.
	 *
	 * da qui in giu pubblica solamente il messaggio odometria per ros!
	 */

	///
	/// publish the odometry message over ROS
	///
	nav_msgs::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = WORLD_FRAME;

	//set the position
	odom.pose.pose.position.x = sats[index].pos.getX() * scale;
	odom.pose.pose.position.y = sats[index].pos.getY() * scale;
	odom.pose.pose.position.z = sats[index].pos.getZ() * scale;

	//set the orientation
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = getSatelliteName(index);
	odom.twist.twist.linear.x = vel[index][0] * scale;
	odom.twist.twist.linear.y = vel[index][1] * scale;
	odom.twist.twist.linear.z = vel[index][2] * scale;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = 0;

	//publish the message
	odomPub[index].publish(odom);
	odomAllPub.publish(odom);



	Eigen::Vector3d translation(sats[index].pos.getX() * scale, sats[index].pos.getY() * scale, sats[index].pos.getZ() * scale);


	Eigen::Vector3d earth = findWorldFromSatelliteSERIO(index, translation, rotationX2Direction);
	//findWorldFromSatellite(index);


	publishEarthVectorSERIO(index, earth);

}




///
/// ??TODO fare una funzione che converta da mondo a frame e viceversa??
///

Eigen::Vector3d OrbitPredictionNode::findWorldFromSatelliteSERIO(int index,
								Eigen::Vector3d translation, Eigen::Quaterniond rotation)
{
	Eigen::Vector3d pEarth(0, 0, 0); // punto che voglio trovare nelle altre coordinate
	Eigen::Vector3d pSat; // risultato

	pSat = rotation.toRotationMatrix().inverse() * (pEarth - translation);

	return pSat;
}

/// TODO
/// TODO
/// TODO
///
///// qui creo un listener e provo ad ascoltare le trasformazioni inviate
///
/// sta soluzione e' una merda, l'avevo usata perche non riuscivo
/// a calcolare direttamente la posizione con la matrice di rotazione.
///
/// la committo tanto per, se nel futuro dovesse servirmi.
///
/// PERO' VA ASSOLUTAMENTE CANCELLATA!!! CANCELLA TUTTA STA ROBA E LE FUNZIONI
/// CHE HANNO L'EQUIVALENTE DEL NOME SENZA 'SERIO'
///
///
/// TODO
/// TODO
/////
//Eigen::Vector3d OrbitPredictionNode::findWorldFromSatellite(int index)
//{
//	// Point that I want to convert
//	geometry_msgs::PointStamped originWorldFrame;
//	originWorldFrame.header.frame_id = WORLD_FRAME;
//	originWorldFrame.header.stamp = currentTime;//ros::Time(0);//ros::Time().now();
//	originWorldFrame.point.x = 0;
//	originWorldFrame.point.y = 0;
//	originWorldFrame.point.z = 0;

//	//to save the result
//	geometry_msgs::PointStamped originSatFrame;

//	try{
//		//transform originWorldFrame in a point in satellite's frame

//		transListener.waitForTransform(WORLD_FRAME, getSatelliteName(index), currentTime, ros::Duration(3.0));
//		transListener.transformPoint(getSatelliteName(index), originWorldFrame, originSatFrame);

////		ROS_INFO("worldFrame(%.2f, %.2f, %.2f)) -----> sat_%d: (%.2f, %.2f, %.2f) at time %.2f",
////				 originWorldFrame.point.x, originWorldFrame.point.y, originWorldFrame.point.z,
////				 index,
////				 originSatFrame.point.x, originSatFrame.point.y, originSatFrame.point.z,
////				 originSatFrame.header.stamp.toSec());

//	}
//	catch(tf::TransformException& ex){
//		ROS_ERROR("Received an exception trying to transform a point from \"world\" to \"sat_n\": %s", ex.what());
//	}

//	Eigen::Vector3d ret(originSatFrame.point.x, originSatFrame.point.y, originSatFrame.point.z);

//	return ret;
//}


///
/// \brief OrbitPredictionNode::initOdomPublishers
/// TO BE CALLED EVERY TIME SATELLITES CHANGE!
///
/// Comunque non mi piace molto come soluzione
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

std::string OrbitPredictionNode::getSatelliteName(int index)
{
	std::stringstream ss;
	ss << "sat_" << index;
	return ss.str();
}



//void OrbitPredictionNode::publishOdometry(int index)
//{
//	///TODO
//	/// in verita' dopo non vorro' un index come argomento,
//	/// perche devo pubblicare i messaggi in un topic
//	/// separato per ogni satellite

//	double x =  sats[index].pos.getX() * scale;
//	double y =  sats[index].pos.getY() * scale;
//	double z =  sats[index].pos.getZ() * scale;

//	double norm = sqrt(x*x + y*y + z*z);

//	double COS45 = 0.70710678118;

//	geometry_msgs::Quaternion odom_quat;
////	odom_quat.x = 0;
////	odom_quat.y = 0;
////	odom_quat.z = 0;
////	odom_quat.w = 1;
//	odom_quat.x = -x/norm * COS45;
//	odom_quat.y = -y/norm * COS45;
//	odom_quat.z = -z/norm * COS45;
//	odom_quat.w =  COS45;



////	double vx = vel[index][0];
////	double vy = vel[index][1];
////	double vz = vel[index][2];

////	double roll = 0;
////	double pitch = atan( sqrt(vx*vx + vy*vy ) / vz );
////	double yaw = atan (vx/-vy);

////	odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

//	///
//	/// first, we'll publish the transform over tf
//	///
//	geometry_msgs::TransformStamped odom_trans;
//	odom_trans.header.stamp = currentTime;
//	odom_trans.header.frame_id = WORLD_FRAME;
//	odom_trans.child_frame_id = getSatelliteName(index);

//	odom_trans.transform.translation.x = sats[index].pos.getX() * scale;
//	odom_trans.transform.translation.y = sats[index].pos.getY() * scale;
//	odom_trans.transform.translation.z = sats[index].pos.getZ() * scale;
//	odom_trans.transform.rotation = odom_quat;

//	//send the transform
//	odomBroadcaster.sendTransform(odom_trans);

//	///
//	/// publish the odometry message over ROS
//	///
//	nav_msgs::Odometry odom;
//	odom.header.stamp = currentTime;
//	odom.header.frame_id = WORLD_FRAME;

//	//set the position
//	odom.pose.pose.position.x = sats[index].pos.getX() * scale;
//	odom.pose.pose.position.y = sats[index].pos.getY() * scale;
//	odom.pose.pose.position.z = sats[index].pos.getZ() * scale;

//	//set the orientation
//	odom.pose.pose.orientation = odom_quat;

//	//set the velocity
//	odom.child_frame_id = getSatelliteName(index);
//	odom.twist.twist.linear.x = vel[index][0] * scale;
//	odom.twist.twist.linear.y = vel[index][1] * scale;
//	odom.twist.twist.linear.z = vel[index][2] * scale;
//	odom.twist.twist.angular.x = 0;
//	odom.twist.twist.angular.y = 0;
//	odom.twist.twist.angular.z = 0;

//	//publish the message
//	odomPub[index].publish(odom);
//	odomAllPub.publish(odom);
//}
//	double roll = 0;
//	double pitch = atan( sqrt(vx*vx + vy*vy ) / vz );
//	double yaw = atan (vx/-vy);
//	odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);


