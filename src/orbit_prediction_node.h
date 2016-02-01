#ifndef ORBITPREDICTIONNODE_H
#define ORBITPREDICTIONNODE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "trilateration/satMeasurement.h"
#include "trilateration/satMeasurementArray.h"

#include "../include/rinex_reader/src/rinex_reader.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Geometry>

/// TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO
/// TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO
/// TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO
/// TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO
///
/// c'e' un po' di casino, una buona idea sarebbe quella di creare (o modificare
/// lo struct gia esistente) una classe "Satellite" che contiene nome, posizione x y z,
/// pseudorange, velocita', (timestamp?), etc. e lavorare con questa.
/// Di conseguenza
/// 1.modificare il messaggio ros
/// 2.far si che ogni satellite sia identificato dal suo nome, e non dall'indice
/// 3.far si che rinex reader mi restituisca sta roba. ora addirittura mi restituisce un
///   gpstk::Triple come velocita', che fa cagare.
/// 4.quando un satellite scompare dal campo visivo, elimino le robe che il suo publisher
///   ha pubblicato come ultime
///

class OrbitPredictionNode
{
public:
	OrbitPredictionNode(char *path_obs, char *path_nav);

	bool processNextEpoch();

	// compute position of sats at observation time + offset
	void computeSatsPositionAfter(double offset);

	// publish sats list
	void publishSatsPositions();

	// publish a sphere that represents the earth
	void publishEarth();

	// to decide the scale of visualization
	void setScale(double value);

	Eigen::Quaterniond rotateSatelliteFrame(int index);

	void publishOdometry(int index, const Eigen::Quaterniond &rotation);

protected:
	void initOdomPublishers();

	void publishSat(int index);
	void publishSatVelocityThroughEndings(int index);
	void publishSatVelocity(int index);
	void publishEarthVector(int index, const Eigen::Vector3d &earth);

	std::string getSatelliteFrame(int index);

	Eigen::Vector3d getEarthFromSatellite(int index, const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation);


public:
	const std::string WORLD_FRAME = "world";

	const double EARTH_RADIUS = 6371000; // meters
	const double METERS = 1;
	const double KILOMETERS = METERS/1000;
	const double MEGAMETERS = KILOMETERS/1000;

protected:
	RinexReader rr;

	ros::Time currentTime;

	std::vector<SatelliteMeasurement> sats;
	std::vector<gpstk::Triple> vel;

	// ROS node handle
	ros::NodeHandle nh;

	// Publishers
	ros::Publisher markerPub;
	ros::Publisher odomAllPub;
	std::vector<ros::Publisher> odomPub;

	tf::TransformBroadcaster transBroadcaster;

	double scale; // provv: per gestire le dimensioni di stampa. forse e' meglio un enum
};

#endif // ORBITPREDICTIONNODE_H
