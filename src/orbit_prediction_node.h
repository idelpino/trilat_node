#ifndef ORBITPREDICTIONNODE_H
#define ORBITPREDICTIONNODE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
#include "trilateration/satMeasurement.h"
#include "trilateration/satMeasurementArray.h"

#include "../include/trilateration/src/trilateration.h"
#include "../include/trilateration/src/structs.h"

#include "../include/rinex_reader/src/rinex_reader.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Geometry> // to calculate quaternion



///
/// TODO link utili
///
/// http://eigen.tuxfamily.org/dox/group__TutorialGeometry.html
///
/// http://docs.ros.org/jade/api/tf/html/c++/transform__datatypes_8h.html
/// http://docs.ros.org/jade/api/tf/html/c++/classtf_1_1Quaternion.html


///
///
/// Quaternion<float> q;  q = AngleAxis<float>(angle_in_radian, axis);
/// axis va normalizzato
///



class OrbitPredictionNode
{
public:
	OrbitPredictionNode(char *path_obs, char *path_nav);

	bool processNextEpoch();

	// compute position of sats at time observation + offset
	void computeSatsPositionAfter(double offset);

	// publish sats list
	void publishSatsPositions();

	// publish a sphere that represents the earth
	void publishEarth();

	// to decide the scale of visualization
	void setScale(double value);

	void publishOdometry(int index); // TODO da togliere

protected:
	void initOdomPublishers();

	void publishSat(int index);
	void publishSatVelocity(int index);
	void publishSatVelocity2(int index);
	//void publishEarthVector(int index);
	void publishEarthVectorSERIO(int index, Eigen::Vector3d earth);


	std::string getSatelliteName(int index);

	Eigen::Vector3d findWorldFromSatelliteSERIO(int index, Eigen::Vector3d translation, Eigen::Quaterniond rotation);
	//Eigen::Vector3d findWorldFromSatellite(int index);

	// todo
	void publishDebugArrow(int id, double x, double y, double z,
					double xx, double yy, double zz, double ww,
					double r = 0.0, double g = 1.0, double b = 0.0);

	void stampeDiDebug(int id);

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

	tf::TransformListener transListener; // todo vedi se serve

	// provv: per gestire le dimensioni di stampa. forse e' meglio un enum
	double scale;
};

#endif // ORBITPREDICTIONNODE_H
