#ifndef RINEXREADERNODE_H
#define RINEXREADERNODE_H

///	TODO
///	da cambiare completamente il codice, in modo che utilizzi la mia libreria
///	TODO


// ROS includes
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
#include "trilateration/satMeasurement.h"
#include "trilateration/satMeasurementArray.h"

#include "iri_asterx1_gps/NavSatFix_ecef.h"
#include "asterx1_node/SatPr.h"
#include "asterx1_node/SatPrArray.h"

// Trilateration library includes
#include "../include/trilateration/src/trilateration.h"
#include "../include/trilateration/src/structs.h"


/// TODO poi vedi quali include servono veramente
// First, let's include Standard Template Library classes
#include <string>
#include <vector>

// Classes for handling observations RINEX files (data)
#include "Rinex3ObsHeader.hpp"
#include "Rinex3ObsData.hpp"
#include "Rinex3ObsStream.hpp"

// Classes for handling satellite navigation parameters RINEX
// files (ephemerides)
#include "Rinex3NavHeader.hpp"
#include "Rinex3NavData.hpp"
#include "Rinex3NavStream.hpp"

// Classes for handling RINEX files with meteorological parameters
#include "RinexMetBase.hpp"
#include "RinexMetData.hpp"
#include "RinexMetHeader.hpp"
#include "RinexMetStream.hpp"

// Class for handling tropospheric models
#include "TropModel.hpp"

// Class for storing "broadcast-type" ephemerides
#include "GPSEphemerisStore.hpp"

// Class for handling RAIM
#include "PRSolution2.hpp"

// Class defining GPS system constants
#include "GNSSconstants.hpp"

//per convertire coordinate da ecef a lla
#include <Position.hpp>
#include <Triple.hpp>
#include "WGS84Ellipsoid.hpp"

// per calcolare la posizione dei satelliti
#include <Matrix.hpp>
#include <PreciseRange.hpp>

//per leggere il tempo decentemente
#include "CivilTime.hpp"
/// TODO fin qui






class RinexReaderNode
{
public:
	RinexReaderNode(char *path_obs, char *path_nav/*, char *path_met = NULL*/);
	~RinexReaderNode();
	int processNextEpoch();
	void publishMeasurements();

	bool isSolutionValid() const;
	bool isFileFinished() const;

	void printEpochRecap();
//	SatelliteMeasurement get vettore di satelliti

	Receiver getReceiverECEF() const;
	Receiver getReceiverLLR() const;

	std::vector<SatelliteMeasurement> getMeasurements() const;

protected:
	void gpstkInit();

protected:
	ros::NodeHandle nh;								// ROS node handle

	//Publishers
	ros::Publisher measurementsPub;					// Publisher for measurements
	ros::Publisher observationsPub;					// Publisher for observations (int the new format, see asterx1_node)// Publisher
	ros::Publisher realFixPub;

	std::vector<SatelliteMeasurement> measurements;	// Measures obtained from satellites
	std::vector<int> satIDs;						// Satellite ID paired to measurements vector (bag solution, but i don't have time to change everything, it's just a quick test

	bool fileFinished;

	gpstk::GPSEphemerisStore bcestore;	// Object to store ephemeris
	gpstk::PRSolution2 raimSolver;		// RAIM solver
	gpstk::ZeroTropModel noTropModel;	// Object for void-type tropospheric model
	gpstk::TropModel *tropModelPtr;		// Pointer to the tropospheric models.

	// Let's compute an useful constant (also found in "GNSSconstants.hpp")
	const double GAMMA = (gpstk::L1_FREQ_GPS/gpstk::L2_FREQ_GPS)*(gpstk::L1_FREQ_GPS/gpstk::L2_FREQ_GPS);
	const double SPEED_OF_LIGHT = 3e8;

	gpstk::Rinex3NavStream rNavFile;    // Object to read Rinex navigation data files
	gpstk::Rinex3NavData rNavData;      // Object to store Rinex navigation data
	gpstk::Rinex3NavHeader rNavHeader;  // Object to read the header of Rinex navigation data files

	gpstk::Rinex3ObsStream rObsFile;    // Object to read Rinex observation data files
	gpstk::Rinex3ObsData rObsData;      // Object to store Rinex observation data
	gpstk::Rinex3ObsHeader rObsHeader;	// Object to read the header of Rinex observation data files
	int indexP1;
	int indexP2;

};

#endif // RINEXREADERNODE_H
