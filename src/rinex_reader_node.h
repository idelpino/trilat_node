#ifndef RINEXREADERNODE_H
#define RINEXREADERNODE_H

/// ROS includes
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
#include "trilateration/satMeasurement.h"
#include "trilateration/satMeasurementArray.h"

/// My library includes
#include "../include/trilateration/src/trilateration.h"
#include "../include/trilateration/src/structs.h"


/// TODO poi vedi quali include servono veramente
// Basic input/output C++ class
#include <iostream>

// Classes for handling observations RINEX files (data)
#include "Rinex3ObsData.hpp"
#include "Rinex3ObsStream.hpp"

// Class to easily extract data from Rinex3ObsData objects
#include "ExtractData.hpp"

// Classes for handling satellite navigation parameters RINEX files
// (Broadcast ephemerides)
#include "Rinex3NavHeader.hpp"
#include "Rinex3NavData.hpp"
#include "Rinex3NavStream.hpp"

// Class to store satellite broadcast navigation data
#include "GPSEphemerisStore.hpp"

// Class to model GPS data for a mobile receiver
#include "ModeledPR.hpp"

// Class to model the tropospheric delays
#include "TropModel.hpp"

// Classes to model ans store ionospheric delays
#include "IonoModel.hpp"
#include "IonoModelStore.hpp"

// Class to solve the equations system using a Weighted Least Mean Square method
#include "SolverWMS.hpp"

// Class to compute the weights to be used for each satellite
#include "MOPSWeight.hpp"

// Basic framework for programs in the GPSTk. The 'process()' method MUST
// be implemented
#include "BasicFramework.hpp"

#include "geometry.hpp"                   // DEG_TO_RAD

// Time-class year-day-second
#include "YDSTime.hpp"
/// TODO fin qui

using namespace gpstk; // TODO da togliere

class RinexReaderNode
{

protected:

	// Measure obtained from satellites
	std::vector<SatelliteMeasurement> measurements;

	// ROS node handle
	ros::NodeHandle nh;

	// Publisher for measurements
	ros::Publisher measurementsPub;

public:

	RinexReaderNode(std::string path2obs, std::string path2nav, std::string path2met = NULL);

	~RinexReaderNode();

	void processNextEpoch();

	void publishMeasurements();



	Rinex3ObsStream rObsFile;     // Object to read Rinex observation data files
	Rinex3ObsData rData;          // Object to store Rinex observation data
	Rinex3NavStream rNavFile;     // Object to read Rinex navigation data files
	Rinex3NavData rNavData;       // Object to store Rinex navigation data
	Rinex3NavHeader rNavHeader;   // Object to read the header of Rinex
								  // navigation data files
	IonoModelStore ionoStore;     // Object to store ionospheric models
	GPSEphemerisStore bceStore;   // Object to store ephemeris
	ModeledPR modelPR;            // Declare a ModeledReferencePR object
	MOPSTropModel mopsTM;         // Declare a MOPSTropModel object
	ExtractData obsC1;            // Declare an ExtractData object
	int indexC1;                  // Index to "C1" observation
	bool useFormerPos;            // Flag indicating if we have an a priori
								  // position
	Position formerPosition;      // Object to store the former position
	IonoModel ioModel;            // Declare a Ionospheric Model object
	SolverWMS solver;             // Declare an object to apply WMS method
	MOPSWeight mopsWeights;       // Object to compute satellites' weights


};

#endif // RINEXREADERNODE_H
