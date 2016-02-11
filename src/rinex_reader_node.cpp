#include "rinex_reader_node.h"

///	TODO
///	da cambiare completamente il codice, in modo che utilizzi la mia libreria
///	TODO

using namespace gpstk;
using namespace std;

RinexReaderNode::RinexReaderNode(char* path_obs, char* path_nav/*, char *path_met*/):
	nh(ros::this_node::getName()),
	rNavFile(path_nav),		//reads the nav file
	rObsFile(path_obs)		//reads the obs file
{
	// Initialize measurements publisher
	measurementsPub = nh.advertise<trilateration::satMeasurementArray>("/gps_measurements", 1000);
	observationsPub = nh.advertise<iri_common_drivers_msgs::SatellitePseudorangeArray>("/sat_pseudoranges", 1000);
	realFixPub = nh.advertise<iri_asterx1_gps::NavSatFix_ecef>("/real_fix", 5000);

	// Initialize all the stuff related to gpstk
	gpstkInit();

}

RinexReaderNode::~RinexReaderNode()
{

}

int RinexReaderNode::processNextEpoch()
{
	measurements.clear();
	satIDs.clear();
	
	if(rObsFile >> rObsData)
	{
		fileFinished = false;

		if(rObsData.epochFlag == 0 || rObsData.epochFlag == 1 )
		{
			//TODO da mettere nella classe probabilmente
			vector<SatID> prnVec;
			vector<double> rangeVec;

			Rinex3ObsData::DataMap::const_iterator it;

			// This part gets the PRN numbers and ionosphere-corrected
			// pseudoranges for the current epoch. They are correspondly fed
			// into "prnVec" and "rangeVec";

			for( it = rObsData.obs.begin(); it!= rObsData.obs.end(); it++ )
			{

				// The RINEX file may have P1 observations, but the current
				// satellite may not have them.
				double P1( 0.0 );
				try
				{
					P1 = rObsData.getObs( (*it).first, indexP1 ).data;
				}
				catch(...)
				{
					// Ignore this satellite if P1 is not found
					continue;
				}

				double ionocorr( 0.0 );

				// If there are P2 observations, let's try to apply the
				// ionospheric corrections
				if( indexP2 >= 0 )
				{

					// The RINEX file may have P2 observations, but the
					// current satellite may not have them.
					double P2( 0.0 );
					try
					{
						P2 = rObsData.getObs( (*it).first, indexP2 ).data;
					}
					catch(...)
					{
						// Ignore this satellite if P1 is not found
						continue;
					}

					// Vector 'vecData' contains RinexDatum, whose public
					// attribute "data" indeed holds the actual data point
					ionocorr = 1.0 / (1.0 - GAMMA) * ( P1 - P2 );

				}

				// Now, we include the current PRN number in the first part
				// of "it" iterator into the vector holding the satellites.
				// All satellites in view at this epoch that have P1 or P1+P2
				// observations will be included.
				prnVec.push_back( (*it).first );

				// The same is done for the vector of doubles holding the
				// corrected ranges
				rangeVec.push_back( P1 - ionocorr );

				// WARNING: Please note that so far no further correction
				// is done on data: Relativistic effects, tropospheric
				// correction, instrumental delays, etc.

			}  // End of 'for( it = rObsData.obs.begin(); it!= rObsData.obs.end(); ...'



			// The default constructor for PRSolution2 objects (like "raimSolver")
			// is to set a RMSLimit of 6.5. We change that here.
			// With this value of 3e6 the solution will have a lot more dispersion.

			//TODO vedi se rimettere
			raimSolver.RMSLimit = 3e6;


			int ret;
			// Compute posiiton
			ret = raimSolver.RAIMCompute( rObsData.time,		// current time
									prnVec,			// vector of visible satellites
									rangeVec,		// vector of corresponding ranges
									bcestore,		// satellite ephemerides
									tropModelPtr );	// pointer to the tropospheric model to be applied

			if (ret!=0)
				cout << "Return value of raimSolver.RAIMCompute:" << ret << endl;

			Matrix<double> calcPos;



			ret = raimSolver.PrepareAutonomousSolution(rObsData.time, //questo e' il TOA (time of arrival), cioe' l'istante nel quale voglio predire la posizione dei satelliti
												 prnVec,
												 rangeVec,
												 bcestore,
												 calcPos); //satellite positions at transmit time, and the corrected pseudorange
			//Return values: 0 ok -4 ephemeris not found for all the satellites
			//NB: verify that the number of good entries (Satellite[.] > 0) is > 4 before proceeding

			if (ret!=0)
				cout << "Return value of raimSolver2.PrepareAutonomousSolution:" << ret << endl;


			cout << "--------------------" << endl;

			cout << "Reception time:\t" << CivilTime(rObsData.time) << "\n";

			for (int i = 0; i < prnVec.size(); ++i)
			{
//				cout << prnVec[i]
//					 << ":\tpos al receive time:\t"
//					 << bcestore.getXvt(prnVec[i], rObsData.time).getPos() << "\n"
//					 << "\tcalc. dal solver:\t("
//					 << calcPos[i][0] << ", "
//					 << calcPos[i][1] << ", "
//					 << calcPos[i][2] << ")\n";

//				cout << "\tpseudorange: "
//					 << rangeVec[i] << " --> "
//					 << calcPos[i][3] << " (il secondo tiene conto del bias! il primo no)\n"


//					 << "\tDifferenza: " << (rangeVec[i]-calcPos[i][3]) << " metri"
//					 << endl;


//				cout << "\tClock bias: " << bcestore.getXvt(prnVec[i], rObsData.time).getClockBias() * 1000000 << " microseconds (satellite's bias)"
//					 << endl <<  endl;

				SatelliteMeasurement sm;
				sm.pos = Point<double>(calcPos[i][0], calcPos[i][1], calcPos[i][2]);
				sm.pseudorange = calcPos[i][3];

				measurements.push_back(sm);
				satIDs.push_back(prnVec[i].id);
			}

			//TODO guarda se ho assegnato la soluzione ai cosi della classe


		}
	} else {
		fileFinished = true;
	}

	/*
	 * Publish true position.
	 * true means the one calculated with raimsolver by gpstk
	 */
	Receiver truePos = getReceiverECEF();
	iri_asterx1_gps::NavSatFix_ecef estFixMsg;
	//TODO fill up header etc
	estFixMsg.x = truePos.pos.getX();
	estFixMsg.y = truePos.pos.getY();
	estFixMsg.z = truePos.pos.getZ();

	realFixPub.publish(estFixMsg);



	return 0;//TODO torna i codici sensati di raimcompute ad esempio
}

void RinexReaderNode::publishMeasurements()
{
	/*
	 * TODO
	 * manca da riempire i campi velocità
	 * mancano i timestamp, (occhio che c'è il timestamp ros e il timestamp gps wnc+tow)
	 */

	trilateration::satMeasurement meas;
	trilateration::satMeasurementArray msg;

	iri_common_drivers_msgs::SatellitePseudorangeArray observation;
	

	for (int i = 0; i < measurements.size(); ++i)
	{
		iri_common_drivers_msgs::SatellitePseudorange ob;

		ob.sat_id = satIDs[i];

		ob.x = meas.x = measurements.at(i).pos.getX();
		ob.y = meas.y = measurements.at(i).pos.getY();
		ob.z = meas.z = measurements.at(i).pos.getZ();
		ob.pseudorange = meas.pseudorange = measurements.at(i).pseudorange;

		msg.measurements.push_back(meas);
		observation.measurements.push_back(ob);
	}

	measurementsPub.publish(msg);
	observationsPub.publish(observation);


}

void RinexReaderNode::gpstkInit()
{
	fileFinished = true;

	// se non ho il file meteo, assegno il modello troposferico void
	tropModelPtr = &noTropModel;

	// parse nav file
	rNavFile >> rNavHeader;
	while(rNavFile >> rNavData) bcestore.addEphemeris(rNavData);

	// Setting the criteria for looking up ephemeris
	bcestore.SearchNear();


	rObsFile >> rObsHeader;		// Let's read the header

	// The following lines fetch the corresponding indexes for some observation types
	// we are interested in. Given that old-style observation types are used, GPS is assumed.
	try
	{
		indexP1 = rObsHeader.getObsIndex( "P1" );
	}
	catch(...)
	{
		cerr << "The observation file doesn't have P1 pseudoranges." << endl;
		exit(1);
	}

	try
	{
		indexP2 = rObsHeader.getObsIndex( "P2" );
	}
	catch(...)
	{
		indexP2 = -1;
	}

	cout << setprecision(12);

}

std::vector<SatelliteMeasurement> RinexReaderNode::getMeasurements() const
{
	return measurements;
}

bool RinexReaderNode::isSolutionValid() const
{
	return raimSolver.isValid();
}

bool RinexReaderNode::isFileFinished() const
{
	return fileFinished;
}

void RinexReaderNode::printEpochRecap()
{
	// If we got a valid solution, let's print it
	if( raimSolver.isValid() )
	{
		cout << "### ECEF: " << getReceiverECEF().toString() << endl;
		cout << "### LLR: " << getReceiverLLR().toString() << endl ;

	} else {
		cout << "Solution is not valid\n";
	}
}

Receiver RinexReaderNode::getReceiverECEF() const
{
	Receiver r;

	if( raimSolver.isValid() )
	{
		r.pos = Point<double>(raimSolver.Solution[0], raimSolver.Solution[1], raimSolver.Solution[2]);
		r.bias = raimSolver.Solution[3];
	}

	return r;
}

Receiver RinexReaderNode::getReceiverLLR() const
{
	Receiver r;

	if( raimSolver.isValid() )
	{
		Triple sol_ecef, sol_llr;
		sol_ecef[0] = raimSolver.Solution[0];
		sol_ecef[1] = raimSolver.Solution[1];
		sol_ecef[2] = raimSolver.Solution[2];

		// conversione in coordinate geodetic
		WGS84Ellipsoid WGS84;
		double AEarth = WGS84.a();
		double eccSquared = WGS84.eccSquared();

		Position::convertCartesianToGeodetic(sol_ecef, sol_llr, AEarth , eccSquared);

		r.pos = Point<double>(sol_llr[0], sol_llr[1], sol_llr[2]);
		r.bias = raimSolver.Solution[3];
	}

	return r;
}





//void RinexReaderNode::spinUp(string path_obs, string path_nav, string path_met)
//{

//	  // From now on, some parts may look similar to 'example3.cpp' and
//	  // 'example4.cpp'
//	  // Activate failbit to enable exceptions
//   rObsFile.exceptions(ios::failbit);

//	  // First, data RINEX reading object
//   try
//   {
//	  rObsFile.open(path_obs.c_str(), std::ios::in);
//   }
//   catch(...)
//   {
//	  cerr << "Problem opening file " << path_obs << endl;
//	  cerr << "Maybe it doesn't exist or you don't have proper read "
//		   << "permissions." << endl;

//	  exit (-1);
//   }

//	  // We need to read the header of the observations file
//   Rinex3ObsHeader roh;
//   rObsFile >> roh;

//	  // We need the index pointing to C1-type observations
//   try
//   {
//	  indexC1 = roh.getObsIndex( "C1" );
//   }
//   catch(...)
//   {
//	  cerr << "The observation file doesn't have C1 pseudoranges." << endl;
//	  exit(1);
//   }


//	  // Activate failbit to enable exceptions
//   rNavFile.exceptions(ios::failbit);

//	  // Read nav file and store unique list of ephemerides
//   try
//   {
//	  rNavFile.open(path_nav.c_str(), std::ios::in);
//   }
//   catch(...)
//   {
//	  cerr << "Problem opening file " << path_nav << endl;
//	  cerr << "Maybe it doesn't exist or you don't have proper read "
//		   << "permissions." << endl;

//	  exit (-1);
//   }

//	  // We will need to read ionospheric parameters (Klobuchar model) from
//	  // the file header
//   rNavFile >> rNavHeader;

//	  // Let's feed the ionospheric model (Klobuchar type) from data in the
//	  // navigation (ephemeris) file header. First, we must check if there are
//	  // valid ionospheric correction parameters in the header
//   if(rNavHeader.valid & Rinex3NavHeader::validIonoCorrGPS)
//   {
//		 // Extract the Alpha and Beta parameters from the header
//	  double* ionAlpha = rNavHeader.mapIonoCorr["GPSA"].param;
//	  double* ionBeta  = rNavHeader.mapIonoCorr["GPSB"].param;

//		 // Feed the ionospheric model with the parameters
//	  ioModel.setModel(ionAlpha, ionBeta);
//   }
//   else
//   {
//	  cerr << "WARNING: Navigation file " << path_nav
//		   << " doesn't have valid ionospheric correction parameters." << endl;
//   }

//	  // WARNING-WARNING-WARNING: In this case, the same model will be used
//	  // for the full data span
//   ionoStore.addIonoModel(CommonTime::BEGINNING_OF_TIME, ioModel);

//	  // Storing the ephemeris in "bceStore"
//   while (rNavFile >> rNavData) bceStore.addEphemeris(rNavData);

//	  // Setting the criteria for looking up ephemeris
//   bceStore.SearchUser();  // This is the default

//	  // This is set to true if the former computed positon will be used as
//	  // a priori position
//   useFormerPos = false;   // At first, we don't have an a priori position

//	  // Prepare for printing later on
//   cout << fixed << setprecision(8);

//}  // End of example5::spinUp()







//void RinexReaderNode::processNextEpoch()
//{
//	if (!(rObsFile >> rData)){
//		cout << "All the epoch have been processed";
//		return;
//	}


//	  // Begin usable data with enough number of satellites
//   if( (rData.epochFlag == 0 || rData.epochFlag == 1) &&
//	   (rData.numSVs > 3) )
//   {

//		 // Number of satellites with valid data in this epoch
//	  int validSats = 0;
//	  int prepareResult;
//	  double rxAltitude;  // Receiver altitude for tropospheric model
//	  double rxLatitude;  // Receiver latitude for tropospheric model

//		 // We need to extract C1 data from this epoch. Skip epoch if not
//		 // enough data (4 SV at least) is available
//	  if( obsC1.getData(rData, indexC1) < 4 )
//	  {
//			// The former position will not be valid next time
//		 useFormerPos = false;
//		 return;
//	  }


//		 // If possible, use former position as a priori
//	  if( useFormerPos )
//	  {

//		 prepareResult = modelPR.Prepare(formerPosition);

//			// We need to seed this kind of tropospheric model with
//			// receiver altitude
//		 rxAltitude = formerPosition.getAltitude();
//		 rxLatitude = formerPosition.getGeodeticLatitude();

//	  }
//	  else
//	  {
//			// Use Bancroft method is no a priori position is available
//		 cerr << "Bancroft method was used at epoch "
//			  << static_cast<YDSTime>(rData.time).sod << endl;

//		 prepareResult = modelPR.Prepare( rData.time,
//										  obsC1.availableSV,
//										  obsC1.obsData,
//										  bceStore );

//			// We need to seed this kind of tropospheric model with
//			// receiver altitude
//		 rxAltitude = modelPR.rxPos.getAltitude();
//		 rxLatitude = modelPR.rxPos.getGeodeticLatitude();
//	  }

//		 // If there were problems with Prepare(), skip this epoch
//	  if( prepareResult )
//	  {
//			// The former position will not be valid next time
//		 useFormerPos = false;
//		 return;
//	  }

//		 // If there were no problems, let's feed the tropospheric model
//	  mopsTM.setReceiverHeight(rxAltitude);
//	  mopsTM.setReceiverLatitude(rxLatitude);
//	  mopsTM.setDayOfYear(static_cast<YDSTime>(rData.time).doy);


//		 // Now, let's compute the GPS model for our observable (C1)
//	  validSats = modelPR.Compute( rData.time,
//								   obsC1.availableSV,
//								   obsC1.obsData,
//								   bceStore,
//								   &mopsTM,
//								   &ionoStore );

//		 // Only get into further computations if there are enough
//		 // satellites
//	  if( validSats >= 4 )
//	  {

//			// Now let's solve the navigation equations using the WMS method
//		 try
//		 {
//			   // First, compute the satellites' weights
//			int goodSv = mopsWeights.getWeights( rData.time,
//												 modelPR.availableSV,
//												 bceStore,
//												 modelPR.ionoCorrections,
//												 modelPR.elevationSV,
//												 modelPR.azimuthSV,
//												 modelPR.rxPos );

//			   // Some minimum checking is in order
//			if ( goodSv != (int)modelPR.prefitResiduals.size() ) return;

//			   // Then, solve the system
//			solver.Compute( modelPR.prefitResiduals,
//							modelPR.geoMatrix,
//							mopsWeights.weightsVector );

//		 }
//		 catch( InvalidSolver& e )
//		 {
//			cerr << "Couldn't solve equations system at epoch "
//				 << static_cast<YDSTime>(rData.time).sod << endl;
//			cerr << e << endl;

//			   // The former position will not be valid next time
//			useFormerPos = false;
//			return;
//		 }

//			// With "solver", we got the difference vector between the
//			// a priori position and the computed, 'real' position. Then,
//			// let's convert the solution to a Position object
//		 Position solPos( (modelPR.rxPos.X() + solver.solution[0]),
//						  (modelPR.rxPos.Y() + solver.solution[1]),
//						  (modelPR.rxPos.Z() + solver.solution[2]) );

//			// Print results
//		 cout << static_cast<YDSTime>(rData.time).sod
//			  << "   ";   // Output field #1
//		 cout << solPos.X() << "   ";                // Output field #2
//		 cout << solPos.Y() << "   ";                // Output field #3
//		 cout << solPos.Z() << "   ";                // Output field #4
//		 cout << solPos.geodeticLatitude() << "   "; // Output field #6
//		 cout << solPos.longitude() << "   ";        // Output field #5
//		 cout << solPos.height() << "   ";           // Output field #7
//		 cout << endl;

//		 formerPosition = solPos;

//			// Next time, former position will be used as a priori
//		 useFormerPos = true;

//	  }  // End of 'if( validSats >= 4 )'
//	  else
//	  {
//			// The former position will not be valid next time
//		 useFormerPos = false;
//	  }

//   }  // End of 'if( (rData.epochFlag == 0 || rData.epochFlag == 1) &&...'
//   else
//   {
//		 // The former position will not be valid next time
//	  useFormerPos = false;
//   }
//}
