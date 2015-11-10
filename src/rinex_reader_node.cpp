#include "rinex_reader_node.h"

RinexReaderNode::RinexReaderNode(std::string path2obs, std::string path2nav, std::string path2met):
	nh(ros::this_node::getName())
{
	// Initialize measurements publisher
	measurementsPub = nh.advertise<trilateration::satMeasurementArray>("/gps_measurements", 1000);



	//todo leggi i rinex file ecc




}

RinexReaderNode::~RinexReaderNode() { }

void RinexReaderNode::processNextEpoch()
{

}

//da lasciare pubblica?
void RinexReaderNode::publishMeasurements()
{

}

