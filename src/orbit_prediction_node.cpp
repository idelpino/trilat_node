#include "orbit_prediction_node.h"

OrbitPredictionNode::OrbitPredictionNode(char *path_obs, char *path_nav):
	rr(path_obs, path_nav)
{

}

