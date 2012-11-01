/*
 * AprilROSNode.cpp
 *
 *  Created on: Nov 1, 2012
 *      Author: slynen
 */

#include "roscpp_apriltag/AprilROSNode.h"

int main( int argc, char **argv )
{

	ros::init(argc, argv, "AprilTagROSNode");
	ROS_INFO("starting AprilTagROSNode with node name %s", ros::this_node::getName().c_str());

	roscpp_apriltagParameters mParameters; //instantiate singleton, read params

	AprilROSNode aprilnode;

	ros::spin();

	std::cout<<"[main] DONE...exit!"<<std::endl;
	return 0;
}
