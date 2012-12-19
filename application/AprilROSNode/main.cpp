/*
 * AprilROSNode.cpp
 *
 *  Created on: Nov 1, 2012
 *      Author: slynen
 */

#include "ethzasl_apriltag/april_interface.h"

int main( int argc, char **argv )
{

	ros::init(argc, argv, "apriltag");
	ROS_INFO("starting April Tracker with node name %s", ros::this_node::getName().c_str());

	ethzasl_apriltagParameters mParameters; //instantiate singleton, read params

	AprilInterface aprilnode;

	ros::spin();

	std::cout<<"[main] DONE...exit!"<<std::endl;
	return 0;
}
