/*
 * AprilROSNode.cpp
 *
 *  Created on: Nov 1, 2012
 *      Author: slynen
 */

#include <roscpp_apriltag/Params.h>

ParamsAccess params_obj;

void FixParams::readFixParams()
{
  ros::NodeHandle nh("~");

  nh.param("tagID", tagID, 4);

}
;

roscpp_apriltag::roscpp_apriltagConfig* ParamsAccess::varParams;
FixParams* ParamsAccess::fixParams;
