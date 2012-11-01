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
  nh.param("focalLengthX", focalLengthX, 614.0);
  nh.param("focalLengthY", focalLengthY, 361.7);
  nh.param("tagSize", tagSize, 0.04);
}
;

roscpp_apriltag::roscpp_apriltagConfig* ParamsAccess::varParams;
FixParams* ParamsAccess::fixParams;
