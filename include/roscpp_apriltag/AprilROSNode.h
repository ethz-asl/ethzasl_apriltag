/*
 * AprilROSNode.h
 *
 *  Created on: Nov 1, 2012
 *      Author: slynen
 */

#ifndef APRILROSNODE_H_
#define APRILROSNODE_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "OpenCVHelper.h"
#include "Log.h"
#include <roscpp_apriltag/Params.h>

//#define TAG_DEBUG_PERFORMANCE 1

class AprilROSNode {
private:
	  ros::NodeHandle nh_, image_nh_;
	  tf::TransformBroadcaster tf_pub_;
	  bool first_frame_;
	  image_transport::Subscriber sub_image_;

	  void imageCallback(const sensor_msgs::ImageConstPtr & img);

public:
	AprilROSNode();
	virtual ~AprilROSNode();
};

#endif /* APRILROSNODE_H_ */
