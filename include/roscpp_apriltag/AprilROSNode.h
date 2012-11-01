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
#include <Eigen/Dense>

//#define TAG_DEBUG_PERFORMANCE 1

class AprilROSNode {
private:
	ros::NodeHandle nh_, image_nh_;
	tf::TransformBroadcaster tf_pub_;
	ros::Publisher pub_pose_;
	bool first_frame_;
	image_transport::Subscriber sub_image_;

	void imageCallback(const sensor_msgs::ImageConstPtr & img);

	tf::Vector3 projectionMatrixToTranslationVector(Eigen::Matrix<double, 3, 4>& M);
	tf::Quaternion projectionMatrixToQuaternion(Eigen::Matrix<double, 3, 4>& M);
	tf::Transform homographyToPose(double fx, double fy, double tagSize, Eigen::Matrix<double, 3,3> H);

public:
	AprilROSNode();
	virtual ~AprilROSNode();
};

#endif /* APRILROSNODE_H_ */
