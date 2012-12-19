/*
 * AprilInterface.h
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
#include <ethzasl_apriltag/Params.h>
#include <Eigen/Dense>

#include <sensor_msgs/CameraInfo.h>

//#define TAG_DEBUG_PERFORMANCE 1

class AprilInterface
{
private:
  ros::NodeHandle nh_, image_nh_;
  tf::TransformBroadcaster tf_pub_;
  ros::Publisher pub_pose_;
  mutable bool first_frame_;
  image_transport::Subscriber sub_image_;
  ros::Subscriber cam_info_sub_;
  std::string parent_frameid;
  unsigned int seq_number_;

  void imageCallback(const sensor_msgs::ImageConstPtr & img);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

  sensor_msgs::CameraInfoConstPtr last_cam_info_msg_; //caches latest cam info

  tf::Vector3 projectionMatrixToTranslationVector(Eigen::Matrix4d& M) const;
  tf::Quaternion projectionMatrixToQuaternion(Eigen::Matrix4d& M) const;
  tf::Transform homographyToPose(double fx, double fy, double scale, double cx, double cy,
                                 const Eigen::Matrix<double, 3, 3>& H) const;
  void publishPoseAndTf(const tf::Transform& transform, double timestamp, std::string frameid, double largestObservedPerimeter);
public:
  AprilInterface();
  virtual ~AprilInterface();
};

#endif /* APRILROSNODE_H_ */
