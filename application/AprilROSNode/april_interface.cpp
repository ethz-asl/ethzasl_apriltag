/*
 * AprilInterface.cpp
 *
 *  Created on: Nov 1, 2012
 *      Author: slynen
 */

#define TAG_DEBUG_PERFORMANCE 0
#define TAG_DEBUG_DRAW 0

#include "ethzasl_apriltag/april_interface.h"
#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/SVD>

using namespace std;
using namespace cv;
using april::tag::INT64;
using april::tag::TagFamily;
using april::tag::TagFamilyFactory;
using april::tag::TagDetector;
using april::tag::TagDetection;
using helper::ImageSource;

cv::Ptr<TagFamily> tagFamily;
cv::Ptr<TagDetector> detector;

#if TAG_DEBUG_PERFORMANCE
Log::Level Log::level = Log::LOG_DEBUG;
#else
Log::Level Log::level = Log::LOG_INFO;
#endif

AprilInterface::AprilInterface() :
    nh_("apriltag"), image_nh_(""), first_frame_(true), seq_number_(0)
{

  parent_frameid = "/invalid";

  std::string topic = image_nh_.resolveName("image");
  if (topic == "/image")
  {
    ROS_WARN("video source: image has not been remapped! Typical command-line usage:\n"
    "\t$ ./AprilInterface image:=<image topic>");
  }

  std::string topic_info = image_nh_.resolveName("camera_info");
  if (topic_info == "/camera_info")
  {
    ROS_WARN("info source: cam info has not been remapped! Typical command-line usage:\n"
    "\t$ ./AprilInterface camera_info:=<info topic>");
  }

  image_transport::ImageTransport it(image_nh_);
  sub_image_ = it.subscribe(topic, 1, &AprilInterface::imageCallback, this,
                            image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay(true)));

  cam_info_sub_ = image_nh_.subscribe(topic_info, 1, &AprilInterface::cameraInfoCallback, this);

  pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);

  //// create tagFamily
  int familyid = ParamsAccess::fixParams->_tagFamilyID;
  tagFamily = TagFamilyFactory::create(familyid);
  if (tagFamily.empty())
  {
    loglne("[main] create TagFamily fail!");
    exit(1);
  }
  detector = new TagDetector(tagFamily);
  if (detector.empty())
  {
    loglne("[main] create TagDetector fail!");
    exit(1);
  }
#if TAG_DEBUG_DRAW
  cv::namedWindow("april");
#endif

  ROS_INFO("waiting for first image");
}

/** Given a 3x3 homography matrix and the focal lengths of the
 * camera, compute the pose of the tag. The focal lengths should
 * be given in pixels. For example, if the camera's focal length
 * is twice the width of the sensor, and the sensor is 600 pixels
 * across, the focal length in pixels is 2*600. Note that the
 * focal lengths in the fx and fy direction will be approximately
 * equal for most lenses, and is not a function of aspect ratio.
 *
 * Theory: The homography matrix is the product of the camera
 * projection matrix and the tag's pose matrix (the matrix that
 * projects points from the tag's local coordinate system to the
 * camera's coordinate frame).
 *
 * [ h00 h01 h02 h03] = [ 1/fx 0     0 0 ] [ R00 R01 R02 TX ]
 * [ h10 h11 h12 h13] = [ 0    1/fy  0 0 ] [ R10 R11 R12 TY ]
 * [ h20 h21 h22 h23] = [ 0    0     1 0 ] [ R20 R21 R22 TZ ]
 *                                         [ 0   0   0   1  ]
 *
 * When observing a tag, the points we project in world space all
 * have z=0, so we can form a 3x3 matrix by eliminating the 3rd
 * column of the pose matrix.
 *
 * [ h00 h01 h02 ] = [ 1/fx 0     0 0 ] [ R00 R01 TX ]
 * [ h10 h11 h12 ] = [ 0    1/fy  0 0 ] [ R10 R11 TY ]
 * [ h20 h21 h22 ] = [ 0    0     1 0 ] [ R20 R21 TZ ]
 *                                      [ 0   0   1  ]
 *
 * (note that these h's are different from the ones above.)
 *
 * We can multiply the right-hand side to yield a set of equations
 * relating the values of h to the values of the pose matrix.
 *
 * There are two wrinkles. The first is that the homography matrix
 * is known only up to scale. We recover the unknown scale by
 * constraining the magnitude of the first two columns of the pose
 * matrix to be 1. We use the geometric average scale. The sign of
 * the scale factor is recovered by constraining the observed tag
 * to be in front of the camera. Once scaled, we recover the first
 * two colmuns of the rotation matrix. The third column is the
 * cross product of these.
 *
 * The second wrinkle is that the computed rotation matrix might
 * not be exactly orthogonal, so we perform a polar decomposition
 * to find a good pure rotation approximation.
 *
 * Tagsize is the size of the tag in your desired units. I.e., if
 * your tag measures 0.25m along the side, your tag size is
 * 0.25. (The homography is computed in terms of *half* the tag
 * size, i.e., that a tag is 2 units wide as it spans from -1 to
 * +1, but this code makes the appropriate adjustment.)
 **/

//slynen: rewritten from java
#define sq(x) ((x)*(x))

tf::Transform AprilInterface::homographyToPose(double fx, double fy, double s, double cx, double cy,
                                               const Eigen::Matrix3d& H) const
{
  using namespace Eigen;

  // flip the homography along the Y axis to align the
  // conventional image coordinate system (y=0 at the top) with
  // the conventional camera coordinate system (y=0 at the
  // bottom).

  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();

  F(1, 1) = -1;
  F(2, 2) = -1;

  Eigen::Matrix3d h = F * H;

  Eigen::Matrix4d M;
  M(0, 0) = h(0, 0) / fx;
  M(0, 1) = h(0, 1) / fx;
  M(0, 3) = h(0, 2) / fx;
  M(1, 0) = h(1, 0) / fy;
  M(1, 1) = h(1, 1) / fy;
  M(1, 3) = h(1, 2) / fy;
  M(2, 0) = h(2, 0);
  M(2, 1) = h(2, 1);
  M(2, 3) = h(2, 2);

  // Compute the scale. The columns of M should be made to be
  // unit vectors. This is over-determined, so we take the
  // geometric average.
  double scale0 = sqrt(sq(M(0,0)) + sq(M(1,0)) + sq(M(2,0)));
  double scale1 = sqrt(sq(M(0,1)) + sq(M(1,1)) + sq(M(2,1)));
  double scale = sqrt(scale0 * scale1);

  M *= 1.0 / scale;

  // recover sign of scale factor by noting that observations must occur in front of the camera.
  if (M(2, 3) > 0)
    M *= -1;

  // The bottom row should always be [0 0 0 1].  We reset the
  // first three elements, even though they must be zero, in
  // order to make sure that they are +0. (We could have -0 due
  // to the sign flip above. This is theoretically harmless but
  // annoying in practice.)
  M(3, 0) = 0;
  M(3, 1) = 0;
  M(3, 2) = 0;
  M(3, 3) = 1;

  // recover third rotation vector by crossproduct of the other two rotation vectors.
  Eigen::Vector3d a;
  a << M(0, 0), M(1, 0), M(2, 0);
  Eigen::Vector3d b;
  b << M(0, 1), M(1, 1), M(2, 1);
  Eigen::Vector3d ab = a.cross(b);

  M(0, 2) = ab(0);
  M(1, 2) = ab(1);
  M(2, 2) = ab(2);

  // pull out just the rotation component so we can normalize it.
  Matrix3d R = M.block<3, 3>(0, 0);

  JacobiSVD<Matrix3d> svd;
  svd.compute(R, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // polar decomposition, R = (UV')(VSV')
  Matrix3d MR = svd.matrixU() * svd.matrixV().transpose();
  M.block<3, 3>(0, 0) = MR;

  // Scale the results based on the scale in the homography. The
  // homography assumes that tags span from -1 to +1, i.e., that
  // they are two units wide (and tall).
  for (int i = 0; i < 3; i++)
    M(i, 3) *= s / 2;

  tf::Transform tf;
  tf.setOrigin(projectionMatrixToTranslationVector(M));
  tf.setRotation(projectionMatrixToQuaternion(M));

  return tf;
}

tf::Vector3 AprilInterface::projectionMatrixToTranslationVector(Eigen::Matrix4d& M) const
{
  return tf::Vector3(M(0, 3), -M(1, 3), -M(2, 3));	// camera looks towards tag
}

tf::Quaternion AprilInterface::projectionMatrixToQuaternion(Eigen::Matrix4d& M) const
{

  double qx, qy, qz, qw;

  double t = M(0, 0) + M(1, 1) + M(2, 2);
  if (t > 0)
  {
    double r = sqrt(1 + t);
    double s = 0.5 / r;
    qw = 0.5 * r;
    qx = (M(2, 1) - M(1, 2)) * s;
    qy = (M(0, 2) - M(2, 0)) * s;
    qz = (M(1, 0) - M(0, 1)) * s;
  }
  else
  {
    if (M(0, 0) > M(1, 1) && M(0, 0) > M(2, 2))
    {
      double r = sqrt(1.0 + M(0, 0) - M(1, 1) - M(2, 2));
      double s = 0.5f / r;
      qw = (M(2, 1) - M(1, 2)) * s;
      qx = 0.5 * r;
      qy = (M(0, 1) + M(1, 0)) * s;
      qz = (M(2, 0) + M(0, 2)) * s;
    }
    else if (M(1, 1) > M(2, 2))
    {
      double r = sqrt(1.0 + M(1, 1) - M(0, 0) - M(2, 2));
      double s = 0.5 / r;
      qw = (M(0, 2) - M(2, 0)) * s;
      qx = (M(0, 1) + M(1, 0)) * s;
      qy = 0.5 * r;
      qz = (M(1, 2) + M(2, 1)) * s;
    }
    else
    {
      double r = sqrt(1.0 + M(2, 2) - M(0, 0) - M(1, 1));
      double s = 0.5 / r;
      qw = (M(1, 0) - M(0, 1)) * s;
    qx = (M(0, 2) + M(2, 0)) * s;
      qy = (M(1, 2) + M(2, 1)) * s;
      qz = 0.5 * r;
    }
  }

  return tf::Quaternion(1, 0, 0, 0) * tf::Quaternion(qx, qy, qz, qw); // camera looks towards tag
}

void AprilInterface::publishPoseAndTf(const tf::Transform& transform, double timestamp, std::string frameid,
                                      double largestObservedPerimeter)
{
  //put together a ROS pose
  tf::Quaternion tfQuat = transform.getRotation();
  tf::Vector3 tfTrans = transform.getOrigin();

  geometry_msgs::Pose ros_pose;

  ros_pose.orientation.w = tfQuat.getW();
  ros_pose.orientation.x = tfQuat.getX();
  ros_pose.orientation.y = tfQuat.getY();
  ros_pose.orientation.z = tfQuat.getZ();

  ros_pose.position.x = tfTrans.getX();
  ros_pose.position.y = tfTrans.getY();
  ros_pose.position.z = tfTrans.getZ();

  //pose msg
  geometry_msgs::PoseWithCovarianceStampedPtr poseCovStPtr(new geometry_msgs::PoseWithCovarianceStamped);
  poseCovStPtr->pose.pose = ros_pose;
  double PoseNoise = ParamsAccess::varParams->pose_noise;

  //calculate the noise dependent on the observed perimeter
  //std::cout<<"Best: "<<largestObservedPerimeter<<std::endl;

  Eigen::Matrix<double, 6, 6> cov = (Eigen::Matrix<double, 1, 6>::Constant(PoseNoise * PoseNoise)).asDiagonal();
  assert(cov.SizeAtCompileTime == 36);
  for (int i = 0; i < cov.SizeAtCompileTime; ++i)
  {
    poseCovStPtr->pose.covariance[i] = cov.coeff(i); //TODO fill this: dependend on viewpoint etc.
  }
  std_msgs::Header header;
  header.frame_id = frameid;
  header.seq = seq_number_++;
  header.stamp = ros::Time(timestamp);
  poseCovStPtr->header = header;

  header.frame_id = parent_frameid;
  pub_pose_.publish(poseCovStPtr);

  //tf msg
  tf::StampedTransform transform_msg;
  transform_msg.setRotation(tfQuat);
  transform_msg.setOrigin(tfTrans);
  transform_msg.frame_id_ = parent_frameid;
  transform_msg.child_frame_id_ = frameid;
  transform_msg.stamp_ = header.stamp;
  tf_pub_.sendTransform(transform_msg);
}

void AprilInterface::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  last_cam_info_msg_ = msg; //cache latest cam info
}

void AprilInterface::imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
     cv_ptr= cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
  }
  catch(cv_bridge::Exception e)
  {
     ROS_ERROR("cv_bridge exception: %s",e.what());
     return;
  }

  FixParams* fixparams = ParamsAccess::fixParams;

  if (first_frame_)
  {
    ROS_INFO("OK Got first image. Running...");
    first_frame_ = false;
    parent_frameid = msg->header.frame_id;
  }

  //track the april tag and publish the tf

  static helper::PerformanceMeasurer PM;

  vector<TagDetection> detections;

  if (last_cam_info_msg_.get() == NULL)
  {
    ROS_WARN_STREAM_THROTTLE(
        2,
        "Did not receive valid camera info message up to now. Did you remap the camera info topic? Discarding image!");
    return;
  }

  double opticalCenter[2] = {last_cam_info_msg_->P[2], last_cam_info_msg_->P[6]};
  PM.tic();

  namespace enc = sensor_msgs::image_encodings;

  detector->process(cv_ptr->image, opticalCenter, detections);

  tf::Transform mostStableTransform;
  double largestObservedPerimeter = 0;

  BOOST_FOREACH(TagDetection& dd, detections){
  if(dd.hammingDistance>0) continue; //better not publish a tf, than publishing the one of a wrong tag

  Eigen::Matrix3d tmp((double*)dd.homography[0]);
  Eigen::Matrix3d H = tmp.transpose();

  //read for currently detected tag, abort processing, if there are no parameters
  if(fixparams->_AprilTags.count(dd.id)==0)
  {
    ROS_WARN("Detected Tag with ID %i but you no parameters were loaded for this tag! Please check the parameters file", dd.id);
    continue;
  }

  AprilTagProperties& tagproperties = fixparams->_AprilTags[dd.id];

  tf::Transform tf_tagfromcam = homographyToPose(
      last_cam_info_msg_->P[0], last_cam_info_msg_->P[5], tagproperties.scale, last_cam_info_msg_->P[2], last_cam_info_msg_->P[6], H);

  //add tag transform
  const tf::Transform& tagFromWorld = tagproperties.transform;
  tf::Transform transform = (tagFromWorld * tf_tagfromcam.inverse()).inverse();
  //tf::Transform transform = tf_tagfromcam * tagFromWorld.inverse();
  //publish
  std::string frameid = "/tag_"+boost::lexical_cast<std::string>(dd.id);
  //publishPoseAndTf(transform, frameid);
  //store best, which we assume to be the pose with the largest appearance in the image
  if(dd.observedPerimeter>largestObservedPerimeter && dd.observedPerimeter > ParamsAccess::varParams->min_observed_tag_size * 4)
  { //size to perimeter
    mostStableTransform = transform;
    largestObservedPerimeter = dd.observedPerimeter;
  }
}

//	tf::Quaternion quat(tf::Vector3(0,0,1),-45./180.*M_PI);
//	std::cout<<"quat "<<quat.getX()<<" "<<quat.getY()<<" "<<quat.getZ()<<" "<<quat.getW()<<std::endl;

//only publish if there is valid transform
  if (largestObservedPerimeter > 0)
  {
    std::string frameid = "/april_fix";
    //publish best
    publishPoseAndTf(mostStableTransform, msg->header.stamp.toSec(), frameid, largestObservedPerimeter);
  }

  //some debugging tools
#if TAG_DEBUG_DRAW
  loglni("[TagDetector] process time = "<<PM.toc()<<" sec.");

  logi(">>> find id: ");
  for (int id = 0; id < (int)detections.size(); ++id)
  {
    TagDetection &dd = detections[id];
    if (dd.hammingDistance > 2)
      continue;

    logi("#"<<dd.id<<"|"<<dd.hammingDistance<<" ");
    cv::putText(img, helper::num2str(dd.id), cv::Point(dd.cxy[0], dd.cxy[1]), CV_FONT_NORMAL, 1, helper::CV_BLUE, 2);

    cv::Mat tmp(3, 3, CV_64FC1, (double*)dd.homography[0]);
    double vm[] = {1, 0, dd.hxy[0], 0, 1, dd.hxy[1], 0, 0, 1};
    cv::Mat Homo = cv::Mat(3, 3, CV_64FC1, vm) * tmp;
    static double crns[4][2] = { {-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    helper::drawHomography(img, Homo, crns);
  }
  logi(endl);

#if TAG_DEBUG_PERFORMANCE
  static int barH = 30;
  static int textH = 12;
  static vector<cv::Scalar> pclut = helper::pseudocolor(10);
  //draw performance bar
  double total = 0;
  for (int i = 0; i < 9; ++i)
  {
    total += detector->steptime[i];
  }
  int lastx = 0;
  int lasty = barH + textH;
  for (int i = 0; i < 9; ++i)
  {
    double thisx = (detector->steptime[i] / total) * imgW + lastx;
    cv::rectangle(img, cv::Point(lastx, 0), cv::Point(thisx, barH), pclut[i], CV_FILLED);
    lastx = thisx;
    cv::putText(img, cv::format("step %d: %05.3f ms", i + 1, detector->steptime[i]), cv::Point(5, lasty - 2),
                CV_FONT_NORMAL, 0.5, pclut[i], 1);
    lasty += textH;
  }
  cv::putText(img, cv::format("fps=%4.3lf", 1000.0 / total), cv::Point(imgW / 2, barH), CV_FONT_NORMAL, 1,
              helper::CV_BLUE, 1);
  loglnd("-------------------------------");
#endif

  imshow("april", img);
  cv::waitKey(1);
#endif

}

AprilInterface::~AprilInterface()
{
}

