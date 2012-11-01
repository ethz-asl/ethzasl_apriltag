/*
 * AprilROSNode.cpp
 *
 *  Created on: Nov 1, 2012
 *      Author: slynen
 */

//#define TAG_DEBUG_PERFORMANCE 1
//#define TAG_DEBUG_DRAW 1

#include "roscpp_apriltag/AprilROSNode.h"
#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/foreach.hpp>

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

AprilROSNode::AprilROSNode() :
						nh_("AprilTracker"), image_nh_(""), first_frame_(true){


	std::string topic = image_nh_.resolveName("image");
	if (topic == "/image")
	{
		ROS_WARN("video source: image has not been remapped! Typical command-line usage:\n"
				"\t$ ./AprilROSNode image:=<image topic>");
	}

	image_transport::ImageTransport it(image_nh_);
	sub_image_ = it.subscribe(topic, 1, &AprilROSNode::imageCallback, this, image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay(true)));



	//// create tagFamily
	int tagid = ParamsAccess::fixParams->tagID;
	tagFamily = TagFamilyFactory::create(tagid);
	if(tagFamily.empty()) {
		loglne("[main] create TagFamily fail!");
		exit(1);
	}
	detector = new TagDetector(tagFamily);
	if(detector.empty()) {
		loglne("[main] create TagDetector fail!");
		exit(1);
	}
	cv::namedWindow("april");
}

void AprilROSNode::imageCallback(const sensor_msgs::ImageConstPtr & msg){
	ROS_ASSERT(msg->encoding == sensor_msgs::image_encodings::MONO8 && msg->step == msg->width);

	VarParams *varParams = ParamsAccess::varParams;

	if(first_frame_){
		std::cout<<"Got first image"<<std::endl;
		//some init?
		first_frame_ = false;
	}


	//track the april tag and publish the tf

	static helper::PerformanceMeasurer PM;
	double imgW=msg->width, imgH=msg->height; //todo slynen: use calibration data!
	vector<TagDetection> detections;
	double opticalCenter[2] = { imgW/2.0, imgH/2.0 };
	PM.tic();

	namespace enc = sensor_msgs::image_encodings;

	cv::Mat img(msg->height, msg->width, CV_8U, const_cast<uint8_t*>(&(msg->data[0])));

	detector->process(img, opticalCenter, detections);

	BOOST_FOREACH(TagDetection& detect, detections){

	}



	//some debugging tools
#if TAG_DEBUG_DRAW
	loglni("[TagDetector] process time = "<<PM.toc()<<" sec.");

	logi(">>> find id: ");
	for(int id=0; id<(int)detections.size(); ++id) {
		TagDetection &dd = detections[id];
		if(dd.hammingDistance>2) continue;

		logi("#"<<dd.id<<"|"<<dd.hammingDistance<<" ");
		cv::putText( img, helper::num2str(dd.id), cv::Point(dd.cxy[0],dd.cxy[1]), CV_FONT_NORMAL, 1, helper::CV_BLUE, 2 );

		cv::Mat tmp(3,3,CV_64FC1, (double*)dd.homography[0]);
		double vm[] = {1,0,dd.hxy[0],0,1,dd.hxy[1],0,0,1};
		cv::Mat Homo = cv::Mat(3,3,CV_64FC1,vm) * tmp;
		static double crns[4][2]={
				{-1, -1},
				{ 1, -1},
				{ 1,  1},
				{-1,  1}
		};
		helper::drawHomography(img, Homo, crns);
	}
	logi(endl);


#if TAG_DEBUG_PERFORMANCE
	static int barH = 30;
	static int textH = 12;
	static vector<cv::Scalar> pclut = helper::pseudocolor(10);
	//draw performance bar
	double total = 0;
	for(int i=0; i<9; ++i) {
		total += detector->steptime[i];
	}
	int lastx=0;
	int lasty=barH+textH;
	for(int i=0; i<9; ++i) {
		double thisx = (detector->steptime[i]/total)*imgW+lastx;
		cv::rectangle(img, cv::Point(lastx,0), cv::Point(thisx,barH), pclut[i], CV_FILLED);
		lastx = thisx;
		cv::putText(img, cv::format("step %d: %05.3f ms",i+1,detector->steptime[i]),
				cv::Point(5,lasty-2), CV_FONT_NORMAL, 0.5, pclut[i], 1 );
		lasty += textH;
	}
	cv::putText(img, cv::format("fps=%4.3lf",1000.0/total), cv::Point(imgW/2,barH), CV_FONT_NORMAL, 1, helper::CV_BLUE, 1);
	loglnd("-------------------------------");
#endif

	imshow("april", img);
	cv::waitKey(1);
#endif

}

AprilROSNode::~AprilROSNode() {
	// TODO Auto-generated destructor stub
}

