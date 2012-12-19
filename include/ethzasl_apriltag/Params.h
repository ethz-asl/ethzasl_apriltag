/*
 * Params.h
 *
 *  Created on: Nov 1, 2012
 *      Author: slynen
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include <string.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <dynamic_reconfigure/server.h>
#include <ethzasl_apriltag/ethzasl_apriltagConfig.h>
#include <Eigen/Dense>
#include <tf/tf.h>

typedef ethzasl_apriltag::ethzasl_apriltagConfig varParamsT;

typedef dynamic_reconfigure::Server<varParamsT> ethzasl_apriltagParamsReconfigureServer;
typedef ethzasl_apriltag::ethzasl_apriltagConfig VarParams;

struct AprilTagProperties{
	int id;
	double scale;
	tf::Transform transform;
};

typedef std::map<int, AprilTagProperties> TagMap_T;

class FixParams
{
public:
//	double _focalLengthX;
//	double _focalLengthY;
//	double _imageCenterX;
//	double _imageCenterY;
	int _tagFamilyID;
	TagMap_T _AprilTags;

	void readFixParams();
};

class ParamsAccess
{
public:
	ParamsAccess(){};
	ParamsAccess(varParamsT* _varParams, FixParams* _fixParams){
		varParams = _varParams;
		fixParams = _fixParams;
	};

	// ptam::PtamParamsConfig* varParams{return varParams;};
	// FixParams* getFixParams(){return fixParams;};

	static varParamsT* varParams;
	static FixParams* fixParams;
};

class ethzasl_apriltagParameters{
private:
	varParamsT mVarParams;
	FixParams mFixParams;

	ethzasl_apriltagParamsReconfigureServer *methzasl_apriltagParamsReconfigureServer;

	void ethzasl_apriltagConfig(varParamsT & config, uint32_t level){
		mVarParams = config;
	};
public:
	ethzasl_apriltagParameters()
	{
		methzasl_apriltagParamsReconfigureServer = new ethzasl_apriltagParamsReconfigureServer(ros::NodeHandle("~"));
		ethzasl_apriltagParamsReconfigureServer::CallbackType ethzasl_apriltagParamCall = boost::bind(&ethzasl_apriltagParameters::ethzasl_apriltagConfig, this, _1, _2);
		methzasl_apriltagParamsReconfigureServer->setCallback(ethzasl_apriltagParamCall);

		mFixParams.readFixParams();

		ParamsAccess pAccess(&mVarParams, &mFixParams);
	}
};

#endif /* PARAMS_H_ */
