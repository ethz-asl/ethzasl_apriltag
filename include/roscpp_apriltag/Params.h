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
#include <roscpp_apriltag/roscpp_apriltagConfig.h>

typedef roscpp_apriltag::roscpp_apriltagConfig varParamsT;

typedef dynamic_reconfigure::Server<varParamsT> roscpp_apriltagParamsReconfigureServer;
typedef roscpp_apriltag::roscpp_apriltagConfig VarParams;

class FixParams
{
public:
	int tagID;
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

class roscpp_apriltagParameters{
private:
	varParamsT mVarParams;
	FixParams mFixParams;

	roscpp_apriltagParamsReconfigureServer *mroscpp_apriltagParamsReconfigureServer;

	void roscpp_apriltagConfig(varParamsT & config, uint32_t level){
		mVarParams = config;
	};
public:
	roscpp_apriltagParameters()
	{
		mroscpp_apriltagParamsReconfigureServer = new roscpp_apriltagParamsReconfigureServer(ros::NodeHandle("~"));
		roscpp_apriltagParamsReconfigureServer::CallbackType roscpp_apriltagParamCall = boost::bind(&roscpp_apriltagParameters::roscpp_apriltagConfig, this, _1, _2);
		mroscpp_apriltagParamsReconfigureServer->setCallback(roscpp_apriltagParamCall);

		mFixParams.readFixParams();

		ParamsAccess pAccess(&mVarParams, &mFixParams);
	}
};

#endif /* PARAMS_H_ */
