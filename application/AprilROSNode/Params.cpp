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

	nh.param("focal_length_x", _focalLengthX, 614.0);
	nh.param("focal_length_y", _focalLengthY, 614.4);
	nh.param("image_center_x", _imageCenterX, 376.0);
	nh.param("image_center_y", _imageCenterY, 240.0);
	nh.param("tag_family_id", _tagFamilyID, 4);

	ROS_INFO("parsing parameters for tag properties");

	//for all possible tags, see whether we have parameters for these
	for(int i = 0;i<587;++i){

		char buffer [50];
		sprintf (buffer, "april%03d", i);
		std::string idname(buffer);
		int tagID;
		nh.param(idname+"/id", tagID, -1); //see whether there is a tagid given for this tag
		if(tagID == -1) continue; //no definition for this tag in the parameter file

		AprilTagProperties tagprops;
		tagprops.id = tagID;

		assert(i == tagID && "Tag ID of one tag in the parameter file is set incorrectly");

		nh.param(idname+"/scale", tagprops.scale, 0.04);

		double px, py, pz, qx, qy, qz, qw;
		nh.param(idname+"/p/x", px, 0.0);
		nh.param(idname+"/p/y", py, 0.0);
		nh.param(idname+"/p/z", pz, 0.0);
		nh.param(idname+"/q/x", qx, 0.0);
		nh.param(idname+"/q/y", qy, 0.0);
		nh.param(idname+"/q/z", qz, 0.0);
		nh.param(idname+"/q/w", qw, 1.0);

		tf::Quaternion quat(qx, qy, qz, qw);
		tf::Vector3 pos(px, py, pz);
		tagprops.transform.setRotation(quat);
		tagprops.transform.setOrigin(pos);

		this->_AprilTags[tagprops.id] = tagprops;
	}

	std::stringstream ss;
	ss<<"Read the following tags from the parameter file:"<<std::endl;
	for(TagMap_T::iterator it = this->_AprilTags.begin(); it!=this->_AprilTags.end();++it){
		ss<<"------------------"<<std::endl;
		ss<<"ID: "<<it->second.id<<std::endl;
		ss<<"scale: "<<it->second.scale<<std::endl;
		ss<<"tf-p xyz: "<<it->second.transform.getOrigin().getX()<<
				" "<<it->second.transform.getOrigin().getY()<<
				" "<<it->second.transform.getOrigin().getZ()<<std::endl;
		ss<<"tf-q wxyz: "<<it->second.transform.getRotation().getW()<<
				" "<<it->second.transform.getRotation().getX()<<
				" "<<it->second.transform.getRotation().getY()<<
				" "<<it->second.transform.getRotation().getZ()<<std::endl;
	}
	ss<<"------------------"<<std::endl;
	ROS_INFO_STREAM(ss);
}
;

roscpp_apriltag::roscpp_apriltagConfig* ParamsAccess::varParams;
FixParams* ParamsAccess::fixParams;
