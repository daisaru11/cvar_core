//
//  PoseUpdaterGNLS.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include "PoseUpdaterGNLS.h"

PoseUpdaterGNLS::PoseUpdaterGNLS()
{
	// do nothing
}
PoseUpdaterGNLS::~PoseUpdaterGNLS()
{
	// do nothing
}

void PoseUpdaterGNLS::init(const Map& mapinfo)
{
	map_size = mapinfo.size;
	map_center = mapinfo.center;
	projector = mapinfo.projector;
}

bool PoseUpdaterGNLS::updatePose(std::vector<Keypoint>& keypoints, cv::Mat& pose)
{
	count = static_cast<int>(keypoints.size());

	cv::Mat Ps(count*2,1, CV_64F);
	cv::Mat Ms(count*2,1, CV_64F);

	double * M_ = (double *)Ms.data;
	double * P_ = (double *)Ps.data;

	for (int k=0; k<count; ++k)
	{
		Keypoint& kp = keypoints[k];

		P_[0] = kp.img_point.x;
		P_[1] = kp.img_point.y;
		M_[0] = kp.map_point.x;
		M_[1] = kp.map_point.y;

		P_+=2;
		M_+=2;
	}

	cv::Mat R_rod(3, 1, CV_64F);
	cv::Rodrigues(pose, R_rod);
	run(R_rod, Ps, Ms, 5, TUKEY_LM);
	cv::Rodrigues(R_rod, pose);

	return true;
}

void PoseUpdaterGNLS::project(const cv::Mat& model, const cv::Mat& m, cv::Mat& projection)
{
	cv::Mat R(3,3,CV_64F);
	cv::Rodrigues(model, R);
	projector.setRotation(R);

	double * f = (double *)m.data; // from
	double * t = (double *)projection.data; // to


	const double dx = static_cast<double>(map_center.x),
		 dy = static_cast<double>(map_center.y);
    double x, y;
	for (int k=0; k<count; ++k)
	{
		// back_projection
		projector.mapBackward(f[0]-dx, f[1]-dy, t[0], t[1]);
		f += 2;
		t += 2;
	}
}

void PoseUpdaterGNLS::reprojErr(const cv::Mat& model, const cv::Mat& observation, const cv::Mat& m, cv::Mat& error)
{
	cv::Mat R(3,3,CV_64F);
	cv::Rodrigues(model, R);
	projector.setRotation(R);

	double * f = (double *)m.data;
	double * t = (double *)observation.data;
	double *err = (double *)error.data;

	const double dx = static_cast<double>(map_center.x),
		 dy = static_cast<double>(map_center.y);
    double x, y;
	for (int k=0; k<count; ++k)
	{
		// back_projection
		projector.mapBackward(f[0]-dx, f[1]-dy, x, y);
		err[0] = t[0] - x;
		err[1] = t[1] - y;
		f += 2;
		t += 2;
		err += 2;
	}
	//std::cout << "ERROR" << std::endl;
	//std::cout << error << std::endl;
}
