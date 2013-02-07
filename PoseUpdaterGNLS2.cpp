//
//  PoseUpdaterGNLS2.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include "PoseUpdaterGNLS2.h"

PoseUpdaterGNLS2::PoseUpdaterGNLS2()
:count(0)
{
	// do nothing
}

PoseUpdaterGNLS2::~PoseUpdaterGNLS2()
{
	// do nothing
}

void PoseUpdaterGNLS2::init(const Map& mapinfo)
{
	map_size = mapinfo.size;
	map_center = mapinfo.center;
	projector = mapinfo.projector;
}

bool PoseUpdaterGNLS2::updatePose(std::vector<Keypoint>& keypoints, cv::Mat& pose)
{
	count = static_cast<int>(keypoints.size());

	cv::Mat Ps(count*3,1, CV_64F);
	cv::Mat Ms(count*3,1, CV_64F);

	//cv::Mat K(3,3,CV_64F,projector.k);
	double *kinv = projector.kinv;
	double scale = projector.scale;

	double * M_ = (double *)Ms.data;
	double * P_ = (double *)Ps.data;

	for (int k=0; k<count; ++k)
	{
		Keypoint& kp = keypoints[k];

		double mx = kp.map_point.x-map_center.x;
		double my = kp.map_point.y-map_center.y;
		double r = sqrt( scale*scale + my*my );

		M_[0] = scale * sin(mx/scale) / r;
		M_[1] = my / r;
		M_[2] = scale * cos(mx/scale) / r;

		double px = kp.img_point.x;
		double py = kp.img_point.y;
		//double pz = 1.0;
		P_[0] = kinv[0] * px + kinv[1] * py + kinv[2];
		P_[1] = kinv[3] * px + kinv[4] * py + kinv[5];
		P_[2] = kinv[6] * px + kinv[7] * py + kinv[8];

		double nP = sqrt(P_[0]*P_[0] + P_[1]*P_[1] + P_[2]*P_[2]);
		P_[0] /= nP;
		P_[1] /= nP;
		P_[2] /= nP;

		M_ += 3;
		P_ += 3;
	}

	cv::Mat R_rod(3, 1, CV_64F);
	cv::Rodrigues(pose, R_rod);
	run(R_rod, Ps, Ms, 5, TUKEY_LM);
	cv::Rodrigues(R_rod, pose);

	return true;
}

void PoseUpdaterGNLS2::project(const cv::Mat& model, const cv::Mat& m, cv::Mat& projection)
{
	cv::Mat R(3,3,CV_64F);
	cv::Rodrigues(model, R);
	const double *R_ = (double *)R.data;

	double * f = (double *)m.data; // from
	double * t = (double *)projection.data; // to

	for(int i = 0; i < count; i++ )
	{
		t[0] = R_[0]*f[0] + R_[1]*f[1] + R_[2]*f[2];
		t[1] = R_[3]*f[0] + R_[4]*f[1] + R_[5]*f[2];
		t[2] = R_[6]*f[0] + R_[7]*f[1] + R_[8]*f[2];

		f += 3;
		t += 3;
	}
}

void PoseUpdaterGNLS2::reprojErr(const cv::Mat& model, const cv::Mat& observation, const cv::Mat& m, cv::Mat& error)
{
	cv::Mat R(3,3,CV_64F);
	cv::Rodrigues(model, R);
	const double *R_ = (double *)R.data;

	double * f = (double *)m.data;
	double * t = (double *)observation.data;
	double *err = (double *)error.data;

	for(int i = 0; i < count; i++ )
	{
		// P = RM
		double a = R_[0]*f[0] + R_[1]*f[1] + R_[2]*f[2];
		double b = R_[3]*f[0] + R_[4]*f[1] + R_[5]*f[2];
		double c = R_[6]*f[0] + R_[7]*f[1] + R_[8]*f[2];
		double e0 = t[0] - (R_[0]*f[0] + R_[1]*f[1] + R_[2]*f[2]);
		double e1 = t[1] - (R_[3]*f[0] + R_[4]*f[1] + R_[5]*f[2]);
		double e2 = t[2] - (R_[6]*f[0] + R_[7]*f[1] + R_[8]*f[2]);

		err[0] = t[0] - (R_[0]*f[0] + R_[1]*f[1] + R_[2]*f[2]);
		err[1] = t[1] - (R_[3]*f[0] + R_[4]*f[1] + R_[5]*f[2]);
		err[2] = t[2] - (R_[6]*f[0] + R_[7]*f[1] + R_[8]*f[2]);

		f += 3;
		t += 3;
		err += 3;
	}
}
