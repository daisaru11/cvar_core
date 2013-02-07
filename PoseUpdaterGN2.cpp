//
//  PoseUpdaterGN2.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include "PoseUpdaterGN2.h"

PoseUpdaterGN2::PoseUpdaterGN2()
{
}

void PoseUpdaterGN2::Project(CvMat* state, CvMat* projection, void *param)
{
	PoseUpdaterGN2 *updater = (PoseUpdaterGN2*)param;

	cv::Mat R_rod(3, 1, CV_64F, &(state->data.db[0+0]));
	double R_[9];
	cv::Mat R_m(3,3,CV_64F,R_);
	cv::Rodrigues(R_rod, R_m);

	double *f = (double *)updater->Ms.data;
	double *p = (double *)projection->data.db;
	for (int k=0; k<updater->count; ++k)
	{
		p[0] = R_[0]*f[0] + R_[1]*f[1] + R_[2]*f[2];
		p[1] = R_[3]*f[0] + R_[4]*f[1] + R_[5]*f[2];
		p[2] = R_[6]*f[0] + R_[7]*f[1] + R_[8]*f[2];
		f+=3;
		p+=3;
	}
}

void PoseUpdaterGN2::init(const Map& mapinfo)
{
	map_size = mapinfo.size;
	map_center = mapinfo.center;
	projector = mapinfo.projector;
}

bool PoseUpdaterGN2::updatePose(const std::vector<Keypoint> keypoints, cv::Mat& pose)
{
	count = static_cast<int>(keypoints.size());
	Ps.create(count*3,1, CV_64F);
	Ms.create(count*3,1, CV_64F);

	//cv::Mat K(3,3,CV_64F,projector.k);
	double *kinv = projector.kinv;
	double scale = projector.scale;

	double * M_ = (double *)Ms.data;
	double * P_ = (double *)Ps.data;

	for (int k=0; k<count; ++k)
	{
		const Keypoint& kp = keypoints[k];

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
	M_ = (double *)Ms.data;
	P_ = (double *)Ps.data;
	for (int k=0; k<count; ++k)
	{
		cv::Mat M(3,1,CV_64F,M_);
		cv::Mat P(3,1,CV_64F,P_);
		//std::cout << P << std::endl;
		//std::cout << pose*M << std::endl;
		M_ += 3;
		P_ += 3;
	}
		
	CvMat image_observations = Ps;

	double rot[3];
	cv::Mat R_rod(3, 1, CV_64F, rot);
	cv::Rodrigues(pose, R_rod);

	CvMat* par = cvCreateMat(3, 1, CV_64F);
	memcpy(&(par->data.db[0+0]), rot, 3*sizeof(double));

	Optimization *opt = new Optimization(3, 3*count);
	
	double foo = opt->Optimize(par, &image_observations, 0.0005, 5, Project, this, Optimization::TUKEY_LM);
	memcpy(rot, &(par->data.db[0+0]), 3*sizeof(double));
	cv::Rodrigues(R_rod, pose);
    
	delete opt;

	cvReleaseMat(&par);
	//cvReleaseMat(&image_observations);
	//cvReleaseMat(&_M);

	return true;
}
