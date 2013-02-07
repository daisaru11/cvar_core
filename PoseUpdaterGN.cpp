//
//  PoseUpdaterGN.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include "PoseUpdaterGN.h"

PoseUpdaterGN::PoseUpdaterGN()
{
	// do nothing
}

void PoseUpdaterGN::init(const Map& mapinfo)
{
	map_size = mapinfo.size;
	map_center = mapinfo.center;
	projector = mapinfo.projector;
}

void PoseUpdaterGN::Project(CvMat* state, CvMat* projection, void *param)
{
	PoseUpdaterGN *updater = (PoseUpdaterGN*)param;

	cv::Mat R_rod(3, 1, CV_64F, &(state->data.db[0+0]));
	double R_data[3][3];
	cv::Mat R_m(3,3,CV_64F,R_data);
	cv::Rodrigues(R_rod, R_m);

	updater->projector.setRotation(R_m);

	const double dx = static_cast<float>(updater->map_center.x),
		 dy = static_cast<float>(updater->map_center.y);
    double x, y;
	int ind = 0;
	for (std::vector<Keypoint>::iterator kp=updater->kps->begin(); kp!=updater->kps->end(); ++kp)
	{
		// back_projection
		updater->projector.mapBackward(kp->map_point.x-dx, kp->map_point.y-dy, x, y);
		projection->data.db[ind*2+0] = x;
		projection->data.db[ind*2+1] = y;
		//std::cout << "projection: " << projection->data.db[ind*2+0] <<", map_point: " << kp->map_point.x << std::endl;
		ind++;
	}
}

bool PoseUpdaterGN::_updatePose(cv::Mat& pose)
{
	int count_points = static_cast<int>(kps->size());
	if(count_points < 6) return false;

	//CvMat* _M = cvCreateMat(count_points, 1, CV_64FC3);
	CvMat* image_observations = cvCreateMat(count_points*2, 1, CV_64F); // [u v u v u v ...]'

	//map<int,Feature>::iterator it;
	int ind = 0;
	for(std::vector<Keypoint>::iterator it=kps->begin(); it!=kps->end(); ++it)
	{
		image_observations->data.db[ind*2+0] = it->img_point.x;
		image_observations->data.db[ind*2+1] = it->img_point.y;
		//std::cout << "observation: " << image_observations->data.db[ind*2+0] <<", map_point: " << it->map_point.x << std::endl;
		ind++;
	}
	// count_points == ind

	double rot[3];
	cv::Mat R_rod(3, 1, CV_64F, rot);
	cv::Rodrigues(pose, R_rod);

	CvMat* par = cvCreateMat(3, 1, CV_64F);
	memcpy(&(par->data.db[0+0]), rot, 3*sizeof(double));
	//par->data.db[3] = 0;

	Optimization *opt = new Optimization(3, 2*ind);
	
	double foo = opt->Optimize(par, image_observations, 0.0005, 5, Project, this, Optimization::TUKEY_LM);
	memcpy(rot, &(par->data.db[0+0]), 3*sizeof(double));
	cv::Rodrigues(R_rod, pose);
    
	delete opt;

	cvReleaseMat(&par);
	cvReleaseMat(&image_observations);
	//cvReleaseMat(&_M);

	return true;
}

bool PoseUpdaterGN::updatePose(std::vector<Keypoint>& keypoints, cv::Mat& pose)
{
	kps = &keypoints;

	_updatePose(pose);

	kps = NULL;

	return true;
}
