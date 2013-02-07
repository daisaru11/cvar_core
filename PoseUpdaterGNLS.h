//
//  PoseUpdaterGNLS.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__PoseUpdaterGNLS__
#define __cvar_core__PoseUpdaterGNLS__

#include <iostream>

#include "OptimizationGN2.h"

class PoseUpdaterGNLS : public OptimizationGN2
{
public:
	PoseUpdaterGNLS();
	virtual ~PoseUpdaterGNLS();

	void init(const Map& mapinfo);
	bool updatePose(std::vector<Keypoint>& keypoints, cv::Mat& pose);

protected:
	virtual void project(const cv::Mat& model, const cv::Mat& m, cv::Mat& projection);
	virtual void reprojErr(const cv::Mat& model, const cv::Mat& m1, const cv::Mat& m2, cv::Mat& error);

	// mapinfo
	cv::Size map_size;
	cv::Point map_center;
	CylindricalProjector projector;

	int count;
};

#endif /* defined(__cvar_core__PoseUpdaterGNLS__) */
