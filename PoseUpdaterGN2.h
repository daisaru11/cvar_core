//
//  PoseUpdaterGN2.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__PoseUpdaterGN2__
#define __cvar_core__PoseUpdaterGN2__

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Map.h"
#include "CylindricalProjector.h"
#include "Keypoint.h"
#include "Optimization.h"

class PoseUpdaterGN2
{
public:
	PoseUpdaterGN2();
	void init(const Map& mapinfo);
	bool updatePose(const std::vector<Keypoint> keypoints, cv::Mat& pose);
	cv::Mat Ps;
	cv::Mat Ms;

private:
	// mapinfo
	cv::Size map_size;
	cv::Point map_center;
	CylindricalProjector projector;

	static void Project(CvMat* state, CvMat* projection, void *param);
	int count;
};

#endif /* defined(__cvar_core__PoseUpdater__) */
