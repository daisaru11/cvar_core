//
//  PoseUpdaterGN.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Map.h"
#include "CylindricalProjector.h"
#include "Keypoint.h"
#include "Optimization.h"

#ifndef __cvar_core__PoseUpdaterGN__
#define __cvar_core__PoseUpdaterGN__

class PoseUpdaterGN {
	
public:

	PoseUpdaterGN();
	void init(const Map& mapinfo);
	bool updatePose(std::vector<Keypoint>& keypoints, cv::Mat& pose);

private:
	
	// mapinfo
	cv::Size map_size;
	cv::Point map_center;
	CylindricalProjector projector;

	std::vector<Keypoint>* kps;
	
	bool _updatePose(cv::Mat& pose);
	static void Project(CvMat* state, CvMat* projection, void *param);
};


#endif /* defined(__cvar_core__PoseUpdaterGN__) */
