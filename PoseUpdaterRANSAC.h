//
//  PoseUpdaterRANSAC.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__PoseUpdaterRANSAC__
#define __cvar_core__PoseUpdaterRANSAC__

#include <iostream>

#include "OptimizationRANSAC.h"

class PoseUpdaterRANSAC : public OptimizationRANSAC
{
public:
	PoseUpdaterRANSAC(const double confidence, const int max_iters, const double reproj_threshold);
	virtual ~PoseUpdaterRANSAC();

	void init(const Map& mapinfo);
	bool updatePose(std::vector<Keypoint>& keypoints, cv::Mat& pose);

protected:
	virtual void fitSubset(const cv::Mat& m1, const cv::Mat& m2, std::vector<int>& subset_idx, cv::Mat& model);
	virtual void computeReprojError( const cv::Mat& m1, const cv::Mat& m2, const cv::Mat& model, cv::Mat& error );
	void fit(const cv::Mat& m1, const cv::Mat& m2, const cv::Mat& mask, cv::Mat& model);

	double confidence;
	int max_iters;
	double reproj_threshold;

	// mapinfo
	cv::Size map_size;
	cv::Point map_center;
	CylindricalProjector projector;
};


#endif /* defined(__cvar_core__PoseUpdaterRANSAC__) */
