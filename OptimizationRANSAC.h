//
//  OptimizationRANSAC.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include <iostream>
#include <opencv2/core/core.hpp>

#include "Map.h"
#include "CylindricalProjector.h"
#include "Keypoint.h"

#ifndef __cvar_core__OptimizationRANSAC__
#define __cvar_core__OptimizationRANSAC__

class OptimizationRANSAC
{
public:
	OptimizationRANSAC(const int num_model_points);
	virtual ~OptimizationRANSAC();

	bool run(const cv::Mat& m1, const cv::Mat& m2, cv::Mat& model, cv::Mat& mask,
			const double reproj_threshold, const double confidence, 
			const int max_iters );

protected:

	bool getSubset(const cv::Mat& m1, const cv::Mat& m2, std::vector<int>& subset_idx);
	int findInliers( const cv::Mat& m1, const cv::Mat& m2, const cv::Mat& model, cv::Mat& err, cv::Mat& mask, double threshold );
	virtual void fitSubset(const cv::Mat& m1, const cv::Mat& m2, std::vector<int>& subset_idx, cv::Mat& model) = 0;
	virtual void computeReprojError( const cv::Mat& m1, const cv::Mat& m2, const cv::Mat& model, cv::Mat& error ) = 0;

	int num_model_points;
};

#endif
