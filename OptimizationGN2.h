//
//  Mapper.h
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

#ifndef __cvar_core__OptimizationGN2__
#define __cvar_core__OptimizationGN2__

class OptimizationGN2
{
public:
	enum Method {
		GAUSSNEWTON = 0,
		TUKEY_LM = 1
	};

	OptimizationGN2();
	virtual ~OptimizationGN2();
	void run( cv::Mat& model, 
		const cv::Mat& m1, const cv::Mat m2,   // m1 = model( m2 )
		int max_iter, Method method);

protected:
	void calcJacobian(const cv::Mat& model, const cv::Mat& m, cv::Mat& Jt);
	double calcTukeyWeight(double residual, double c);
	virtual void project(const cv::Mat& model, const cv::Mat& m, cv::Mat& projection) = 0;
	virtual void reprojErr(const cv::Mat& model, const cv::Mat& m1, const cv::Mat& m2, cv::Mat& error) = 0;

	// mapinfo
	int n_params;
	int n_meas;
};


#endif
