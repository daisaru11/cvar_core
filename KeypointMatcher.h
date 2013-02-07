//
//  KeypointMatcher.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__KeypointMatcher__
#define __cvar_core__KeypointMatcher__

#include "cvarconf.h"

#include <iostream>
#include <vector>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class KeypointMatcher {

public:
	KeypointMatcher(int ncc_range, double threshold, bool refine_subpx);

	bool matchPoint(const cv::Mat& src, const cv::Point2d src_pt, const cv::Mat& target, cv::Point2d& target_pt) const;
	bool matchPatch(const cv::Mat& patch, const cv::Mat& target, cv::Point2d& target_pt) const;

protected:
private:
	int ncc_winsize;
	int ncc_winsize_dbl;
	int ncc_range;
	int ncc_range_dbl;
	bool refine_subpx;
	double threshold;

};

inline int irange(int x, int a, int b) {
	if (x>a)
		if (x<b)
			return x;
		else
			return b;
	else
		return a;
}


#endif /* defined(__cvar_core__KeypointMatcher__) */
