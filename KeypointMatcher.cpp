//
//  KeypointMatcher.cpp
//  cvtest0
//
//  Created by 酒井 大地 on 2013/01/16.
//  Copyright (c) 2013年 Daichi Sakai. All rights reserved.
//

#include "KeypointMatcher.h"
#include <iostream>

KeypointMatcher::KeypointMatcher(int _ncc_range, double _threshold, bool _refine_subpx)
:ncc_winsize(3), ncc_range(_ncc_range), 
ncc_winsize_dbl(3*2+1), ncc_range_dbl(_ncc_range*2+1),
threshold(_threshold),
refine_subpx(_refine_subpx)
{
	//nothing to do
}

bool KeypointMatcher::matchPoint(const cv::Mat& src, const cv::Point2d src_pt, const cv::Mat& target, cv::Point2d& target_pt) const
{
	const int wx = irange(static_cast<int>(src_pt.x)-ncc_winsize,0,src.cols),
		wy = irange(static_cast<int>(src_pt.y)-ncc_winsize,0,src.rows),
		ww = wx+ncc_winsize_dbl>src.cols? src.cols-wx:ncc_winsize_dbl,
		wh = wy+ncc_winsize_dbl>src.rows? src.rows-wy:ncc_winsize_dbl;

	cv::Mat win(src, cv::Rect(wx,wy,ww,wh));

	bool found = matchPatch(win, target, target_pt);

	if (!found)
		return false;

	target_pt.x += ncc_winsize;
	target_pt.y += ncc_winsize;

	return true;
}

bool KeypointMatcher::matchPatch(const cv::Mat& patch, const cv::Mat& target, cv::Point2d& target_pt) const {

	const int rx = irange(static_cast<int>(target_pt.x)-ncc_range,0,target.cols),
		ry = irange(static_cast<int>(target_pt.y)-ncc_range,0,target.rows),
		rw = rx+ncc_range_dbl>target.cols? target.cols-rx:ncc_range_dbl,
		rh = ry+ncc_range_dbl>target.rows? target.rows-ry:ncc_range_dbl;
	cv::Mat search_range(target, cv::Rect(rx,ry,rw,rh));
	
	if (rw < patch.cols || rh < patch.rows)
		return false;

	double peak;
	cv::Mat result;
	cv::Point peak_pt;
	/* */
	cv::matchTemplate(search_range, patch, result, CV_TM_CCORR_NORMED);
	cv::minMaxLoc(result, NULL, &peak, NULL, &peak_pt);

	if (peak < threshold) {
		return false;
	}
	/* *
	cv::matchTemplate(search_range, patch, result, CV_TM_SQDIFF_NORMED);
	cv::minMaxLoc(result, &peak, NULL, &peak_pt, NULL);
	/* */

	double peak_x = static_cast<double>(peak_pt.x);
	double peak_y = static_cast<double>(peak_pt.y);
	
	if (refine_subpx)
	{
		//Quadratic Interpolation
		const double z_00   = static_cast<double>( result.at<float>(peak_y,peak_x) );
		const double z_p10  = static_cast<double>( result.at<float>(peak_y,peak_x+1) );
		const double z_m10  = static_cast<double>( result.at<float>(peak_y,peak_x-1) );
		const double z_0m1  = static_cast<double>( result.at<float>(peak_y-1,peak_x) );
		//const double z_p1m1 = static_cast<double>( result.at<float>(peak_y-1,peak_x+1) );
		//const double z_m1m1 = static_cast<double>( result.at<float>(peak_y-1,peak_x-1) );
		const double z_0p1  = static_cast<double>( result.at<float>(peak_y+1,peak_x) );
		//const double z_p1p1 = static_cast<double>( result.at<float>(peak_y+1,peak_x+1) );
		//const double z_m1p1 = static_cast<double>( result.at<float>(peak_y+1,peak_x-1) );

		//const double a = z_00;
		const double b = (z_p10 - z_m10) / 2;
		const double c = (z_0p1 - z_0m1) / 2;
		const double d = -z_00 + (z_p10 + z_m10) / 2;
		const double e = -z_00 + (z_0p1 + z_0m1) / 2;
		
		target_pt.x = rx + peak_x + b/(2*d);
		target_pt.y = ry + peak_y + c/(2*e);
		//std::cout << "===" << std::endl;
		//std::cout << target_pt.x << std::endl;
		//std::cout << rx+peak_x << std::endl;
	}
	else
	{
		target_pt.x = rx + peak_x;
		target_pt.y = ry + peak_y;
	}
	
	return true;
}

