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
		/*
		double interp_x, interp_y;
		cv::Point2f interp_peak_pt;
		if (0 < peak_pt.x && peak_pt.x < result.cols-1)
		{
			unsigned int y = peak_pt.y;
			float a = result.at<float>(y,peak_pt.x-1);
			float b = result.at<float>(y,peak_pt.x);
			float g = result.at<float>(y,peak_pt.x+1);
			interp_peak_pt.x = peak_pt.x + ((a-g) / ((a-2*b+g)*2));
			
		} else {
			interp_peak_pt.x = static_cast<float>(peak_pt.x);
		}
		if (0 < peak_pt.y && peak_pt.y < result.rows-1)
		{
			unsigned int x = peak_pt.x;
			float a = result.at<float>(peak_pt.y-1,x);
			float b = result.at<float>(peak_pt.y,x);
			float g = result.at<float>(peak_pt.y+1,x);
			interp_peak_pt.y = peak_pt.y + ((a-g) / ((a-2*b+g)*2));
			
		} else {
			interp_peak_pt.y = static_cast<float>(peak_pt.y);
		}
		*/
		target_pt.x = rx + peak_x;
		target_pt.y = ry + peak_y;
	}
	else
	{
		target_pt.x = rx + peak_x;
		target_pt.y = ry + peak_y;
	}
	
	return true;
}

