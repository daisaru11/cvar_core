//
//  Relocalizer.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__Relocalizer__
#define __cvar_core__Relocalizer__

#include <iostream>
#include "cvmath.h"

class Relocalizer
{
public:
	Relocalizer();
	//bool isKeyframe(const double roll, const double pitch, const double yaw,
	//	int& roll_idx, int& pitch_idx, int& yaw_idx);
	//bool isKeyframe(const cv::Mat& euler,
	//	int& roll_idx, int& pitch_idx, int& yaw_idx);
	bool checkKeyframe(const double roll, const double pitch, const double yaw,
		int& roll_idx, int& pitch_idx, int& yaw_idx);
	bool checkKeyframe(const cv::Mat& euler,
		int& roll_idx, int& pitch_idx, int& yaw_idx);
	bool isRegistered(const int roll_idx, const int pitch_idx, const int yaw_idx);
	void registerKeyframe(const int roll_idx, const int pitch_idx, const int yaw_idx, const cv::Mat& rawframe, const cv::Mat& euler);
	void showKeyframes();
	bool searchKeyframe(const cv::Mat& rawframe, cv::Mat& euler);


private:
	
	double roll_range;
	double pitch_range;
	double yaw_range;
	int roll_bin;
	int pitch_bin;
	int yaw_bin;
	double scale;
	std::vector<std::vector<std::vector<cv::Mat> > > keyframes;
	std::vector<std::vector<std::vector<cv::Mat> > > orientations; // euler angle matrices

	bool getAngleIdx(const double range, const int bin, const double angle, int& idx)
	{
		double step=range/bin;
		idx = static_cast<int>(angle/step) + bin;
		return 0 <= idx && idx < 2*bin;
	}
	
	/*
	bool nearKeyAngle(const double range, const int bin, const double th, const double angle, int& idx)
	{
		double step, w;
		step=range/bin;
		w = -range;
		for (int i=-bin ; i<=bin; ++i, w+=step)
		{
			if ( w-th<angle && angle<w+th )
			{
				idx = i+bin;
				return true;
			}
		}
		return false;
	}
	*/


};

#endif /* defined(__cvar_core__Relocalizer__) */
