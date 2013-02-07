//
//  Relocalizer.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Relocalizer.h"

/*
Relocalizer::Relocalizer()
:roll_range(cvmath::PI/6.0), 
pitch_range(cvmath::PI), 
yaw_range(cvmath::PI/2),
roll_bin(4),
pitch_bin(12),
yaw_bin(6),
scale(0.25),
keyframes(roll_bin*2+1, std::vector<std::vector<cv::Mat> >(
	pitch_bin*2+1, std::vector<cv::Mat>(yaw_bin*2+1)))
{
}
*/
Relocalizer::Relocalizer()
:roll_range(cvmath::PI/6.0), 
pitch_range(cvmath::PI), 
yaw_range(cvmath::PI/2),
roll_bin(4),
pitch_bin(12),
yaw_bin(6),
scale(0.25),
keyframes(roll_bin*2, std::vector<std::vector<cv::Mat> >(
	pitch_bin*2, std::vector<cv::Mat>(yaw_bin*2))),
orientations(roll_bin*2, std::vector<std::vector<cv::Mat> >(
	pitch_bin*2, std::vector<cv::Mat>(yaw_bin*2)))
{
	// do nothing
}

/*
bool Relocalizer::isKeyframe(const double roll, const double pitch, const double yaw,
		int& roll_idx, int& pitch_idx, int& yaw_idx)
{
	if ( nearKeyAngle(roll_range, roll_bin, 0.03, roll, roll_idx)
		&& nearKeyAngle(pitch_range, pitch_bin, 0.075, pitch, pitch_idx)
		&& nearKeyAngle(yaw_range, yaw_bin, 0.075, yaw, yaw_idx) )
	{
		return true;
	}
	return false;
}

bool Relocalizer::isKeyframe(const cv::Mat& euler,
		int& roll_idx, int& pitch_idx, int& yaw_idx)
{
	return isKeyframe(euler.at<double>(0,0), euler.at<double>(0,1), euler.at<double>(0,2), roll_idx, pitch_idx, yaw_idx);
}
*/

bool Relocalizer::checkKeyframe(const double roll, const double pitch, const double yaw,
		int& roll_idx, int& pitch_idx, int& yaw_idx)
{
	if (getAngleIdx(roll_range, roll_bin, roll, roll_idx)
		&& getAngleIdx(pitch_range, pitch_bin, pitch, pitch_idx)
		&& getAngleIdx(yaw_range, yaw_bin, yaw, yaw_idx) )
	{
		return ! isRegistered(roll_idx, pitch_idx, yaw_idx);
	}

	return false;
}

bool Relocalizer::checkKeyframe(const cv::Mat& euler,
		int& roll_idx, int& pitch_idx, int& yaw_idx)
{
	return checkKeyframe(euler.at<double>(0,0), euler.at<double>(0,1), euler.at<double>(0,2), roll_idx, pitch_idx, yaw_idx);
}

bool Relocalizer::isRegistered(const int roll_idx, const int pitch_idx, const int yaw_idx)
{
	return ! keyframes[roll_idx][pitch_idx][yaw_idx].empty();
}

void Relocalizer::registerKeyframe(const int roll_idx, const int pitch_idx, const int yaw_idx, const cv::Mat& rawframe, const cv::Mat& euler)
{
	const int kw = static_cast<int>(rawframe.cols*scale);
	const int kh = static_cast<int>(rawframe.rows*scale);
	const cv::Size keyframe_size(kw,kh);

	cv::Mat& keyframe = keyframes[roll_idx][pitch_idx][yaw_idx];
	cv::resize(rawframe, keyframe, keyframe_size);
	cv::GaussianBlur(keyframe, keyframe, cv::Size(5,5), 1.5);

	orientations[roll_idx][pitch_idx][yaw_idx] = euler;
}

void Relocalizer::showKeyframes()
{
	for (size_t i=0; i<keyframes.size(); ++i)
	{
		for (size_t j=0; j<keyframes[0].size(); ++j)
		{
			for (size_t k=0; k<keyframes[0][0].size(); ++k)
			{
				if (!keyframes[i][j][k].empty())
				{
					cv::imshow("keyframe", keyframes[i][j][k]);
					cv::waitKey();
				}
			}
		}
	}
}

bool Relocalizer::searchKeyframe(const cv::Mat& rawframe, cv::Mat& euler)
{
	const int kw = static_cast<int>(rawframe.cols*scale);
	const int kh = static_cast<int>(rawframe.rows*scale);
	const cv::Size keyframe_size(kw,kh);
	cv::Mat frame;
	cv::resize(rawframe, frame, keyframe_size);
	cv::GaussianBlur(frame, frame, cv::Size(5,5), 1.5);
	
	const double th = 0.90;
	double score_max = 0;
	size_t max_i, max_j, max_k;

	for (size_t i=0; i<keyframes.size(); ++i)
	{
		for (size_t j=0; j<keyframes[0].size(); ++j)
		{
			for (size_t k=0; k<keyframes[0][0].size(); ++k)
			{
				if (!keyframes[i][j][k].empty())
				{
					double peak;
					cv::Mat result;
					/* */
					cv::matchTemplate(frame, keyframes[i][j][k], result, CV_TM_CCORR_NORMED);
					cv::minMaxLoc(result, NULL, &peak, NULL, NULL);
					if (peak>score_max)
					{
						score_max = peak;
						max_i = i; max_j = j; max_k = k;
					}
				}
			}
		}
	}


	if (score_max > th) {
		orientations[max_i][max_j][max_k].copyTo(euler);
		return true;
	}
	else 
		return false;
	
}
