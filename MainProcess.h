//
//  MainProcess.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__MainProcess__
#define __cvar_core__MainProcess__

#include "cvarconf.h"

#include <iostream>

#include "cvmath.h"
#include "Mapper.h"
#include "Relocalizer.h"
#include "PlanarFinder.h"
#include "TimerManager.h"

#include "PoseUpdaterRANSAC.h"
#include "PoseUpdaterGN.h"
//#include "PoseUpdaterGN2.h"
#include "PoseUpdaterGNLS.h"
//#include "PoseUpdaterGNLS2.h"

class MainProcess
{
public: 
	MainProcess(const cv::Mat& K, const double projector_scale, const cv::Size map_size, const cv::Size img_size,
		const std::vector<double>& map_scales,
		const std::vector<size_t>& max_keypoints_cnt_per_cell
	);

	void start(const cv::Mat& pose);
	bool process();
	bool findPlanar(const cv::Point2d& target_pt, cv::Mat& debug);
	bool updatePlanar(const double scale, cv::Mat& debug);
	bool adjustPlanar(cv::Mat& debug);
	void trackPointsBackProject(double* dst);
	void trackPointsBackProject(cv::Point2d* dst);
	void trackPointsBackProject(PlanarRect& planar);
	void trackPointsProject(const cv::Point2d* dst);
	void trackPointsProject(const PlanarRect& planar);
	void debugOutput(cv::Mat& output);
	void debugOutput2(cv::Mat& output);

	void setFrame(const size_t map_scale_idx, const cv::Mat& frame, const cv::Mat& gray);
	double getFrameScale(const size_t map_scale_idx);

	Mapper mapper;
	Relocalizer relocalizer;

	PoseUpdaterGN updater_gn_s0;
	PoseUpdaterGN updater_gn_s1;
	//PoseUpdaterGNLS updater_gn_s0;
	//PoseUpdaterGNLS updater_gn_s1;
	//PoseUpdaterGN2 updater_gn2_s0;
	PoseUpdaterRANSAC updater_rs_s0;
	PoseUpdaterRANSAC updater_rs_s1;

	void mulRotation(cv::Mat& R)
	{
		current_pose = current_pose * R;
		mapper.setProjectorRotation(current_pose);
	}

	void getPoseRotation(cv::Mat& R)
	{
		current_pose.copyTo(R);
	}

	bool hasTrackPoints()
	{
		return is_trackpoints_mapped;
	}

private:

	double trackRoughly(const int search_range, std::vector<Keypoint>& keypoints);
	double track(const int search_range, std::vector<Keypoint>& keypoints);
	bool relocalize();

	cv::Mat current_pose;

	enum ProcessState {
		STATE_NONE = 0,
		STATE_TRACKING = 1,
		STATE_LOST = 2
	};
	ProcessState state;
	int non_relialbe_itr; //count non-reliable tracking iterated

	// planar estimation
	cv::Size tracking_src_size;
	cv::Point2d tracked_mappoints[4];
	bool is_trackpoints_mapped;

	// timer
	TimerManager debug_timer;

};

#endif /* defined(__cvar_core__MainProcess__) */
