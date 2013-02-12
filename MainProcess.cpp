//
//  MainProcess.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include "MainProcess.h"

MainProcess::MainProcess(const cv::Mat& K, const double projector_scale, const cv::Size map_size, const cv::Size img_size,
	const std::vector<double>& map_scales,
	const std::vector<size_t>& max_keypoints_cnt_per_cell
)
:mapper(map_size, img_size, map_scales, max_keypoints_cnt_per_cell),
//tracker(),
relocalizer(),
updater_gn_s0(),
updater_gn_s1(),
updater_rs_s0(0.995,50,0.048),
updater_rs_s1(0.998,50,0.028),
state(STATE_NONE),
is_trackpoints_mapped(false),
debug_timer()
{
	current_pose = cv::Mat::eye(3,3,CV_64F);
	mapper.setProjectorParams(K, projector_scale, current_pose);
	updater_gn_s0.init(mapper.maps[0]);
	updater_gn_s1.init(mapper.maps[1]);
	//updater_gn2_s0.init(mapper.maps[0]);
	updater_rs_s0.init(mapper.maps[0]);
	updater_rs_s1.init(mapper.maps[1]);
}

void MainProcess::start(const cv::Mat& pose)
{
	current_pose = pose;
	mapper.setProjectorRotation(pose);

	//state = STATE_TRACKING;
	state = STATE_NONE;
	non_relialbe_itr = 0;
}

void MainProcess::setFrame(const size_t map_scale_idx, const cv::Mat& frame, const cv::Mat& gray)
{
	mapper.setFrameImage(map_scale_idx, frame);
	mapper.setFrameGray(map_scale_idx, gray);
}

double MainProcess::getFrameScale(const size_t map_scale_idx)
{
	return mapper.getScale(map_scale_idx);
}

bool MainProcess::process()
{
    clock_t t0,t1,t2,t3,t4,t5;
	double rscore = 1.0;

	t0 = clock();
	if ( state == STATE_TRACKING )
	{
		// track on low-resolution map
		double l_track_score;
		const double l_track_score_th = 0.4;
		std::vector<Keypoint> l_keypoints;

		debug_timer.start("Rough Tracking");
#ifdef CVAR_IOS
		l_track_score = trackRoughly(8, l_keypoints);
#elif defined CVAR_PC_MOCK
		l_track_score = trackRoughly(8, l_keypoints);
#else
		l_track_score = trackRoughly(12, l_keypoints);
#endif
		if ( l_track_score < l_track_score_th )
		{
			if ( ++non_relialbe_itr > 3 )
			{
				state = STATE_LOST;
			}
			return false;
		}
		debug_timer.stop("Rough Tracking");
		debug_timer.print("Rough Tracking");

		debug_timer.start("Rough PoseUpdate");
#ifdef CVAR_USE_UPDATE_RANSAC
		//updater_rs_s1.updatePose(l_keypoints, current_pose);
		updater_gn_s1.updatePose(l_keypoints, current_pose);
		mapper.setProjectorRotation(current_pose);
#else
		updater_gn_s1.updatePose(l_keypoints, current_pose);
		mapper.setProjectorRotation(current_pose);
#endif
		debug_timer.stop("Rough PoseUpdate");
		debug_timer.print("Rough PoseUpdate");


		// refine tracking
		debug_timer.start("Refine Tracking");
		double track_score;
		const double track_score_th = 0.4;
		std::vector<Keypoint> keypoints;

#ifdef CVAR_IOS
		track_score = track(4, keypoints);
#else
		track_score = track(6, keypoints);
#endif
		if ( track_score < track_score_th )
		{
			if ( ++non_relialbe_itr > 3 )
			{
				state = STATE_LOST;
			}
			return false;
		}
		debug_timer.stop("Refine Tracking");
		debug_timer.print("Refine Tracking");


#ifdef CVAR_USE_UPDATE_RANSAC
		debug_timer.start("RANSAC PoseUpdate");

		//updater_rs_s0.updatePose(keypoints, current_pose);
		updater_gn_s0.updatePose(keypoints, current_pose);
		mapper.setProjectorRotation(current_pose);

		debug_timer.stop("RANSAC PoseUpdate");
		debug_timer.print("RANSAC PoseUpdate");
#else

		debug_timer.start("GAUSS PoseUpdate");

		updater_gn_s0.updatePose(keypoints, current_pose);
		mapper.setProjectorRotation(current_pose);

		debug_timer.stop("GAUSS PoseUpdate");
		debug_timer.print("GAUSS PoseUpdate");
#endif

		rscore = track_score;
	}
	else if ( state == STATE_LOST )
	{
		// recover pose
		bool found = relocalize();
		if (!found) 
		{
			return false;
		}

		mapper.updateCellStatus();

		// re-tracking
		// track on low-resolution map
		double l_track_score;
		const double l_track_score_th = 0.4;
		std::vector<Keypoint> l_keypoints;

#ifdef CVAR_IOS
		l_track_score = trackRoughly(12, l_keypoints);
#elif defined CVAR_PC_MOCK
		l_track_score = trackRoughly(12, l_keypoints);
#else
		l_track_score = trackRoughly(16, l_keypoints);
#endif
		if ( l_track_score < l_track_score_th )
		{
			return false;
		}

		updater_gn_s1.updatePose(l_keypoints, current_pose);
		mapper.setProjectorRotation(current_pose);

		// refine tracking
		double track_score;
		const double track_score_th = 0.4;
		std::vector<Keypoint> keypoints;

		track_score = track(5, keypoints);
		if ( track_score < track_score_th )
		{
			return false;
		}

		updater_gn_s0.updatePose(keypoints, current_pose);
		mapper.setProjectorRotation(current_pose);

		rscore = 0.0;
	}

	// success tracking
	// continue tracking in next frame
	state = STATE_TRACKING;
	non_relialbe_itr = 0;
	
	mapper.updateCellStatus();

	// register keyframe
	// roll: up-down
	// pitch: right-left
	// yaw: 
	cv::Mat eul(1,3,CV_64F);
	cvmath::mat2euler(current_pose, eul);
	//std::cout << "euler: " << eul << std::endl;
		
	int roll_idx, pitch_idx, yaw_idx;
	if( relocalizer.checkKeyframe(eul, roll_idx, pitch_idx, yaw_idx) )
	{
		relocalizer.registerKeyframe(roll_idx, pitch_idx, yaw_idx, mapper.getFrameGray(0), eul);
		//relocalizer.showKeyframes();
	}
		
	//mapping
	if (rscore>0.6)
	{
		mapper.updateMap();
	}
	t1 = clock();
	//std::cout << "total: " << (double)(t1-t0)/CLOCKS_PER_SEC << std::endl;
		
	return true;
}

double MainProcess::trackRoughly(const int search_range, std::vector<Keypoint>& keypoints)
{
    clock_t t0,t1;
	int kp_count, drop_count;

#ifdef CVAR_IOS
	mapper.trackPointsRoughly(search_range, keypoints, drop_count, 30);
#elif defined CVAR_PC_MOCK
	mapper.trackPointsRoughly(search_range, keypoints, drop_count, 30);
#else
	mapper.trackPointsRoughly(search_range, keypoints, drop_count, 60);
#endif
	kp_count = static_cast<int>(keypoints.size());

//	std::cout << "rough keypoint size: " << kp_count
//	  << ", drop_count: " << drop_count
//	  << std::endl;

	// calc score
#ifdef CVAR_IOS
	if ( kp_count > 24) {
#elif defined CVAR_PC_MOCK
	if ( kp_count > 24) {
#else
	if ( kp_count > 48 ) {
#endif
		return static_cast<double>(kp_count)/(kp_count+drop_count);
	}

	return 0.0;
}


double MainProcess::track(const int search_range, std::vector<Keypoint>& keypoints)
{
    clock_t t0,t1;
	int kp_count, drop_count;

	t0 = clock();

#ifdef CVAR_IOS
	mapper.trackPoints(search_range, keypoints, drop_count, 50);
#elif defined CVAR_PC_MOCK
	mapper.trackPoints(search_range, keypoints, drop_count, 50);
#else
	mapper.trackPoints(search_range, keypoints, drop_count, 100);
#endif
	kp_count = static_cast<int>(keypoints.size());

	std::cout << "refine keypoint size: " << kp_count
		<< ", drop_count: " << drop_count
		<< std::endl;
	//t1 = clock();
	//std::cout << "refine track point: " << (double)(t1-t0)/CLOCKS_PER_SEC << std::endl;

	// calc score
#ifdef CVAR_IOS
	if ( kp_count > 40 ) {
#elif defined CVAR_PC_MOCK
	if ( kp_count > 40 ) {
#else
	if ( kp_count > 80 ) {
#endif
		return static_cast<double>(kp_count)/(kp_count+drop_count);
	}

	return 0.0;
}


bool MainProcess::relocalize()
{
    clock_t t0,t1;
	bool found;

	t0 = clock();
	cv::Mat eul;
	found = relocalizer.searchKeyframe(mapper.getFrameGray(0), eul);
	t1 = clock();
	//std::cout << "search keyframe: " << (double)(t1-t0)/CLOCKS_PER_SEC << std::endl;

	if (found) {
		cvmath::euler2mat(eul, current_pose);

		//apply pose
		mapper.setProjectorRotation(current_pose);
		
		return true;
	}

	return false;
}

bool MainProcess::findPlanar(const cv::Point2d& target_pt, cv::Mat& debug)
{
	PlanarFinder finder(&debug_timer);

	const cv::Mat& gray = mapper.getFrameGray(0);
	cv::Mat pose;
	getPoseRotation(pose);

	cv::Point2d dst[4];

	// Extract edges and make clusters
	finder.findEdgeClusters(gray);

	// Find Rect
	bool found = finder.find(gray, pose, target_pt, dst, debug);
	if (!found) {
		// Set default size rect
#ifdef CVAR_IOS
		const double dw = 100;
		const double dh = 60;
#else
		const double dw = 180;
		const double dh = 120;
#endif
		const double x0 = target_pt.x>dw? target_pt.x-dw:0,
			x1 = gray.cols>target_pt.x+dw? target_pt.x+dw:gray.cols,
			y0 = target_pt.y>dh? target_pt.y-dh:0,
			y1 = gray.rows>target_pt.y+dh? target_pt.y+dh:gray.rows;
		const Line l0( cv::Point2d(x0,y0), cv::Point2d(x0,y1) );
		const Line l1( cv::Point2d(x1,y0), cv::Point2d(x1,y1) );
		const PlanarRect target_rect(l0, l1);

		found = finder.updatePlanar(gray, pose, target_rect, dst, debug);
		if (found) 
		{
			found = true;
		}

	}

	if (found) {
		trackPointsProject(dst);
		is_trackpoints_mapped = true;
	}

	return found;
}

bool MainProcess::movePlanar(const double delta_x, const double delta_y, cv::Mat& debug)
{
	if (!hasTrackPoints()) return false;
	PlanarFinder finder(&debug_timer);

	PlanarRect planar;
	trackPointsBackProject(planar);

	planar.translate(delta_x, delta_y);

//	const cv::Mat& gray = mapper.getFrameGray(0);
//	cv::Mat pose;
//	getPoseRotation(pose);
//
//	cv::Point2d dst[4];
//
//	// Extract edges and make clusters
//	finder.findEdgeClusters(gray);
//
//	bool found = finder.updatePlanar(gray, pose, planar, dst, debug);
//	if (found)
//	{
//		trackPointsProject(dst);
//	}
//
//	return found;
	trackPointsProject(planar);
	return true;
}

bool MainProcess::updatePlanar(const double scale, cv::Mat& debug)
{
	if (!hasTrackPoints()) return false;

	PlanarFinder finder(&debug_timer);

	PlanarRect planar;
	trackPointsBackProject(planar);

	planar.scale(scale);

//	const cv::Mat& gray = mapper.getFrameGray(0);
//	cv::Mat pose;
//	getPoseRotation(pose);
//
//	cv::Point2d dst[4];
//
//	// Extract edges and make clusters
//	finder.findEdgeClusters(gray);
//
//	bool found = finder.updatePlanar(gray, pose, planar, dst, debug);
//	if (found)
//	{
//		trackPointsProject(dst);
//	}
//	return found;

	trackPointsProject(planar);
	return true;
}

bool MainProcess::adjustPlanar(cv::Mat& debug)
{
	if (!hasTrackPoints()) return false;

	PlanarFinder finder(&debug_timer);

	PlanarRect planar;
	trackPointsBackProject(planar);

	const cv::Mat& gray = mapper.getFrameGray(0);
	cv::Mat pose;
	getPoseRotation(pose);

	cv::Point2d dst[4];

	// Extract edges and make clusters
	finder.findEdgeClusters(gray);

	bool found = finder.adjustPlanar(gray, pose, planar, dst, debug);
	if (found)
	{
		trackPointsProject(dst);
	}

	return found;
}

void MainProcess::trackPointsBackProject(double* dst) //
{
	for (size_t i=0; i<4; ++i) {
		cv::Point2d _d;
		mapper.backProjectPointToImage(tracked_mappoints[i], _d);
		dst[i*2+0] = _d.x;
		dst[i*2+1] = _d.y;
	}
}

void MainProcess::trackPointsBackProject(cv::Point2d* dst)
{
	for (size_t i=0; i<4; ++i) {
		mapper.backProjectPointToImage(tracked_mappoints[i], dst[i]);
	}
}

void MainProcess::trackPointsBackProject(PlanarRect& planar)
{
	cv::Point2d d00,d01,d10,d11;
	mapper.backProjectPointToImage(tracked_mappoints[0], d00);
	mapper.backProjectPointToImage(tracked_mappoints[1], d01);
	mapper.backProjectPointToImage(tracked_mappoints[2], d10);
	mapper.backProjectPointToImage(tracked_mappoints[3], d11);
	planar = PlanarRect(d00, d01, d10, d11);
}

void MainProcess::trackPointsProject(const cv::Point2d* dst)
{
	for (size_t i=0; i<4; ++i) {
		mapper.projectPointToMap(dst[i], tracked_mappoints[i]);
	}
}

void MainProcess::trackPointsProject(const PlanarRect& planar)
{
	mapper.projectPointToMap(planar.vt_seg0.x1, tracked_mappoints[0]);
	mapper.projectPointToMap(planar.vt_seg0.x2, tracked_mappoints[1]);
	mapper.projectPointToMap(planar.vt_seg1.x1, tracked_mappoints[2]);
	mapper.projectPointToMap(planar.vt_seg1.x2, tracked_mappoints[3]);
}

void MainProcess::debugOutput(cv::Mat& output)
{
	const cv::Mat& gray = mapper.getFrameGray(1);
	mapper.drawDebugMap(output, 1, gray.size());
}

void MainProcess::debugOutput2(cv::Mat& output)
{
	cv::Point map_center = mapper.maps[0].center;
	cv::Point2d p0,p1,p2,p3;
	cv::Point2d s0(map_center.x-50,map_center.y-25);
	cv::Point2d s1(map_center.x-50,map_center.y+25);
	cv::Point2d s2(map_center.x+50,map_center.y+25);
	cv::Point2d s3(map_center.x+50,map_center.y-25);
	mapper.backProjectPointToImage(s0, p0);
	mapper.backProjectPointToImage(s1, p1);
	mapper.backProjectPointToImage(s2, p2);
	mapper.backProjectPointToImage(s3, p3);
	cv::circle(output, p0, 4, cv::Scalar(255,0,0), -1);
	cv::circle(output, p1, 4, cv::Scalar(255,0,0), -1);
	cv::circle(output, p2, 4, cv::Scalar(255,0,0), -1);
	cv::circle(output, p3, 4, cv::Scalar(255,0,0), -1);
}
