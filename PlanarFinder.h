//
//  PlanarFinder.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__PlanarFinder__
#define __cvar_core__PlanarFinder__

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "cvmath.h"
#include "JLinkage.h"
//#include "LineExtracter.h"
#include "LineExtracterLSD.h"
#include "TimerManager.h"

class PlanarRect
{
public:
	PlanarRect() {};
	PlanarRect(const Line& vt_seg0, const Line& vt_seg1)
	:vt_seg0(vt_seg0), vt_seg1(vt_seg1) {}
	PlanarRect(const cv::Point2d& p00, const cv::Point2d& p01, const cv::Point2d& p10, const cv::Point2d& p11)
	:vt_seg0(p00, p01), vt_seg1(p10, p11) {}
	Line vt_seg0;
	Line vt_seg1;

	void draw(cv::Mat& output, const cv::Scalar& color) const
	{
		cv::line(output, vt_seg0.x1, vt_seg0.x2, color, 1);
		cv::line(output, vt_seg0.x2, vt_seg1.x2, color, 1);
		cv::line(output, vt_seg1.x2, vt_seg1.x1, color, 1);
		cv::line(output, vt_seg1.x1, vt_seg0.x1, color, 1);
	}
	void scale(const double factor)
	{
		cv::Point2d& p00 = vt_seg0.x1;
		cv::Point2d& p01 = vt_seg0.x2;
		cv::Point2d& p10 = vt_seg1.x1;
		cv::Point2d& p11 = vt_seg1.x2;

		// diagonal line
		cv::Vec3d c0( p00.y - p11.y, p11.x - p00.x, p00.x*p11.y - p00.y*p11.x );
		cv::Vec3d c1( p01.y - p10.y, p10.x - p01.x, p01.x*p10.y - p01.y*p10.x );
		
		// center point of the rect
		cv::Point2d pc = findIntersection2d(c0, c1);
		
		p00 = pc + factor*(p00-pc);
		p01 = pc + factor*(p01-pc);
		p10 = pc + factor*(p10-pc);
		p11 = pc + factor*(p11-pc);

		vt_seg0 = Line(p00, p01);
		vt_seg1 = Line(p10, p11);
	}
};

class PlanarFinder
{
public:
	PlanarFinder(TimerManager* debug_timer);
	void findEdgeClusters(const cv::Mat& img);
	bool find(const cv::Mat& img, const cv::Mat& pose, const cv::Point2d& target_pt, cv::Point2d* dst, cv::Mat& debug);
	bool updatePlanar(const cv::Mat& img, const cv::Mat& pose,
			const PlanarRect& target_rect, cv::Point2d* dst_rect, cv::Mat& debug);
	bool adjustPlanar(const cv::Mat& img, const cv::Mat& pose,
			const PlanarRect& target_rect, cv::Point2d* dst_rect, cv::Mat& debug);
private:

	int estimateVertical(const std::vector<cv::Point3d>& vps, const cv::Mat& pose);
	void buildKpIndex(const cv::Mat& img);
	bool existKpNeighbor(const cv::Point2d point, const float radius);
	bool selectAutoRect(const cv::Mat& img, 
			const int vert_idx,
			const cv::Point2d& target_pt, cv::Point2d* dst, cv::Mat& debug);
	int calcHorizontalDominantVp(const int vert_idx, const PlanarRect& target_rect);
	void calcVpHistogram(const PlanarRect& target_rect, std::vector<std::vector<int> >& idx_hist);

	std::vector<Line> edges;
	std::vector<cv::Point3d> vps;
	std::vector<std::vector<int> > clusters;

	cv::flann::Index kp_index;

	// debug timer
	TimerManager* debug_timer;
};

#endif /* defined(__cvar_core__PlanarFinder__) */
