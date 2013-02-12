//
//  PlanarFinder.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//
//
#include "PlanarFinder.h"

PlanarFinder::PlanarFinder(TimerManager* debug_timer)
:debug_timer(debug_timer)
{
	//do_nothing
}

bool PlanarFinder::updatePlanar(const cv::Mat& img, const cv::Mat& pose, 
		const PlanarRect& target_rect, cv::Point2d* dst_rect, cv::Mat& debug)
{
	// Vertical VP
	const int vert_idx = estimateVertical(vps, pose);
	if (vert_idx < 0) {
		return false;
	}

#ifdef CVAR_PC_DEBUG
	target_rect.draw(debug, cv::Scalar(120,0,120));
	
	cv::Scalar colors[3] = {
		cv::Scalar(0, 0, 255),
		cv::Scalar(0, 255, 0),
		cv::Scalar(255, 0, 0)
	};
	for (size_t h=0; h<vps.size(); ++h)
	{
		//if (vert_idx == h) continue;
		std::vector<int>& cluster_h = clusters[h];
		for (int c=0; c<edges.size(); ++c) {
			if (!cluster_h[c]) continue;
			Line& line = edges[c];
			cv::line(debug, line.x1, line.x2, colors[h%3], 1);
		}
	}
#endif
	
	const int hrz_idx = calcHorizontalDominantVp(vert_idx, target_rect);
	if (hrz_idx<0) {
		return false;
	}

	const cv::Point3d& vvp = vps[vert_idx];
	const cv::Point3d& hvp = vps[hrz_idx];
	const cv::Point2d& mv0 = target_rect.vt_seg0.mean;
	const cv::Point2d& mv1 = target_rect.vt_seg1.mean;
	const cv::Point2d mh0(
		(target_rect.vt_seg0.x1.x+target_rect.vt_seg1.x1.x)/2,
		(target_rect.vt_seg0.x1.y+target_rect.vt_seg1.x1.y)/2 );
	const cv::Point2d mh1(
		(target_rect.vt_seg0.x2.x+target_rect.vt_seg1.x2.x)/2,
		(target_rect.vt_seg0.x2.y+target_rect.vt_seg1.x2.y)/2 );

	cv::Vec3d vl0( mv0.y*vvp.z - vvp.y, vvp.x - mv0.x*vvp.z, mv0.x*vvp.y - mv0.y*vvp.x );
	cv::Vec3d vl1( mv1.y*vvp.z - vvp.y, vvp.x - mv1.x*vvp.z, mv1.x*vvp.y - mv1.y*vvp.x );
	cv::Vec3d hl0( mh0.y*hvp.z - hvp.y, hvp.x - mh0.x*hvp.z, mh0.x*hvp.y - mh0.y*hvp.x );
	cv::Vec3d hl1( mh1.y*hvp.z - hvp.y, hvp.x - mh1.x*hvp.z, mh1.x*hvp.y - mh1.y*hvp.x );

	cv::Point2d p00 = findIntersection2d(vl0, hl0);
	cv::Point2d p01 = findIntersection2d(vl0, hl1);
	cv::Point2d p10 = findIntersection2d(vl1, hl0);
	cv::Point2d p11 = findIntersection2d(vl1, hl1);

	dst_rect[0] = p00; dst_rect[1] = p01;
	dst_rect[2] = p10; dst_rect[3] = p11;

#ifdef CVAR_PC_DEBUG
	Line new_vl0(p00, p01);
	Line new_vl1(p10, p11);

	PlanarRect dst_(new_vl0, new_vl1);
	dst_.draw(debug, cv::Scalar(120,120,0));
	cv::imshow("Lines", debug);
#endif

	return true;
}

bool PlanarFinder::adjustPlanar(const cv::Mat& img, const cv::Mat& pose, 
		const PlanarRect& target_rect, cv::Point2d* dst_rect, cv::Mat& debug)
{
	// Vertical VP
	int vert_idx = estimateVertical(vps, pose);
	if (vert_idx < 0) {
		return false;
	}

	std::vector<std::vector<int> > idx_hist;
	calcVpHistogram(target_rect, idx_hist);


	// find dominant horizontal vp
	int hist_max = 0;
	int hrz_idx = -1;
	for (size_t h=0; h<vps.size(); ++h)
	{
		if (vert_idx == h) continue;
		if (idx_hist[h].size()>hist_max) {
			hrz_idx = h;
			hist_max = idx_hist[h].size();
		}
	}

	if (hrz_idx < 0) {
		return false;
	}

	std::vector<int>& v_edge_idx = idx_hist[vert_idx];
	std::vector<int>& h_edge_idx = idx_hist[hrz_idx];

	if (v_edge_idx.size() <=2 || h_edge_idx.size() <= 2) {
		return false;
	}

	// Build Keypoint IndexTree
	buildKpIndex(img);

	// adjust edge
	debug_timer->start("Adjust Edge");
	const cv::Point2d& t00 = target_rect.vt_seg0.x1;
 	const cv::Point2d& t01 = target_rect.vt_seg0.x2;
	const cv::Point2d& t10 = target_rect.vt_seg1.x1;
	const cv::Point2d& t11 = target_rect.vt_seg1.x2;

	const std::vector<int>& cluster_v = clusters[vert_idx];
	const std::vector<int>& cluster_h = clusters[hrz_idx];
	const int line_count = edges.size();

	const float kp_neighbor_r = 4.0;
	const double len_min_th = 20;

	bool found = false;

	for (int ci0=0; ci0<v_edge_idx.size(); ++ci0) { // Vertical Edge
		const Line& li0 = edges[ v_edge_idx[ci0] ];

		for (int ci1=ci0+1; ci1<v_edge_idx.size(); ++ci1) {
			const Line& li1 = edges[ v_edge_idx[ci1] ];

			for (int cj0=0; cj0<h_edge_idx.size(); ++cj0) { // Horizontal Edge
				const Line& lj0 = edges[ h_edge_idx[cj0] ];

				// get intersection points
				cv::Point2d pt00 = findIntersection2d(li0, lj0);
				if (pt00.x < 0 || img.cols < pt00.x || pt00.y < 0 || img.rows < pt00.y) continue;
				if (!existKpNeighbor(pt00, kp_neighbor_r)) continue;
				//if (!testInternalPoint(t00, t01, t11, t10, pt00) ) continue;


				cv::Point2d pt10 = findIntersection2d(li1, lj0);
				if (pt10.x < 0 || img.cols < pt10.x || pt10.y < 0 || img.rows < pt10.y) continue;
				if (!existKpNeighbor(pt10, kp_neighbor_r)) continue;
				//if (!testInternalPoint(t00, t01, t11, t10, pt10) ) continue;

				const double dh = cv::norm(pt00-pt10);
				if (dh<len_min_th) continue;

				for (int cj1=cj0+1; cj1<h_edge_idx.size(); ++cj1) {
					const Line& lj1 = edges[ h_edge_idx[cj1] ];

					// get intersection points
					cv::Point2d pt01 = findIntersection2d(li0, lj1);
					if (pt01.x < 0 || img.cols < pt01.x || pt01.y < 0 || img.rows < pt01.y) continue;
					if (!existKpNeighbor(pt01, kp_neighbor_r)) continue;
					//if (!testInternalPoint(t00, t01, t11, t10, pt01) ) continue;

					cv::Point2d pt11 = findIntersection2d(li1, lj1);
					if (pt11.x < 0 || img.cols < pt11.x || pt11.y < 0 || img.rows < pt11.y) continue;
					if (!existKpNeighbor(pt11, kp_neighbor_r)) continue;
					//if (!testInternalPoint(t00, t01, t11, t10, pt11) ) continue;

					// ratio of vertical and horizontal sides
					const double dv = cv::norm(pt00-pt01);
					if (dv<len_min_th) continue;

					//double dth = 0.3;
					//if ( (dv<dh && dv/dh < dth) || (dh<dv && dh/dv < dth) ) continue;
					// len max
					//if (dv>len_max_th || dh>len_max_th) continue;
			
					setAndAlign(dst_rect, pt00, pt01, pt10, pt11);
					//dst_rect[0] = pt00; dst_rect[1] = pt01;
					//dst_rect[2] = pt10; dst_rect[3] = pt11;
					found = true;
					goto endloop;

				}
			}
		}

	}
endloop:
	debug_timer->stop("Adjust Edge");
	debug_timer->print("Adjust Edge");

	return found;
}

int PlanarFinder::calcHorizontalDominantVp(const int vert_idx, const PlanarRect& target_rect)
{
	int *hist = new int(vps.size());
    memset(hist, 0, sizeof(int)*vps.size());

	const size_t line_count = edges.size();
	for (int c=0; c<line_count; ++c)
	{
		Line& l = edges[c];
		for (size_t h=0; h<vps.size(); ++h)
		{
			if (vert_idx == h) continue;
			if (!clusters[h][c]) continue;

			if ( testInternalPoint(
					target_rect.vt_seg0.x1, target_rect.vt_seg0.x2,
					target_rect.vt_seg1.x2, target_rect.vt_seg1.x1,
					l.mean) )
			{
				hist[h]++;
			}
		}
	}

	int hist_max = 0;
	int hrz_idx = -1;
	for (size_t h=0; h<vps.size(); ++h)
	{
		if (hist[h]>hist_max) {
			hrz_idx = h;
			hist_max = hist[h];
		}
	}
	delete [] hist;

	return hrz_idx;
}

void PlanarFinder::calcVpHistogram(const PlanarRect& target_rect, std::vector<std::vector<int> >& idx_hist)
{
	idx_hist.resize(vps.size());

	const size_t line_count = edges.size();
	for (int c=0; c<line_count; ++c)
	{
		Line& l = edges[c];
		for (size_t h=0; h<vps.size(); ++h)
		{
			if (!clusters[h][c]) continue;

			if ( testInternalPoint(
					target_rect.vt_seg0.x1, target_rect.vt_seg0.x2,
					target_rect.vt_seg1.x2, target_rect.vt_seg1.x1,
					l.mean) )
			{
				idx_hist[h].push_back(c);
			}
		}
	}
}


void PlanarFinder::findEdgeClusters(const cv::Mat& img)
{
	// Extract lines by LSD
	LineExtracterLSD extracter;

	debug_timer->start("Extract Line");
	edges.clear();
	extracter.extractLine(img, edges);
	debug_timer->stop("Extract Line");
	debug_timer->print("Extract Line");

	//Tardif's algorithm
	JLinkage jlinkage;

	debug_timer->start("JLinkage");
	clusters.clear();
	vps.clear();
	jlinkage.findVanPoints(edges, clusters, vps);
	debug_timer->stop("JLinkage");
	debug_timer->print("JLinkage");
}

bool PlanarFinder::find(const cv::Mat& img, const cv::Mat& pose, const cv::Point2d& target_pt, cv::Point2d* dst, cv::Mat& debug)
{

	// Vertical VP
	int vert_idx = estimateVertical(vps, pose);
	if (vert_idx < 0)
	{
		return false;
	}

	// Build Keypoint IndexTree
	buildKpIndex(img);

#ifdef CVAR_PC_DEBUG
	cv::Scalar colors[3] = {
		cv::Scalar(0, 0, 255),
		cv::Scalar(0, 255, 0),
		cv::Scalar(255, 0, 0)
	};
	std::vector<cv::KeyPoint> kps;
	cv::FastFeatureDetector kp_detector(10);
	kp_detector.detect(img, kps);
	for (size_t k=0; k<kps.size(); ++k)
	{
		cv::circle(debug, kps[k].pt, 2, cv::Scalar(100,0,0));
	}


	for (size_t h=0; h<vps.size(); ++h)
	{
		//if (vert_idx == h) continue;
		std::vector<int>& cluster_h = clusters[h];
		for (int c=0; c<edges.size(); ++c) {
			if (!cluster_h[c]) continue;
			Line& line = edges[c];
			cv::line(debug, line.x1, line.x2, colors[h%3], 1);
		}
	}
#endif

	bool found = selectAutoRect(img, vert_idx, target_pt, dst, debug);
	
	return found;
}

bool PlanarFinder::selectAutoRect(const cv::Mat& img, 
	const int vert_idx,
	const cv::Point2d& target_pt, cv::Point2d* dst, cv::Mat& debug)
{
	// Rectangle Selection
	const size_t line_count = edges.size();
#ifdef CVAR_IOS
	const double ldist_min_th = 10;
	const double ldist_max_th = 200;
	const double h_range_l = target_pt.x - 40;
	const double h_range_r = target_pt.x + 40;
	const float kp_neighbor_r = 4.0;
	const double len_max_th = 160;
#else
	const double ldist_min_th = 10;
	const double ldist_max_th = 400;
	const double h_range_l = target_pt.x - 60;
	const double h_range_r = target_pt.x + 60;
	const float kp_neighbor_r = 4.0;
	const double len_max_th = 320;
#endif

#ifdef CVAR_PC_DEBUG
	cv::Scalar colors2[3] = {
		cv::Scalar(0, 255, 255),
		cv::Scalar(255, 255, 0),
		cv::Scalar(255, 0, 255)
	};
#endif

	bool found = false;

	debug_timer->start("Select Rectangle");
	for (size_t h=0; h<vps.size(); ++h)
	{
		if (vert_idx == h) continue;

		const std::vector<int>& cluster_v = clusters[vert_idx];
		const std::vector<int>& cluster_h = clusters[h];

		for (int ci0=0; ci0<line_count; ++ci0) {
			if (!cluster_v[ci0]) continue;

			const Line& li0 = edges[ci0];

			for (int ci1=ci0+1; ci1<line_count; ++ci1) {
				if (!cluster_v[ci1]) continue;

				const Line& li1 = edges[ci1];

				// line distance
				double dist_v = lineDistance(li0, li1.mean);
				if ( dist_v < ldist_min_th || dist_v > ldist_max_th ) continue;

				for (int cj0=0; cj0<line_count; ++cj0) {
					if (!cluster_h[cj0]) continue;

					const Line& lj0 = edges[cj0];
					if (lj0.mean.x < h_range_l || h_range_r < lj0.mean.x) continue;

					// get intersection points
					cv::Point2d pt00 = findIntersection2d(li0, lj0);
					if (pt00.x < 0 || img.cols < pt00.x || pt00.y < 0 || img.rows < pt00.y) continue;
					//if (!existKpNeighbor(pt00, kp_neighbor_r)) continue;

					cv::Point2d pt10 = findIntersection2d(li1, lj0);
					if (pt10.x < 0 || img.cols < pt10.x || pt10.y < 0 || img.rows < pt10.y) continue;
					//if (!existKpNeighbor(pt10, kp_neighbor_r)) continue;

					// len max
					const double dh = cv::norm(pt00-pt10);
					if (dh>len_max_th) continue;

					for (int cj1=cj0+1; cj1<line_count; ++cj1) {
						if (!cluster_h[cj1]) continue;

						const Line& lj1 = edges[cj1];
						if (lj1.mean.x < h_range_l || h_range_r < lj1.mean.x) continue;

						// line distance
						double dist_h = lineDistance(lj0, lj1.mean);
						if ( dist_h < ldist_min_th || dist_h > ldist_max_th ) continue;

						// get intersection points
						cv::Point2d pt01 = findIntersection2d(li0, lj1);
						if (pt01.x < 0 || img.cols < pt01.x || pt01.y < 0 || img.rows < pt01.y) continue;
						//if (!existKpNeighbor(pt01, kp_neighbor_r)) continue;

						cv::Point2d pt11 = findIntersection2d(li1, lj1);
						if (pt11.x < 0 || img.cols < pt11.x || pt11.y < 0 || img.rows < pt11.y) continue;
						//if (!existKpNeighbor(pt11, kp_neighbor_r)) continue;


						// actual lines is on the rectangle contour
						if ( !isMidPoint(pt00, pt01, li0.mean)
							|| !isMidPoint(pt01, pt11, lj1.mean)
							|| !isMidPoint(pt11, pt10, li1.mean)
							|| !isMidPoint(pt10, pt01, lj0.mean) ) continue;

						// ratio of vertical and horizontal sides
						const double dv = cv::norm(pt00-pt01);
						double dth = 0.4;
						if ( (dv<dh && dv/dh < dth) || (dh<dv && dh/dv < dth) ) continue;
						// len max
						if (dv>len_max_th || dh>len_max_th) continue;


						if ( testInternalPoint(pt00, pt01, pt11, pt10, target_pt) )
						{
#ifdef CVAR_PC_DEBUG
							cv::line(debug, pt00, pt01, colors2[h%3], 1);
							cv::line(debug, pt01, pt11, colors2[h%3], 1);
							cv::line(debug, pt11, pt10, colors2[h%3], 1);
							cv::line(debug, pt10, pt00, colors2[h%3], 1);
#endif
							setAndAlign(dst, pt00, pt01, pt10, pt11);
							//dst[0] = pt00; dst[1] = pt01;
							//dst[2] = pt10; dst[3] = pt11;
							found = true;
							goto endloop;
						}
					}
				}
			}
		}

	}
endloop:
	debug_timer->stop("Select Rectangle");
	debug_timer->print("Select Rectangle");
	
#ifdef CVAR_PC_DEBUG
	cv::imshow("lines", debug);
	cv::waitKey();
#endif

	return found;
}

int PlanarFinder::estimateVertical(const std::vector<cv::Point3d>& vps, const cv::Mat& pose)
{

	cv::Point2d tmp_img_center(240, 180);
	double th = 0.1;
	double min = cvmath::PI;
	int min_idx = -1;

	for (size_t k=0; k<vps.size(); ++k)
	{
		const cv::Point3d& vp = vps[k];

		double x =vp.x/vp.z;
		double y =vp.y/vp.z;
		double a = fabs( atan2(
			tmp_img_center.y - (vp.y/vp.z),
			tmp_img_center.x - (vp.x/vp.z) ));
		a = fabs(a - cvmath::PI/2);

		if (a<min && a < th)
		{
			min = a;
			min_idx = static_cast<int>(k);
		}
	}

	return min_idx;
}

void PlanarFinder::buildKpIndex(const cv::Mat& img)
{
	std::vector<cv::KeyPoint> kps;
	cv::FastFeatureDetector kp_detector(20);
	kp_detector.detect(img, kps);

	cv::Mat features(static_cast<int>(kps.size()), 2, CV_32F);
	float *fptr = (float *)features.data;
	for (std::vector<cv::KeyPoint>::iterator iter=kps.begin(); iter!=kps.end(); ++iter)
	{
		fptr[0] = iter->pt.x;
		fptr[1] = iter->pt.y;
		fptr += 2;
	}

	kp_index.build(features, cv::flann::KDTreeIndexParams(4));
}

bool PlanarFinder::existKpNeighbor(const cv::Point2d point, const float radius)
{
	std::vector<float> query(2);
	query[0] = static_cast<float>(point.x);
	query[1] = static_cast<float>(point.y);

	std::vector<int> results;
	std::vector<float> dists;

	int c = kp_index.radiusSearch(query, results, dists, radius, 1);

	return c > 0;
}
