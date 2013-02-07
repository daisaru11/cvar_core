//
//  Mapper.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__Mapper__
#define __cvar_core__Mapper__

#include "cvarconf.h"

#include <iostream>
#include <vector>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "KeypointMatcher.h"
#include "CylindricalProjector.h"
#include "Map.h"
#include "MapCell.h"
#include "MapCellStatus.h"


class Mapper {
    
public:
    std::vector<Map> maps;
    
    Mapper(const cv::Size nrm_map_size, const cv::Size img_size, 
		const std::vector<double> scales,
		const std::vector<size_t> max_kp_per_cell);
	~Mapper();
	void updateCellStatus();
    void updateMap();
	void trackPoints(const int search_range, std::vector<Keypoint>& keypoints, int& drop_count, const int using_kp_max=-1);
	void trackPointsRoughly(const int search_range, std::vector<Keypoint>& keypoints, int& drop_count, const int using_kp_max=-1);
	void backProjectPointToImage(const cv::Point2d& src, cv::Point2d& dst);
	void projectPointToMap(const cv::Point2d& src, cv::Point2d& dst, const size_t scale_idx=0);
	void drawDebugMap(cv::Mat& output, const size_t scale_idx, const cv::Size& frame_size);

	void setProjectorParams(const cv::Mat& K, double pscale, const cv::Mat& R)
	{
		// norm projector
		maps[0].projector.setCameraParams(K, R);
		maps[0].projector.scale = pscale;

		// down-scaled projector
		for (size_t i=1; i<maps.size(); ++i)
		{
			double scale = scales[i];
			cv::Mat _K = cv::Mat::eye(3,3,CV_64F);
			_K.at<double>(0,0) = K.at<double>(0,0)*scale;
			_K.at<double>(1,1) = K.at<double>(1,1)*scale;
			_K.at<double>(0,2) = K.at<double>(0,2)*scale;
			_K.at<double>(1,2) = K.at<double>(1,2)*scale;
			maps[i].projector.setCameraParams(_K, R);
			maps[i].projector.scale = pscale * scale;
		}
	}

	void setProjectorRotation(const cv::Mat& R)
	{
		for (size_t i=0; i<maps.size(); ++i)
		{
			maps[i].projector.setRotation(R);
		}
	}

	double getScale(size_t scale_idx)
	{
		return scales[scale_idx];
	}

	void setFrameImage(const size_t scale_idx, const cv::Mat& img)
	{
		img_store[scale_idx] = img;
	}
	cv::Mat& getFrameImage(const size_t scale_idx)
	{
		return img_store[scale_idx];
	}

	void setFrameGray(size_t scale_idx, const cv::Mat& gray)
	{
		gray_store[scale_idx] = gray;
	}
	cv::Mat& getFrameGray(const size_t scale_idx)
	{
		return gray_store[scale_idx];
	}

protected:
private:
	
    cv::Size nrm_map_size;
	cv::Size nrm_cell_size;
	std::vector<double> scales;

    cv::Size img_size;
    cv::Point img_center;

    int cell_rows;
    int cell_cols;
	std::vector<std::vector<MapCellStatus> > cells_st;

	cv::FastFeatureDetector kp_detector;
	int kp_id_cnt;
	std::vector<size_t> max_kp_per_cell;

	std::vector<cv::Mat> img_store;  //pointer should be used?
	std::vector<cv::Mat> gray_store;

	bool detectResultCell();
	bool detectInnerCell(const int center_cell_i, const int center_cell_j);
	void fillInnerRec(const int i, const int j);
	void resetCellStatus();
	void mapCellRoi(const cv::Mat& img, const int i, const int j);
	void copyCellRoi(const cv::Mat& src, cv::Mat& dst_roi);
	void detectKeypointsInCell(const int i, const int j); // with all scaled maps
	void detectKeypointsInCell(Map& map, size_t max_kp_count, const int i, const int j); 
	void matchPointsOnCells(const cv::Mat& img, Map& map, const KeypointMatcher& matcher, std::vector<Keypoint>& keypoints, int& drop_count, const int using_kp_max=-1);
	bool matchKeypoint(const cv::Mat& img, Map& map, const KeypointMatcher& matcher, Keypoint& kp);
	bool matchWarpedPatch(const cv::Mat& img, Map& map, const KeypointMatcher& matcher, Keypoint& kp);
	inline int getNewKeypointId()
	{
		return ++kp_id_cnt;
	}
    
};


#endif /* defined(__cvar_core__Mapper__) */
