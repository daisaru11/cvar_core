//
//  Keypoint.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__Keypoint__
#define __cvar_core__Keypoint__

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>

class Keypoint {
public:
	enum TrackingStatus {
		STATUS_NONE = 0
	};
	int id;

	cv::Point2d img_point;
	cv::Point2d map_point;

	Keypoint();

protected:
private:
};

#endif /* defined(__cvar_core__Keypoint__) */
