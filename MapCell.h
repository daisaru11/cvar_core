//
//  MapCell.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__MapCell__
#define __cvar_core__MapCell__

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>

#include "Keypoint.h"

class MapCell {
public:
	cv::Rect roi;
	int col;
	int row;

	std::vector<Keypoint> keypoints;

	MapCell();

protected:
private:
};

#endif /* defined(__cvar_core__MapCell__) */
