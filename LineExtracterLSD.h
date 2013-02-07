//
//  LineExtracterLSD.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//
//  Extracting lines by LSD detector.
//  See "LSD: a Line Segment Detector" http://www.ipol.im/pub/art/2012/gjmr-lsd/

#ifndef __cvar_core__LineExtracterLSD__
#define __cvar_core__LineExtracterLSD__

#include <iostream>
#include <opencv2/core/core.hpp>

#include "Line.h"
#include "lsd.h"
#include "cvmath.h"

class LineExtracterLSD
{
public:
	void extractLine(const cv::Mat& gray, std::vector<Line>& lines, const double minlen=20.0);
};

#endif /* defined(__cvar_core__LineExtracterLSD__) */
