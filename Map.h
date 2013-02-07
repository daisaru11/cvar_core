//
//  Map.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__Map__
#define __cvar_core__Map__

#include <iostream>
#include <opencv2/core/core.hpp>
#include "MapCell.h"
#include "CylindricalProjector.h"

class Map {
    
public:
    cv::Mat img;
	cv::Mat gray;
    cv::Size size;
    cv::Point center;

	CylindricalProjector projector;

	cv::Size cell_size;
	int cell_rows;
	int cell_cols;
	std::vector<std::vector<MapCell> > cells;

    Map(const cv::Size map_size, const cv::Size cell_size, const int _cell_rows, const int _cell_cols);
protected:
private:

};

#endif /* defined(__cvar_core__Map__) */
