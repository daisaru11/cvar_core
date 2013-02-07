//
//  Map.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include "Map.h"

Map::Map(const cv::Size _map_size, const cv::Size _cell_size, const int _cell_rows, const int _cell_cols)
:img(_map_size,CV_8UC3), 
gray(_map_size,CV_8UC1), 
size(_map_size),
center(_map_size.width/2,_map_size.height/2),
cell_size(_cell_size),
cell_rows(_cell_rows),
cell_cols(_cell_cols),
cells(cell_rows, std::vector<MapCell>(cell_cols))
{
    for (int j=0; j<cell_rows; ++j) {
        for (int i=0; i<cell_cols; ++i) {
			MapCell& cell = cells[j][i];
            cv::Rect& roi = cell.roi;
			cell.col = i;
			cell.row = j;
            roi.x = i*cell_size.width;
            roi.y = j*cell_size.height;
            roi.width = cell_size.width;
            roi.height = cell_size.height;
        }
    }
}
