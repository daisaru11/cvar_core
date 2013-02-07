//
//  LineExtracterLSD.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//
//  Extracting lines by LSD detector.
//  See "LSD: a Line Segment Detector" http://www.ipol.im/pub/art/2012/gjmr-lsd/

#include "LineExtracterLSD.h"

void LineExtracterLSD::extractLine(const cv::Mat& gray, std::vector<Line>& lines, const double minlen)
{
	const int cols = gray.cols;
	const int rows = gray.rows;
    double *dat = new double[rows*cols];
	double *dp = dat;
    for(int y = 0; y < rows; y++)
	{
		const unsigned char* sp = gray.ptr<unsigned char>(y);
		for(int x = 0; x < cols; x++) {
			(*dp) = sp[x];
			++dp;
		}
	}

	double* lsd_lines;
	int n_lines;
	lsd_lines = lsd_scale(&n_lines, dat, cols, rows, 1.0);

	for(int i=0; i<n_lines; i++)
	{
		const double *l = &lsd_lines[i * 7];
		if(10 < l[6]) // NFA
		{
			const double len = sqrt(cvmath::sqr(l[0]-l[2])+cvmath::sqr(l[1]-l[3]));
			if (len>minlen) {
				Line line(l[0], l[1], l[2], l[3]);
				/*
				line.x1[0] = l[0]; //x
				line.x1[1] = l[1]; //y
				line.x2[0] = l[2];
				line.x2[1] = l[3];
				line.mean[0] = (l[0]+l[2])/2;
				line.mean[1] = (l[1]+l[3])/2;
				//line.theta = theta; // not needed in process following?
				//line.r = r;
				line.l[0] = l[1] - l[3];
				line.l[1] = l[2] - l[0];
				line.l[2] = l[0]*l[3] - l[2]*l[1];
				*/
				lines.push_back(line);
			}
        }
    }
	delete [] dat;
}
