//
//  Line.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__Line__
#define __cvar_core__Line__

class Line
{
public:
	Line() {}

	Line(const double p1x, const double p1y, const double p2x, const double p2y)
	:x1(p1x, p1y), x2(p2x, p2y),
	mean( (p1x+p2x)/2, (p1y+p2y)/2 ),
	l( p1y-p2y, p2x-p1x, p1x*p2y-p2x*p1y ) {}

	Line(const cv::Point2d& p1, const cv::Point2d& p2)
	:x1(p1), x2(p2),
	mean( (p1.x+p2.x)/2, (p1.y+p2.y)/2 ),
	l( p1.y-p2.y, p2.x-p1.x, p1.x*p2.y-p2.x*p1.y ) {}

	cv::Point2d x1;     /* end point 1 */
	cv::Point2d x2;     /* end point 2 */
	cv::Point2d mean;   /* middle point */
	cv::Vec3d l;      /* line representation in homogeneous coordinate */
	//double theta;     /* line angle */
	//double r;         /* line lenght */
};

#endif
