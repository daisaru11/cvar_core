//
//  Header.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__cvutil__
#define __cvar_core__cvutil__

#include <opencv2/core/core.hpp>
#include <math.h>

inline cv::Point3d findIntersection(const Line& line1, const Line& line2)
{
	return cv::Point3d(
		line1.l[1] * line2.l[2] - line1.l[2] * line2.l[1],
		line1.l[2] * line2.l[0] - line1.l[0] * line2.l[2],
		line1.l[0] * line2.l[1] - line1.l[1] * line2.l[0] );
}

inline cv::Point2d findIntersection2d(const Line& line1, const Line& line2)
{
	const double z = line1.l[0] * line2.l[1] - line1.l[1] * line2.l[0];
	return cv::Point2d(
		(line1.l[1] * line2.l[2] - line1.l[2] * line2.l[1]) / z,
		(line1.l[2] * line2.l[0] - line1.l[0] * line2.l[2]) / z );
}

inline cv::Point2d findIntersection2d(const cv::Vec3d& l1, const cv::Vec3d& l2)
{
	const double z = l1[0] * l2[1] - l1[1] * l2[0];
	return cv::Point2d(
		(l1[1] * l2[2] - l1[2] * l2[1]) / z,
		(l1[2] * l2[0] - l1[0] * l2[2]) / z );
}

inline double findOrthDistance(const Line& line, const cv::Point3d point)
{
	double lhat[3];
	lhat[0] = line.mean.y * point.z - point.y;
	lhat[1] = point.x - line.mean.x * point.z;
	lhat[2] = line.mean.x * point.y - line.mean.y * point.x;

	return fabs(lhat[0]*line.x1.x + lhat[1]*line.x1.y + lhat[2]) / sqrt(lhat[0]*lhat[0] + lhat[1]*lhat[1]);
}

inline double lineDistance(const Line& line, const cv::Point2d& point)
{
	return fabs(line.l[0]*point.x + line.l[1]*point.y + line.l[2]) / sqrt(line.l[0]*line.l[0] + line.l[1]*line.l[1]);
}

/*
inline double jaccardDist(std::vector<int>& A, std::vector<int>& B)
{
	int n1 = 0;
	int n2 = 0;

	for(size_t i = 0, len=A.size(); i < len; i++)
	{
		if(A[i] == 1 || B[i] == 1)
			n1++;
		if(A[i] == 1 && B[i] == 1)
			n2++;
	}

	double dis = static_cast<double>(n1 - n2) / n1;
	return dis;
}
*/

inline double cross_z(const cv::Point2d& ps, const cv::Point2d& pa, const cv::Point2d& pb)
{
	return (pa.x-ps.x)*(pb.y-ps.y) - (pa.y-ps.y)*(pb.x-ps.x);
}

inline void alignRectPoint(cv::Point2d& pa1, cv::Point2d& pa2, cv::Point2d& pb1, cv::Point2d& pb2, const bool flg)
{
	bool cross = (pa1-pa2).dot(pb1-pa2) < (pa1-pa2).dot(pb2-pa2);
	if ( (flg && !cross ) || (!flg && cross) )
	{
		cv::Point2d tmp = pb1;
		pb1 = pb2;
		pb2 = tmp;
		return;
	}
}

inline bool testInternalPoint(const cv::Point2d& p0, const cv::Point2d& p1, const cv::Point2d& p2, const cv::Point2d& p3,
					   const cv::Point2d& t0)
{
	const double d0 = cross_z(p0, p1, t0);
	const double d1 = cross_z(p1, p2, t0);
	const double d2 = cross_z(p2, p3, t0);
	const double d3 = cross_z(p3, p0, t0);
	
	return ( d0>0 && d1>0 && d2>0 && d3>0 ) || ( d0<0 && d1<0 && d2<0 && d3<0 );
}

inline bool isMidPoint(const cv::Point2d& p0, const cv::Point2d& p1, const cv::Point2d& t)
{
	return ((p0.x < t.x && t.x < p1.x) || (p1.x < t.x && t.x < p0.x))
		&& ((p0.y < t.y && t.y < p1.y) || (p1.y < t.y && t.y < p0.y)) ;
}


#endif
