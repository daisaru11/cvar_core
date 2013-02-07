//
//  JLinkage.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//
//  This is implementation vanishing lines clustering by J-Linkage algorithm.
//  See "J.-P. Tardif. Non-iterative approach for fast and accurate vanishing point detection."
//  And, we refered to the following link for the implementation.
//  http://srinathsridhar.com/research/eecs442

#ifndef __cvar_core__JLinkage__
#define __cvar_core__JLinkage__

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Line.h"
#include "cvutil.h"
#include <bitset>

typedef struct
{
	//Line line1;
	//Line line2;
	size_t line1_idx;
	size_t line2_idx;
	cv::Point3d intersection_pt;
} Model;

#define JLINKAGE_MODEL_SIZE 500
typedef std::bitset<JLINKAGE_MODEL_SIZE> PSMatColBits ;

inline double jaccardDist(const PSMatColBits& A, const PSMatColBits& B)
{
	int n1 = (A|B).count();
	int n2 = (A&B).count();
	return static_cast<double>(n1 - n2) / n1;
}

class JLinkage
{

public:
	JLinkage();
	~JLinkage();
	void findVanPoints(const std::vector<Line>& edges, 
		std::vector<std::vector<int> >& clusters, std::vector<cv::Point3d>& vps);

private:

	size_t line_count;

	std::vector<Model> models;
	std::vector<PSMatColBits> PSMatrix; //TODO change to bitset


	void selectMinimalSets(const std::vector<Line>& edges);
	void makePSMatrix(const std::vector<Line>& edges);
	void clusterPSMatrix(std::vector<std::vector<int> >& result);
	cv::Point3d estimateVanPoint(const std::vector<Line>& edges, const std::vector<int>& cluster);

};

#endif /* defined(__cvar_core__JLinkage__) */
