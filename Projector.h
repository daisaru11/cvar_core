//
//  Projector.h
//  cvtest0
//
//  Created by 酒井 大地 on 2013/01/15.
//  Copyright (c) 2013年 Daichi Sakai. All rights reserved.
//

#ifndef __cvtest0__Projector__
#define __cvtest0__Projector__

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

class Projector {
public:
    double scale;
    double k[9];
    double kinv[9];
    double rinv[9];
    double r_kinv[9];
    double k_r[9];
    double t[3];
	
    void setCameraParams(const Mat &K = Mat::eye(3, 3, CV_64F),
                         const Mat &R = Mat::eye(3, 3, CV_64F),
                         const Mat &T = Mat::zeros(3, 1, CV_64F));
	void setRotation(const Mat &R);

protected:
private:
	
};

#endif /* defined(__cvtest0__Projector__) */
