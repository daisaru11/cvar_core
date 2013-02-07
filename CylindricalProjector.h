//
//  CylindricalProjector.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__CylindricalProjector__
#define __cvar_core__CylindricalProjector__

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Projector.h"

using namespace cv;

class CylindricalProjector : public Projector
{
public:
    void mapForward(double x, double y, double &u, double &v);
    void mapBackward(double u, double v, double &x, double &y);
};

inline
void CylindricalProjector::mapForward(double x, double y, double &u, double &v)
{
    double x_ = r_kinv[0] * x + r_kinv[1] * y + r_kinv[2];
    double y_ = r_kinv[3] * x + r_kinv[4] * y + r_kinv[5];
    double z_ = r_kinv[6] * x + r_kinv[7] * y + r_kinv[8];

    u = scale * atan2(x_, z_);
    v = scale * y_ / sqrt(x_ * x_ + z_ * z_);
}


inline
void CylindricalProjector::mapBackward(double u, double v, double &x, double &y)
{
    u /= scale;
    v /= scale;

    double x_ = sin(u);
    double y_ = v;
    double z_ = cos(u);

    double z;
    x = k_r[0] * x_ + k_r[1] * y_ + k_r[2] * z_;
    y = k_r[3] * x_ + k_r[4] * y_ + k_r[5] * z_;
    z = k_r[6] * x_ + k_r[7] * y_ + k_r[8] * z_;

    if (z > 0) { x /= z; y /= z; }
    else x = y = -1;
}



#endif /* defined(__cvar_core__CylindricalProjector__) */
