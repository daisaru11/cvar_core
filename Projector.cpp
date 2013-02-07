//
//  Projector.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include "Projector.h"

void Projector::setCameraParams(const Mat &K, const Mat &R, const Mat &T)
{
    CV_Assert(K.size() == Size(3, 3) && K.type() == CV_64F);
    CV_Assert(R.size() == Size(3, 3) && R.type() == CV_64F);
    CV_Assert((T.size() == Size(1, 3) || T.size() == Size(3, 1)) && T.type() == CV_64F);

    Mat_<double> K_(K);
    k[0] = K_(0,0); k[1] = K_(0,1); k[2] = K_(0,2);
    k[3] = K_(1,0); k[4] = K_(1,1); k[5] = K_(1,2);
    k[6] = K_(2,0); k[7] = K_(2,1); k[8] = K_(2,2);

    Mat_<double> Kinv = K.inv();
    kinv[0] = Kinv(0,0); kinv[1] = Kinv(0,1); kinv[2] = Kinv(0,2);
    kinv[3] = Kinv(1,0); kinv[4] = Kinv(1,1); kinv[5] = Kinv(1,2);
    kinv[6] = Kinv(2,0); kinv[7] = Kinv(2,1); kinv[8] = Kinv(2,2);

    Mat_<double> Rinv = R.t();
    rinv[0] = Rinv(0,0); rinv[1] = Rinv(0,1); rinv[2] = Rinv(0,2);
    rinv[3] = Rinv(1,0); rinv[4] = Rinv(1,1); rinv[5] = Rinv(1,2);
    rinv[6] = Rinv(2,0); rinv[7] = Rinv(2,1); rinv[8] = Rinv(2,2);

    Mat_<double> R_Kinv = Rinv * Kinv;
    r_kinv[0] = R_Kinv(0,0); r_kinv[1] = R_Kinv(0,1); r_kinv[2] = R_Kinv(0,2);
    r_kinv[3] = R_Kinv(1,0); r_kinv[4] = R_Kinv(1,1); r_kinv[5] = R_Kinv(1,2);
    r_kinv[6] = R_Kinv(2,0); r_kinv[7] = R_Kinv(2,1); r_kinv[8] = R_Kinv(2,2);

    Mat_<double> K_R = K * R;
    k_r[0] = K_R(0,0); k_r[1] = K_R(0,1); k_r[2] = K_R(0,2);
    k_r[3] = K_R(1,0); k_r[4] = K_R(1,1); k_r[5] = K_R(1,2);
    k_r[6] = K_R(2,0); k_r[7] = K_R(2,1); k_r[8] = K_R(2,2);

    Mat_<double> T_(T.reshape(0, 3));
    t[0] = T_(0,0); t[1] = T_(1,0); t[2] = T_(2,0);
}

void Projector::setRotation(const Mat &R)
{
    CV_Assert(R.size() == Size(3, 3) && R.type() == CV_64F);

	Mat K(3, 3, CV_64F, k);
	Mat Kinv(3, 3, CV_64F, kinv);

    Mat_<double> Rinv = R.t();
    rinv[0] = Rinv(0,0); rinv[1] = Rinv(0,1); rinv[2] = Rinv(0,2);
    rinv[3] = Rinv(1,0); rinv[4] = Rinv(1,1); rinv[5] = Rinv(1,2);
    rinv[6] = Rinv(2,0); rinv[7] = Rinv(2,1); rinv[8] = Rinv(2,2);

    Mat_<double> R_Kinv = Rinv * Kinv;
    r_kinv[0] = R_Kinv(0,0); r_kinv[1] = R_Kinv(0,1); r_kinv[2] = R_Kinv(0,2);
    r_kinv[3] = R_Kinv(1,0); r_kinv[4] = R_Kinv(1,1); r_kinv[5] = R_Kinv(1,2);
    r_kinv[6] = R_Kinv(2,0); r_kinv[7] = R_Kinv(2,1); r_kinv[8] = R_Kinv(2,2);

    Mat_<double> K_R = K * R;
    k_r[0] = K_R(0,0); k_r[1] = K_R(0,1); k_r[2] = K_R(0,2);
    k_r[3] = K_R(1,0); k_r[4] = K_R(1,1); k_r[5] = K_R(1,2);
    k_r[6] = K_R(2,0); k_r[7] = K_R(2,1); k_r[8] = K_R(2,2);
}
