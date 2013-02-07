//
//  PoseUpdaterRANSAC.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include "PoseUpdaterRANSAC.h"

PoseUpdaterRANSAC::PoseUpdaterRANSAC(const double confidence, const int max_iters, const double reproj_threshold)
:OptimizationRANSAC(3),
confidence(confidence), max_iters(max_iters), reproj_threshold(reproj_threshold)
{
	// do nothing
}

PoseUpdaterRANSAC::~PoseUpdaterRANSAC()
{
	// do nothing
}

void PoseUpdaterRANSAC::init(const Map& mapinfo)
{
	map_size = mapinfo.size;
	map_center = mapinfo.center;
	projector = mapinfo.projector;
}

bool PoseUpdaterRANSAC::updatePose(std::vector<Keypoint>& keypoints, cv::Mat& pose)
{
	const int count = static_cast<int>(keypoints.size());
	cv::Mat Ps(count, 3, CV_64F);
	cv::Mat Ms(count, 3, CV_64F);

	//cv::Mat K(3,3,CV_64F,projector.k);
	double *kinv = projector.kinv;
	double scale = projector.scale;

	for (int k=0; k<count; ++k)
	{
		Keypoint& kp = keypoints[k];

		double mx = kp.map_point.x-map_center.x;
		double my = kp.map_point.y-map_center.y;
		double r = sqrt( scale*scale + my*my );

		double * M_ = Ms.ptr<double>(k);
		M_[0] = scale * sin(mx/scale) / r;
		M_[1] = my / r;
		M_[2] = scale * cos(mx/scale) / r;

		double * P_ = Ps.ptr<double>(k);
		double px = kp.img_point.x;
		double py = kp.img_point.y;
		//double pz = 1.0;
		P_[0] = kinv[0] * px + kinv[1] * py + kinv[2];
		P_[1] = kinv[3] * px + kinv[4] * py + kinv[5];
		P_[2] = kinv[6] * px + kinv[7] * py + kinv[8];

		double nP = sqrt(P_[0]*P_[0] + P_[1]*P_[1] + P_[2]*P_[2]);
		P_[0] /= nP;
		P_[1] /= nP;
		P_[2] /= nP;
	}


	cv::Mat R;
	cv::Mat mask;
	bool success = run(Ps, Ms, R, mask, reproj_threshold, confidence, max_iters);

	if (success)
	{
		fit(Ps, Ms, mask, pose);
	}
}

void PoseUpdaterRANSAC::fitSubset(const cv::Mat& m1, const cv::Mat& m2, std::vector<int>& subset_idx, cv::Mat& model)
{
	cv::Mat C = cv::Mat::zeros(3,3,CV_64F);
	double *C_ = reinterpret_cast<double *>(C.data);
	int count = static_cast<int>(subset_idx.size());

	for (int i=0; i<count; ++i)
	{
		int idx = subset_idx[i];

		const double *P_ = m1.ptr<double>(idx);
		const double *M_ = m2.ptr<double>(idx);
		
		C_[0] += P_[0]*M_[0]; C_[1] += P_[0]*M_[1]; C_[2] += P_[0]*M_[2];
		C_[3] += P_[1]*M_[0]; C_[4] += P_[1]*M_[1]; C_[5] += P_[1]*M_[2];
		C_[6] += P_[2]*M_[0]; C_[7] += P_[2]*M_[1]; C_[8] += P_[2]*M_[2];
	}

	cv::SVD svd(C, cv::SVD::MODIFY_A|cv::SVD::FULL_UV);
	cv::Mat scale_mat = cv::Mat::eye(3,3,CV_64F);
	scale_mat.at<double>(2,2) = cv::determinant( svd.u * svd.vt ) > 0 ? 1.0:-1.0;

	model = svd.u * scale_mat * svd.vt;
}

void PoseUpdaterRANSAC::fit(const cv::Mat& m1, const cv::Mat& m2, const cv::Mat& mask, cv::Mat& model)
{
	cv::Mat C = cv::Mat::zeros(3,3,CV_64F);
	double *C_ = reinterpret_cast<double *>(C.data);
	int count = m1.rows;

	for (int i=0; i<count; ++i)
	{
		if (!mask.at<uchar>(i,0)) continue;
		const double *P_ = m1.ptr<double>(i);
		const double *M_ = m2.ptr<double>(i);
		
		C_[0] += P_[0]*M_[0]; C_[1] += P_[0]*M_[1]; C_[2] += P_[0]*M_[2];
		C_[3] += P_[1]*M_[0]; C_[4] += P_[1]*M_[1]; C_[5] += P_[1]*M_[2];
		C_[6] += P_[2]*M_[0]; C_[7] += P_[2]*M_[1]; C_[8] += P_[2]*M_[2];
	}

	cv::SVD svd(C, cv::SVD::MODIFY_A|cv::SVD::FULL_UV);
	cv::Mat scale_mat = cv::Mat::eye(3,3,CV_64F);
	scale_mat.at<double>(2,2) = cv::determinant( svd.u * svd.vt ) > 0 ? 1.0:-1.0;

	model = svd.u * scale_mat * svd.vt;
}

void PoseUpdaterRANSAC::computeReprojError( const cv::Mat& m1, const cv::Mat& m2, const cv::Mat& model, cv::Mat& error )
{
	int count = m1.rows;
	const double *R = reinterpret_cast<double *>(model.data);
	double *err = reinterpret_cast<double *>(error.data);

	for(int i = 0; i < count; i++ )
	{
		// P = RM
		const double *f = m2.ptr<double>(i); // from
		const double *t = m1.ptr<double>(i); // to

		double a = R[0]*f[0] + R[1]*f[1] + R[2]*f[2] - t[0];
		double b = R[3]*f[0] + R[4]*f[1] + R[5]*f[2] - t[1];
		double c = R[6]*f[0] + R[7]*f[1] + R[8]*f[2] - t[2];

		err[i] = sqrt(a*a + b*b + c*c);
	}
}
