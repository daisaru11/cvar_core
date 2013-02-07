//
//  Optimization.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__Optimization__
#define __cvar_core__Optimization__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class Optimization
{

private:

	void *estimate_param;
	CvMat *J;
	CvMat *JtJ;
	CvMat *W;
	CvMat *diag;
	CvMat *tmp;
	CvMat *err;
	CvMat *delta;
	CvMat *x_plus;
	CvMat *x_minus;
	CvMat *x_tmp1;
	CvMat *x_tmp2;
	CvMat *tmp_par;

	double CalcTukeyWeight(double residual, double c);
	double CalcTukeyWeightSimple(double residual, double c);

	double lambda;

public:

	enum OptimizeMethod
	{
		GAUSSNEWTON,
		LEVENBERGMARQUARDT,
		TUKEY_LM
	};

	Optimization(int n_params, int n_meas);
	~Optimization();

	CvMat *GetErr() { return err; }

	typedef void (*EstimateCallback)(CvMat* state, CvMat *projection, void *param);

	void CalcJacobian(CvMat* x, CvMat* J, EstimateCallback Estimate);

	double Optimize(CvMat*					parameters,
				    CvMat*					measurements,
					double					stop,
					int						max_iter,
					EstimateCallback		Estimate,
					void *param				= 0,
					OptimizeMethod method	= LEVENBERGMARQUARDT,
					CvMat* parameters_mask	= 0,
					CvMat* J_mat			= 0,
					CvMat* weights			= 0); 

};

#endif
