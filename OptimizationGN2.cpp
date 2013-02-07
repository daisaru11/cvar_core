//
//  OptimizationGN2.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include "OptimizationGN2.h"

OptimizationGN2::OptimizationGN2()
:n_params(0), n_meas(0)
{
	// do nothing
}

OptimizationGN2::~OptimizationGN2()
{
	// do nothing
}


void OptimizationGN2::calcJacobian(const cv::Mat& model, const cv::Mat& m, cv::Mat& Jt)
{
	const double step = 0.001;
	Jt.create(n_params, n_meas, CV_64F);
	cv::Mat proj1(n_meas, 1, CV_64F);
	cv::Mat proj2(n_meas, 1, CV_64F);

	for (int i=0; i<Jt.rows; ++i)
	{
		cv::Mat Jt_row(n_meas, 1, CV_64F, Jt.ptr<double>(i));
		cv::Mat x_plus = model.clone();
		cv::Mat x_minus = model.clone();

		x_plus.at<double>(i,0)  += step;
		x_minus.at<double>(i,0) -= step;

		project(x_plus, m, proj1);
		project(x_minus, m, proj2);
		cv::subtract(proj1, proj2, Jt_row);
		Jt_row *= 1.0/(2*step);
	}
}

double OptimizationGN2::calcTukeyWeight(double residual, double c)
{
	//const double c = 3; // squared distance in the model tracker
	double ret=0;

	if(fabs(residual) <= c)
	{
		double tmp = 1.0-((residual/c)*(residual/c));
		ret = ((c*c)/6.0)*(1.0-tmp*tmp*tmp);
	}
	else
		ret = (c*c)/6.0;
	
	if(residual)
		ret = fabs(sqrt(ret)/residual);
	else
		ret = 1.0; // ???
	
	return ret;
}


void OptimizationGN2::run( cv::Mat& model, 
	const cv::Mat& observation, const cv::Mat m,
	int max_iter, Method method)
{
	n_params = model.rows;
	n_meas = observation.rows;

	double error_new = 0;
	double error_old = 0;
	double n1, n2;
	double stop = 0.0005;
	double lambda = 0.001;


	cv::Mat J, Jt, JtJinv, JtJ;
	cv::Mat err(n_meas, 1, CV_64F);
	cv::Mat delta;
	cv::Mat diag;
	cv::Mat W(n_meas, n_meas, CV_64F);
	cv::Mat tmp_model(n_params, 1, CV_64F);

	int iter = 0;
	while (true)
	{
		calcJacobian(model, m, Jt);
		
		reprojErr(model, observation, m, err);

		switch(method)
		{
			case (GAUSSNEWTON) :
				JtJinv = (Jt * Jt.t()).inv(DECOMP_SVD);
				delta = JtJinv * Jt * err;
				
				model += delta;

				n1 = cv::norm(delta);
				n2 = cv::norm(model);

				if ( ((n1/n2) < stop) || (iter >= max_iter) )
					goto end; 

			break;

			case (TUKEY_LM) :
							
				cv::Mat diag = cv::Mat::eye(n_params,n_params,CV_64F)*lambda;

				// Tukey weights
				for(int k = 0; k < W.rows; ++k)
				{
					W.at<double>(k,k) = calcTukeyWeight(err.at<double>(k,0), 3);
				}

				JtJ = Jt * W * Jt.t();
				JtJ += diag;
				JtJinv = JtJ.inv(DECOMP_SVD);

				delta = JtJinv * Jt * W * err;
				tmp_model = model + delta;

				error_old = cv::norm(err);
				reprojErr(tmp_model, observation, m, err);

				error_new = cv::norm(err);
				
				if(error_new < error_old)
				{
					tmp_model.copyTo(model);
					lambda = lambda/10.0;
				}
				else
				{
					lambda = lambda*10.0;
				}
				if(lambda>10) lambda = 10;
				if(lambda<0.00001) lambda = 0.00001;

				n1 = cv::norm(delta);
				n2 = cv::norm(model);

				if( ((n1/n2) < stop) ||
					(iter >= max_iter) )
				{
					goto end;
				}

			break;
		}
		++iter;

	}
end:
	
	return;
}
