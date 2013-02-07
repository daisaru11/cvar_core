//
//  OptimizationRANSAC.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//


#include "OptimizationRANSAC.h"

OptimizationRANSAC::OptimizationRANSAC(const int num_model_points)
:num_model_points(num_model_points)
{
	// do nothing
}

OptimizationRANSAC::~OptimizationRANSAC()
{
	// do nothing
}

bool OptimizationRANSAC::getSubset(const cv::Mat& m1, const cv::Mat& m2, std::vector<int>& subset_idx)
{
	subset_idx.clear();
	int count = m1.rows;
	const int max_attempts = 300;
	int i, iters;
	for(i=0, iters=0; i<num_model_points && iters<max_attempts; )
	{
		int idx_i = rand() % count;
		int j;
		iters++;
		for (j=0; j<i; j++)
			if( idx_i == subset_idx[j] )
				break;
		if( j < i )
			continue;
		subset_idx.push_back(idx_i);
		i++;
	}

	return i == num_model_points && iters < max_attempts;
}

int RANSACUpdateNumIters( double p, double ep,
                        int model_points, int max_iters )
{
	// avoid inf's & nan's
	double num = MAX(1. - p, DBL_MIN);
	double denom = 1. - pow(1. - ep,model_points);
	if( denom < DBL_MIN )
		return 0;

	num = log(num);
	denom = log(denom);

	return denom >= 0 || -num >= max_iters*(-denom) ?
		max_iters : round(num/denom);
}

bool OptimizationRANSAC::run(const cv::Mat& m1, const cv::Mat& m2, 
			cv::Mat& model, cv::Mat& mask,
			const double reproj_threshold, const double confidence, 
			const int max_iters )
{
	bool result = false;
	const int count = m1.rows;
	int good_count_max = 0;
	std::vector<int> subset_idx;

	cv::Mat model_tmp;
	cv::Mat err(count, 1, CV_64F);
	cv::Mat tmask(count, 1, CV_8UC1);
	mask.create(count, 1, CV_8UC1);

	int niters = max_iters;
	for( int iter = 0; iter < niters; iter++ )
	{
		int good_count;
		if( count > num_model_points )
		{
			subset_idx.clear();
			bool found = getSubset( m1, m2, subset_idx );
			if( !found )
			{
				if( iter == 0 )
					return false;
				break;
			}
		}

		fitSubset(m1, m2, subset_idx, model_tmp);

		good_count = findInliers( m1, m2, model_tmp, err, tmask, reproj_threshold );
		if (good_count > good_count_max && good_count > num_model_points)
		{
			std::swap(tmask, mask);
			model_tmp.copyTo(model);
			good_count_max = good_count;
			niters = RANSACUpdateNumIters( confidence,
				(double)(count - good_count)/count, num_model_points, niters );
		}
	}

	if (good_count_max>0)
	{
		result = true;
	}

	return result;
}


int OptimizationRANSAC::findInliers( const cv::Mat& m1, const cv::Mat& m2,
		const cv::Mat& model, cv::Mat& err,
		cv::Mat& mask, double threshold )
{
	const int count = err.rows;
	int good_count = 0;
	const double* _err = reinterpret_cast<double *>(err.data);
	uchar* _mask = (uchar *)mask.data;

    threshold *= threshold;
	computeReprojError( m1, m2, model, err );
	for(int i = 0; i < count; i++ )
	{
		//std::cout << _err[i] << ", ";
		good_count += _mask[i] = _err[i] <= threshold;
	}
	//std::cout << std::endl;
	return good_count;
}

