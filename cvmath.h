//
//  cvmath.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__cvmath__
#define __cvar_core__cvmath__

#include <math.h>
#include <iostream>
#include <opencv2/core/core.hpp>

class cvmath
{

 public:
  static const double PI = 3.14159265;
    
  static inline double sqr(double x) { return x*x ; }

  //   round x to the nearest integer
  static inline int round(const double x) ;

  //   return the sign of x (+-1)
  static inline int sign(double x) ;

  // test if a number equals 0 (with threshold value)
  static inline bool nul(double x, double s=0.001);

  // test if two numbers are equals (with a user defined threshold)
  static inline bool equal(double x, double y, double s=0.001);

  /*!
    Find the maximum between two numbers (or other).
    \param a : First number.
    \param b : Second number.
    \return The maximum of the two numbers.
  */
  template <class Type> static Type maximum(const Type& a, const Type& b)
  {
    return (a > b) ? a : b;
  }

  /*!
    Find the minimum between two numbers (or other).
    \param a : First number.
    \param b : Second number.
    \return The minimum of the two numbers.
  */
  template <class Type> static Type minimum(const Type& a, const Type& b)
  {
    return (a < b) ? a : b;
  }

  static inline void euler2mat(const cv::Mat& eular, cv::Mat& mat);

  static inline void mat2euler(const cv::Mat& mat, cv::Mat& eular);

};

/*!
  Round x to the nearest integer.

  \param x : Value to round.

  \return Nearest integer of x.

*/
int cvmath::round(const double x)
{
  if (sign(x) > 0)
    {
      if ((x-(int)x) <= 0.5) return (int)x ;
      else return (int)x+1 ;
    }
  else
    {
      if (fabs(x-(int)x) <= 0.5) return (int)x ;
      else return (int)x-1 ;
    }
}

/*!
  Return the sign of x.

  \return -1 if x is negative, +1 if positive.

*/
int cvmath::sign(double x)
{
  if (fabs(x) < 1e-15) return 0 ;else
    {
      if (x<0) return -1 ; else return 1 ;
    }
}

/*!
  Compares  \f$ | x | \f$ to \f$ s \f$.
  \param x : Value to test.
  \param s : Tolerance threshold
  \return true if \f$ | x | < s \f$.

*/
bool cvmath::nul(double x, double s)
{
  return(fabs(x)<s);
}

/*!
  Compares  \f$ | x - y | \f$ to \f$ s \f$.
  \param x : x value.
  \param y : y value.
  \param s : Tolerance threshold.
  \return true if \f$ | x - y | < s \f$.
*/
bool cvmath::equal(double x, double y, double s)
{
  return( nul(x-y, s) );
}


void cvmath::euler2mat(const cv::Mat& eular, cv::Mat& mat)
{
    CV_Assert(eular.size() == cv::Size(3, 1) && eular.type() == CV_64F);
    CV_Assert(mat.size() == cv::Size(3, 3) && mat.type() == CV_64F);

	cv::Mat_<double> mat_(mat);
	cv::Mat_<double> eular_(eular);
	double cosx = cos(eular_(0,0));
	double cosy = cos(eular_(0,1));
	double cosz = cos(eular_(0,2));
	double sinx = sin(eular_(0,0));
	double siny = sin(eular_(0,1));
	double sinz = sin(eular_(0,2));
	
	mat_(0,0) = cosz * cosy;
	mat_(0,1) = cosz * siny * sinx - sinz * cosx;
	mat_(0,2) = cosz * siny * cosx + sinz * sinx;
	mat_(1,0) = sinz * cosy;
	mat_(1,1) = sinz * siny * sinx + cosz * cosx;
	mat_(1,2) = sinz * siny * cosx - cosz * sinx;
	mat_(2,0) = -siny;
	mat_(2,1) = cosy * sinx;
	mat_(2,2) = cosy * cosx;
}

void cvmath::mat2euler(const cv::Mat& mat, cv::Mat& eular)
{
    CV_Assert(eular.size() == cv::Size(3, 1) && eular.type() == CV_64F);
    CV_Assert(mat.size() == cv::Size(3, 3) && mat.type() == CV_64F);
	cv::Mat_<double> mat_(mat);
	cv::Mat_<double> eular_(eular);

	eular_(0,0) = atan2( mat_(2,1), mat_(2,2) );
	eular_(0,1) = atan2( -mat_(2,0), sqrt(mat_(2,1)*mat_(2,1) + mat_(2,2)* mat_(2,2)) );
	eular_(0,2) = atan2( mat_(1,0), mat_(0,0) );
}

#endif
