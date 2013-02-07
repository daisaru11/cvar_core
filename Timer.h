//
//  Timer.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__Timer__
#define __cvar_core__Timer__

#include "cvarconf.h"

#include <iostream>

#ifdef CVAR_IOS
class Timer
{
public:
	Timer()
	:s(0), tmp(0), result(0), active(false)
	{
	}
	void start()
	{
		result = 0;
		s = clock();
		active = true;
	}
	void resume()
	{
		if (active)
		{
			tmp += clock() - s;
		}
		active = false;
	}
	void stop()
	{
		if (active)
		{
			result = tmp + clock() - s;
		}
		tmp = 0;
		active = false;
	}
	void print(const std::string& title)
	{
		std::cout << title << ": " << (double)result/CLOCKS_PER_SEC << std::endl;
	}
private:
	clock_t s, tmp, result;
	bool active;
};
#else

#include <boost/timer/timer.hpp>
class Timer
{
public:
	void start()
	{
		t.start();
	}
	void resume()
	{
		t.resume();
	}
	void stop()
	{
		t.stop();
	}
	void print(const std::string& title)
	{
		std::cout<< title << ": " << t.format(10,"%ws wall, %us user + %ss system = %ts CPU (%p%)\n")<< std::endl;
	}
private:
	boost::timer::cpu_timer t;
};
#endif

#endif /* defined(__cvar_core) */
