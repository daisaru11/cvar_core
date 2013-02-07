//
//  TimerManager.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__TimerManager__
#define __cvar_core__TimerManager__

#include "cvarconf.h"

#include <iostream>
#include <map>
#include "Timer.h"

class TimerManager
{
public:

	void start(const std::string& name)
	{
		timer_map[name].start();
	}
	void resume(const std::string& name)
	{
		timer_map[name].resume();
	}
	void stop(const std::string& name)
	{
		timer_map[name].stop();
	}
	void print(const std::string& name)
	{
		timer_map[name].print(name);
	}

private:
	std::map<std::string, Timer> timer_map;
};

#endif /* defined(__cvar_core__TimerManager__) */
