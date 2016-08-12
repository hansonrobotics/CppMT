/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2013, 2014, 2015 Michal Uricar
 * Copyright (C) 2013, 2014, 2015 Michal Uricar
 */

#ifndef __CTIMER__H_
#define __CTIMER__H_

#include <cstdio>
#include <cstdlib>
#include <ctime>

class CTimer {

public:

	void tic()
	{
		gettimeofday(&start, 0x0);
	}

	double toc()
	{
		gettimeofday(&end, 0x0);
		sec = end.tv_sec - start.tv_sec;
		usec = end.tv_usec - start.tv_usec;
		duration = (sec + usec/1000000.0) * 1000.0;
		return duration;
	}

private:
	timeval start, end;
	tm* local;
	long sec, usec;
	double duration;
};

#endif // __CTIMER__H_
