// time.c

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Logic dealing with wall-clock time
   Conversions, formatting, timezones
 */

#include <time.h>

#include "base.h"
#include "time.h"

static ub4 msgfile;
#include "msg.h"

void initime(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

void sec70toyymmdd(ub4 secs, char *dst, ub4 dstlen)
{
  time_t t = (time_t)secs;
  struct tm *tp = gmtime(&t);
  mysnprintf(dst,0,dstlen,"%04u-%02u-%02u %02u:%02u", tp->tm_year+1900, tp->tm_mon+1,tp->tm_mday,tp->tm_hour,tp->tm_min);
}
