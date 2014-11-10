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
#include "util.h"
#include "mem.h"
#include "time.h"

static ub4 msgfile;
#include "msg.h"

#include "os.h"

static ub4 epochmin,eramin;

// todo utc-based

static ub4 daysinmon[12] = {31,28,31,30,31,30,30,31,30,31,30,31};
static ub4 daysinmon2[12] = {31,29,31,30,31,30,30,31,30,31,30,31};
static ub4 *yymm2daytab;

void initime(int iter)
{
  char buf[256];
  ub4 now;
  struct tm tm;
  time_t sec;
  ub4 years,months,days,yy,mm,d;

  oclear(tm);

  msgfile = setmsgfile(__FILE__);
  iniassert();
  if (iter == 0) {
    years = Erayear - Epochyear;
    months = years * 12;
    days = 0;
    yymm2daytab = alloc(months,ub4,0,"misc",0);
    for (yy = 0; yy < years; yy++) {
      for (mm = 0; mm < 12; mm++) {
        d = (yy % 4) ? daysinmon[mm] : daysinmon2[mm];
        yymm2daytab[yy * 12 + mm] = days;
        days += d;
      }
    }
    tm.tm_year = Epochyear - 1900;
    tm.tm_mday = 1;
    sec = mktime(&tm);
    epochmin = (ub4)(sec / 60);
  } else {
    now = (ub4)time(NULL);
    sec70toyymmdd(now,buf,sizeof(buf));
    info(0,"current time %s : expect UTC",buf);
  }
}

void sec70toyymmdd(ub4 secs, char *dst, ub4 dstlen)
{
  time_t t = (time_t)secs;
  struct tm *tp = gmtime(&t);
  mysnprintf(dst,0,dstlen,"%04u-%02u-%02u %02u:%02u", tp->tm_year+1900, tp->tm_mon+1,tp->tm_mday,tp->tm_hour,tp->tm_min);
}

void mintoyymmdd(ub4 min, char *dst, ub4 dstlen)
{
  time_t t = (time_t)(min + epochmin) * 60;
  struct tm *tp = gmtime(&t);
  mysnprintf(dst,0,dstlen,"%04u-%02u-%02u %02u:%02u", tp->tm_year+1900, tp->tm_mon+1,tp->tm_mday,tp->tm_hour,tp->tm_min);
}

// 20140226 to minutes since epoch
ub4 yymmdd2min(ub4 cd)
{
  struct tm tm;
  ub4 d,m,y,mm,yy,dm,days;

  oclear(tm);

//  info(0,"cd time %u",cd);
  d = cd % 100;
  if (d == 0) { warning(0,"day in %u zero",cd); d = 1; }
  else if (d > 31) { warning(0,"day %u above 31",d); d = 31; }
  cd /= 100;
  m = cd % 100;
  if (m == 0) { warning(0,"month in %u zero",cd); m = 1; }
  else if (m > 12) { warning(0,"month %u above 12",m); m = 12; }
  dm = daysinmon2[m-1];
  if (d > dm) { warning(0,"invalid day %u in month %u",d,m); d = dm; }

  y = cd / 100;
  if (y < Epochyear) { warning(0,"year %u before lowest supported %u",y,Epochyear); y = Epochyear; }
  else if (y >= Erayear) {
    warning(0,"year %u after highest supported %u",y,Erayear);
    y = Erayear - 1; m = 12; d = 31;
  }

  yy = y - Epochyear;
  mm = m - 1;
  days = yymm2daytab[yy * 12 + mm] + (d - 1);

  return (days * 1440);
}

// weekday of minute time
ub4 min2wday(ub4 min)
{
  ub4 day = min / 1440;
  ub4 wday = (day + Epochwday) % 7;
  return wday;
}

ub4 gettime_sec(void)
{
  ub8 usec = gettime_usec();
  return (ub4)(usec / (1000 * 1000));
}
