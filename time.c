// time.c

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Logic dealing with wall-clock time
   Conversions, formatting, timezones

   Internal standard is minutes UTC since Epoch, typ 2000
   This is kept in an unsigned 32 bit integer
   UTC offset is minutes plus 12 hours kept as an unsigned integer.
   weekdays start at monday = 0 .. sunday = 6

   in some places a 'coded decimal' unsigned integer is used for a date
 */

#include <time.h>

#include "base.h"
#include "util.h"
#include "mem.h"
#include "time.h"

static ub4 msgfile;
#include "msg.h"

#include "os.h"

static ub4 eramin;

static ub4 daysinmon[12] =  {31,28,31,30,31,30,31,31,30,31,30,31};
static ub4 daysinmon2[12] = {31,29,31,30,31,30,31,31,30,31,30,31};
static ub4 *yymm2daytab;
static ub4 *day2cdtab;
static ub4 day2cdmax;

void initime(int iter)
{
  char buf[256];
  ub4 now;
  time_t sec;
  ub4 years,months,days,yy,mm,dd,d,cd,m;

  msgfile = setmsgfile(__FILE__);
  iniassert();

  // create calendar dates to minute table, supporting typically 10 years around now
  // mktime() is hardly useful as it refers to a fixed system TZ.
  if (iter == 0) {
    years = Erayear - Epochyear;
    months = years * 12;
    days = 0;
    yymm2daytab = alloc(months,ub4,0,"misc",0);
    day2cdtab = alloc(months * 31,ub4,0,"misc",0);
    day2cdmax = months * 31 - 1;

    for (yy = 0; yy < years; yy++) {
      for (mm = 0; mm < 12; mm++) {
        dd = (yy % 4) ? daysinmon[mm] : daysinmon2[mm];
        yymm2daytab[yy * 12 + mm] = days;
        for (d = 0; d < dd; d++) day2cdtab[days+d] = (yy + Epochyear) * 10000 + (mm + 1) * 100 + d + 1;
        days += dd;
      }
    }

  } else {
    now = (ub4)time(NULL);
    sec70toyymmdd(now,buf,sizeof(buf));
    info(0,"current time %s : expect UTC",buf);

#if 0
    for (yy = 13; yy < 15; yy++) {
      for (mm = 0; mm < 12; mm++) {
        cd = (yy + Epochyear) * 10000 + (mm + 1) * 100 + 1;
        m = yymmdd2min(cd,12 * 60);
        info(0,"%u.%u.1 %u %u %u daysto %u",yy,mm,cd,m,lmin2cd(m),yymm2daytab[yy * 12 + mm]);
      }
    }
#endif
  }
}

// add utcofs
ub4 min2lmin(ub4 min,ub4 utcofs)
{
  if (min <= utcofs) { warning(0,"time %u for utc offset %u before Epoch UTC",min,utcofs); return min; }
  else if (utcofs < 12 * 60) return min - (utcofs - 12 * 60);
  else return min + (12 * 60 - utcofs);
}

// sub utcofs
ub4 lmin2min(ub4 lmin,ub4 utcofs)
{
  if (lmin <= utcofs) { warning(0,"time %u at utc offset %u before Epoch UTC",lmin,utcofs); return lmin; }
  else if (utcofs < 12 * 60) return lmin + (12 * 60 - utcofs);
  else if (utcofs > 12 * 60) return lmin - (utcofs - 12 * 60);
  else return lmin;
}

void sec70toyymmdd(ub4 secs, char *dst, ub4 dstlen)
{
  time_t t = (time_t)secs;
  struct tm *tp = gmtime(&t);
  mysnprintf(dst,0,dstlen,"%04u-%02u-%02u %02u:%02u", tp->tm_year+1900, tp->tm_mon+1,tp->tm_mday,tp->tm_hour,tp->tm_min);
}

// minutes since epoch localtime to decimal-coded 20140522
ub4 lmin2cd(ub4 min)
{
  ub4 yr,mon,day;

  day = min / 1440;
  return day2cdtab[min(day,day2cdmax)];
}

// day to coded decimal yyyymmdd
ub4 day2cd(ub4 day)
{
  return day2cdtab[min(day,day2cdmax)];
}

// coded decimal day to day, tz-agnostic
ub4 cd2day(ub4 cd)
{
  ub4 d,m,y,mm,yy,dm,day;

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
  day = yymm2daytab[yy * 12 + mm] + (d - 1);
  return day;
}

// 20140226 localtime to minutes utc since epoch
ub4 yymmdd2min(ub4 cd,ub4 utcofs)
{
  ub4 d,m,y,mm,yy,dm,days;
  ub4 lmin;

  days = cd2day(cd);

  lmin = days * 1440;
  return lmin2min(lmin,utcofs);
}

// coded decimal day to weekday, tz-agnostic
ub4 cdday2wday(ub4 cd)
{
  ub4 day = cd2day(cd);
  ub4 wday = (day + Epochwday) % 7;
  return wday;
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
