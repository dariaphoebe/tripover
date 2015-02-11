// time.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define Epochyear 2000

// 1-1-2000 = sat
#define Epochwday 5

// time horizon
#define Erayear 2020

// c11 langage only
#if defined  __STDC_VERSION__ && __STDC_VERSION__ >= 201101
  _Static_assert(Epochyear > 1969,"time before 1970 not handled");
  _Static_assert(Epochyear < 2100,"time after  2100 not handled");
  _Static_assert(Epochyear < Erayear,"must have a time span");
  _Static_assert(Erayear - Epochyear < 100,"time span too large");
  _Static_assert(Erayear > Epochyear,"time span too small");
#endif

extern ub4 gettime_sec(void);
extern void sec70toyymmdd(ub4 secs, char *dst, ub4 dstlen);
extern ub4 yymmdd2min(ub4 cd,ub4 utcofs);
extern ub4 lmin2cd(ub4 min);
extern ub4 min2wday(ub4 min);
extern ub4 min2lmin(ub4 min,ub4 utcofs);
extern ub4 lmin2min(ub4 lmin,ub4 utcofs);
extern ub4 utc12ofs(ub4 uo12);
extern ub4 cdday2wday(ub4 cd);
extern ub4 cd2day(ub4 cd);
extern ub4 day2cd(ub4 day);
extern ub4 hhmm2min(ub4 ct);
extern ub4 nix2min(ub4 xmin);

extern void initime(int iter);
