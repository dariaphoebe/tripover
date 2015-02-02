// gtfsprep.h - prepare gtfs feeds

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

struct gtstop {
  ub4 idofs,codeofs,nameofs,latofs,lonofs;
  ub4 idlen,codelen,namelen,latlen,lonlen;
  ub4 parent,loctype;
};

struct gtfsnet {
  ub4 agencycnt;
  ub4 calendarcnt;
  ub4 caldatescnt;
  ub4 routecnt;
  ub4 stopcnt;
  ub4 tripcnt;
  ub4 stoptimescnt;
  block agencymem;
  block calendarmem;
  block caldatesmem;
  block routemem;
  block stopmem;
  block tripmem;
  block stoptimesmem;
  char *agencylines;
  char *calendarlines;
  char *caldateslines;
  char *routelines;
  char *stoplines;
  char *triplines;
  char *stoptimeslines;
  ub4 agencylinepos;
  ub4 calendarlinepos;
  ub4 caldateslinepos;
  ub4 routelinepos;
  ub4 stoplinepos;
  ub4 triplinepos;
  ub4 stoptimeslinepos;
};
typedef struct gtfsnet gtfsnet;
