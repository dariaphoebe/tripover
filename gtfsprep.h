// gtfsprep.h - prepare gtfs feeds

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define Nearstop 256

struct gtstop {
  ub4 id;

  ub4 gidofs,codeofs,nameofs,parentofs,latofs,lonofs,descofs;
  ub4 gidlen,codelen,namelen,parentlen,latlen,lonlen,desclen;
  bool isparent,hasparent;
  ub4 parent,loctype;
  double lat,lon,rlat,rlon;
  ub4 nearcnt,enearcnt;
  ub4 nears[Nearstop];
  ub4 group,iparent;
  char name[64];
  char parentname[64];
};

struct bucket {
  ub4 sofs;
  ub4 slen;
  ub4 code;
  ub4 data;
};

struct hashtab {
  ub4 len;
  ub4 eqlen;
  ub4 maxeq;
  ub4 itemcnt;
  char *strpool;
  ub4 spoollen,sofs;
  struct bucket *bkts;
  block bktmem;
  block strmem;
  const char *desc;

};
typedef struct hashtab hash;

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
  block stopmem,estopmem;
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

  hash *routes;
  hash *trips;
  hash *stops;
};
typedef struct gtfsnet gtfsnet;
