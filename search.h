// search.h - defines for actual journey search

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define Maxevs 2048

// rudimentary
// port and hop refs are global
struct srcctx {
  char desc[256];

  ub4 trip[Nxleg];
  ub2 tripparts[Nxleg];
  ub4 tripports[Nxleg];
  ub4 triplen;
  ub4 tripcnt;
  ub4 dist;
  ub4 dep,arr;
  ub4 vias[Nvia];
  ub4 viacnt;
  ub4 deptmin,deptmax;
  ub4 lostop,histop;
  ub4 stop;
  ub4 costlim;

  ub4 lodt,hidt;
  ub4 lodist,hidist,geodist;

  ub4 locvarcnt;
  ub4 locnocnt;
  ub4 locsrccnt;
  ub4 dvarcnt,tvarcnt,avarcnt,dvarxcnt,tvarxcnt,avarxcnt;

  struct timepat *tps[Nxleg];
  ub4 dcnts[Nxleg];
  ub4 dtlos[Nxleg];
  ub4 *depevs[Nxleg];
  ub4 *evpool;

};
typedef struct srcctx search;

extern void inisearch(void);
extern int plantrip(search *src,char *ref,ub4 dep,ub4 arr,ub4 nstoplo,ub4 nstophi);
