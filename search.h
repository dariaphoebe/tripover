// search.h - defines for actual journey search

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define Maxevs 2048

#define Topdts 32

// port and hop refs are global
struct srcctx {
  char desc[256];

  char resbuf[2 * Nxleg * 256];
  ub4 reslen;
  ub8 querytlim;

  struct trip trips[2];

  ub4 dep,arr;
  ub4 vias[Nvia];
  ub4 viacnt;
  ub4 deptmin,deptmax;
  ub4 deptmin_cd,depttmin_cd,tspan;
  ub4 utcofs12;
  ub4 lostop,histop;
  ub4 stop;
  ub4 costlim;
  ub4 costperstop;

  ub4 lodt,hidt;
  ub4 lodist,hidist,geodist;
  ub4 lot,lotid;

  ub4 curdts[Nxleg];
  ub4 curdurs[Nxleg];
  ub4 curts[Nxleg];
  ub4 curtids[Nxleg];
  ub4 curdt,curt;

  ub4 locvarcnt;
  ub4 locnocnt;
  ub4 locsrccnt;
  ub4 varcnt;
  ub4 dvarcnt,tvarcnt,avarcnt,dvarxcnt,tvarxcnt,avarxcnt;
  ub4 stat_noprv;
  ub4 stat_nxtlim;
  ub4 stat_nxt0,stat_nxt3;
  ub4 duraccs[Nxleg];

  ub4 hop1s[Nxleg];
  ub4 hop2s[Nxleg];
  ub4 parts[Nxleg];

  ub4 dcnts[Nxleg];
  ub4 dtlos[Nxleg];
  ub4 dtcurs[Nxleg];
  ub4 devcurs[Nxleg];
  ub4 *depevs[Nxleg];

  ub4 topdts[Topdts];

  ub4 *evpool;
};
typedef struct srcctx search;

extern void inisearch(void);
extern int plantrip(search *src,char *ref,ub4 dep,ub4 arr,ub4 nstoplo,ub4 nstophi);
