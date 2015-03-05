// search.h - defines for actual journey search

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define Maxevs 8192

#define Topdts 32

// port and hop refs are global
struct srcctx {
  char desc[256];

  // result
  char resbuf[2 * Nxleg * 256];
  ub4 reslen;
  ub8 querytlim,queryt0;
  struct trip trips[2];

  // main search args
  ub4 dep,arr;
  ub4 vias[Nvia];
  ub4 viacnt;

  // search params
  ub4 deptmin,deptmax;
  ub4 deptmin_cd,depttmin_cd,tspan;
  ub4 utcofs12;
  ub4 lostop,histop,nethistop;
  ub4 mintt,maxtt;
  ub4 walklimit,sumwalklimit;
  ub4 stop;
  ub4 costlim;
  ub4 costperstop;

  // workspace
  ub4 lodt,hidt;
  ub4 lodist;
  ub4 hidist;
  ub4 geodist;
  ub4 lot,lotid;
  ub4 timestop;

  // store current best trip
  ub4 curdts[Nxleg];
  ub4 curdurs[Nxleg];
  ub4 curts[Nxleg];
  ub4 curtids[Nxleg];
  ub4 curfares[Nxleg];
  ub4 cursdeps[Nxleg];
  ub4 cursarrs[Nxleg];
  ub4 curdt,curt; // shorthand for overall props

  ub4 locvarcnt;
  ub4 locnocnt;
  ub4 locsrccnt;
  ub4 varcnt;
  ub4 dvarcnt,tvarcnt,avarcnt,dvarxcnt,tvarxcnt,avarxcnt;

  ub8 combicnt,totevcnt;

  ub4 stat_noprv;
  ub4 stat_nxtlim;
  ub4 stat_nxt0,stat_nxt3;

  ub4 nleg;

  ub4 duraccs[Nxleg];
  ub4 hop1s[Nxleg];
  ub4 hop2s[Nxleg];
  ub4 parts[Nxleg];

  ub4 dcnts[Nxleg];   // #events in dev[leg]

  ub4 dtcurs[Nxleg];  // low cost (=biased dt) for [0.. curleg]
  ub4 devcurs[Nxleg]; // dev index for above

  ub4 *depevs[Nxleg]; // candidate event+attr store: time,tid,dt,dur,cost

  ub4 dtlos[Nxleg];

  ub4 topdts[Topdts];

  ub4 *evpool;
};
typedef struct srcctx search;

extern void inisearch(void);
extern int plantrip(search *src,char *ref,ub4 dep,ub4 arr,ub4 nstoplo,ub4 nstophi);
