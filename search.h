// search.h - defines for actual journey search

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

// rudimentary
// port and hop refs are global
struct srcctx {
  ub4 trip[Nxleg];
  ub2 tripparts[Nxleg];
  ub4 tripports[Nxleg];
  ub4 triplen;
  ub4 tripcnt;
  ub4 dist;
  ub4 lodist;
  ub4 dep,arr;
  ub4 vias[Nvia];
  ub4 viacnt;
  ub4 lostop,histop;
  ub4 stop;
  ub4 costlim;
};
typedef struct srcctx search;

extern void inisearch(void);
extern int searchgeo(search *src,char *ref,ub4 dep,ub4 arr,ub4 nstoplo,ub4 nstophi);
