// cfg.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

// search limits: practical

// max stops aka transfers supported within a single part
#define Nstop 8

// idem, inter-part. typically 3 * Nstop
#define Nxstop 20

// not yet
#define Nvia 16

#define Querycnt 256

#define Maxquerysize (1024 * 64)

// time in seconds to accept a query
#define Queryage 60

// end of limits

enum Engvars { Eng_periodlim,Eng_partsize,Eng_cnt };
enum Netvars {
  Net_partsize,
  Net_sumwalklimit,
  Net_walklimit,
  Net_walkspeed,
  Net_tpat0,
  Net_tpat1,
  Net_tpatmintt,
  Net_tpatmaxtt,
  Net_mintt,
  Net_maxtt,
  Net_cnt
};

sassert(Net_cnt < sizeof(globs.netvars),"globs.netvars < Net_cnt")
sassert(Net_cnt < Elemcnt(globs.netvars),"globs.netvars < Net_cnt")
sassert(Eng_cnt < sizeof(globs.engvars),"globs.engvars < Eng_cnt")

#define Cfgcl (1U << 31)
#define Cfgdef (1U << 30)

extern int readcfg(const char *name);
extern int inicfg(void);
extern int inicfgcl(void);
extern const char *runlvlnames(enum Runlvl lvl);
