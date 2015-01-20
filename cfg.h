// cfg.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

// compile-time limits: architectural
#define Hopcnt (1024 * 1024)

// compile-time limits: practical
#define Portcnt (1024 * 64 - 2)

// search limits: practical

// max stops aka transfers supported within a single part
#define Nstop 8

// idem, inter-part. typically 3 * Nstop
#define Nxstop 20

#define Nvia 16

#define Querycnt 256
#define Maxquerysize (1024 * 64)
#define Queryage 60

// end of limits

#define Cfgcl (1U << 31)
#define Cfgdef (1U << 30)

extern int readcfg(const char *name);
extern int inicfg(void);
extern int inicfgcl(void);
extern const char *runlvlnames(enum Runlvl lvl);
