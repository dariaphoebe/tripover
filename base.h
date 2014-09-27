// base.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

// to be determined what goes in here

typedef unsigned char ub1;
typedef unsigned char bool;
typedef unsigned short ub2;
typedef unsigned int ub4;
typedef unsigned long ub8;

typedef short sb2;
typedef int sb4;

#define Version_maj 0
#define Version_min 1
#define Version_phase "initial"

// handful of useful macros
#define hi16 0xffff
#define hi32 0xffffffff

#define Elemcnt(a) (sizeof(a) / sizeof(*a))

#ifndef max
 #define max(a,b) ((a) > (b) ? (a) : (b))
#endif
#ifndef min
 #define min(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef NULL
 #define NULL (void*)0
#endif

#define clear(p) memset((p),0,sizeof (*p) )
#define aclear(p) memset((p),0,sizeof (p) )
#define oclear(p) memset(&(p),0,sizeof (p) )

#define strcopy(dst,src) strncpy((dst),(src),sizeof (dst) )

// program-wide global vars go here
extern struct globs {
  char *progname;
  ub2 nosteps;
  ub2 dorandnet;
  ub2 doreadnet;
  ub2 doinit;
  ub2 doserver;
  char cfgfile[1024];
  char netfile[1024];
  char netdir[1024];
  char querydir[256];
  ub4 serverid;
  ub4 msglvl;
  ub4 vrblvl;
  ub4 maxports;
  ub4 maxhops;
  ub4 maxstops;

  ub4 writext;
  ub4 extdec;

  ub4 testa,testb;
  ub4 testcnt;
  ub4 testset[16];
} globs;

struct myfile {
  int exist,direxist,alloced;
  ub4 basename;
  size_t len;
  unsigned long mtime;
  char name[256];
  char localbuf[1024];
  char *buf;
};

// 360 at 40k = 111 km / lon
#define Latscale 100000
#define Lonscale 100000
