// base.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

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
#define Version_min 17
#define Version_phase "developing"
#define Program_name "tripover"
#define Program_desc "broad-search journey planner"

// handful of useful macros
#define hi16 0xffff
#define hi24 0xffffff
#define hi32 0xffffffff
#define hi64 0xffffffffffffffff

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

#define clear(p) memset((p),0,sizeof(*(p)))
#define nclear(p,n) memset((p),0,(n) * sizeof(*(p)))
#define nsethi(p,n) memset((p),0xff,(n) * sizeof(*(p)))

// c11 langage only
#if defined  __STDC_VERSION__ && __STDC_VERSION__ >= 201101
 #define sassert(expr,msg) _Static_assert(expr,msg)

 #define aclear(p) { _Static_assert(sizeof(p) > 8,"need array, not pointer"); memset((p),0,sizeof(p)); }
 #define strcopy(dst,src) { _Static_assert(sizeof(dst) > 8,"need array, not pointer"); strncpy((dst),(src),sizeof(dst)-1); }
#else
 #define sassert(expr,msg)
 #define aclear(p) memset((p),0,sizeof(p));
 #define strcopy(dst,src) strncpy((dst),(src),sizeof(dst)-1 );
#endif

enum Runlvl { Runread,Runbaseprep,Runprep,Runcompound,Runpart,Runmknet,Runnet0,Runnetn,Runserver,Runend,Runcnt };

// program-wide global vars go here
extern struct globs {
  const char *progname;

  ub1 doruns[Runcnt];
  enum Runlvl stopat,stopatcl;
  char stopatstr[64];

  ub4 argc;
  char args[16][1024];

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

  ub4 maxvm;

  int netok;

  ub4 writext;
  ub4 writpdf;
  ub4 writgtfs;
  ub4 extdec;

  int msg_fd;
  char logname[256];
  int pid;
  int sigint,sig;

  ub4 limassert;
  ub4 testa,testb;
  ub4 testcnt;
  ub4 testset[16];

  ub4 netvars[64];   // checked for Net_cnt in cfg
  ub4 engvars[64];   // checked for Eng_cnt in cfg

  int nomsgsum;

} globs;

struct myfile {
  int exist,direxist,alloced;
  int isdir,isfile;
  ub4 basename;
  size_t len;
  unsigned long mtime;
  char name[1024];
  char localbuf[4096];
  char *buf;
};

#define NVALGRIND
#ifdef NVALGRIND
 #define vg_set_undef(p,n)
 #define vg_chk_def(p,n) 0
#else
 #include <valgrind/memcheck.h>
 #define vg_set_undef(p,n) VALGRIND_MAKE_MEM_UNDEFINED((p),(n))
 #define vg_chk_def(p,n) VALGRIND_CHECK_MEM_IS_DEFINED((p),(n))
#endif
