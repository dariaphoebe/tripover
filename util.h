// util.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

struct cmdval { // matched commandline arg
  ub4 uval;
  ub4 valcnt;
  char *sval;
  const char *subarg;
  struct cmdarg *args;
  ub4 argndx;
  int retval;
};

struct cmdarg { // commandline arg defines
  const char *arg;
  const char *val;
  const char *desc;
  int (*fn)(struct cmdval *cp);
};

// global name.
struct gname {
  spool *strpool;  // vars below index here 
  ub4 name;        // common name
  ub4 code;        // formal coded name, e.g. IATA
  ub4 locname;     // name in local language
  ub4 intname;     // international name
  ub4 descr;
};

extern int filecreate(const char *name);
extern int filewrite(int fd, const void *buf,ub4 len,const char *name);
extern int fileread(int fd,void *buf,ub4 len,const char *name);
extern int fileclose(int fd,const char *name);

extern int readfile(struct myfile *mf,const char *name, int mustexist);

extern int cmdline(int argc, char *argv[], struct cmdarg *cap);

#define memcopy(d,s,n) memcopyfln((d),(s),(n),FLN);
extern void memcopyfln(char *dst,const char *src,ub4 len,ub4 fln);

extern int str2ub4(const char *s, ub4 *pv);
extern int hex2ub4(const char *s, ub4 *pv);

extern int dorun(enum Runlvl stage);

extern int writeppm(const char *name,ub4 *data,ub4 nx,ub4 ny);

extern int iniutil(void);
