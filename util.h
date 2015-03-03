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
  const char *valname;
  const char *subarg;
  struct cmdarg *args;
  ub4 argndx;
  int retval;
  const char *progname;
  const char *progdesc;
};

struct cmdarg { // commandline arg defines
  const char *arg;
  const char *val;
  const char *desc;
  int (*fn)(struct cmdval *cp);
};

extern int filecreate(const char *name,int mustsucceed);
extern int fileopen(const char *name,int mustexist);
extern int filewrite(int fd, const void *buf,ub4 len,const char *name);
extern int fileread(int fd,void *buf,ub4 len,const char *name);
extern int fileclose(int fd,const char *name);

extern int readfile(struct myfile *mf,const char *name, int mustexist,ub4 maxlen);
extern int readpath(struct myfile *mf,const char *dir,const char *name, int mustexist,ub4 maxlen);
extern int freefile(struct myfile *mf);

extern ub4 sort8(ub8 *p,ub4 n,ub4 fln,const char *desc);
extern ub4 isearch4(ub4 *p,ub4 n,ub4 key,ub4 fln,const char *desc);

extern int cmdline(int argc, char *argv[], struct cmdarg *cap,const char *desc);
extern int shortusage(void);

#define memcopy(d,s,n) { sassert(sizeof(d) == sizeof(s),"pointer size mismatch") memcopyfln((d),(s),(n),FLN); }
extern void memcopyfln(void *dst,const void *src,size_t len,ub4 fln);

#define strcomp(a,b) strcompfln((a),(b),#a,#b,FLN)
extern int strcompfln(const char *a,const char *b,const char *sa,const char *sb,ub4 fln);

extern ub4 str2ub4(const char *s, ub4 *pv);
extern int hex2ub4(const char *s, ub4 *pv);

#define oclear(p) do_clear(&(p),sizeof(p))
extern void do_clear(void *p,size_t len);

#define limit(a,b,c) if ((a) > (b)) { (a) = (b); warningfln(FLN,0,"limit %s:%u to %s:%u for %s:%u",#a,(a),#b,(b),#c,(c)); }

#define m2geo(m) ((m) / (1000 / Geoscale))
#define geo2m(g) ((g) * (1000 / Geoscale))

extern int dorun(ub4 fln,enum Runlvl stage,bool silent);

extern int getwatchitems(const char *name,ub8 *list,ub4 listlen);
extern void addwatchitem(const char *name,ub4 val,ub4 fln,int hex);

extern int writeppm(const char *name,ub4 *data,ub4 nx,ub4 ny);

extern int iniutil(int pass);
extern void exiutil(void);
