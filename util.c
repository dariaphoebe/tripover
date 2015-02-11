// util.c - generic utility functions

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* This file contains utiity functions that do not deserve their own place

 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "base.h"
#include "os.h"

static ub4 msgfile;
#include "msg.h"

#include "mem.h"
#include "cfg.h"
#include "util.h"
#include "time.h"

static const char logitemfile[] = "watches.cfg";

ub4 str2ub4(const char *s, ub4 *pv)
{
  unsigned long n;

  char *ep;

  *pv = 0;
  if (!s || !*s) return 0;
  n = strtoul(s,&ep,0);
  if (ep == s) return 0;
  if (n > hi32) *pv = hi32;
  else *pv = (ub4)n;
  return (ub4)(ep - s);
}

int hex2ub4(const char *s, ub4 *pv)
{
  unsigned long n;
  char *ep;

  *pv = 0;
  if (!s || !*s) return 1;
  n = strtoul(s,&ep,16);
  if (ep == s) return 1;
  if (n > hi32) *pv = hi32;
  else *pv = (ub4)n;
  return 0;
}

void memcopyfln(void *dst,const void *src,size_t len,ub4 fln)
{
  char *d = dst;
  const char *s = src;

  if (len == 0) infofln(fln,0,"zero copy");
  else if (s < d && s + len > d) errorfln(fln,Exit,FLN,"overlapping copy: %p %p %lu",src,dst,(unsigned long)len);
  else if (s > d && d + len > s) errorfln(fln,Exit,FLN,"overlapping copy: %p %p %lu",src,dst,(unsigned long)len);
  else memcpy(dst,src,len);
}

void do_clear(void *p,size_t len) { memset(p,0,len); }

int strcompfln(const char *a,const char *b,const char *sa,const char *sb,ub4 fln)
{
  vrbfln(fln,0,"cmp %s %s",sa,sb);
  if (a == NULL) return errorfln(fln,Exit,FLN,"strcmp(%s:nil,%s",sa,sb);
  else if (b == NULL) return errorfln(fln,Exit,FLN,"strcmp(%s,%s:nil",sa,sb);
  else return strcmp(a,b);
}

int filecreate(const char *name,int mustsucceed)
{
  int fd = oscreate(name);
  if (fd == -1) {
    if (mustsucceed) oserror(0,"cannot create %s",name);
    else osinfo(0,"not creating %s",name);
  }
  return fd;
}

int fileopen(const char *name,int mustexist)
{
  int fd = osopen(name);
  if (fd == -1) {
    if (mustexist) oserror(0,"cannot open %s",name);
    else osinfo(0,"not reading %s",name);
  }
  return fd;
}

int filewrite(int fd, const void *buf,ub4 len,const char *name)
{
  long n;

  if (len == 0) return error(0,"nil write to %s",name);
  n = oswrite(fd,buf,len);
  if (n == -1) return oserror(0,"cannot write \ah%u bytes to %s",len,name);
  else if (n != (long)len) return error(0,"partial write \ah%ld of \ah%u bytes to %s",n,len,name);
  else return 0;
}

int fileread(int fd,void *buf,ub4 len,const char *name)
{
  long n;

  if (len == 0) return error(0,"nil read from %s",name);

  n = osread(fd,buf,len);

  if (n == -1) return oserror(0,"cannot read from %s",name);
  else if (n  == 0) return error(0,"eof on reading \ah%u bytes from %s",len,name);
  else if (n != (long)len) return error(0,"partial read \ah%ld of \ah%u bytes from %s",n,len,name);
  else return 0;
}

int fileclose(int fd,const char *name)
{
  int rv = osclose(fd);
  if (rv) oserror(0,"cannot close %s",name);
  return rv;
}

int dorun(ub4 fln,enum Runlvl stage)
{
  int run;

  vrb(0,"dorun stage %u stopat %u",stage, globs.stopat);
  if (stage >= globs.stopat) run = 0;
  else if (stage >= Runcnt) run = 1;
  else run = globs.doruns[stage];

  info0(User,"");
  infofln(fln,0,"--- %s stage %u %s ---",run ? "run" : "skip",stage,runlvlnames(stage));
  return run;
}

// adapted from cs.clackamas.cc.or.us/molatore/cs260Spr03/combsort.htm
static ub4 combsort8(ub8 *p,ub4 n)
{
  ub4 iter = 0;
  ub4 gap = n;
  ub8 v0,v1;
  int swap;
  static ub4 gaps[7] = {11,8,6,4,3,2,1};
  ub4 i,gi = 0;

  do { // gapped stage
    if (gap > 14) gap = gap * 10 / 13;
    else gap = gaps[gi++];
    iter++;
    for (i = 0; i + gap < n; i++) {
      v0 = p[i]; v1 = p[i+gap];
      if (v0 > v1) { p[i] = v1; p[i+gap] = v0; }
    }
  } while (gap > 1);

  do { // final bubble stage
    swap = 0;
    iter++;
    for (i = 1; i < n; i++) {
      v0 = p[i-1]; v1 = p[i];
      if (v0 > v1) {
        p[i-1] = v1; p[i] = v0;
        swap = 1;
      }
    }
  } while (swap);
  return iter;
}

ub4 sort8(ub8 *p,ub4 n,ub4 fln,const char *desc)
{
  ub4 i,rv = 1;
  ub8 v;

  switch (n) {  // trivia
  case 0: return warnfln(fln,0,"sort of nil items for %s",desc);
  case 1: return 0;
  case 2: v = p[0]; if (v > p[1]) { p[0] = p[1]; p[1] = v; } return 1;
  };

  for (i = 1; i < n; i++) if (p[i] < p[i-1]) break;
  if (i == n) { vrbfln(fln,0,"csort of %u started in order",n); return 0; }

  for (i = 1; i < n; i++) if (p[i] > p[i-1]) break;
  if (i == n) {
    infofln(fln,Iter,"csort of %u started in reverse order",n);
    for (i = 0; i < n / 2; i++) {
      v = p[i]; p[i] = p[n-i-1]; p[n-i-1] = v;
    }
  } else rv = combsort8(p,n);

  for (i = 1; i < n; i++) if (p[i] < p[i-1]) break;
  if (i < n) {
    warnfln(fln,0,"csort of %u not in order in %u runs",n,rv);
    for (i = 0; i < n; i++) info(0,"%u %u+%u",i,(ub4)(p[i] >> 32),(ub4)(p[i] & hi32));
    error_z(0,0);
  }
  return rv;
}

// interpolation search, after en.wikipedia.org/wiki/Interpolation_search
ub4 isearch4(ub4 *p,ub4 n,ub4 key,ub4 fln,const char *desc)
{
  ub4 lo,hi,mid,i,di;
  ub4 v,dv,kv;

  switch (n) {  // trivia
  case 0: return warnfln(fln,0,"search in nil items for %s",desc);
  case 1: return *p == key ? 0 : 1;
  case 2: if (p[0] == key) return 0;
          else if (p[1] == key) return 1;
          else return 2;
  case 3: if (p[0] == key) return 0;
          else if (p[1] == key) return 1;
          else if (p[2] == key) return 2;
          else return 3;
  case 4: case 5: case 6: case 7: case 8: case 9: case 10:
          i = 0; while (i < n) if (p[i] == key) return i;
          return n;
  };

  lo = 0; hi = n - 1;
  dv = p[hi] - p[lo];
  di = hi - lo;
  kv = key - p[lo];

  while (dv > 10 && di > 10) {
    mid = lo + (kv * di) / dv;
    v = p[mid];
    if (v < key) {
      hi = mid - 1;
      kv = key - p[lo];
    } else if (v == key) return mid;
    else {
      lo = mid + 1;
      kv = p[hi] - key;
    }
    di = hi - lo;
    dv = p[hi] - p[lo];
  }

  while (lo < hi) if (p[lo++] == key) return lo;
  return n;
}

int readfile(struct myfile *mf,const char *name, int mustexist)
{
  int fd;
  char *buf;
  size_t len;
  long nr;

  mf->len = 0;
  mf->buf = mf->localbuf;

  error_zp(name,0);
  if (*name == 0) return errorfln(FLN,0,0,"empty filename %p passed to readfile",name);
  fd = osopen(name);
  if (fd == -1) {
    if (mustexist) return oserror(0,"cannot open %s",name);
    else return info(0,"optional %s is not present",name);
  }
  if (osfdinfo(mf,fd)) { oserror(0,"cannot get info for %s",name); osclose(fd); return 1; }
  mf->exist = 1;
  len = mf->len;
  if (len == 0) { osclose(fd); return info(0,"%s is empty",name); }
  if (len < sizeof(mf->localbuf)) buf = mf->localbuf;
  else {
    buf = alloc((ub4)len,char,0,name,0);
    mf->alloced = 1;
  }
  mf->buf = buf;
  nr = osread(fd,buf,len);
  if (nr == -1) { oserror(0,"cannot read %s",name); osclose(fd); return 1; }
  osclose(fd);
  if (nr != (long)len) return error(0,"partial read of %s",name);
  return 0;
}

// write simple 2D pixmap
int writeppm(const char *name,ub4 *data,ub4 nx,ub4 ny)
{
  int fd;
  ub4 v0,v1,dv,v,x,y,xy,pos,mval;
  char buf[8192];

  v0 = hi32;
  v1 = 0;
  for (xy = 0; xy < nx * ny; xy++) {
    v0 = min(v0,data[xy]);
    v1 = max(v1,data[xy]);
  }
  dv = v1 - v0;
  info(0,"ppm: lo %u hi \ah%u range %u",v0,v1,dv);
  fd = oscreate(name);
  if (fd == -1) return oserror(0,"cannot create ppm file %s",name);

  pos = fmtstring(buf,"P3\n%u %u\n255\n",nx,ny);
  oswrite(fd,buf,pos);

  pos = 0;
  for (y = 0; y < ny; y++) {
    for (x = 0; x < nx; x++) {
      mval = data[y * ny + x];
      if (dv) v = (mval - v0) * 256 / dv;
      else v = 0;
      pos += mysnprintf(buf,pos,sizeof(buf),"%u %u %u ",v,v,v);
      if (pos >= 4096) { oswrite(fd,buf,pos); pos = 0; }
    }
    pos += mysnprintf(buf,pos,sizeof(buf),"\n");
  }
  if (pos) oswrite(fd,buf,pos);
  osclose(fd);

  return 0;
}

static const char license[] =
"This work is licensed under the Creative Commons\n\
Attribution-NonCommercial-NoDerivatives 4.0 International License.\n\
To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/.\n\
";

static int showlicense(struct cmdval *cv)
{
  info(User,"%s",license);
  return cv->retval;
}

static int showvers(struct cmdval *cv)
{
  char ccstr[256];
  ub4 ccmaj,ccmin;
  char nowstr[64];
  const char *tz;

#ifdef NOW
  sec70toyymmdd(NOW,nowstr,sizeof(nowstr));
  tz = "utc";
#else
  strcopy(nowstr,__DATE__);
  tz = "localtime";
#endif

  info(User,"tripover server version %u.%u  %s %s", Version_maj,Version_min,nowstr,tz);

#if defined(__clang__) && defined(__clang_major__) && defined(__clang_minor__)
  ccmaj = __clang_major__; ccmin = __clang_minor__;
  fmtstring(ccstr,"clang %u.%u",ccmaj,ccmin);
#elif defined(__GNUC__) && defined(__GNUC_MINOR__)
  ccmaj = __GNUC__; ccmin = __GNUC_MINOR__;
  fmtstring(ccstr,"gcc %u.%u",ccmaj,ccmin);
#else
  return cv->retval;
#endif

  info(User,"compiled using %s",ccstr);
  return cv->retval;
}

int shortusage(void)
{
  info(User,"usage: %s [options] <cmd> <args>\n",globs.progname);
  info(User,"'%s help' shows commandline usage",globs.progname);
  return 1;
}

static int usage(struct cmdval *cv)
{
  struct cmdarg *ap = cv->args;
  const char *arg,*desc,*val,*type,*alt;
  char cnv;
  ub4 vlen;
  char valstr[64];
  char argstr[64];
  char optstr[128];

  info(User,"%s - %s",cv->progname,cv->progdesc);
  info(User,"usage: %s [options] %s\n",globs.progname,cv->valname);

  info0(User,"options:");

  while (ap->arg) {
    arg = ap->arg;
    val = ap->val;
    desc = ap->desc;
    ap++;
    if (*arg == '.') continue;
    alt = strchr(arg,'|');
    if (alt) {
      aclear(argstr);
      memcpy(argstr,arg,alt - arg);
      arg = argstr;
    }
    if (!val) {
      info(User|Ind|2,"-%-25s%s",arg,desc);
    } else {
      vlen = (ub4)strlen(val);
      if (vlen > 2 && val[vlen-2] == '%') {
        cnv = val[vlen-1];
        vlen -= 2;
      } else cnv = 's';
      switch (cnv) {
      case 's': type = "string"; break;
      case 'u': type = "integer"; break;
      default: type = "";
      }
      aclear(valstr);
      aclear(optstr);
      if (vlen > 1 && *val == '[') {
        strncpy(valstr,val+1,vlen-2);
        fmtstring(optstr,"-%s=%s",arg,valstr);
        info(User|Ind|2,"%-26s%s (%s,opt)",optstr,desc,type);
      } else {
        strncpy(valstr,val,vlen);
        fmtstring(optstr,"-%s=%s",arg,valstr);
        info(User|Ind|2,"%-26s%s (%s,req)",optstr,desc,type);
      }
    }
    if (alt) {
      info(User|Ind|3,"-%s",alt+1);
    }
  }
  info0(User," ");
  return 1;
}

// common options for programs
static struct cmdarg allargs[64] = {
  { "help|h|?", NULL, "show usage", usage },
  { "version|V", NULL, "show version", showvers },
  { "license|L", NULL, "show license", showlicense }
};

static int streq(const char *s,const char *q) { return !strcmp(s,q); }
static int memeq(const char *s,const char *q,ub4 n) { return !memcmp(s,q,n); }

static struct cmdarg *findarg(const char *arg,struct cmdarg *cap)
{
  const char *a,*b;

  while (cap->arg) {
    a = cap->arg;
    while (a) {
      b = strchr(a,'|');
      if (b) {
        if (memeq(arg,a,(ub4)(b - a))) return cap;
        a = b + 1;
      } else {
        if (*a == '.') a++;
        if (streq(arg,a)) return cap;
        else break;
      }
    }
    cap++;
  }
  return cap;
}

int cmdline(int argc, char *argv[], struct cmdarg *cmdargs,const char *desc)
{
  char *eq,*valp,*sub,cnv;
  const char *arg,*vp;
  ub4 argno,vlen;
  struct cmdarg *plainap,*cap = cmdargs,*ap = allargs;
  struct cmdval cv;
  char msg[256];
  int rv;

  if (!argc) return 0;

  oclear(cv);

  while(ap->arg) ap++;
  while(cap->arg) {
    ap->arg = cap->arg;
    ap->val = cap->val;
    ap->desc = cap->desc;
    ap->fn = cap->fn;
    cap++; ap++;
  }
  plainap = cap;

  for (argno = 1; argno < (ub4)argc; argno++) {
    arg = argv[argno];
    if (!arg || !*arg) continue;

    fmtstring(msg,"arg %u '%s' : ", argno,arg);
    valp = NULL;
    if (*arg == '-') {
      arg++;
      if (*arg == '-') arg++;
      if (*arg == 0) { warning(User,"%signoring empty option after '%s'",msg,argv[argno-1]); continue; }

      eq = strchr(arg,'=');
      if (eq == arg) { warning(User,"%signoring malformed arg",msg); continue; }
      else if (eq) {
        *eq = 0;
        valp = eq + 1;
      }

      ap = findarg(arg,allargs);
      if (!ap->arg) {
        warning(User,"%signoring unknown argument",msg);
        continue;
      }
      oclear(cv);
      cv.retval = 1;
      sub = strchr(ap->arg,'-');
      if (sub) cv.subarg = sub + 1;
      else cv.subarg = ap->arg;
      vp = ap->val;
      if (vp) {
        cv.valcnt = 1;
        vlen = (ub4)strlen(vp);
        if (vlen > 2 && vp[vlen-2] == '%') {
          cnv = vp[vlen-1];
        } else cnv = 's';
        if (*vp != '[' && !valp) { warning(User,"%smissing value",msg); continue; }
        if (cnv == 'u') {
          if (str2ub4(valp,&cv.uval) == 0) {
          if (*vp != '[' || valp)
            warning(User,"%signoring non-integer value %s",msg,valp); continue;
          }
        }
        cv.sval = valp;
      } else {
        if (valp) warning(User,"%signoring value for arg",msg);
      }
      cv.args = allargs;
      cv.argndx = (ub4)(ap - allargs);
      cv.valname = cap->val;
      cv.progname = cap->desc;
      cv.progdesc = desc;
      if (ap->fn) {
        rv = (*ap->fn)(&cv); 
        if (rv) return rv;
      }
    } else {
      cv.args = allargs;
      cv.valname = cap->val;
      cv.progname = cap->desc;
      cv.progdesc = desc;
      if (streq(arg,"help")) return usage(&cv);
      else {
        cv.sval = (char *)arg;
        (*plainap->fn)(&cv);
      }
    }
  }
  return 0;
}

static struct logitem {
  char name[16];
  ub4 nlen;
  ub8 val;
  ub8 lvl;
  ub4 fln;
  int hex;
} logitems[64];

int getwatchitems(const char *name,ub8 *list,ub4 listlen)
{
  ub4 len = (ub4)strlen(name);
  ub4 n = 0;
  ub8 x;
  struct logitem *li;

  for (li = logitems; li < logitems + Elemcnt(logitems); li++) {
    if (li->nlen != len || memcmp(name,li->name,len)) continue;
    if (n >= listlen) break;
    x = li->val | (li->lvl << 32);
    info(0,"watching %s %lu at lvl %u",name,li->val,(ub4)li->lvl);
    list[n++] = x;
  }
  return n;
}

void addwatchitem(const char *name,ub4 val,ub4 fln,int hex)
{
  ub4 len = (ub4)strlen(name);
  struct logitem *li;

  for (li = logitems; li < logitems + Elemcnt(logitems); li++) {
    if (li->nlen == len && memeq(name,li->name,len)) return;
    else if (li->nlen == 0) {
      memcpy(li->name,name,len);
      li->nlen = len;
      li->val = val;
      li->fln = fln;
      li->hex = hex;
      return;
    }
  }
}

static void wrwatchitems(void)
{
  struct logitem *li;
  int fd;
  ub4 n,fln;
  char buf[256];
  char flnbuf[64];

  for (li = logitems; li < logitems + Elemcnt(logitems); li++) if (li->fln) break;
  if (li == logitems + Elemcnt(logitems)) return;

  fd = oscreate(logitemfile);
  if (fd == -1) return;

  for (li = logitems; li < logitems + Elemcnt(logitems); li++) {
    if (li->nlen == 0) continue;
    fln = li->fln;
    if (fln) fmtstring(flnbuf," # %u.%u",fln >> 16,fln & hi16);
    else *flnbuf = 0;
    if (li->hex) n = fmtstring(buf,"%s x%x%s\n",li->name,(ub4)li->val,flnbuf);
    else n = fmtstring(buf,"%s ,%u%s\n",li->name,(ub4)li->val,flnbuf);
    oswrite(fd,buf,n);
  }
  osclose(fd);
}

static void rdwatchitems(void)
{
  struct myfile mf;
  enum states {Out,Name,Val0,Hex,Dec,Lvl,Fls} state;
  ub4 pos,len,nlen,val,lvl,item = 0;
  struct logitem *li = logitems;
  char *buf,c;
  char name[16];

  if (readfile(&mf,logitemfile,0)) return;
  len = (ub4)mf.len;
  if (len == 0) return;

  buf = mf.buf;
  pos = nlen = val = 0;

  state = Out;
  while (pos < len && item < Elemcnt(logitems)) {
    c = buf[pos++];

    switch(state) {
    case Out:
      if (c == '#') state = Fls;
      else if (c >= 'A' && c <= 'z') { name[0] = c; nlen = 1; state = Name; }
      else if (c != '\n') state = Fls;
    break;

    case Name:
      if (c == ' ') state = Val0;
      else if (c >= 'A' && c <= 'z' && nlen < Elemcnt(name)) name[nlen++] = c;
      else if (c == '\n') state = Out;
      else state = Fls;
    break;

    case Val0:
      memcpy(li->name,name,nlen);
      li->nlen = nlen;
      if (c == ',') { val = 0; state = Dec; }
      else if (c == 'x') { val = 0; state = Hex; }
      else if (c == '*') {
        li->val = hi32;
        li++; item++;
        state = Fls;
      } else state = Fls;
    break;

    case Dec:
      if (c >= '0' && c <= '9') val = val * 10 + c - '0';
      else if (c == '\n') {
        li->val = val;
        li++; item++;
        state = Out;
      } else if (c == ' ') state = Lvl;
      else state = Fls;
    break;

    case Hex:
      if (c >= '0' && c <= '9') val = (val << 4) | (c - '0');
      else if (c >= 'a' && c <= 'f') val = (val << 4) | (c - 'a' + 10);
      else if (c == '\n') {
        li->val = val;
        li++; item++;
        state = Out;
      } else if (c == ' ') state = Lvl;
      else state = Fls;
    break;

    case Lvl:
      if (c >= '0' && c <= '9') {
        lvl = c - '0';
        li->lvl = lvl;
        state = Fls;
      } else if (c == '\n') state = Out;
      else state = Fls;
      li++; item++;
    break;

    case Fls: if (c == '\n') state = Out; break;

    }
  }
}

int iniutil(int pass)
{
  static int passed = 0;

  if (pass == 0 && passed == 0) {
    msgfile = setmsgfile(__FILE__);
    iniassert();

    memset(globs.doruns,1,Runcnt);
    passed = 1;
    return 0;
  } else if (pass == 1 && passed == 1) {
    rdwatchitems();
    passed = 2;
  }
  return 0;
}

void exiutil(void)
{
  wrwatchitems();
}
