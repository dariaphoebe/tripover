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
#include "util.h"
#include "time.h"

int str2ub4(const char *s, ub4 *pv)
{
  unsigned long n;
  char *ep;

  *pv = 0;
  if (!s || !*s) return 1;
  n = strtoul(s,&ep,0);
  if (ep == s) return 1;
  if (n > hi32) *pv = hi32;
  else *pv = (ub4)n;
  return 0;
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

int readfile(struct myfile *mf,const char *name, int mustexist)
{
  int fd = osopen(name);
  char *buf;
  size_t len;
  ssize_t nr;

  clear(mf);
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
  if (nr != (ssize_t)len) return error(0,"partial read of %s",name);
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

#ifdef NOW
  sec70toyymmdd(NOW,nowstr,sizeof(nowstr));
#else
  strcopy(nowstr,__DATE__);
#endif

  info(User,"tripover server version %u.%u  %s utc", Version_maj,Version_min,nowstr);

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

static int usage(struct cmdval *cv)
{
  struct cmdarg *ap = cv->args;
  const char *arg,*desc,*val,*type,*alt;
  char cnv;
  ub4 vlen;
  char valstr[64];
  char argstr[64];
  char optstr[128];

  info0(User,"tripover server - flight+train search engine");
  info0(User,"usage: tripover [options] [args]\n");

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

int cmdline(int argc, char *argv[], struct cmdarg *cmdargs)
{
  char *eq,*valp,*sub,cnv;
  const char *arg,*vp;
  ub4 argno,vlen;
  struct cmdarg *cap = cmdargs,*ap = allargs;
  struct cmdval cv;
  char msg[256];
  int rv;

  if (!argc) return 0;

  while(ap->arg) ap++;
  while(cap->arg) {
    ap->arg = cap->arg;
    ap->val = cap->val;
    ap->desc = cap->desc;
    ap->fn = cap->fn;
    cap++; ap++;
  }

  globs.progname = argv[0];

  for (argno = 1; argno < (ub4)argc; argno++) {
    arg = argv[argno];
    if (!arg || !*arg) continue;

    fmtstring(msg,"arg %u '%s' : ", argno,arg);
    if (*arg == '-') {
      arg++;
      if (*arg == '-') arg++;
      if (*arg == 0) { warning(User,"%signoring empty option after '%s'",msg,argv[argno-1]); continue; }
    }

    eq = strchr(arg,'=');
    if (eq == arg) { warning(User,"%signoring malformed arg",msg); continue; }
    else if (eq) {
      *eq = 0;
      valp = eq + 1;
    } else valp = NULL;

    ap = findarg(arg,allargs);
    if (!ap->arg) {
      warning(User,"%signoring unknown argument",msg);
      continue;
    }
    clear(&cv);
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
        if (str2ub4(valp,&cv.uval)) {
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
    if (ap->fn) {
      rv = (*ap->fn)(&cv); 
      if (rv) return rv;
    }
  }
  return 0;
}

int iniutil(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();

  return 0;
}
