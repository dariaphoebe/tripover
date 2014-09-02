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

#include "base.h"

static ub4 msgfile;
#include "msg.h"

#include "util.h"
#include "time.h"

int str2ub4(const char *s, ub4 *pv)
{
  unsigned long n;
  char *ep;

  *pv = 0;
  n = strtoul(s,&ep,0);
  if (ep == s) return 1;
  if (n > hi32) *pv = hi32;
  else *pv = (ub4)n;
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
  char *eq,*valp,cnv;
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

  argno = 1;

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
    vp = ap->val;
    if (vp) {
      cv.valcnt = 1;
      vlen = (ub4)strlen(vp);
      if (vlen > 2 && vp[vlen-2] == '%') {
        cnv = vp[vlen-1];
      } else cnv = 's';
      if (*vp != '[' && !valp) { warning(User,"%smissing value",msg); continue; }
      if (cnv == 'u') {
        if (str2ub4(valp,&cv.uval)) { warning(User,"%signoring non-integer value %s",msg,valp); continue; }
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

void iniutil(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}
