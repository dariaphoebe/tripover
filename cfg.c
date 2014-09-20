// cfg.c - process configuration file

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* process config file
 */

#include <string.h>

#include "base.h"
#include "mem.h"
#include "util.h"

static ub4 msgfile;
#include "msg.h"

#include "os.h"
#include "cfg.h"

void inicfg(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

#define Varlen 64
#define Vallen 256

static ub4 linno;
static const char *cfgname;

enum Cfgvar { Maxhops,Maxports,Net2pdf,Cfgcnt };
enum Cfgcnv { String, Uint, Bool,None };
static struct cfgvar {
  const char *name;
  enum Cfgcnv cnv;
  enum Cfgvar var;
} cfgvars[Cfgcnt] = {
  {"maxhops",Uint,Maxhops},
  {"maxports",Uint,Maxports},
  {"net.pdf",Bool,Net2pdf}
};

extern struct globs globs;

static int memeq(const char *s,const char *q,ub4 n) { return !memcmp(s,q,n); }

static int writecfg(void)
{
  struct cfgvar *vp = cfgvars;
  ub4 uval,pos;
  int fd;
  char buf[4096];
  const char *name;
  char *sval = (char *)"";

  fd = oscreate("tripover.curcfg");
  if (fd == -1) return 1;

  while (vp < cfgvars + Cfgcnt) {
    name = vp->name;

    uval = 0;
    switch(vp->var) {
    case Maxhops: uval = globs.maxhops; break;
    case Maxports: uval = globs.maxports; break;
    case Net2pdf: break;
    case Cfgcnt: break;
    }

    switch(vp->cnv) {
    case Uint: pos = fmtstring(buf,"%s %u\n",name,uval); break;
    case Bool: pos = fmtstring(buf,"%s %u\n",name,uval); break;
    case String: pos = fmtstring(buf,"%s %s\n",name,sval);break;
    case None: pos = fmtstring(buf,"%s\n",name);break;
    }
    oswrite(fd,buf,pos);
    vp++;
  }
  osclose(fd);
  return 0;
}

static int addvar(char *var,char *val,ub4 varlen,ub4 vallen)
{
  struct cfgvar *vp = cfgvars;
  ub4 n,uval = 0;
  const char *name;

  var[varlen] = 0;
  while (vp < cfgvars + Cfgcnt) {
    name = vp->name;
    n = (ub4)strlen(name);
    if (n == varlen && memeq(name,var,n)) break;
    vp++;
  }
  if (vp == cfgvars + Cfgcnt) return error(0,"%s.%u: unknown config var '%s'",cfgname,linno,var);

  if (vallen == 0) return error(0,"%s.%u: '%s' needs arg",cfgname,linno,var);

  switch(vp->cnv) {
  case Uint: val[vallen] = 0;
             if (str2ub4(val,&uval)) return error(0,"%s.%u: %s : '%s' needs numerical arg",cfgname,linno,var,val);
             break;
  case Bool: break;
  case String: break;
  case None: break;
  }

  switch(vp->var) {
  case Maxhops: globs.maxhops = uval; break;
  case Maxports: globs.maxports = uval; break;
  case Net2pdf: break;
  case Cfgcnt: break;
  }
  return 0;
}

int config(const char *name)
{
  struct myfile cfg;
  enum states { Out,Var,Val0,Val,Val9,Fls } state;
  ub4 pos,len,varlen,vallen;
  char var[Varlen];
  char val[Vallen];
  int rv = 0;
  char *buf,c;

  info(0,"check config in %s",name);
  rv = readfile(&cfg,name,0);
  if (rv) return 1;
  if (cfg.exist == 0) return 0;

  len = (ub4)cfg.len;

  if (len == 0) return 0;

  buf = cfg.buf;

  cfgname = name;
  state = Out;
  pos = 0;
  linno = 1;
  varlen = vallen = 0;
  while(pos < len && rv == 0) {
    c = buf[pos++];
    if (c == '\n') linno++;

    switch(state) {
    case Out:
      switch(c) {
      case '#': state = Fls; break;
      case '\n': break;
      case ' ': case '\t': break;
      default: var[0] = c; varlen = 1; state = Var;
      } break;

    case Var:
      if (varlen >= Varlen-1) return error(0,"%s.%u: variable name exceeds %u chars",cfgname,linno, Varlen);
      switch(c) {
      case '#': state = Val9; break;
      case '\n': rv = addvar(var,val,varlen,vallen); state = Out; break;
      case ' ': case '\t': case '=': state = Val0; break;
      default: var[varlen++] = c;
      } break;

    case Val0:
      switch(c) {
      case '#': state = Val9; break;
      case '\n': rv = addvar(var,val,varlen,vallen); state = Out; break;
      case ' ': case '\t': case '=': break;
      default: val[0] = c; vallen = 1; state = Val;
      } break;

    case Val:
      if (vallen >= Vallen-1) return error(0,"%s.%u: variable exceeds %u chars",cfgname,linno, Vallen);
      switch(c) {
      case '#': state = Val9; break;
      case '\n': rv = addvar(var,val,varlen,vallen); state = Out; break;
      case ' ': case '\t': state = Val9; break;
      default: val[vallen++] = c;
      } break;

    case Val9:
      if (c == '\n') state = Out;
      else state = Fls;
      rv = addvar(var,val,varlen,vallen);
      break;

    case Fls:
      if (c == '\n') state = Out;
      break;
    } // switch state

  } // each char
  switch(state) {
  case Var: case Val0: case Val: case Val9: rv = addvar(var,val,varlen,vallen); break;
  case Out: case Fls: break;
  }

  if (rv) return rv;
  writecfg();
  return 0;
}
