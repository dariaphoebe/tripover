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

#include "cfg.h"
#include "time.h"

#define Varlen 64
#define Vallen 256

static ub4 linno,colno;
static const char *cfgname;

static const char *lvlnames[Runcnt+1];
static ub4 setruns[Runcnt]; // line number where set

const char *runlvlnames(enum Runlvl lvl)
{
  return lvlnames[min(lvl,Runcnt)];
}

static int streq(const char *s,const char *q) { return !strcmp(s,q); }

int inicfgcl(void)
{
  enum Runlvl l,lvl = 0;
  ub4 pos,v;
  char buf[1024];
  char *stagename = globs.stopatstr;

  if (*stagename) {
    if (str2ub4(stagename,&v)) lvl = v;
    else { while (lvl < Runcnt && strcomp(globs.stopatstr,lvlnames[lvl])) lvl++; }

    if (lvl >= Runcnt) {
      pos = fmtstring(buf,"%u known stages:\n   ", Runcnt);
      for (l = 0; l < Runcnt; l++) pos += mysnprintf(buf,pos,sizeof(buf),"%s ",lvlnames[l]);
      info0(0,buf);
      return error(0,"unknown stage for cmdline '-stopat=%s'",globs.stopatstr);
    }
    info(0,"stop at stage %s:%u (commandline)",lvlnames[lvl],lvl);
    globs.stopatcl = lvl;
  } else globs.stopatcl = Runcnt;
  return 0;
}

enum Cfgvar {
  Maxhops,Maxports,Maxstops,
  Maxvm,
  Querydir,
  Stopat,Enable,Disable,
  Net_gen,
  Net2pdf,Net2ext,
  Section,
  Eng_gen,
  Eng_opt,
  Cfgcnt
 };

enum Cfgcnv { String,Uint,EnumRunlvl,Bool,None };

static struct cfgvar {
  const char *name;
  enum Cfgcnv cnv;
  enum Cfgvar var;
  ub4 subvar;
  ub4 lo,hi,def;
  const char *desc;
} cfgvars[] = {

  {"limits",Bool,Section,0,0,0,0,"limits"},

  {"maxhops",Uint,Maxhops,0,0,1024 * 1024,1024 * 1024,"maximum number of hops"},
  {"maxports",Uint,Maxports,0,0,1024 * 1024,1024 * 1024,"maximum number of ports"},
  {"maxstops",Uint,Maxstops,0,0,Nstop-1,1,"max transfers to precompute"},

  {"maxvm",Uint,Maxvm,0,1,hi24,hi24,"virtual memory limit in GB"},

  {"run",Bool,Section,0,0,0,0,"determines which stages to run"},

  {"stopat",EnumRunlvl,Stopat,0,0,Runcnt,Runcnt,"stop at given stage"},
  {"enable",EnumRunlvl,Enable,0,0,1,1,"enable a given stage"},
  {"disable",EnumRunlvl,Disable,0,0,1,1,"disable a given stage"},

  {"network",Bool,Section,0,0,0,0,"network settings"},
  {"net.partsize",Uint,Net_gen,Net_partsize,1,50000,6000,"aimed partition size"},
  {"net.walklimit",Uint,Net_gen,Net_walklimit,0,10000,1000,"maximum walk distance in meters for a single go"},
  {"net.walkspeed",Uint,Net_gen,Net_walkspeed,0,10000,5000,"walk speed in meters per hour"},
  {"net.sumwalklimit",Uint,Net_gen,Net_sumwalklimit,0,10000,3000,"maximum summed up walk distance in meters"},
  {"net.mintxtime",Uint,Net_gen,Net_mintt,0,180,5,"minimum transfer time in minutes"},
  {"net.maxtxtime",Uint,Net_gen,Net_maxtt,2,60 * 48,120,"maximum transfer time in minutes"},
  {"net.periodstart",Uint,Net_gen,Net_period0,0,20201231,0,"start day of schedule period"},
  {"net.periodend",Uint,Net_gen,Net_period1,0,20201231,0,"end day of schedule period"},
  {"net.patternstart",Uint,Net_gen,Net_tpat0,0,20201231,20150215,"start day of transfer pattern base"},
  {"net.patternend",Uint,Net_gen,Net_tpat1,0,20201231,20150315,"end day of transfer pattern base"},
  {"net.patternmintt",Uint,Net_gen,Net_tpatmintt,0,120,3,"minimum tranfser time for transfer pattern"},
  {"net.patternmaxtt",Uint,Net_gen,Net_tpatmaxtt,2,60 * 48,120,"maximum tranfser time for transfer pattern"},

  // interface
  {"interface",Bool,Section,0,0,0,0,"configure client-server interface"},
  {"querydir",String,Querydir,0,0,0,0,"client query queue directory"},

  {"files",Bool,Section,0,0,0,0,"determines which files to generate"},
  {"net.pdf",Bool,Net2pdf,0,0,1,0,"write network to pdf"},
  {"net.ext",Bool,Net2ext,0,0,1,0,"write network to ext"},

  {"engineering",Bool,Section,0,0,0,0,"engineering settings"},
  {"eng.periodlim",Uint,Eng_gen,Eng_periodlim,0,365*2,365,"schedule period limit"},
  {"eng.options",String,Eng_opt,0,0,0,0,"engineering options"},
  {NULL,0,0,0,0,0,0,NULL}
};

int inicfg(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();

  lvlnames[Runread] = "readnet";
  lvlnames[Runbaseprep] = "prepbase";
  lvlnames[Runprep] = "prepnet";
  lvlnames[Runmknet] = "mknet";
  lvlnames[Runnet0] = "mknet0";
  lvlnames[Runpart] = "partition";
  lvlnames[Runcompound] = "compound";
  lvlnames[Runnetn] = "mknetn";
  lvlnames[Runserver] = "server";
  lvlnames[Runend] = "end";
  lvlnames[Runcnt] = "end";

  struct cfgvar *vp;
  ub4 x,gb = meminfo() >> 10;
  ub4 now;

  if (gb == 0) return vrbfln(FLN,0,"cfg defaults not adjusted to available memory");
  vrb0(0,"adjusting cfg defaults to %u GB memory",gb);

  // determine some defaults at runtime
  for (vp = cfgvars; vp->name; vp++) {
    if (vp->var != Net_gen) continue;
    switch(vp->subvar) {
    case Net_tpat0:
      now = nix2min(gettime_sec() / 60);
      vp->def = day2cd(now / 1440);
      break;
    case Net_tpat1:
      now = nix2min(gettime_sec() / 60);
      vp->def = day2cd(now / 1440 + 30);
      break;

    case Net_partsize:
      if (gb < 4) x = 3000;
      else if (gb < 8) x = 6000;
      else if (gb < 16) x = 12000;
      else if (gb < 64) x = 20000;
      else if (gb < 256) x = 40000;
      else x = 50000;
      vp->def = x;
      break;
    default: break;
    }
  }

  return 0;
}

static int varseen[Cfgcnt];

static int memeq(const char *s,const char *q,ub4 n) { return !memcmp(s,q,n); }

static int writecfg(int havecfg,const char *cname)
{
  struct cfgvar *vp = cfgvars;
  ub4 uval,pos;
  ub4 now;
  int fd;
  char fname[1024];
  char buf[4096];
  char nowstr[64];
  const char *name;
  const char *desc;
  char *sval;
  enum Runlvl lvl;
  char *origin;

  if (havecfg) {
    fmtstring(fname,"%s/%s.cur",globs.netdir,cname);
  } else {
    fmtstring(fname,"%s/%s",globs.netdir,cname);
  }
  fd = filecreate(fname,1);
  if (fd == -1) return 1;

  now = gettime_sec();
  sec70toyymmdd(now,nowstr,sizeof(nowstr));
  if (havecfg) {
    pos = fmtstring(buf,"# tripover %u.%u config in effect at %s\n\n",Version_maj,Version_min,nowstr);
  } else {
    pos = fmtstring(buf,"# tripover %u.%u config created at %s\n\n",Version_maj,Version_min,nowstr);
  }
  pos += mysnprintf(buf,pos,sizeof(buf),"# name value  '#' origin description default\n\n");
  if (filewrite(fd,buf,pos,fname)) return 1;

  for (vp = cfgvars; vp->name; vp++) {
    name = vp->name;
    desc = vp->desc;

    if (vp->var == Enable || vp->var == Disable) continue;
    else if (vp->var == Section) {
      pos = fmtstring(buf,"\n[%s] - %s\n",name,desc);
      if (filewrite(fd,buf,pos,fname)) return 1;
      continue;
    }

    uval = 0;
    sval = (char *)"";
    switch(vp->var) {
    case Maxhops:  uval = globs.maxhops; break;
    case Maxports: uval = globs.maxports; break;
    case Maxstops: uval = globs.maxstops; break;
    case Maxvm:    uval = globs.maxvm; break;
    case Querydir: sval = globs.querydir; break;
    case Stopat:   uval = globs.stopat; break;
    case Enable: case Disable: break;
    case Net2pdf:  uval = globs.writpdf; break;
    case Net2ext:  uval = globs.writext; break;
    case Eng_gen:  uval = globs.engvars[vp->subvar]; break;
    case Net_gen:  uval = globs.netvars[vp->subvar]; break;
    case Eng_opt: break;
    case Cfgcnt: case Section: break;
    }

    if (uval & Cfgcl) origin = "cmdln";
    else if (uval & Cfgdef) origin = ".cfg ";
    else origin = "def  ";
    uval &= ~(Cfgcl | Cfgdef);

    switch(vp->cnv) {
    case Uint:       pos = fmtstring(buf,"%s %u\t# %s %s [%u]\n",name,uval,origin,desc,vp->def); break;
    case Bool:       pos = fmtstring(buf,"%s %u\t# %s %s [%u]\n",name,uval,origin,desc,vp->def); break;
    case EnumRunlvl: if (uval <= Runcnt) pos = fmtstring(buf,"%s %s=%u\t# %s %s\n",name,lvlnames[uval],uval,origin,desc);
                     else pos = fmtstring(buf,"%s unknown-%u\t# %s %s\n",name,uval,origin,desc); break;
    case String:     pos = fmtstring(buf,"%s %s\t# %s %s\n",name,sval,origin,desc);break;
    case None:       pos = fmtstring(buf,"%s\t# %s %s\n",name,origin,desc);break;
    }
    if (filewrite(fd,buf,pos,fname)) return 1;
  }
  filewrite(fd,"\n",1,fname);

  for (lvl = 0; lvl < Runcnt; lvl++) {
    if (globs.doruns[lvl]) pos = fmtstring(buf,"enable %s\n",lvlnames[lvl]);
    else pos = fmtstring(buf,"disable %s\n",lvlnames[lvl]);
    if (filewrite(fd,buf,pos,fname)) return 1;
  }

  fileclose(fd,fname);
  info(0,"wrote config to %s",fname);
  return 0;
}

static void limitval(struct cfgvar *vp,ub4 *puval)
{
  ub4 newval,defcl;

  defcl = *puval & (Cfgdef | Cfgcl);
  if (*puval & Cfgdef) newval = *puval & ~(Cfgcl | Cfgdef);
  else newval = vp->def;
  if (newval < vp->lo) { newval = vp->lo; warning(0,"config var %s below %u",vp->name,vp->lo); }
  else if (newval > vp->hi) { newval = vp->hi; warning(0,"config var %s above \ah%u",vp->name,vp->hi); }
  *puval = newval | defcl;
}

static int limitvals(void)
{
  struct cfgvar *vp;

  for (vp = cfgvars; vp->name; vp++) {
    switch(vp->var) {
    case Maxhops: limitval(vp,&globs.maxhops); break;
    case Maxports: limitval(vp,&globs.maxports); break;
    case Maxstops: limitval(vp,&globs.maxstops); break;
    case Maxvm:    limitval(vp,&globs.maxvm); break;
    case Stopat: limitval(vp,&globs.stopat); break;
    case Enable: case Disable: break;
    case Net2pdf: limitval(vp,&globs.writpdf); break;
    case Net2ext: limitval(vp,&globs.writext); break;
    case Querydir: break;
    case Eng_gen:  limitval(vp,globs.engvars + vp->subvar); break;
    case Net_gen:  limitval(vp,globs.netvars + vp->subvar); break;
    case Eng_opt: break;
    case Cfgcnt: case Section: break;
    }
  }
  return 0;
}

static void finalval(ub4 *puval)
{
  *puval &= ~(Cfgcl | Cfgdef);
}

static void finalize(void)
{
  struct cfgvar *vp;

  for (vp = cfgvars; vp->name; vp++) {
    switch(vp->var) {
    case Maxhops: finalval(&globs.maxhops); break;
    case Maxports: finalval(&globs.maxports); break;
    case Maxstops: finalval(&globs.maxstops); break;
    case Maxvm:    finalval(&globs.maxvm); break;
    case Stopat: finalval(&globs.stopat); break;
    case Enable: case Disable: break;
    case Net2pdf: finalval(&globs.writpdf); break;
    case Net2ext: finalval(&globs.writext); break;
    case Eng_gen:  finalval(globs.engvars + vp->subvar); break;
    case Net_gen:  finalval(globs.netvars + vp->subvar); break;
    case Querydir: break;
    case Eng_opt: break;
    case Cfgcnt: case Section: break;
    }
  }
}

static void setval(struct cfgvar *vp,ub4 *puval,ub4 newval)
{
  if (*puval & Cfgcl) info(0,"config var %s set on commandline",vp->name);
  else *puval = newval | Cfgdef;
}

static void eng_opt(const char *val,ub4 len)
{
  if (len == 9 && memeq(val,"no-msgsum",len)) globs.nomsgsum = 1;
  else if (len) warn(0,"line %u: ignoring unknown option %s",linno,val);
}

static int addvar(char *varname,char *val,ub4 varlen,ub4 vallen)
{
  struct cfgvar *vp = cfgvars;
  ub4 n,prvline,uval = 0;
  const char *name,*eq;
  enum Cfgvar var;
  char fln[1024];

  varname[varlen] = 0;
  val[vallen] = 0;

  fmtstring(fln,"%s ln %u col %u var '%s': ",cfgname,linno,colno,varname);

  if (varlen == 0) return error(0,"%s: empty var",fln);

  varname[varlen] = 0;
  while (vp->name) {
    name = vp->name;
    n = (ub4)strlen(name);
    if (n == varlen && memeq(name,varname,n)) break;
    vp++;
  }
  if (vp->name == NULL) return error(0,"%s: unknown config var",fln);
  var = vp->var;
  error_ge(var,Cfgcnt);

  if (var != Enable && var != Disable && var != Eng_opt && var != Eng_gen && var != Net_gen) {
    prvline = varseen[var];
    if (prvline) return warning(0,"%s: previously defined at line %u",fln,prvline);
  }
  varseen[var] = linno;

  if (var != Enable && var != Disable && var != Eng_opt && var != Eng_gen) {
    if (vallen == 0) return error(0,"%s: needs arg",fln);
  } else if (vallen == 0) return 0;

  switch(vp->cnv) {

  case Uint: val[vallen] = 0;
             if (str2ub4(val,&uval) == 0) return error(0,"%s: needs numerical arg, has '%s'",fln,val);
             break;

  case Bool: if (vallen) {
               if (*val == '0' || *val == 'n' || *val == 'f') uval = 0;
               else if (*val == '1' || *val == 'y' || *val == 't') uval = 1;
             }
             break;

  case EnumRunlvl:
             eq = strchr(val,'=');
             if (eq) vallen = (ub4)(eq - val);
             val[vallen] = 0;
             uval = 0;
             while (uval < Runcnt && strcmp(lvlnames[uval],val)) uval++;
             if (uval >= Runcnt) return error(0,"%s: unknown option '%s'",fln,val);
             vrb(0,"%s: stage %s",varname,lvlnames[uval]);
             break;

  case String: break;
  case None: break;
  }

  switch(var) {
  case Maxhops:  setval(vp,&globs.maxhops,uval); break;
  case Maxports: setval(vp,&globs.maxports,uval); break;
  case Maxstops: setval(vp,&globs.maxstops,uval); break;
  case Maxvm:    setval(vp,&globs.maxvm,uval); break;
  case Querydir: memcpy(globs.querydir,val,min(vallen,sizeof(globs.querydir)-1)); break;
  case Stopat:   setval(vp,&globs.stopat,uval); break;
  case Enable:   if (setruns[uval]) return error(0,"%s: previously set at line %u",val,setruns[uval]);
                 globs.doruns[uval] = 1; setruns[uval] = linno;
                 break;
  case Disable:  if (setruns[uval]) return error(0,"%s: previously set at line %u",val,setruns[uval]);
                 globs.doruns[uval] = 0; setruns[uval] = linno;
                 break;
  case Net2pdf:  setval(vp,&globs.writpdf,uval); break;
  case Net2ext:  setval(vp,&globs.writext,uval); break;
  case Net_gen:  setval(vp,globs.netvars + vp->subvar,uval); break;
  case Eng_gen:  setval(vp,globs.engvars + vp->subvar,uval); break;
  case Eng_opt:  eng_opt(val,vallen); break;
  case Cfgcnt: case Section: break;
  }
  return 0;
}

static int rdcfg(const char *name,int *havecfg)
{
  struct myfile cfg;
  char fname[1024];
  enum states { Out,Var,Val0,Val,Val9,Fls } state;
  ub4 pos,len,varlen,vallen;
  char var[Varlen];
  char val[Vallen];
  int rv = 0;
  char *buf,c;

  fmtstring(fname,"%s/%s",globs.netdir,name);
  info(0,"check config in %s",fname);
  rv = readfile(&cfg,fname,0);
  if (rv) return 1;

  if (cfg.exist == 0) {
    if (streq(globs.netdir,".")) return 0;
    info(0,"check config in %s",name);
    rv = readfile(&cfg,name,0);
    if (rv) return 1;
    if (cfg.exist == 0) return 0;
  }

  *havecfg = 1;

  len = (ub4)cfg.len;
  vrb(CC,"parse config in %s",fname);

  if (len == 0) return 0;

  buf = cfg.buf;

  cfgname = name;
  state = Out;
  pos = 0;
  linno = 1; colno = 1;
  varlen = vallen = 0;
  while(pos < len && rv == 0) {
    c = buf[pos++];
    if (c == '\n') { linno++; colno = 1; }
    else colno++;

    vrb(1,"state %u varlen %u",state,varlen);

    switch(state) {
    case Out:
      switch(c) {
      case '#': state = Fls; break;
      case '[': state = Fls; break;
      case '\n': break;
      case ' ': case '\t': break;
      default: var[0] = c; varlen = 1; state = Var;
      } break;

    case Var:
      if (varlen >= Varlen-1) return error(0,"%s.%u: variable name exceeds %u chars",cfgname,linno, Varlen);
      switch(c) {
      case '#': state = Val9; break;
      case '\n': rv = addvar(var,val,varlen,vallen); state = Out; break;
      case ' ': case '\t': case '=': vallen = 0; state = Val0; break;
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
      if (varlen) rv = addvar(var,val,varlen,vallen);
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

  return 0;
}

int readcfg(const char *name)
{
  int havecfg = 0;

  error_ge(Eng_cnt,Elemcnt(globs.engvars));

  if (rdcfg(name,&havecfg)) return 1;
  if (limitvals()) return 1;
  writecfg(havecfg,name);
  finalize();
  globs.stopat = min(globs.stopat,globs.stopatcl);
  vrb0(0,"done config file %s",name);
  infocc(globs.stopat < Runend,0,"stop at %s",lvlnames[globs.stopat]);
  return 0;
}
