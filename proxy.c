// proxy.c - web proxy

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */


#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "base.h"
struct globs globs;

#include "os.h"
#include "mem.h"
#include "util.h"
#include "time.h"

static ub4 msgfile;
#include "msg.h"

static const ub4 version = 0;
static const ub4 apiversion = 0;

static ub4 port = 7001; // set on commandline

static ub4 geoscale = 1000000; // matches .js

static const char repdir[] = "rep";

static int init0(char *progname)
{
  char mtimestr[64];
  char *p;

  setsigs();

  p = strrchr(progname,'/');
  globs.progname = (p ? p + 1 : progname);

  setmsglvl(Info,0,0);
  inimsg(progname,"proxy.log",Msg_stamp|Msg_pos|Msg_type);
  msgfile = setmsgfile(__FILE__);
  iniassert();

#ifdef NOW
  sec70toyymmdd(NOW,mtimestr,sizeof(mtimestr));
#else
  strcopy(mtimestr,__DATE__);
#endif

  info(User,"proxy %u.%u %s %s\n", Version_maj,Version_min,Version_phase,mtimestr);

  if (iniutil(0)) return 1;
  inimem();
  inios();
  initime(0);
  globs.maxvm = 2;
  return 0;
}

extern const char *runlvlnames(enum Runlvl lvl);
const char *runlvlnames(enum Runlvl lvl) { return lvl ? "n/a" : "N/A"; }

static const ub4 maxreqlen = 65536;
#define Maxval 64

static int parserr(ub4 pos,ub4 fln)
{
  return errorfln(fln,0,FLN,"parse error at pos %u",pos);
}

struct toreq {
  ub4 id;
  ub4 cmd;
  ub4 deplat,deplon;
  ub4 arrlat,arrlon;
  ub4 dep,arr;
  ub4 date;
  ub4 utcofs;
  ub4 walklim;
};

struct torep {
  ub4 id;
  ub4 len,maxlen;
  char txt[4096];
};

enum cmds { Cmd_plan,Cmd_geo2name };
enum urlvals { Aver,Ver,Id,Cmd,Deplat,Deplon,Arrlat,Arrlon,Dep,Arr,Date,Utcofs,Delay,Walklim,Vcnt };

static char rspline[] =
"HTTP/1.1 200 OK\r\n\
Access-Control-Allow-Origin: *\r\n\
Content-Type: text/plain; charset=UTF-8\r\n\
Content-Length: ";

static int putrep(int fd,struct toreq *treq,struct torep *trep)
{
  char reply[4096];
  ub4 pos,len,nw;
  long rx;

  if (treq->id != trep->id) return error(0,"req id %u rep %u",treq->id,trep->id);

  len = fmtstring(reply,"%s%u\r\n\r\n%s\n%u.",rspline,trep->len + 5,trep->txt,treq->id);
  info(0,"write %s",reply);
  nw = 0; pos = 0;
  while (pos < len) {
    rx = oswrite(fd,reply + pos,len - pos);
    if (rx == -1) return oserror(0,"cannot write to fd %u pos %u",fd,pos);
    else if (rx == 0) return error(0,"eof on fd %u pos %u",fd,pos);
    pos += rx;
  }
  return 0;
}

static int getreq(int fd,struct toreq *tr)
{
  ub4 nr = 0,pos,chunk;
  char buf[4096];
  long rx;
  char c;
  ub4 linecnt = 0;
  enum states { Out,M1,M2,M3,U1,U2,R0,R1,R2,R3,Eoh } state;
  ub4 vals[Maxval];
  ub4 val = 0,valndx = 0;
  ub4 cmd;

  state = Out;
  aclear(vals);

  while (nr < maxreqlen && state != Eoh) {
    chunk = min(maxreqlen - nr,sizeof(buf));
    rx = osread(fd,buf,chunk);
    if (rx == -1) return oserror(0,"read error at pos %u",nr);
    else if (rx == 0) return error(0,"eof at pos %u",nr);
    msg_write("\n",1);
    msg_write(buf,(ub4)rx);
    msg_write("\n",1);
    pos = 0;
    while (pos < rx && state != Eoh) {
      c = buf[pos++];
//      info(0," c %c state %u ",c,state);
      if (c == '\n') linecnt++;
      switch (state) {
      case Out: if (c == 'G') state = M1; else return parserr(pos,FLN); break;
      case M1: if (c == 'E') state = M2; else return parserr(pos,FLN); break;
      case M2: if (c == 'T') state = M3; else return parserr(pos,FLN); break;
      case M3: if (c == ' ') { val = 0; state = U1; break; } else return parserr(pos,FLN);
      case U1: if (c == '/') { val = 0; state = U2; break; } else return parserr(pos,FLN);
      case U2: if (c >= '0' && c <= '9') val = (val * 10) + (c - '0');
               else if (c == '/') {
                 vals[valndx++] = val; val = 0;
                 if (valndx >= Maxval) return error(0,"exceeded max %u dirs",valndx);
                 break;
               } else if (c == ' ' || c == '?' || c == '#') { vals[valndx++] = val; state = R0; }
               else if (c == '\r') state = R1;
               else return parserr(pos,FLN);
               break;
       case R0: if (c == '\r') state = R1; break;
       case R1: if (c == '\n') state = R2; else return parserr(pos,FLN); break;
       case R2: if (c == '\r') state = R3; else state = R0; break;
       case R3: if (c == '\n') state = Eoh; break;
       case Eoh: break;
      }
    }
  }
  if (nr == maxreqlen) return error(0,"exceeded max %u reqlen: %u vars %u lines",nr,valndx,linecnt);
  info(0,"eoh at pos %u: %u vars %u lines",nr,valndx,linecnt);
  if (valndx <= Cmd) return error(0,"missing cmd arg, only %u",valndx);
  if (vals[Aver] != apiversion) return error(0,"incompatible api version %u vs %u",vals[Aver],apiversion);
  tr->id = vals[Id];
  cmd = vals[Cmd];
  switch(cmd) {
  case Cmd_plan:
    if (valndx <= Date) return error(0,"missing args ver,dep,arr, only %u",valndx);
    tr->deplat = vals[Deplat];
    tr->deplon = vals[Deplon];
    tr->arrlat = vals[Arrlat];
    tr->arrlon = vals[Arrlon];
    tr->dep = vals[Dep];
    tr->arr = vals[Arr];
    tr->date = vals[Date];
    tr->utcofs = vals[Utcofs];
    tr->cmd = cmd;
    break;
  case Cmd_geo2name:
    if (valndx <= Arrlon) return error(0,"missing args ver,dep,arr, only %u",valndx);
    tr->deplat = vals[Deplat];
    tr->deplon = vals[Deplon];
    tr->arrlat = vals[Arrlat];
    tr->arrlon = vals[Arrlon];
    tr->cmd = cmd;
    break;
  default: return error(0,"unknown command %u",cmd);
  }
  info(0,"req ver %u id %u dep %u,%u arr %u,%u date %u delay %u",vals[Ver],tr->id,tr->deplat,tr->deplon,tr->arrlat,tr->arrlon,tr->date,vals[Delay]);
  osmillisleep(vals[Delay]);
  return 0;
}

static double ilat2lat(ub4 lat)
{
  double x = (double)lat / geoscale;
  return x - 90;
}

static double ilon2lon(ub4 lon)
{
  double x = (double)lon / geoscale;
  return x - 180;
}

static int run(struct toreq *treq,struct torep *trep,ub4 seq,char *envp[])
{
  char arg0[256];
  char arg1[128];
  char arg2[128];
  char arg3[128];
  char arg4[128];
  char arg5[128];
  char arg6[128];
  char arg7[128];
  char *argv[9] = { arg0,arg1,arg2,arg3,arg4,arg5,arg6,arg7,NULL };
  char repname[1024];
  const char *cmd = "plantrip";
  struct myfile mf;
  ub4 len,utcofs;
  char utc_ofs[64];

  utcofs = treq->utcofs;
  int utchh = (utcofs / 100) - 12;
  int utcmm = utcofs % 100;
  if (utchh < 0) fmtstring(utc_ofs,"%02d:%02u",utchh,utcmm);
  else fmtstring(utc_ofs,"+%02d:%02u",utchh,utcmm);

  oclear(mf);
  strcopy(arg0,cmd);
  switch(treq->cmd) {
  case Cmd_plan:
    fmtstring(repname,"%s-plan-%u.rep",cmd,seq);
    strcpy(arg1,"-x");
    fmtstring(arg2,"-o=%s",repname);
    strcpy(arg3,"iplan");
    fmtstring(arg4,"%u",treq->dep);
    fmtstring(arg5,"%u",treq->arr);
    fmtstring(arg6,"date=%u",treq->date);
    fmtstring(arg7,"utcoffset=%s",utc_ofs);
    argv[7] = NULL;
    info(0,"running %s %s %s %s %s %s %s %s",cmd,arg1,arg2,arg3,arg4,arg5,arg6,arg7);
    break;
  case Cmd_geo2name:
    fmtstring(repname,"%s-geo-%u.rep",cmd,seq);
    strcpy(arg1,"-x");
    fmtstring(arg2,"-o=%s",repname);
    strcpy(arg3,"geocode");
    fmtstring(arg4,"%f",ilat2lat(treq->deplat));
    fmtstring(arg5,"%f",ilon2lon(treq->deplon));
    info(0,"running %s %s %s %s %s %s",cmd,arg1,arg2,arg3,arg4,arg5);
    argv[6] = NULL;
    break;
  default: return error(0,"skip unknown command %u",treq->cmd);
  }
  if (osrun(cmd,argv,envp)) return 1;
  info(0,"reading file %s",repname);
  if (readpath(&mf,repdir,repname,1,trep->maxlen)) return 1;
  len = min((ub4)mf.len,trep->maxlen);
  if (len == 0) return 0;
  memcpy(trep->txt,mf.buf,len);
  trep->len = len;
  if (mf.alloced) afree(mf.buf,repname);
  return 0;
}

static int proxy(char *envp[])
{
  int sfd,cfd;
  struct osnetadr ai;
  struct toreq treq;
  struct torep trep;
  ub4 seq = 0;
  ub8 now;

  info(0,"entering proxy loop on port %u, version %u",port,version);

  sfd = ossocket(1);
  if (sfd == -1) return 1;

  if (osbind(sfd,port)) return 1;

  if (oslisten(sfd,2)) return 1;

  do {
    info(0,"wait for new conn %u",seq);
    cfd = osaccept(sfd,&ai);
    if (cfd == -1) return 1;
    seq++;
    oclear(treq);
    oclear(trep);
    trep.maxlen = (ub4)sizeof(trep.txt);
    now = gettime_sec();
    trep.len = fmtstring(trep.txt,"test at \ad%u.%u\n",(ub4)(now / 60),(ub4)(now % 60));
    if (getreq(cfd,&treq)) { osclose(cfd); continue; }
    trep.id = treq.id;
    run(&treq,&trep,seq,envp);
    putrep(cfd,&treq,&trep);
    osclose(cfd);
  } while (globs.sigint == 0);

  return 0;
}

static int cmd_vrb(struct cmdval *cv) {
  if (cv->valcnt) globs.msglvl = cv->uval + Error;
  else globs.msglvl++;
  setmsglvl(globs.msglvl,0,globs.limassert);
  return 0;
}

static int cmd_port(struct cmdval *cv) {
  port = cv->uval;
  return 0;
}

static int cmd_limassert(struct cmdval *cv) {
  globs.limassert = cv->uval;
  setmsglvl(globs.msglvl,0,globs.limassert);
  return 0;
}

static int cmd_arg(struct cmdval *cv)
{
  ub4 argc = globs.argc;
  char *dst;
  ub4 maxlen = sizeof(globs.args[0]);

  if (argc + 1 >= Elemcnt(globs.args)) return error(0,"exceeded %u arg limit",argc);

  dst = globs.args[argc];
  vrb(0,"add arg %s", cv->sval);
  strncpy(dst, cv->sval,maxlen-1);
  globs.argc = argc + 1;
  return 0;
}

static struct cmdarg cmdargs[] = {
  { "verbose|v", "[level]%u", "set or increase verbosity", cmd_vrb },
  { "port|p", "port%u", "listen port", cmd_port },
  { "assert-limit", "[limit]%u", "stop at this #assertions", cmd_limassert },
  { NULL, "dir", "proxy", cmd_arg }
};

int main(int argc, char *argv[],char *envp[])
{
  init0(argv[0]);

  if (cmdline(argc,argv,cmdargs,"proxy")) return 1;

  if (proxy(envp)) return 1;

  return 0;
}
