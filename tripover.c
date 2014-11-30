// tripover.c - main program

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Perform basic initialisation and setup
   Enter a loop processing queries
   Initially, only direct commandline queries
*/

#include <string.h>

#include "base.h"
#include "cfg.h"
#include "os.h"
#include "time.h"
#include "mem.h"
#include "util.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "server.h"

#include "bitfields.h"
#include "netbase.h"
#include "net.h"
#include "netio.h"
#include "netprep.h"
#include "event.h"
#include "condense.h"
#include "compound.h"
#include "search.h"

struct globs globs;

static const char copyright[] = "Copyright (C) 2014, and Creative Commons CC-by-nc-nd'd by Joris van der Geer";

static int streq(const char *s,const char *q) { return !strcmp(s,q); }

static int init0(char *progname)
{
  char nowstr[64];

  setsigs();

  inimsg(progname,"tripover.log",Msg_init|Msg_stamp|Msg_pos|Msg_type);
  msgfile = setmsgfile(__FILE__);
  iniassert();

#ifdef NOW
  sec70toyymmdd(NOW,nowstr,sizeof(nowstr));
#else
  strcopy(nowstr,__DATE__);
#endif

  info(User,"tripover %u.%u %s %s\n%s\n", Version_maj,Version_min,Version_phase,nowstr,copyright);

  if (iniutil(0)) return 1;
  if (inicfg()) return 1;
  inimem();
  inios();
  iniserver();
  initime(0);
  inimath();
  ininetbase();
  ininet();
  ininetio();
  ininetprep();
  inievent(0);
  inicondense();
  inicompound();
  inisearch();

  sec70toyymmdd(gettime_sec(),nowstr,sizeof(nowstr));
  info(0,"starting at %s utc",nowstr);

  return 0;
}

static void exit0(void)
{
  exiutil();
  eximem();
  eximsg();
}

// read or generate base network
static int getbasenet(void)
{
  ub4 portcnt = globs.maxports;
  ub4 hopcnt = globs.maxhops;
  netbase *basenet = getnetbase();
  int rv;

  error_ovf(portcnt,ub2);

  if (*globs.netdir) {  // todo read compiled net
    if (dorun(FLN,Runread)) {
      rv = readextnet(basenet,globs.netdir);
      if (rv) return rv;
      if (dorun(FLN,Runbaseprep)) rv = prepbasenet();
      if (rv) return rv;
      if (dorun(FLN,Runprep)) rv = prepnet(basenet);
      info(0,"rv %d",rv);
      return rv;
    } else return 0;
  }
  info(0,"generate random %u port %u hop net", globs.maxports, globs.maxhops);
  rv = mkrandnet(portcnt,hopcnt);
  if (rv) return rv;
  if (dorun(FLN,Runbaseprep)) rv = prepbasenet();
  if (rv) return rv;
  if (dorun(FLN,Runprep)) rv = prepnet(basenet);
//  net2pdf(net);
  return rv;
}

static int cmd_vrb(struct cmdval *cv) {
  if (cv->valcnt) globs.msglvl = cv->uval + Error;
  else globs.msglvl++;
  setmsglvl(globs.msglvl,0,globs.limassert);
  return 0;
}

static int cmd_limassert(struct cmdval *cv) {
  globs.limassert = cv->uval;
  setmsglvl(globs.msglvl,0,globs.limassert);
  return 0;
}

static int cmd_max(struct cmdval *cv) {
  ub4 val = cv->uval | Cfgcl | Cfgdef;

  if (streq(cv->subarg,"ports")) globs.maxports = val;
  else if (streq(cv->subarg,"hops")) globs.maxhops = val;
  else if (streq(cv->subarg,"stops")) globs.maxstops = val;
  return 0;
}

static int cmd_test(struct cmdval *cv) {
  if (streq(cv->subarg,"a")) globs.testa = cv->uval;
  else if (streq(cv->subarg,"b")) globs.testb = cv->uval;
  else if (streq(cv->subarg,"set") && globs.testcnt < Elemcnt(globs.testset)) {
    vrb(User,"test %u %u",globs.testcnt,cv->uval);
    globs.testset[globs.testcnt++] = cv->uval;
  } else info(User,"ignoring arg test-%s",cv->subarg);
  return 0;
}

static int cmd_cfg(struct cmdval *cv)
{
  strcopy(globs.cfgfile, cv->sval);
  return 0;
}

static int cmd_stopat(struct cmdval *cv)
{
  strcopy(globs.stopatstr, cv->sval);

  return 0;
}

static int cmd_arg(struct cmdval *cv) {
  vrb(0,"add arg %s", cv->sval);
  strcopy(globs.netdir, cv->sval);

  return 0;
}

static struct cmdarg cmdargs[] = {
  { "verbose|v", "[level]%u", "set or increase verbosity", cmd_vrb },
  { "assert-limit", "[limit]%u", "stop at this #assertions", cmd_limassert },
  { "max-ports", "limit%u", "limit #ports", cmd_max },
  { "max-hops", "limit%u", "limit #hops", cmd_max },
  { "max-stops", "limit%u", "limit #stops", cmd_max },
  { "stopat|runto","stage","run only up to the given stage",cmd_stopat },
  { ".test-a", "test%u", "test", cmd_test },
  { ".test-b", "test%u", "test", cmd_test },
  { ".test-set", "test%u", "tests", cmd_test },
  { "config|c", "file", "specify config file", cmd_cfg },
  { NULL, "nets...", "tripover", cmd_arg }
};

int main(int argc, char *argv[])
{
  // temporary defaults
  globs.msglvl = Info;
  strcopy(globs.cfgfile,"tripover.cfg");
  strcopy(globs.querydir,"queries");

  setmsglvl(globs.msglvl,0,0);
  if (init0(argv[0])) return 1;

  if (cmdline(argc,argv,cmdargs)) return 1;

  if (inicfgcl()) return 1;

  oslimits();

  initime(1);

  if (readcfg(globs.cfgfile)) return 1;

  iniutil(1);
  inievent(1);

  if (getbasenet()) return 1;

  if (mknet(globs.maxstops)) return 1;

  if (globs.testcnt > 1) {
    ub4 dep,arr,lostop = 0, histop = 3;
    int rv;
    search src;

    dep = globs.testset[0];
    arr = globs.testset[1];
    if (globs.testcnt > 3) {
      lostop = globs.testset[2];
      histop= globs.testset[3];
    }
    info(0,"test plan %u to %u minstop %u maxstop %u",dep,arr,lostop,histop);

    rv = searchgeo(&src,dep,arr,lostop,histop);
    if (rv) warning(0,"search returned error %d",rv);
    else if (src.tripcnt) info(0,"%u to %u = \av%u%p distance %u\n",dep,arr,src.lostop+2,src.tripports,src.lodist);
    else info(0,"%u to %u : no trip\n",dep,arr);
  }

  if (dorun(FLN,Runserver)) {
    serverloop();
  }

  exit0();

  return 0;
}
