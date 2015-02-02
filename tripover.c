// tripover.c - main program

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

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
#include "netio.h"
#include "gtfs.h"
#include "event.h"
#include "net.h"
#include "netev.h"
#include "netprep.h"
#include "condense.h"
#include "compound.h"
#include "partition.h"
#include "search.h"

struct globs globs;

static const char copyright[] = "Copyright (C) 2014-2015, and Creative Commons CC-by-nc-nd'd by Joris van der Geer";

static int streq(const char *s,const char *q) { return !strcmp(s,q); }

static int init0(char *progname)
{
  char mtimestr[64];
  char *p;

  setsigs();

  p = strrchr(progname,'/');
  globs.progname = (p ? p + 1 : progname);

  inimsg(progname,"tripover.log",Msg_stamp|Msg_pos|Msg_type);
  msgfile = setmsgfile(__FILE__);
  iniassert();

#ifdef NOW
  sec70toyymmdd(NOW,mtimestr,sizeof(mtimestr));
#else
  strcopy(mtimestr,__DATE__);
#endif

  info(User,"\ntripover %u.%u %s %s\n%s\n", Version_maj,Version_min,Version_phase,mtimestr,copyright);

  if (iniutil(0)) return 1;
  if (inicfg()) return 1;
  inimem();
  inios();
  iniserver();
  initime(0);
  inimath();
  ininetbase();
  inigtfs();
  ininet();
  ininetio();
  ininetev();
  ininetprep();
  inievent(0);
  inicondense();
  inipartition();
  inicompound();
  inisearch();

  return 0;
}

static int do_eximsg; // enable at net init

static void exit0(void)
{
  exiutil();
  eximem();
  if (do_eximsg) eximsg();
}

// init network
static int initnet(void)
{
  ub4 portcnt = globs.maxports;
  ub4 hopcnt = globs.maxhops;
  netbase *basenet = getnetbase();
  gnet *gnet;
  int rv = 0;

  if (*globs.netdir == 0) return 1;

  if (dorun(FLN,Runread)) rv = readextnet(basenet,globs.netdir);
  else return 0;
  if (rv) return rv;

  if (dorun(FLN,Runbaseprep)) rv = prepbasenet();
  else return 0;
  if (rv) return rv;

  if (globs.writgtfs) rv = writegtfs(basenet,globs.netdir);
  if (rv) return rv;

  if (dorun(FLN,Runprep)) rv = prepnet(basenet);
  else return 0;
  if (rv) return rv;

  gnet = getgnet();

  if (dorun(FLN,Runcompound)) rv = compound(gnet);
  else return 0;
  if (rv) return rv;

  if (dorun(FLN,Runpart)) rv = partition(gnet);
  else return 0;
  if (rv) return rv;

  return rv;
}

static int do_main(void)
{
  ub4 argc = globs.argc;
  struct myfile nd;
  const char *cmdstr;

  if (argc == 0) return shortusage();

  cmdstr = globs.args[0];
  if (*cmdstr == 0) return shortusage();

  if (streq(cmdstr,"run")) {
    info0(0,"command 'run'"); 
  } else if (streq(cmdstr,"gtfsout")) {
    info0(0,"cmd: 'gtfsout' TODO: read net, process events and write normalised gtfs");
    globs.stopat = Runprep;
    globs.writgtfs = 1;
  } else if (streq(cmdstr,"init")) {
    return 0;
  } else return error(0,"unknown command '%s': known are 'run','init'",cmdstr);

  if (argc < 2) return error0(0,"commands 'run' and 'gtfsout' need network dir arg");
  strcopy(globs.netdir,globs.args[1]);

  if (*globs.netdir) {
    oclear(nd);
    if (osfileinfo(&nd,globs.netdir)) return oserror(0,"cannot access net directory %s",globs.netdir);
    else if (nd.isdir == 0) return error(0,"net arg %s is not a directory",globs.netdir);
    if (setmsglog(globs.netdir,"tripover.log")) return 1;
  }

  char nowstr[64];
  sec70toyymmdd(gettime_sec(),nowstr,sizeof(nowstr));
  info(0,"starting at %s utc",nowstr);

  inievent(1);

  oslimits();

  do_eximsg = 1;

  if (initnet()) return 1;

  if (mknet(globs.maxstops)) return 1;

  if (globs.testcnt > 1 && globs.netok) {
    ub4 dep,arr,lostop = 0, histop = 3;
    int rv;
    search src;

    oclear(src);

    dep = globs.testset[0];
    arr = globs.testset[1];
    if (globs.testcnt > 3) {
      lostop = globs.testset[2];
      histop = globs.testset[3];
    }
    info(0,"test plan %u to %u minstop %u maxstop %u",dep,arr,lostop,histop);

    rv = plantrip(&src,(char *)"buildin test",dep,arr,lostop,histop);
    if (rv) warning(0,"search returned error %d",rv);
    else if (src.trips[0].cnt) info(0,"%u to %u = \av%u%p distance %u\n",dep,arr,src.lostop+2,src.trips[0].port,src.lodist);
    else info(0,"%u to %u : no trip\n",dep,arr);
  }

  showmemsums();

  if (globs.netok && dorun(FLN,Runserver)) {
    return serverloop();
  }

  return 0;
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
  int rv = 1;

  // temporary defaults
  globs.msglvl = Info;
  strcopy(globs.querydir,"queries");

  setmsglvl(globs.msglvl,0,0);
  if (init0(argv[0])) return 1;
  fmtstring(globs.cfgfile,"%s.cfg",globs.progname);

  if (cmdline(argc,argv,cmdargs,Program_desc)) return 1;

  if (inicfgcl()) return 1;

  initime(1);

  if (globs.argc == 0) return shortusage();

  if (globs.netdir[0] == 0) globs.netdir[0] = '.';

  const char *cfgfile = globs.cfgfile;
  if (globs.argc == 1 && streq(globs.args[0],"init")) {
    info(0,"prepare new default config in %s",cfgfile);
    osremove(cfgfile);
  }

  if (readcfg(cfgfile)) return 1;

  iniutil(1);

  rv = do_main();

  exit0();

  return rv;
}
