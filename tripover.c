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
struct globs globs;

#include "time.h"
#include "cfg.h"
#include "os.h"
#include "mem.h"
#include "util.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "server.h"

#include "netbase.h"
#include "netio.h"
#include "gtfs.h"
#include "event.h"
#include "net.h"
#include "netn.h"
#include "netev.h"
#include "netprep.h"
#include "condense.h"
#include "compound.h"
#include "partition.h"
#include "search.h"
#include "fare.h"

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

  inibase();
  if (iniutil(0)) return 1;
  initime(0);
  if (inicfg()) return 1;
  inimem();
  inios();
  iniserver();
  inimath();
  ininetbase();
  inigtfs();
  ininet();
  ininetn();
  ininetio();
  ininetev();
  ininetprep();
  inievent(0);
  inicondense();
  inipartition();
  inicompound();
  inisearch();
  inifare();
  return 0;
}

static int do_eximsg; // enable at net init

static int background;

static void exit0(void)
{
  exiutil();
  eximem();
  if (do_eximsg) eximsg(1);
}

// init network
static int initnet(void)
{
  netbase *basenet = getnetbase();
  gnet *gnet;
  int rv = 0;
  ub4 mintt,maxtt;
  ub4 walklimit,sumwalklimit,walkspeed;

  // check params
  mintt = globs.netvars[Net_mintt];
  maxtt = globs.netvars[Net_maxtt];
  if (maxtt <= mintt) {
    warn(0,"default for max transfer %u time cannot be below minimum %u",maxtt,mintt);
    maxtt = mintt + 2;
  }
  globs.mintt = mintt; globs.maxtt = maxtt;

  walklimit = globs.netvars[Net_walklimit];
  sumwalklimit = globs.netvars[Net_sumwalklimit];
  if (sumwalklimit < walklimit) {
    warn(0,"max distance for single go walk %u above summed max %u",walklimit,sumwalklimit);
    sumwalklimit = walklimit;
  }

  walkspeed = globs.netvars[Net_walkspeed];
  if (walkspeed == 0) {
    warn(0,"walk speed zero for max distance %u",walklimit);
  }
  globs.walkspeed = walkspeed;
  globs.walklimit = walklimit;
  globs.sumwalklimit = sumwalklimit;

  ub4 t0_cd = globs.netvars[Net_period0];
  ub4 t1_cd = globs.netvars[Net_period1];

  ub4 t0,t1;
  if (t0_cd < Epochyear) t0 = t0_cd;
  else t0 = cd2day(t0_cd);
  if (t1_cd < Epochyear) t1 = t1_cd;
  else t1 = cd2day(t1_cd);
  if (t0 < Epochyear && t1 < Epochyear && t0 > t1) {
    warn(0,"relative period start %u after end %u",t0_cd,t1_cd);
    t1 = t0 + 1440;
  } else if (t0 >= Epochyear && t1 >= Epochyear && t0 > t1) {
    warn(0,"period start \ad%u after end \ad%u",t0,t1);
    t1 = t0 + 1440;
  }
  globs.periodt0 = t0;
  globs.periodt1 = t1;

  if (*globs.netdir == 0) return 1;

  if (dorun(FLN,Runread,0)) rv = readextnet(basenet,globs.netdir);
  else return 0;
  if (rv) return rv;

  if (dorun(FLN,Runbaseprep,0)) rv = prepbasenet();
  else return 0;
  if (rv) return rv;

  if (globs.writgtfs) rv = writegtfs(basenet,globs.netdir);
  if (rv) return rv;

  if (dorun(FLN,Runprep,0)) rv = prepnet(basenet);
  else return 0;
  if (rv) return rv;

  gnet = getgnet();

  if (dorun(FLN,Runcompound,0)) rv = compound(gnet);
  else return 0;
  if (rv) return rv;

  if (dorun(FLN,Runpart,0)) rv = partition(gnet);
  else return 0;
  if (rv) return rv;

  return rv;
}

static int do_main(void)
{
  ub4 argc = globs.argc;
  struct myfile nd;
  const char *cmdstr;
  char logdir[1024];
  int rv;

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
    if (setmsglog(globs.netdir,"tripover.log",0)) return 1;

    fmtstring(logdir,"%s/log",globs.netdir);
    rv = osexists(logdir);
    if (rv == -1) return oserror(0,"cannot access dir %s",logdir);
    else if (rv == 0) {
      if (osmkdir(logdir)) return oserror(0,"cannot create dir %s",logdir);
    }
  }

  char nowstr[64];
  sec70toyymmdd(gettime_sec(),nowstr,sizeof(nowstr));
  info(0,"starting at %s utc",nowstr);

  inievent(1);

  oslimits();

  do_eximsg = 1;

  if (background) osbackground();

  if (initnet()) return 1;

  if (mknet(globs.maxstops)) return 1;

  if (globs.testcnt > 1 && globs.netok) {
    ub4 dep,arr,lostop = 0, histop = 3;
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
    else if (src.trips[0].cnt) info(0,"%u to %u = \av%u%p distance %u\n",dep,arr,src.lostop+2,(void *)src.trips[0].port,src.lodist);
    else info(0,"%u to %u : no trip\n",dep,arr);
  }

  showmemsums();

  gnet *net = getgnet();
  if (globs.netok && dorun(FLN,Runserver,0)) {

    info(0,"overall schedule period \ad%u to \ad%u",net->t0,net->t1);
    info(0,"connectivity precomputed for %u transfer\as",net->histop);
    info(0,"default min transfer time %u min, max %u min",globs.mintt,globs.maxtt);
    info(0,"max walking distance %u m, summed %u",geo2m(net->walklimit),geo2m(net->sumwalklimit));
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

static int cmd_bg(struct cmdval *cv)
{
  background = 1;
  return info(0,"%s set",cv->subarg);
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
  { "max-stops", "limit%u", "limit #stops", cmd_max },
  { "background|b",NULL,"run in background",cmd_bg },
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
