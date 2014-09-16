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
#include "os.h"
#include "time.h"
#include "mem.h"
#include "util.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "bitfields.h"
#include "netbase.h"
#include "net.h"
#include "netio.h"

struct globs globs;

static const char copyright[] = "Copyright (C) 2014, and Creative Commons CC-by-nc-nd'd by Joris van der Geer";

static int streq(const char *s,const char *q) { return !strcmp(s,q); }

static int init0(char *progname)
{
  setsigs();

  inimsg(progname,1,Msg_init|Msg_stamp|Msg_pos|Msg_type,globs.msglvl, 0);
  msgfile = setmsgfile(__FILE__);
  iniassert();

  info(User,"tripover %u.%u %s\n%s\n", Version_maj,Version_min,Version_phase,copyright);

  if (iniutil()) return 1;
  initime();
  inimem();
  inios();
  inimath();
  ininetbase();
  ininet();
  ininetio();

  return 0;
}

// read or generate base network
static int getbasenet(void)
{
  ub4 portcnt = globs.maxports;
  ub4 hopcnt = globs.maxhops;

  error_ovf(portcnt,ub2);

  if (*globs.netfile) {  // todo read compiled net
    info(0,"TODO read compiled net from %s",globs.netfile);
    return 1;
  }
  info(0,"generate random %u port %u hop net", globs.maxports, globs.maxhops);
  if (mkrandnet(portcnt,hopcnt)) return 1;
//  net2pdf(net);
  return 0;
}

static int cmd_vrb(struct cmdval *cv) {
  if (cv->valcnt) globs.msglvl = cv->uval + Error;
  else globs.msglvl++;
  return 0;
}

static int cmd_max(struct cmdval *cv) {
  if (streq(cv->subarg,"ports")) globs.maxports = cv->uval;
  else if (streq(cv->subarg,"hops")) globs.maxhops = cv->uval;
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

static int cmd_arg(struct cmdval *cv) {
// add plain arg
  info(0,"add arg %s", cv->sval);
  strcopy(globs.netfile, cv->sval);

  return 0;
}

static struct cmdarg cmdargs[] = {
  { "verbose|v", "[level]%u", "set or increase verbosity", cmd_vrb },
  { "max-ports", "limit%u", "limit #ports", cmd_max },
  { "max-hops", "limit%u", "limit #hops", cmd_max },
  { ".test-a", "test%u", "test", cmd_test },
  { ".test-b", "test%u", "test", cmd_test },
  { ".test-set", "test%u", "tests", cmd_test },
  { "config|c", "file", "specify config file", cmd_cfg },
  { NULL, "files...", "tripover", cmd_arg }
};

int main(int argc, char *argv[])
{
  netbase *base;

  globs.msglvl = Info;

  if (init0(argv[0])) return 1;

  if (cmdline(argc,argv,cmdargs)) return 1;
  setmsglvl(globs.msglvl,0);

  if (getbasenet()) return 1;
  base = getnetbase();
  if (mknet(base)) return 1;

  return 0;
}
