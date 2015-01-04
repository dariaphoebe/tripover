// server.c

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* server side of local client-server interface

  A remote client, typically only one, conects to a set of local clients via a proxy.
  A local client connects to a set of local servers.

  A local tripover server receives and answers journey queries.
  Servers may serve specific regions. Both local and remote clients get enough network data
  from the server at init that they can validate locations and direct to regions.

  The server loads and inialises the network, then loops for queries
  it may restart to get a fresh network snapshot, or update real-time changes
  multiple server instances may be running to support above reloads, and provide redundancy.
  However, these may run on different machines. the remote client need to know the location of such instances.

  The client interface posts its queries, and one server picks it up
  at timeout, client can re-post

  a separate network proxy can provide http-style interface, forwarding to a local client
 */

#include <string.h>

#include "base.h"
#include "cfg.h"
#include "os.h"
#include "mem.h"
#include "server.h"

static ub4 msgfile;
#include "msg.h"

#include "util.h"

#include "bitfields.h"
#include "netbase.h"
#include "net.h"

#include "search.h"

extern struct globs globs;

enum Cmds { Cmd_nil,Cmd_plan,Cmd_stop,Cmd_cnt };

// to be elaborated: temporary simple interface
static int cmd_plan(struct myfile *req,struct myfile *rep)
{
  char *lp = req->buf;
  ub4 n,pos = 0,len = (ub4)req->len;
  ub4 dep,arr,lostop = 0,histop = 1;
  int rv;
  search src;

  if (len == 0) return 1;

  n = str2ub4(lp,&dep);
  if (n == 0) return error(0,"expected integer departure port for %s",lp);
  pos += n;
  while (pos < len && lp[pos] == ' ') pos++;
  if (pos == len) return error(0,"missing arrival port for %s",lp);
  n = str2ub4(lp+pos,&arr);
  if (n == 0) return error(0,"expected integer arrival port for %s",lp);
  pos += n;
  while (pos < len && lp[pos] == ' ') pos++;
  if (pos < len && lp[pos] != '\n') {
    if (str2ub4(lp+pos,&histop) == 0) {
      warning(0,"expected integer number of stops in '%s'",lp);
      histop = 1;
    }
  }
  if (dep == arr) warning(0,"dep %u equal to arr",dep);

  // invoke actual plan here
  info(0,"plan %u to %u in %u stop\as",dep,arr,histop);
  oclear(src);

  rv = plantrip(&src,req->name,dep,arr,lostop,histop);

  // prepare reply
  rep->buf = rep->localbuf;
  if (rv) len = fmtstring(rep->localbuf,"reply plan %u-%u error code %d\n",dep,arr,rv);
  else if (src.trip.cnt && src.reslen) {
    len = src.reslen;
    memcpy(rep->localbuf,src.resbuf,len);
  } else len = fmtstring(rep->localbuf,"reply plan %u-%u : no trip found\n",dep,arr);
  vrb0(0,"reply len %u",len);
  rep->len = len;
  osmillisleep(100);
  return 0;
}

int serverloop(void)
{
  const char *querydir = globs.querydir;
  struct myfile req,rep;
  int rv;
  enum Cmds cmd = Cmd_nil;
  ub4 prvseq = 0,seq = 0;
  char c;
  const char *region = "glob"; // todo

  info(0,"entering server loop for id %u",globs.serverid);

  do {
    infovrb(seq > prvseq,0,"wait for new cmd %u",seq);
    rv = getqentry(querydir,&req,region,".sub");
    if (rv) break;

    prvseq = seq;

    if (req.direxist == 0) osmillisleep(5000);
    else if (req.exist == 0) osmillisleep(500);
    else {
      info(0,"new client entry %s",req.name);
      c = req.name[req.basename];
      switch(c) {
      case 's': cmd = Cmd_stop; break;
      case 'p': cmd = Cmd_plan; break;
      default: info(0,"unknown command '%c'",c);
      }
      if (cmd == Cmd_plan) {
        oclear(rep);
        rv = cmd_plan(&req,&rep);
        if (rv) info(0,"plan returned %d",rv);
        if (req.alloced) afree(req.buf,"client request");
        setqentry(&req,&rep,".rep");
        rv = 0;
        seq++;
      }
    }
  } while (rv == 0 && cmd != Cmd_stop && globs.sigint == 0);

  info(0,"leaving server loop for id %u",globs.serverid);

  return rv;
}

void iniserver(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

