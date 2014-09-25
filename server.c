// server.c

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

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

extern struct globs globs;

enum Cmds { Cmd_nil,Cmd_plan,Cmd_stop,Cmd_cnt };

// to be elaborated: temporary simple interface
static int cmd_plan(struct myfile *req,struct myfile *rep)
{
  char *lp = req->buf;
  ub4 pos=0,len = (ub4)req->len;
  ub4 dep,arr;

  if (len == 0) return 1;

  if (str2ub4(lp,&dep)) return error(0,"expected integer departure port for %s",lp);
  while (pos < len && lp[pos] != ' ') pos++;
  if (pos == len) return error(0,"expected integer departure port for %s",lp);
  if (str2ub4(lp+pos,&arr)) return error(0,"expected integer arrival port for %s",lp);

  // invoke actual plan here
  info(0,"plan %u to %u",dep,arr);

  // prepare reply
  rep->buf = rep->localbuf;
  len = fmtstring(rep->localbuf,"reply plan %u-%u = 1234\n",dep,arr);
  info(0,"reply len %u",len);
  rep->len = len;
  osmillisleep(500);
  return 0;
}

int serverloop(void)
{
  const char *querydir = globs.querydir;
  struct myfile req,rep;
  int rv;
  enum Cmds cmd = Cmd_nil;
  char c;
  const char *region = "glob"; // todo

  info(0,"entering server loop for id %u",globs.serverid);

  do {
    rv = getqentry(querydir,&req,region,".sub");
    if (rv) break;

    if (req.direxist == 0) osmillisleep(5000);
    else if (req.exist == 0) osmillisleep(1000);
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
        if (req.alloced) afree(req.buf,"client request");
        setqentry(&req,&rep,".rep");
        rv = 0;
      }
    }
  } while(rv == 0 && cmd != Cmd_stop);

  info(0,"leaving server loop for id %u",globs.serverid);

  return rv;
}

void iniserver(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

