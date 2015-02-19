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

#include "netbase.h"
#include "net.h"
#include "fare.h"

#include "search.h"

static int memeq(const char *s,const char *q,ub4 n) { return !memcmp(s,q,n); }

static ub1 hexmap[256];

static void mkhexmap(void)
{
  char c;

  memset(hexmap,0xff,256);
  for (c = '0'; c <= '9'; c++) hexmap[(ub4)c] = (ub1)(c - '0');
  for (c = 'a'; c <= 'f'; c++) hexmap[(ub4)c] = (ub1)(c + 10 - 'a');
  for (c = 'A'; c <= 'F'; c++) hexmap[(ub4)c] = (ub1)(c + 10 - 'A');
  hexmap[9] = 0x20;
  hexmap[0x0a] = 0xfe;
  hexmap[0x20] = 0x20;
}

static ub4 bitsinmask[256];

static void mkbitmask(void)
{
  ub4 x,xx,n;

  for (x = 0; x < 256; x++) {
    n = 0; xx = x;
    while (xx) { if (xx & 1) n++; xx >>= 1; }
    bitsinmask[x] = n;
  }
}

enum Cmds { Cmd_nil,Cmd_plan,Cmd_upd,Cmd_stop,Cmd_cnt };

// to be elaborated: temporary simple interface
static int cmd_plan(struct myfile *req,struct myfile *rep,search *src)
{
  char *vp,*lp = req->buf;
  ub4 n,pos = 0,len = (ub4)req->len;
  ub4 ival;
  ub4 varstart,varend,varlen,valstart,valend,type;

  ub4 dep = 0,arr = 0,lostop = 0,histop = 3,tdep = 0,ttdep = 0,tspan = 3,utcofs=2200;
  ub4 costperstop = 10;
  ub4 mintt = globs.mintt;
  ub4 maxtt = globs.maxtt;
  ub4 walklimit = globs.walklimit;
  ub4 sumwalklimit = globs.sumwalklimit;
  ub4 nethistop = hi32;

  int rv;
  enum Vars {
    Cnone,
    Cdep,
    Carr,
    Ctdep,
    Cttdep,
    Ctspan,
    Clostop,
    Chistop,
    Cmintt,
    Cmaxtt,
    Ccostperstop,
    Cwalklimit,
    Csumwalklimit,
    Cnethistop,
    Cutcofs
  } var;

  ub4 *evpool;

  if (len == 0) return 1;

  while (pos < len && lp[pos] >= 'a' && lp[pos] <= 'z') {
    ival = 0;
    varstart = varend = pos;
    while (varend < len && lp[varend] >= 'a' && lp[varend] <= 'z') varend++;
    varlen = varend - varstart; pos = varend;
    if (varlen == 0) break;

    while (pos < len && lp[pos] == ' ') pos++;
    if (pos == len) break;
    type = lp[pos++];
    if (type == '\n' || pos == len) break;
    while (pos < len && lp[pos] == ' ') pos++;
    lp[varend] = 0;

    valstart = valend = pos;
    while (valend < len && lp[valend] != '\n') valend++;
    if (valend == len) break;
    pos = valend;
    while (pos < len && lp[pos] != '\n') pos++;
    if (lp[pos] == '\n') pos++;
    if (pos == len) break;
    lp[valend] = 0;

    if (type == 'i') {
      n = str2ub4(lp + valstart,&ival);
      if (n == 0) return error(0,"expected integer for %s",lp + valstart);
    }
    vp = lp + varstart;
    if (varlen == 3 && memeq(vp,"dep",3)) var = Cdep;
    else if (varlen == 3 && memeq(vp,"arr",3)) var = Carr;
    else if (varlen == 7 && memeq(vp,"deptmin",7)) var = Ctdep;
    else if (varlen == 8 && memeq(vp,"depttmin",8)) var = Cttdep;
    else if (varlen == 5 && memeq(vp,"tspan",5)) var = Ctspan;
    else if (varlen == 6 && memeq(vp,"lostop",6)) var = Clostop;
    else if (varlen == 6 && memeq(vp,"histop",6)) var = Chistop;
    else if (varlen == 5 && memeq(vp,"mintt",5)) var = Cmintt;
    else if (varlen == 5 && memeq(vp,"maxtt",5)) var = Cmaxtt;
    else if (varlen == 11 && memeq(vp,"costperstop",11)) var = Ccostperstop;
    else if (varlen == 9 && memeq(vp,"walklimit",9)) var = Cwalklimit;
    else if (varlen == 12 && memeq(vp,"sumwalklimit",12)) var = Csumwalklimit;
    else if (varlen == 9 && memeq(vp,"nethistop",9)) var = Cnethistop;
    else if (varlen == 6 && memeq(vp,"utcofs",6)) var = Cutcofs;
    else {
      warn(0,"ignoring unknown var '%s'",vp);
      var = Cnone;
    }
    switch (var) {
    case Cnone: break;
    case Cdep: dep = ival; break;
    case Carr: arr = ival; break;
    case Ctdep: tdep = ival; break;
    case Cttdep: ttdep = ival; break;
    case Ctspan: tspan = ival; break;
    case Clostop:  lostop = ival; break;
    case Chistop: histop = ival; break;
    case Cmintt: mintt = ival; break;
    case Cmaxtt: maxtt = ival; break;
    case Ccostperstop: costperstop = ival; break;
    case Cwalklimit: walklimit = ival; break;
    case Csumwalklimit: sumwalklimit = ival; break;
    case Cnethistop: nethistop = ival; break;
    case Cutcofs: utcofs = ival; break;
    }
  }

  if (mintt > maxtt) {
    warn(0,"min transfer time %u cannot be above max %u",mintt,maxtt);
    maxtt = mintt + 2;
  }
  if (sumwalklimit < walklimit) {
    warn(0,"max distance for single go walk %u above summed max %u",walklimit,sumwalklimit);
    sumwalklimit = walklimit;
  }

  if (dep == arr) warning(0,"dep %u equal to arr",dep);
  evpool = src->evpool;
  clear(src);
  src->evpool = evpool;

  src->depttmin_cd = ttdep;
  src->deptmin_cd = tdep;
  src->utcofs12 = utcofs;
  src->tspan = tspan;
  src->nethistop = min(nethistop,histop);
  src->mintt = mintt;
  src->maxtt = maxtt;
  src->costperstop = costperstop;

  src->walklimit = m2geo(walklimit);
  src->sumwalklimit = m2geo(sumwalklimit);

  // invoke actual plan here
  info(0,"plan %u to %u in %u to %u stop\as from %u for %u days",dep,arr,lostop,histop,tdep,tspan);

  rv = plantrip(src,req->name,dep,arr,lostop,histop);

  // prepare reply
  rep->buf = rep->localbuf;
  if (rv) len = fmtstring(rep->localbuf,"reply plan %u-%u error code %d\n",dep,arr,rv);
  else if (src->reslen) {
    len = min(src->reslen,sizeof(rep->localbuf));
    memcpy(rep->localbuf,src->resbuf,len);
  } else len = fmtstring(rep->localbuf,"reply plan %u-%u : no trip found\n",dep,arr);
  vrb0(0,"reply len %u",len);
  rep->len = len;
//  osmillisleep(10);
  return 0;
}

static int updfares(gnet *net,ub4 *vals,ub4 valcnt)
{
  ub4 rrid = vals[0];
  ub4 dep = vals[1];
  ub4 arr = vals[2];
  ub4 t0 = vals[3];
  ub4 vndx = 4;
  ub4 rid,t,dt,mask,n;
  ub4 *rrid2rid = net->rrid2rid;
  ub4 hirrid = net->hirrid;
  struct route *rp;
  struct port *pdep,*parr,*ports = net->ports;
  ub4 portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 *portsbyhop = net->portsbyhop;

  if (net->fareposcnt == 0) return warn(0,"no reserved routes to update for %u-%u",dep,arr);
  if (dep == arr) return error(0,"dep %u equals arr",dep);
  else if (dep >= portcnt) return error(0,"dep %u above %u",dep,portcnt);
  else if (arr >= portcnt) return error(0,"arr %u above %u",arr,portcnt);
  pdep = ports + dep;
  parr = ports + arr;
  info(0,"%u-%u %s to %s",dep,arr,pdep->name,parr->name);

  if (rrid > hirrid) return error(0,"rrid %u above max %u",rrid,hirrid);
  rid = rrid2rid[rrid];
  if (rid >= net->ridcnt) return error(0,"rid %u above max %u",rid,net->ridcnt);

  vrb0(0,"r.rid %u.%u t \ad%u",rrid,rid,t0);

  rp = net->routes + rid;
  if (rp->reserve == 0) return warn(0,"ignoring rrid %u for nonreserved route",rrid);
  if (rp->hopcnt == 0) return warn(0,"no hops on rrid %u",rrid);

  // get hop from rid,dep,arr. orgs in case of compound
  ub4 hopndx = 0,h1ndx = 0,h2ndx = 0;
  ub4 h,chop,hop1 = hi32,hop2 = hi32;
  while (hopndx < rp->hopcnt && (hop1 == hi32 || hop2 == hi32)) {
    h = rp->hops[hopndx];
    if (portsbyhop[h * 2] == dep) { hop1 = h; h1ndx = hopndx; }
    if (portsbyhop[h * 2 + 1] == arr) { hop2 = h;  h2ndx = hopndx; }
    hopndx++;
  }
  if (hop1 == hi32 || hop2 == hi32) return error(0,"no hop found for %u-%u",dep,arr);
  else if (hop1 >= hopcnt) return error(0,"invalid hop %u found for %u-%u",hop1,dep,arr);
  else if (hop2 >= hopcnt) return error(0,"invalid hop %u found for %u-%u",hop2,dep,arr);
  if (hop1 == hop2) chop = hop1;
  else chop = rp->hop2chop[h1ndx * Chainlen + h2ndx];
  vrb0(0,"found hop %u,%u = %u",hop1,hop2,chop);

  while (vndx + 2 < valcnt) {
    dt = vals[vndx];
    mask = vals[vndx+1];
    vndx += 2;
    t = t0 + dt;
    vrb0(0,"dt %u t \ad%u mask %u",dt,t,mask);
    n = bitsinmask[mask & (Faregrp-1)];
    fareupd(net,rid,hop1,hop2,chop,t,mask,n,vals + vndx);
    vndx += n;
  }
  return 0;
}

#define Maxvals 1024
/* handle an update command
   fare availability:
     rid t (dt grpmask fare1 fare2 ..)+
     rid  = route id : rrid
     t = time in minutes since epoch
     dt = idem, relative to above
     grpmask is bitmap for each known fare group
 */
static int cmd_upd(struct myfile *req,ub4 seq)
{
  char c,*lp = req->buf;
  ub4 pos = 0,len = (ub4)req->len;
  ub4 valcnt,val,x;
  enum states { Out, Val0, Val1, Item, Fls } state;
  enum cmd { Upd_fare };
  ub4 vals[Maxvals];

  info(0,"update seq %u len %u",seq,len);

  if (len == 0) return 0;
  gnet *net = getgnet();

  state = Out; valcnt = val = 0;
  while (pos < len) {
    c = lp[pos++];
    x = hexmap[(ub4)(c & 0x7f)];

//  info(0,"c %x x %x state %u",c,x,state);
    switch(state) {
    case Out:  if (x < 0x10) { val = x; state = Val1; }
               else if (x != 0xfe) state = Fls;
               break;
    case Val0: if (x < 0x10) { val = x; state = Val1; }
               else if (x == 0xfe) state = Item;
               else if (x != 0x20) state = Fls;
               break;
    case Val1: if (x < 0x10) val = (val << 4) + x;
               else if (x == 0x20) { vals[valcnt++] = val; state = Val0; }
               else if (x == 0xfe) state = Item;
               else state = Fls;
               break;

    case Fls:  if (x == 0xfe) state = Out; break;

    case Item: info(0,"%u vals",valcnt); break;
    }
    if (state == Item) {
      state = Out;
      info(0,"%u vals",valcnt);
      if (valcnt == 0) break;
      else if (vals[0] == Upd_fare && valcnt > 7) updfares(net,vals+1,valcnt-1);
      break;
    }
  }

  return 0;
}

/* currently a directory queue based interface
 future plan :
  use a single local proxy on same system that interfaces here with sockets
  have at least a set of 2 servers, allow network rebuild
  remote client interfaces to proxy at e.g. port 80
  proxy on behalf of client to here, synchronously. proxy forks per request
  server accepts one conn only, proxy knows list of servers and iterates
  have directory-based queue for real-time status updates only
  status updates synchronously, rely on having a set of servers
 */
int serverloop(void)
{
  const char *querydir = globs.querydir;
  struct myfile req,rep;
  int prv,rv = 1;
  enum Cmds cmd = Cmd_nil;
  ub4 prvseq = 0,seq = 0,useq = 0;
  char c;
  const char *region = "glob"; // todo
  search src;

  info(0,"entering server loop for id %u",globs.serverid);

  oclear(src);

  do {
    infovrb(seq > prvseq,0,"wait for new cmd %u",seq);
    rv = getqentry(querydir,&req,region,".sub");
    if (rv) break;

    prvseq = seq;

    if (req.direxist == 0) osmillisleep(2000);
    else if (req.exist == 0) osmillisleep(20);  // for linux only we may use inotify instead
    else {
      info(0,"new client entry %s",req.name);
      c = req.name[req.basename];
      switch(c) {
      case 's': cmd = Cmd_stop; break;
      case 'p': cmd = Cmd_plan; break;
      case 'u': cmd = Cmd_upd; break;
      default: info(0,"unknown command '%c'",c);
      }
      if (cmd == Cmd_plan) {
        oclear(rep);
        prv = cmd_plan(&req,&rep,&src);
        if (prv) info(0,"plan returned %d",prv);
        if (req.alloced) afree(req.buf,"client request");
        setqentry(&req,&rep,".rep");
        seq++;
      } else if (cmd == Cmd_upd) {
        prv = cmd_upd(&req,useq);
        if (prv) info(0,"update returned %d",prv);
        if (req.alloced) afree(req.buf,"client request");
        useq++;
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
  mkhexmap();
  mkbitmask();
}
