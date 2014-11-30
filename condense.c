// condense.c - make network more dense

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/*
  Replace a list of ports on a single route to a single condensed port.
 */

#include <string.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "util.h"
#include "bitfields.h"
#include "net.h"
#include "condense.h"

#undef hdrstop

void inicondense(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

// create condensed net out of full net
int condense(struct gnetwork *net)
{
  ub4 port,portcnt = net->portcnt;
  ub4 hop,hopcnt = net->hopcnt;
  ub4 zportcnt = 0,zhopcnt = 0,nzportcnt = 0;
  struct port *pp,*parr,*pdep,*zpp,*zepp,*ports = net->ports;
  struct hop *hp,*hops = net->hops;
  ub4 dep,arr,noda = 0,norid = 0,noda2 = 0,norid2 = 0;;
  ub4 nd,na,zlen,dhop,ahop,zdhop,eahop,edhop,drid,zid;
  int docondense;
  ub4 iv;
  ub4 stats[20];
  aclear(stats);

  if (portcnt == 0) return info0(0,"skip condense on 0 ports");
  if (hopcnt == 0) return info0(0,"skip condense on 0 hops");

  docondense = dorun(FLN,Runcondense);

  if (portcnt < 200) docondense = 0;  // todo heuristic

  ub4 *port2zport = alloc(portcnt,ub4,0xff,"condense",portcnt);
  ub4 *zport2port = alloc(portcnt,ub4,0xff,"condense",portcnt);

  if (docondense == 0) {
    for (port = 0; port < portcnt; port++) {
      port2zport[port] = port;
      zport2port[port] = port;
    }
    net->zportcnt = portcnt;
    net->port2zport = port2zport;
    return info(0,"no condense for %u ports",portcnt);
  }

  info(0,"condensing %u ports",portcnt);

  ub4 *portsbyhop = net->portsbyhop;

  // single case a-b-c
  for (port = 0; port < portcnt; port++) {
    if (port2zport[port] != hi32) continue;
    pp = ports + port;
    nd = pp->ndep;
    na = pp->narr;
    if (nd != 1 || na != 1) {
      stats[0]++;
      pp->zid = hi32;
      continue;
    }

    // follow and merge back
    ahop = pp->arrs[0];
    drid = pp->arids[0];
    zlen = 1;
    do {
      dep = portsbyhop[ahop * 2];
      pdep = ports + dep;
      if (pdep->ndep > 1) { stats[1]; break; }  // do include terminus
      else if (pdep->narr > 1) { stats[2]++; break; }  // do include terminus
      else if (pdep->drids[0] != drid) { stats[3]++; break; }
      zlen++;
      ahop = pdep->arrs[0];
    } while (1);

    arr = portsbyhop[ahop * 2 + 1];
    parr = ports + arr;

    zpp = zepp = parr;
    dhop = zdhop = zpp->deps[0];

    zpp->zid = zid = zportcnt;
    zlen = 1;

    port2zport[port] = zid;
    zport2port[zid] = port;

    // follow and merge forward
    do {
      arr = portsbyhop[dhop * 2 + 1];
      if (arr == zdhop) break;
      parr = ports + arr;
      nd = parr->ndep; na = parr->narr;
      if (na > 1) { stats[4]++; break; }
      else if (nd > 1) { stats[5]++; break; }
      else if (na && parr->arids[0] != drid) { stats[6]++; break; }
      else if (na == 0) info(0,"port %u has no arrs %s",arr,parr->name);
      zepp = parr;
      parr->zid = zid;
      zlen++;
      port2zport[arr] = zid;
      dhop = parr->deps[0];
    } while (1);

    zpp->zlen = zlen;

    if (zepp == zpp) { zpp->zid = hi32; stats[7]++; continue; }

    error_lt(zlen,2);
    stats[8]++;
    zportcnt++;

    // now zpp .. zepp is condensable chain.
    // store next hop
    if (zepp->ndep) edhop = zepp->deps[0];
    else edhop = hi32;
    zpp->zedhop = edhop;

    eahop = zepp->arrs[0];

    dhop = zpp->deps[0];
  }

  // double case a=b=c : 2 rid's, but same dep,arr
  for (port = 0; port < portcnt; port++) {
    if (port2zport[port] != hi32) continue;
    pp = ports + port;
    if (pp->zid != hi32) continue;
    stats[9]++;
    nd = pp->ndep;
    na = pp->narr;

    vrb(0,"abc port %u nd %u na %u %u %u %u %u %s",port,nd,na,pp->arrs[0],pp->arrs[1],pp->deps[0],pp->deps[1],pp->name);

    if (nd != 2 || na != 2) {
      stats[10]++;
      continue;
    } else if (pp->arids[0] != pp->arids[1]) {
      stats[11]++;
      continue;
    } else if (pp->drids[0] != pp->drids[1]) {
      stats[12]++;
      continue;
    }

    // follow and merge back
    ahop = pp->arrs[0];
    drid = pp->arids[0];
    zlen = 1;
    do {
      dep = portsbyhop[ahop * 2];
      if (port2zport[dep] != hi32) break;
      pdep = ports + dep;
      if (pdep->ndep != 2 || pdep->drids[0] != drid || pdep->drids[1] != drid) {
        break;   // include terminus
      } else if (pdep->narr >= 1 && pdep->arids[0] != drid) break;
      else if (pdep->narr == 2 && pdep->arids[1] != drid) break;
      zlen++;
      ahop = pdep->arrs[0];
    } while (zlen < 100);

    arr = portsbyhop[ahop * 2 + 1];
    parr = ports + arr;

    zpp = zepp = parr;
    dhop = zdhop = zpp->deps[0];

    zpp->zid = zid = zportcnt;
    zlen = 1;

    port2zport[port] = zid;
    zport2port[zid] = port;

    // follow and merge forward
    do {
      arr = portsbyhop[dhop * 2 + 1];
      if (port2zport[arr] == zdhop) break;
      parr = ports + arr;
      nd = parr->ndep; na = parr->narr;
      if (na > 2 || nd > 2) { stats[13]++; break; }
      else if (na >= 1 && parr->arids[0] != drid) { stats[14]++; break; }
      else if (na == 2 && parr->arids[1] != drid) { stats[15]++; break; }
      zepp = parr;
      parr->zid = zid;
      zlen++;
      port2zport[arr] = zid;
      dhop = parr->deps[0];
    } while (zlen < 100);

    zpp->zlen = zlen;

    if (zepp == zpp) { zpp->zid = hi32; stats[18]++; continue; }

    error_lt(zlen,2);

    info(0,"double zport at %s-%s",zpp->name,zepp->name);
    zportcnt++;
    stats[19]++;

    // now zpp .. zepp is condensable chain.
    // store next hop
    if (zepp->ndep) edhop = zepp->deps[0];
    else edhop = hi32;
    zpp->zedhop = edhop;

    eahop = zepp->arrs[0];

    dhop = zpp->deps[0];
    info(0,"dhop %u",dhop);
  }

  for (port = 0; port < portcnt; port++) {
    if (port2zport[port] == hi32) {
      nzportcnt++;
      port2zport[port] = port;
    }
  }

  info(0,"condensed %u+%u from %u ports",zportcnt,nzportcnt,portcnt);

  for (iv = 0; iv < Elemcnt(stats); iv++) if (stats[iv]) info(0,"stat %u: %u",iv,stats[iv]);

  net->zportcnt = zportcnt;
//  net->zhopcnt = zhopcnt; todo
  net->port2zport = port2zport;

  return 0;
}
