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
  ub4 nd,na,zlen,dhop,ahop,eahop,edhop,drid,zid;
  int docondense;

  if (portcnt == 0) return info0(0,"skip condense on 0 ports");
  if (hopcnt == 0) return info0(0,"skip condense on 0 hops");

  docondense = dorun(Runcondense);

  if (portcnt < 200) docondense = 0;  // todo heuristic

  if (docondense == 0) return info(0,"no condense for %u ports",portcnt);

  info(0,"condensing %u ports",portcnt);

  ub4 *portsbyhop = net->portsbyhop;

  ub4 *port2zport = alloc(portcnt,ub4,0xff,"condense",portcnt);
  ub4 *zport2port = alloc(portcnt,ub4,0xff,"condense",portcnt);

  for (port = 0; port < portcnt; port++) {
    if (port2zport[port] != hi32) continue;
    pp = ports + port;
    nd = pp->ndep;
    na = pp->narr;
    if (nd != 1 || na != 1) {
      noda++;
      pp->zid = hi32;
      nzportcnt++;
      continue;
    }

    // follow and merge back
    ahop = pp->arrs[0];
    drid = pp->arids[0];
    zlen = 1;
    do {
      dep = portsbyhop[ahop * 2];
      pdep = ports + dep;
      if (pdep->ndep != 1 || pdep->narr != 1) { noda2++; break; }  // do not include terminus
      if (pdep->drids[0] != drid) { norid++; break; }
      zlen++;
      ahop = pdep->arrs[0];
    } while (1);

    arr = portsbyhop[ahop * 2 + 1];
    parr = ports + arr;

    zpp = zepp = parr;
    dhop = zpp->deps[0];

    zpp->zid = zid = zportcnt;
    zlen = 1;

    port2zport[port] = zid;
    zport2port[zid] = port;

    // follow and merge forward
    do {
      arr = portsbyhop[dhop * 2 + 1];
      parr = ports + arr;
      if (parr->narr != 1 || parr->ndep != 1) break;
      if (parr->arids[0] != drid) break;
      zepp = parr;
      parr->zid = zid;
      zlen++;
      port2zport[arr] = zid;
      dhop = parr->deps[0];
    } while (1);

    zpp->zlen = zlen;

    if (zepp == zpp) { zpp->zid = hi32; nzportcnt++; continue; }

    error_lt(zlen,2);
    zportcnt++;

    // now zpp .. zepp is condensable chain.
    // store next hop
    if (zepp->ndep) edhop = zepp->deps[0];
    else edhop = hi32;
    zpp->zedhop = edhop;

    eahop = zepp->arrs[0];

    dhop = zpp->deps[0];
//    rv = rangedur(net,dhop,eahop,&evinfo);
  }

  info(0,"condensed %u+%u from %u ports %u %u",zportcnt,nzportcnt,portcnt,noda,norid);

  net->zportcnt = zportcnt;
  net->zhopcnt = zhopcnt;
//  net->zports = zports;
//  net->zhops = zhops;

  return 1;
}
