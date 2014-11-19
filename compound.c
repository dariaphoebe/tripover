// compound.h - create compound hops from routes

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* add virtual non-stop hops between any two ports on the same route.
 a route is defined here as in public transport : ports are visited in succession
 without actual tranfers.

 such compound hop points to its first and last actual hop.
 These in turn contain the depart and arrive times.

 at network read time, these routes are ignored, yet the ID is marked.
 Note that the 'trip', the complete sequences, are stored. IDs of these are in the time tables
 after condense, reconstruct the 'direct' hops that do not need a physical transfer
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
#include "compound.h"

#undef hdrstop

#define Maxchainports 128

static ub4 rid2log = hi32;

void inicompound(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

// check if all deps from hop1 exist at hop2 as well.
// todo
static int setgos(struct network *net,struct hop *nhp,ub4 hop1,ub4 hop2,ub4 dep,ub4 arr)
{
  struct hop *hops,*hp1,*hp2;
  struct timepat *tp1,*tpn;
  struct chain *cp,*chains = net->chains;
  ub4 *evs,*events = net->events;
  ub8 *chp,*chainhops = net->chainhops;

  ub4 hopndx,tid,tdep,ccnt;

  hops = net->hops;

  hp1 = hops + hop1;
  hp2 = hops + hop2;
  error_ne(hp1->dep,dep);
  error_ne(hp2->arr,arr);

  tp1 = &hp1->tp;

  tpn = &nhp->tp;

  *tpn = *tp1;

  ub4 cnt = 0,tidno = 0;
  evs = events + tp1->evofs;

  while(cnt < tp1->evcnt) {
    tid = evs[cnt * 2 + 1];
    if (tid == 0) { cnt++; continue; }

    cp = chains + tid;
    hopndx = tdep = 0;
    chp = chainhops + cp->hopofs;
    ccnt = cp->hopcnt;
    while (hopndx < ccnt && (chp[hopndx] & hi32) != hop1) {
      if ( (chp[hopndx] & hi32) == hop2) tdep = chp[hopndx] >> 32;
      hopndx++;
    }
    if (hopndx == ccnt) return error(0,"hop %u tid %x not in chain",hop1,tid);
    else if (tdep) {
      tidno++;
    }
    cnt++;
  }
  return 0;
}

// add compound hops
int compound(struct network *net)
{
  ub4 part = net->part;
  ub4 rportcnt,port2,rport2,portcnt = net->portcnt;
  ub4 hop,ghop,nhop,newhopcnt,hopcnt = net->hopcnt;
  ub4 rid,pridcnt,ridcnt = net->ridcnt;
  ub4 chain,chaincnt;
  struct hop *nhp,*newhops,*hops = net->hops;
  struct route *routes,*rp;
  struct chain *chains,*cp;
  ub8 *chp,*chainhops = net->chainhops;
  ub4 dep,arr,ndep,narr;
  ub2 rdep,rarr;
  ub4 hcnt,unicnt,dupci;
  ub8 code;
  int docompound;
  ub2 rarrs[Maxchainports];
  ub1 rdeparr[Maxchainports * Maxchainports];
  ub4 rport2port[Maxchainports];
  ub4 maxchainlen = Maxchainports - 1;

  if (portcnt < 3) return info(0,"skip compound on %u port\as",portcnt);
  if (hopcnt < 2) return info(0,"skip compound on %u hop\as",hopcnt);
  port2 = portcnt * portcnt;

  docompound = dorun(Runcompound);

  if (docompound == 0) return info0(0,"compound not enabled");

  info(0,"compounding %u ports %u hops",portcnt,hopcnt);

  ub4 *rcp,*routechains  = net->routechains;

  ub4 *g2phop = net->g2phop;

  ub4 rawcmphopcnt = 0;
  ub2 *port2rport = alloc(portcnt,ub2,0xff,"port2rport",portcnt);

  routes = net->routes;
  chains = net->chains;
  chaincnt = net->chaincnt;

  error_zp(routes,part);
  error_zp(chains,part);

  pridcnt = 0;
  for (rid = 0; rid < ridcnt; rid++) {
    rp = routes + rid;
    if (rp->part != part) continue;
    pridcnt++;
    hcnt = rp->hichainlen;
    rawcmphopcnt += hcnt * hcnt;  // = (portcnt-1)^2
  }
  info(0,"%u of %u routes in part %u",pridcnt,ridcnt,part);
  rawcmphopcnt = rawcmphopcnt * 4 / 6; // rough estimate is 1/2

  info(0,"estimated \ah%u compound hops",rawcmphopcnt);

  newhopcnt = hopcnt + rawcmphopcnt;
  newhops = alloc(newhopcnt,struct hop,0,"cmp newhops",newhopcnt);

  memcpy(newhops,hops,hopcnt * sizeof(struct hop));

  nhp = newhops + hopcnt;
  nhop = hopcnt;

  ub4 *orgportsbyhop = net->portsbyhop;
  ub4 *portsbyhop = alloc(newhopcnt * 2,ub4,0,"cmp portsbyhop",newhopcnt);
  memcpy(portsbyhop,orgportsbyhop,hopcnt * 2 * sizeof(ub4));

  ub4 ci,chcnt,hi,a2no,da,cdep,carr,arr2;
  ub4 hichaincnt = 0;
  for (rid = 0; rid < ridcnt; rid++) {
    rp = routes + rid;
    if (rp->part != part) continue;
    hichaincnt = max(hichaincnt,rp->chaincnt);
  }

  ub8 *unichains = alloc(hichaincnt,ub8,0,"cmp tmp",hichaincnt);

  for (rid = 0; rid < ridcnt; rid++) {
    rp = routes + rid;
    if (rp->part != part) continue;

    chcnt = rp->chaincnt;
    rcp = routechains + rp->chainofs;
    memset(unichains,0,chcnt * sizeof(ub8));
    memset(port2rport,0xff,portcnt * sizeof(ub2));
    memset(rport2port,0,sizeof(rport2port));

    rportcnt = unicnt = 0;
    for (ci = 0; ci < chcnt; ci++) {
      chain = rcp[ci];
      error_ge(chain,chaincnt);
      cp = chains + chain;
      hcnt = cp->hopcnt;
      if (hcnt < 3) continue;
      limit(hcnt,maxchainlen,chain);

      // filter out chains with identical sequence
      code = cp->code;
      dupci = 0;
      while (dupci < unicnt && (unichains[dupci] & hi32) != code) dupci++;
      if (dupci < unicnt) continue;
      unichains[unicnt++] = ((ub8)chain << 32) | code;
      cp->uni = 1;

      // pass 1: create relative portlist per route
      chp = chainhops + cp->hopofs;
      for (hi = 0; hi < hcnt; hi++) {
        ghop = (ub4)chp[hi];
        hop = g2phop[ghop];
        dep = portsbyhop[hop * 2];
        arr = portsbyhop[hop * 2 + 1];
        if (dep == arr) warning(0,"dep %u == arr",dep);
        if (port2rport[dep] == hi16) {
          port2rport[dep] = (ub2)rportcnt;
          rport2port[rportcnt++] = dep;
        }
        if (port2rport[arr] == hi16) {
          port2rport[arr] = (ub2)rportcnt;
          rport2port[rportcnt++] = arr;
        }
      }
    } // each chain
    rp->portcnt = rportcnt;
    if (rportcnt != rp->hichainlen + 1) info(0,"route %u hi portcnt %u hi chainlen %u",rid,rportcnt,rp->hichainlen+1);
    rport2 = rportcnt * rportcnt;
    memset(rdeparr,0,rport2);

    for (ci = 0; ci < chcnt; ci++) {
      chain = rcp[ci];
      cp = chains + chain;
      hcnt = min(cp->hopcnt,maxchainlen);
      if (hcnt < 3 || cp->uni == 0) continue;

      // pass 2
      chp = chainhops + cp->hopofs;
      for (hi = 0; hi < hcnt; hi++) {
        ghop = (ub4)chp[hi];
        hop = g2phop[ghop];
        arr = portsbyhop[hop * 2 + 1];
        rarr = port2rport[arr];
        rarrs[hi] = rarr;
      }
      for (hi = 0; hi < hcnt; hi++) {
        ghop = (ub4)chp[hi];
        hop = g2phop[ghop];
        dep = portsbyhop[hop * 2];
        arr = portsbyhop[hop * 2 + 1];
        rdep = port2rport[dep];
        rarr = port2rport[arr];
        rarrs[hi] = rarr;
        rdeparr[rdep * rportcnt + rarr] = 1;
        for (a2no = hi+1; a2no < hcnt; a2no++) {
          arr2 = rarrs[a2no];
          da = rdep * rportcnt + arr2;
          if (rdeparr[da] != 1) rdeparr[da] = 2;
        }
      }
    }

    for (rdep = 0; rdep < rportcnt; rdep++) {
      for (rarr = 0; rarr < rportcnt; rarr++) {
        if (rdep == rarr) continue;
        if (rdeparr[rdep * rportcnt + rarr] != 2) continue;

        // generate compound
        narr = ndep = 0;
        while (narr < rportcnt && rdeparr[rdep * rportcnt + narr] != 1) narr++;
        while (ndep < rportcnt && rdeparr[ndep * rportcnt + rarr] != 1) ndep++;
        if (narr == rportcnt) { warning(0,"no first arr for compound %u-%u route %u",rdep,rarr,rid); continue; }
        if (ndep == rportcnt) { warning(0,"no first dep for compound %u-%u route %u",rdep,rarr,rid); continue; }
        dep = rport2port[rdep];
        arr = rport2port[rarr];
        if (dep == arr) { warning(0,"chop %u dep %u == arr",nhop,dep); continue; }
        cdep = rport2port[ndep];
        carr = rport2port[narr];
        nhp->dep = dep;
        nhp->arr = arr;
        nhp->cdep = cdep;
        nhp->carr = carr;
        error_ge(nhop,newhopcnt);
        portsbyhop[nhop * 2] = dep;
        portsbyhop[nhop * 2 + 1] = arr;
        fmtstring(nhp->name,"chop %u %u-%u",nhop,dep,arr);
        nhp++;
        nhop++;
      }
    }
  }

  info(0,"%u of %u estimated compound hops",nhop,newhopcnt);

  net->hopcnt = nhop;
  net->hops = newhops;
  net->portsbyhop = portsbyhop;

  return 0;
}
