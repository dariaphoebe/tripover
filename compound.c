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

void inicompound(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

// add compound hops
int compound(struct network *net)
{
  ub4 part = net->part;
  ub4 rportcnt,port2,portcnt = net->portcnt;
  ub4 hop,chop,newhopcnt,hopcnt = net->hopcnt;
  ub4 rid,ridcnt = net->ridcnt;
  ub4 chaincnt;
  struct hop *hp,*hops = net->hops;
  struct route *routes;
  struct chain *chains;
  ub4 dep,arr;
  ub2 rdep,rarr;
  int docompound;
  ub4 chainlen;

  if (portcnt < 3) return info(0,"skip compound on %u port\as",portcnt);
  if (hopcnt < 2) return info(0,"skip compound on %u hop\as",hopcnt);
  if (ridcnt == 0) return info(0,"skip compound on no rids for %u hop\as",hopcnt);

  net->chopcnt = hopcnt;

  if (net->istpart) return info0(0,"no compound for topnet");

  port2 = portcnt * portcnt;

  docompound = dorun(FLN,Runcompound);

  if (docompound == 0) return info0(0,"compound not enabled");

  info(0,"compounding %u ports %u hops",portcnt,hopcnt);

  ub4 *orgportsbyhop = net->portsbyhop;
  ub4 *orghopdist = net->hopdist;
  error_zp(orghopdist,hopcnt);

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    dep = orgportsbyhop[hop * 2];
    arr = orgportsbyhop[hop * 2 + 1];
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    if (orghopdist[hop] == 0) warning(Iter,"hop %u has distance 0",hop);
  }

  ub4 rawcmphopcnt = 0;
  ub2 *port2rport = alloc(portcnt,ub2,0xff,"port2rport",portcnt);

  routes = net->routes;
  chains = net->chains;
  chaincnt = net->chaincnt;

  error_z(chaincnt,ridcnt);

  error_zp(routes,part);
  error_zp(chains,part);

  ub4 hichainlen = 0;
  for (rid = 0; rid < ridcnt; rid++) {
    chainlen = 0;
    for (hop = 0; hop < hopcnt; hop++) {
      hp = hops + hop;
      if (hp->rid == rid) chainlen++;
    }
    if (chainlen < 3) continue;
    hichainlen = max(chainlen,hichainlen);
    rawcmphopcnt += (chainlen) * (chainlen);  // = (portcnt-1)^2
  }

  newhopcnt = hopcnt + rawcmphopcnt;

  chop = hopcnt;

  ub4 *portsbyhop = alloc(newhopcnt * 2,ub4,0,"cmp portsbyhop",newhopcnt);
  ub4 *choporg = alloc(newhopcnt * 2,ub4,0,"cmp choporg",newhopcnt);
  memcpy(portsbyhop,orgportsbyhop,hopcnt * 2 * sizeof(ub4));

  ub4 *hopdist = alloc(newhopcnt,ub4,0,"cmp hopdist",newhopcnt);
  memcpy(hopdist,orghopdist,hopcnt * sizeof(ub4));

  ub4 hirportcnt = hichainlen + 1;
  hirportcnt *= hirportcnt;
  ub4 *rdeparr = alloc(hirportcnt,ub4,0xff,"cmp rdeparr",hichainlen);

  ub4 *rport2port = alloc(hirportcnt,ub4,0,"cmp",hichainlen);

  ub4 *cmpdeps = alloc(hirportcnt,ub4,0,"cmp seq",hichainlen);
  ub4 *cmparrs = alloc(hirportcnt,ub4,0,"cmp seq",hichainlen);
  ub4 *cmphops = alloc(hirportcnt,ub4,0,"cmp seq",hichainlen);
  ub4 *arrcmps = alloc(hirportcnt,ub4,0,"cmp seq",hichainlen);

  ub4 hop1,hop2,hop3;
  ub4 cmpno,cmplen,cmp1,cmp2,cmp3,cmp4;
  ub4 rdep1,rdep2,rdep3,rarr1,rarr2;
  ub4 legcnt;
  ub4 dist;

  for (rid = 0; rid < ridcnt; rid++) {

    // assign
    nclear(rdeparr,hirportcnt);
    memset(port2rport,0xff,portcnt * sizeof(ub2));
    rportcnt = chainlen = 0;
    for (hop = 0; hop < hopcnt; hop++) {
      hp = hops + hop;
      if (hp->rid != rid) continue;
      chainlen++;
      dep = portsbyhop[hop * 2];
      arr = portsbyhop[hop * 2 + 1];
      error_ge(dep,portcnt);
      error_ge(arr,portcnt);
      rdep = port2rport[dep];
      error_gt(rportcnt,hichainlen,rid);
      if (rdep == hi16) {
        rdep = port2rport[dep] = (ub2)rportcnt++;
        rport2port[rdep] = dep;
      }
      rarr = port2rport[arr];
      if (rarr == hi16) {
        rarr = port2rport[arr] = (ub2)rportcnt++;
        rport2port[rarr] = arr;
      }
    }
    if (chainlen < 3) continue;

    // mark
    cmpno = 0;
    for (hop = 0; hop < hopcnt; hop++) {
      hp = hops + hop;
      if (hp->rid != rid) continue;
      dep = portsbyhop[hop * 2];
      arr = portsbyhop[hop * 2 + 1];
      error_ge(dep,portcnt);
      error_ge(arr,portcnt);
      rdep = port2rport[dep];
      rarr = port2rport[arr];
      rdeparr[rdep * rportcnt + rarr] = hop;
      cmpdeps[cmpno] = rdep;
      cmparrs[cmpno] = rarr;
      cmphops[cmpno] = hop;
      cmpno++;
    }

    if (cmpno != chainlen) warning(0,"rid %u seq %u chainlen %u",rid,cmpno,chainlen);
    cmplen = cmpno;

    for (cmp1 = 0; cmp1 < cmplen; cmp1++) {
      rarr1 = cmparrs[cmp1];
      arrcmps[rarr1] = cmp1;
    }

    for (cmp1 = 0; cmp1 < cmplen; cmp1++) {
      rdep1 = cmpdeps[cmp1];
      rarr1 = cmparrs[cmp1];
      hop1 = cmphops[cmp1];
      dist = hopdist[hop1];

      for (cmp2 = 0; cmp2 < cmplen; cmp2++) { // for all combi's
        if (cmp1 == cmp2) continue;
        rdep2 = cmpdeps[cmp2];
        rarr2 = cmparrs[cmp2];
        if (rarr1 == rdep2 || rdep1 == rarr2) continue;  // existing non-compound
        hop2 = cmphops[cmp2];
        dist = hopdist[hop1];
        legcnt = 0; cmp3 = cmp1; rdep3 = rdep1;
        while (legcnt++ < cmplen && cmp3 != cmp2) {
          cmp3 = arrcmps[rdep3];
          rdep3 = cmpdeps[cmp3];
          hop3 = cmphops[cmp3];
          dist += hopdist[hop3];
        }
        if (legcnt == cmplen) continue;
        cmp4 = 0;
        while (cmp4 < cmplen && !(cmpdeps[cmp4] == rdep1 && cmparrs[cmp4] == rarr2)) cmp4++;
        if (cmp4 < cmplen) continue;

        // generate compound
        error_ge(chop,newhopcnt);
        error_ge(rdep1,rportcnt);
        error_ge(rarr2,rportcnt);
        dep = rport2port[rdep1];
        arr = rport2port[rarr2];
        error_ge(dep,portcnt);
        error_ge(arr,portcnt);
        portsbyhop[chop * 2] = dep;
        portsbyhop[chop * 2 + 1] = arr;
        choporg[chop * 2] = hop1;
        choporg[chop * 2 + 1] = hop2;
        hopdist[chop] = dist;
        chop++;
      }
    }

  }

  info(0,"%u of %u estimated compound hops",chop,newhopcnt);

  net->chopcnt = chop;
  net->choporg = choporg;
  net->portsbyhop = portsbyhop;
  net->hopdist = hopdist;

  return 0;
}
