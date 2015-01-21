// compound.c - create compound hops from routes

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* add virtual non-stop hops between any two ports on the same route.
 a route is defined here as in public transport : ports are visited in succession
 without actual tranfers.

 such compound hop points to its first and last actual hop.
 These in turn contain the depart and arrive times.

 todo: generate only partial combis, let n-stop and search handle continuation
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

static const ub4 maxperm = min(130,Chainlen);
static const ub4 maxperm2 = maxperm * maxperm;

// add compound hops
int compound(struct network *net)
{
  ub4 part = net->part;
  ub4 rportcnt,port2,portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 ghopcnt = net->ghopcnt;
  ub4 whopcnt = net->whopcnt;
  ub4 hop,chop,newhopcnt = 0;
  ub4 rid,rrid,ridcnt = net->ridcnt;
  ub4 chain,chaincnt = net->chaincnt;
  ub4 chainhopcnt = net->chainhopcnt;
  ub4 hichainlen = 0,hirid = 0,hirrid = 0;
  struct hop *hp,*hops = net->hops;
  struct port *pdep,*parr,*ports = net->ports;
  struct route *routes,*rp;
  struct chain *cp,*chains = net->chains;
  struct chainhop *chp,*chainhops = net->chainhops;
  ub4 *g2phop = net->g2phop;

  ub4 dep,arr,deparr,gdep,garr;
  ub2 rdep,rarr;
  int docompound,dbg;
  ub4 chainlen,hopofs;

  net->chopcnt = whopcnt;

  if (portcnt < 3) return info(0,"skip compound on %u port\as",portcnt);
  if (hopcnt < 2) return info(0,"skip compound on %u hop\as",hopcnt);
  if (ridcnt == 0) return info(0,"skip compound on no rids for %u hop\as",hopcnt);

  port2 = portcnt * portcnt;

  docompound = dorun(FLN,Runcompound);

  if (docompound == 0) return info0(0,"compound not enabled");

  info(0,"compounding %u ports %u hops",portcnt,hopcnt);

  ub4 *orgportsbyhop = net->portsbyhop;
  ub4 *orghopdist = net->hopdist;

  ub4 *orghopdur = net->hopdur;
  ub4 midur;

  ub1 *duphops = alloc(port2,ub1,0,"cmp duphops",portcnt);

  ub4 ghop,hop1,hop2,dep1,arr2;
  ub4 dist1,dist2,midur1,midur2,dist = 0;
  ub4 tdep,tarr,tdep1,tdep2,tarr2;
  ub4 ci,ci1,ci2,pchlen,cmpcnt;

  ub4 pchain[Chainlen];
  ub4 pdeps[Chainlen];
  ub4 parrs[Chainlen];
  ub4 ptdep[Chainlen];
  ub4 ptarr[Chainlen];
  ub4 pdist[Chainlen];

  error_zp(orghopdist,hopcnt);

#if 0
  // sanity check
  ub4 *duprids = alloc(port2,ub4,0xff,"cmp duprids",portcnt);
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    rid = hp->rid;
    dep = orgportsbyhop[hop * 2];
    arr = orgportsbyhop[hop * 2 + 1];
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    deparr = dep * portcnt + arr;
//    if (orghopdist[hop] == 0) warning(Iter,"hop %u has distance 0",hop);
    if (duprids[deparr] == rid) warning(Iter,"duplicate hop %u %u-%u for rid %u",hop,dep,arr,rid);
    duprids[deparr] = rid;
  }
  afree(duprids,"cmp duprids");
#endif

  ub4 cnt,rawcmphopcnt = 0;

  routes = net->routes;
  chains = net->chains;
  chaincnt = net->chaincnt;

  error_z(chaincnt,ridcnt);

  error_zp(routes,part);
  error_zp(chains,part);

  rrid = hi32;

  struct eta eta;

  // pass 1: count
  for (rid = 0; rid < ridcnt; rid++) {
    if (progress(&eta,"compound rid %u of %u for %u chains pass 1",rid,ridcnt,chaincnt)) return 1;

    nclear(duphops,port2);
    for (dep = 0; dep < portcnt; dep++) duphops[dep * portcnt + dep] = 1;

    for (chain = 0; chain < chaincnt; chain++) {
      cp = chains + chain;
      cnt = cp->hopcnt;
      if (cp->rid != rid || cnt < 3) continue;

      pchlen = 0;
      for (ci = 0; ci < cnt; ci++) {
        chp = chainhops + cp->hopofs + ci;
        ghop = chp->hop;
        error_ge(ghop,ghopcnt);
        hop = g2phop[ghop];
        if (hop == hi32) continue;

        if (pchlen == maxperm) { warning(0,"limiting rid %u chain to %u",rid,maxperm); break; }

        dep = orgportsbyhop[hop * 2];
        arr = orgportsbyhop[hop * 2 + 1];
        error_ge(dep,portcnt);
        error_ge(arr,portcnt);
        pdeps[pchlen] = dep;
        parrs[pchlen] = arr;
        tdep = chp->tdep;
        tarr = chp->tarr;
        error_lt(tarr,tdep);
        pchlen++;
      }
      if (pchlen < 3) continue;

      // generate all not yet existing compounds
      // let compounds span ports not in part: search supposed to handle
      cmpcnt = 0;
      for (ci1 = 0; ci1 < pchlen - 1; ci1++) {
        dep1 = pdeps[ci1];
        for (ci2 = ci1 + 1; ci2 < pchlen; ci2++) {
          arr2 = parrs[ci2];
          deparr = dep1 * portcnt + arr2;
          if (duphops[deparr]) continue;
          duphops[deparr] = 1;
          newhopcnt++;
          cmpcnt++;
//          if (cmpcnt >= maxperm2) break;
        } // each c2
        if (cmpcnt >= maxperm2) {
          warning(0,"limiting compound on rid %u to %u combis",rid,cmpcnt);
          break;
        }
      } // each c1
    } // each chain
  } // each rid
  info(0,"\ah%u compound hops",newhopcnt);

  info(0,"compound %u chains in %u rids pass 2",chaincnt,ridcnt);

  if (newhopcnt == 0) return 0;

  newhopcnt += whopcnt;

  chop = whopcnt;

  ub4 *portsbyhop = alloc(newhopcnt * 2,ub4,0,"cmp portsbyhop",newhopcnt);

  memcpy(portsbyhop,orgportsbyhop,whopcnt * 2 * sizeof(ub4));
  afree(orgportsbyhop,"net portsbyhop");
  net->portsbyhop = portsbyhop;

  ub4 *hopdist = alloc(newhopcnt,ub4,0,"cmp hopdist",newhopcnt);
  memcpy(hopdist,orghopdist,whopcnt * sizeof(ub4));
  afree(orghopdist,"net hopdist");
  net->hopdist = hopdist;

  ub4 *hopdur = alloc(newhopcnt,ub4,0,"cmp hopdur",newhopcnt);
  memcpy(hopdur,orghopdur,whopcnt * sizeof(ub4));
  afree(orghopdur,"net hopdur");
  net->hopdur = hopdur;

  ub4 *choporg = alloc(newhopcnt * 2,ub4,0,"cmp choporg",newhopcnt);

  midur = 0;

  ub8 cumpchainlen = 0, cumgchainlen = 0, pchaincnt = 0;

  for (rid = 0; rid < ridcnt; rid++) {
    if (progress(&eta,"compound rid %u of %u for %u chains pass 2",rid,ridcnt,chaincnt)) return 1;
    nclear(duphops,port2);
    for (dep = 0; dep < portcnt; dep++) duphops[dep * portcnt + dep] = 1;

    for (chain = 0; chain < chaincnt; chain++) {
      cp = chains + chain;
      cnt = cp->hopcnt;
      if (cp->rid != rid || cnt < 3) continue;

      pchaincnt++;

      pchlen = 0;
      for (ci = 0; ci < cnt; ci++) {
        chp = chainhops + cp->hopofs + ci;
        ghop = chp->hop;
        hop = g2phop[ghop];
        if (hop == hi32) continue;
        error_ge(hop,hopcnt);
        dep = portsbyhop[hop * 2];
        arr = portsbyhop[hop * 2 + 1];
        pdep = ports + dep;
        parr = ports + arr;

        for (ci2 = 0; ci2 < pchlen; ci2++) {
          hop2 = pchain[ci2];
          if (hop != hop2) continue;
          error(Exit,"rid %u chain %u hop %u pos %u equals pos %u %s to %s",rid,chain,hop,pchlen,ci2,pdep->name,parr->name);
        }
        pchain[pchlen] = hop;
        pdeps[pchlen] = dep;
        parrs[pchlen] = arr;
        pdist[pchlen] = chp->dist;
        ptdep[pchlen] = chp->tdep;
        ptarr[pchlen] = chp->tarr;
        pchlen++;
        if (pchlen == maxperm) { warning(0,"limiting rid %u chain to %u",rid,maxperm); break; }
      }
      cumpchainlen += pchlen;
      cumgchainlen += cnt;
      if (pchlen < 3) continue;

      // generate all not yet existing compounds
      // let compounds span ports not in part: search supposed to handle
      cmpcnt = 0;
      for (ci1 = 0; ci1 < pchlen - 1; ci1++) {
        dep1 = pdeps[ci1];
        for (ci2 = ci1 + 1; ci2 < pchlen; ci2++) {
          arr2 = parrs[ci2];
          deparr = dep1 * portcnt + arr2;
          if (duphops[deparr]) continue;

          if (chop >= newhopcnt) {
            warn(0,"limiting compound to %u hops",chop - whopcnt);
            break;
          }

          duphops[deparr] = 1;

          // generate compound
          hop1 = pchain[ci1];
          hop2 = pchain[ci2];
          dist1 = pdist[ci1];
          dist2 = pdist[ci2];
          tdep1 = ptdep[ci1];
          tarr2 = ptarr[ci2];

          error_eq(hop1,hop2);

          portsbyhop[chop * 2] = dep1;
          portsbyhop[chop * 2 + 1] = arr2;
          choporg[chop * 2] = hop1;
          choporg[chop * 2 + 1] = hop2;

//          error_lt(dist2,dist1);
          dist = dist2 - dist1;
          hopdist[chop] = dist;

          error_lt(tarr2,tdep1);   // todo take average over chains if not constant
          midur = tarr2 - tdep1;
          hopdur[chop] = midur;

          chop++;
          cmpcnt++;
//          if (cmpcnt >= maxperm2) break;
        } // each c2
        if (chop >= newhopcnt) break;
        else if (cmpcnt >= maxperm2) {
          warning(0,"limiting compound on rid %u to %u combis",rid,cmpcnt);
          break;
        }
      } // each c1
      if (chop >= newhopcnt) break;
    } // each chain
    if (chop >= newhopcnt) break;
  } // each rid

  for (hop = 0; hop < chop; hop++) {
    error_ge(choporg[hop * 2],hopcnt);
    error_ge(choporg[hop * 2 + 1],hopcnt);
  }

  info(0,"%u of %u estimated compound hops",chop - whopcnt,rawcmphopcnt);
  info(0,"avg chain len %lu, global %lu",cumpchainlen / pchaincnt,cumgchainlen / pchaincnt);

  net->chopcnt = chop;
  net->choporg = choporg;

  return 0;
}
