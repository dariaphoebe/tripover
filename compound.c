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

static const ub4 maxperm = 80 * 80;

// add compound hops
int compound(struct network *net)
{
  ub4 part = net->part;
  ub4 rportcnt,port2,portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 whopcnt = net->whopcnt;
  ub4 hop,chop,newhopcnt;
  ub4 rid,rrid,ridcnt = net->ridcnt;
  ub4 chain,chaincnt = net->chaincnt;
  ub4 chainhopcnt = net->chainhopcnt;
  ub4 hichainlen = 0,hirid = 0;
  struct hop *hp,*hops = net->hops;
  struct port *pdep,*parr,*ports = net->ports;
  struct route *routes;
  struct chain *cp,*chains = net->chains;
  ub8 *chp,*chainhops = net->chainhops;
  ub8 *chmp,*chainmets = net->chainmets;
  ub4 *g2phop = net->g2phop;

  ub4 dep,arr,deparr,gdep,garr;
  ub2 rdep,rarr;
  int docompound,dbg;
  ub4 chainlen,hopofs;

  if (portcnt < 3) return info(0,"skip compound on %u port\as",portcnt);
  if (hopcnt < 2) return info(0,"skip compound on %u hop\as",hopcnt);
  if (ridcnt == 0) return info(0,"skip compound on no rids for %u hop\as",hopcnt);

  net->chopcnt = whopcnt;

  error_zp(chainmets,0);

  port2 = portcnt * portcnt;

  docompound = dorun(FLN,Runcompound);

  if (docompound == 0) return info0(0,"compound not enabled");

  info(0,"compounding %u ports %u hops",portcnt,hopcnt);

  ub4 *orgportsbyhop = net->portsbyhop;
  ub4 *orghopdist = net->hopdist;

  ub4 *orghopdur = net->hopdur;
  ub4 midur;
  ub8 met1,met2;

  ub4 *duprids = alloc(port2,ub4,0xff,"cmp duprids",portcnt);

  error_zp(orghopdist,hopcnt);

  // sanity check
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

  ub4 rawcmphopcnt = 0;

  routes = net->routes;
  chains = net->chains;
  chaincnt = net->chaincnt;

  error_z(chaincnt,ridcnt);

  error_zp(routes,part);
  error_zp(chains,part);

  for (rid = 0; rid < ridcnt; rid++) {
    chainlen = 0; rrid = hi32;
    for (hop = 0; hop < hopcnt; hop++) {
      hp = hops + hop;
      if (hp->rid != rid) continue;
      if (chainlen == 0) rrid = hp->rrid;
      else if (rrid != hp->rrid) error(Exit,"hop %u rid %u rrid %u vs %u",hop,rid,hp->rrid,rrid);
      dep = orgportsbyhop[hop * 2];
      arr = orgportsbyhop[hop * 2 + 1];
      pdep = ports + dep;
      parr = ports + arr;
      infocc(rid == 1795,0,"hop %u=%u %u-%u %s to %s",hop,hp->gid,dep,arr,pdep->name,parr->name);
      chainlen++;
    }
    if (chainlen < 3) continue;
    infocc(rid == 1795,0,"rid %u rrid %u len %u",rid,rrid,chainlen);
    // 2 directions for a single route, plus variants
    warncc(chainlen > net->hichainlen * 2 + 5,0,"rid %u rrid %u len %u max %u",rid,rrid,chainlen,net->hichainlen);

    if (chainlen > hichainlen) { hichainlen = chainlen; hirid = rid; }
    rawcmphopcnt += chainlen * chainlen;
  }
  info(0,"estimated %u compound hops, longest chain %u for rid %u",rawcmphopcnt,hichainlen,rid);

  newhopcnt = whopcnt + rawcmphopcnt;

  chop = whopcnt;

  ub4 *portsbyhop = alloc(newhopcnt * 2,ub4,0,"cmp portsbyhop",newhopcnt);
  ub4 *choporg = alloc(newhopcnt * 2,ub4,0,"cmp choporg",newhopcnt);
  memcpy(portsbyhop,orgportsbyhop,whopcnt * 2 * sizeof(ub4));

  ub4 *hopdist = alloc(newhopcnt,ub4,0,"cmp hopdist",newhopcnt);
  memcpy(hopdist,orghopdist,whopcnt * sizeof(ub4));

  ub4 *hopdur = alloc(newhopcnt,ub4,0,"cmp hopdur",newhopcnt);
  memcpy(hopdur,orghopdur,whopcnt * sizeof(ub4));

  ub1 *duphops = alloc(port2,ub1,0,"cmp duphops",portcnt);

  ub4 ghop,hop1,hop2,dep1,arr2;
  ub4 dist1,dist2,midur1,midur2,dist = 0;
  ub4 ci,ci1,ci2,pchlen,cnt,cmpcnt;

  ub4 pchain[Chainlen];
  ub4 pdeps[Chainlen];
  ub4 parrs[Chainlen];
  ub8 pmets[Chainlen];

  midur = 0;

  ub8 cumpchainlen = 0, cumgchainlen = 0, pchaincnt = 0;

  for (rid = 0; rid < ridcnt; rid++) {
    nclear(duphops,port2);
    for (dep = 0; dep < portcnt; dep++) duphops[dep * portcnt + dep] = 1;

    for (chain = 0; chain < chaincnt; chain++) {
      cp = chains + chain;
      cnt = cp->hopcnt;
      if (cp->rid != rid || cnt < 3) continue;

      pchaincnt++;

      aclear(pchain);
      aclear(pdeps);
      aclear(parrs);
      pchlen = 0;
      chp = chainhops + cp->hopofs;
      chmp = chainmets + cp->hopofs;
      for (ci = 0; ci < cnt; ci++) {
        ghop = chp[ci] & hi32;
        hop = g2phop[ghop];
        if (hop == hi32) continue;

        pchain[pchlen] = hop;
        dep = portsbyhop[hop * 2];
        arr = portsbyhop[hop * 2 + 1];
        pdeps[pchlen] = dep;
        parrs[pchlen] = arr;
        pmets[pchlen] = chmp[ci];
        pchlen++;
        if (pchlen == Chainlen) { warning(0,"limiting rid %u chain to %u",rid,pchlen); break; }
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
          duphops[deparr] = 1;

          // generate compound
          hop1 = pchain[ci1];
          hop2 = pchain[ci2];
          met1 = pmets[ci1];
          met2 = pmets[ci2];

          error_ge(chop,newhopcnt);
          portsbyhop[chop * 2] = dep1;
          portsbyhop[chop * 2 + 1] = arr2;
          choporg[chop * 2] = hop1;
          choporg[chop * 2 + 1] = hop2;

          dist1 = met1 >> 32;
          dist2 = met2 >> 32;
          error_lt(dist2,dist1);
          dist = dist2 - dist1;
          hopdist[chop] = dist;

          midur1 = met1 & hi32;
          midur2 = met2 & hi32;
          if (midur1 == hi32 || midur2 == hi32) midur = hi32;
          else {
            error_lt(midur2,midur1);
            midur = midur2 - midur1;
          }
          hopdur[chop] = midur;

          chop++;
          cmpcnt++;
          if (cmpcnt == maxperm) break;
        }
        if (cmpcnt == maxperm) {
          warning(0,"limiting compound on rid %u to %u combis",rid,cmpcnt);
          break;
        }
      }
    }
  }

  info(0,"%u of %u estimated compound hops",chop - whopcnt,rawcmphopcnt);
  info(0,"avg chain len %lu, global %lu",cumpchainlen / pchaincnt,cumgchainlen / pchaincnt);

  net->chopcnt = chop;
  net->choporg = choporg;
  net->portsbyhop = portsbyhop;
  net->hopdist = hopdist;
  net->hopdur = hopdur;

  return 0;
}
