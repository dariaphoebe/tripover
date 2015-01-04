// compound.h - create compound hops from routes

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

  todo: use chains aka tid instead of route aka rid and detect duplicates
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
  ub4 hop,chop,newhopcnt,hopcnt = net->hopcnt;
  ub4 rid,rrid,ridcnt = net->ridcnt;
  ub4 chaincnt;
  ub4 hichainlen = 0,hirid = 0;
  struct hop *hp,*hops = net->hops;
  struct port *pdep,*parr,*ports = net->ports;
  struct route *routes;
  struct chain *chains;
  ub4 dep,arr,gdep,garr;
  ub2 rdep,rarr;
  int docompound,dbg;
  ub4 chainlen;

  if (portcnt < 3) return info(0,"skip compound on %u port\as",portcnt);
  if (hopcnt < 2) return info(0,"skip compound on %u hop\as",hopcnt);
  if (ridcnt == 0) return info(0,"skip compound on no rids for %u hop\as",hopcnt);

  net->chopcnt = hopcnt;

//  if (net->istpart) return warning(0,"todo: no compound for topnet in part %u",part);

  port2 = portcnt * portcnt;

  docompound = dorun(FLN,Runcompound);

  if (docompound == 0) return info0(0,"compound not enabled");

  info(0,"compounding %u ports %u hops",portcnt,hopcnt);

  ub4 *orgportsbyhop = net->portsbyhop;
  ub4 *orghopdist = net->hopdist;

  ub4 *orghopdur = net->hopdur;
  ub8 midur,midur1;

  ub4 *duprids = alloc(port2,ub4,0xff,"cmp duprids",portcnt);

  error_zp(orghopdist,hopcnt);

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    rid = hp->rid;
    dep = orgportsbyhop[hop * 2];
    arr = orgportsbyhop[hop * 2 + 1];
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    if (orghopdist[hop] == 0) warning(Iter,"hop %u has distance 0",hop);
    if (duprids[dep * portcnt + arr] == rid) warning(Iter,"duplicate hop %u %u-%u for rid %u",hop,dep,arr,rid);
    duprids[dep * portcnt + arr] = rid;
  }

  ub4 rawcmphopcnt = 0;
  ub2 *port2rport = alloc(portcnt,ub2,0xff,"port2rport",portcnt);

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
    warncc(chainlen > net->hichainlen,0,"rid %u rrid %u len %u max %u",rid,rrid,chainlen,net->hichainlen);

    if (chainlen > hichainlen) { hichainlen = chainlen; hirid = rid; }
    rawcmphopcnt += (chainlen) * (chainlen);  // = (portcnt-1)^2
  }
  info(0,"estimated %u compound hops, longest chain %u for rid %u",rawcmphopcnt,hichainlen,rid);
  net->hichainlen = hichainlen;

  newhopcnt = hopcnt + rawcmphopcnt;

  chop = hopcnt;

  ub4 *portsbyhop = alloc(newhopcnt * 2,ub4,0,"cmp portsbyhop",newhopcnt);
  ub4 *choporg = alloc(newhopcnt * 2,ub4,0,"cmp choporg",newhopcnt);
  memcpy(portsbyhop,orgportsbyhop,hopcnt * 2 * sizeof(ub4));

  ub4 *hopdist = alloc(newhopcnt,ub4,0,"cmp hopdist",newhopcnt);
  memcpy(hopdist,orghopdist,hopcnt * sizeof(ub4));

  ub4 *hopdur = alloc(newhopcnt,ub4,0,"cmp hopdur",newhopcnt);
  memcpy(hopdur,orghopdur,hopcnt * sizeof(ub4));

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
  ub4 legcnt,cmpcnt;
  ub4 dist,dist1;

  for (rid = 0; rid < ridcnt; rid++) {

    dbg = (rid == ridcnt);

    cmpcnt = 0;

    // assign
    nclear(rdeparr,hirportcnt);
    memset(port2rport,0xff,portcnt * sizeof(ub2));
    memset(rdeparr,0xff,hirportcnt * sizeof(ub4));
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
      if (rportcnt >= hichainlen) {
        warn(0,"hop %u exceeds chain len %u rid %u",hop,rportcnt,rid);
        break;
      }
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
    if (chainlen < 3 || rportcnt >= hichainlen) continue;

    // mark
    cmpno = 0;
    for (hop = 0; hop < hopcnt; hop++) {
      hp = hops + hop;
      if (hp->rid != rid) continue;
      dep = portsbyhop[hop * 2];
      arr = portsbyhop[hop * 2 + 1];
      error_ge(dep,portcnt);
      error_ge(arr,portcnt);
      gdep = net->p2gport[dep];
      garr = net->p2gport[arr];
      if (hp->gid == 21472 || hp->gid == 21439) dbg = 1;
      if (dbg) {
        pdep = ports + dep; parr = ports + arr;
        info(0,"hop %u %u-%u rid %u rrid %u dur %u %s to %s route %s",hop,gdep,garr,rid,hp->rrid,hopdur[hop],pdep->name,parr->name,hp->name);
      }
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
      dist1 = hopdist[hop1];
      midur1 = hopdur[hop1];

      for (cmp2 = 0; cmp2 < cmplen; cmp2++) { // for all combi's
        if (cmp1 == cmp2) continue;
        rdep2 = cmpdeps[cmp2];
        rarr2 = cmparrs[cmp2];
        if (rarr1 == rdep2 || rdep1 == rarr2) continue;  // existing non-compound
        hop2 = cmphops[cmp2];
        dist = dist1 + hopdist[hop2];
        midur = midur1 + hopdur[hop2];
        legcnt = 1; cmp3 = cmp1; rdep3 = rdep1;
        while (legcnt++ < cmplen && cmp3 != cmp2) {
          cmp3 = arrcmps[rdep3];
          rdep3 = cmpdeps[cmp3];
          hop3 = cmphops[cmp3];
          dist += hopdist[hop3];
          midur += hopdur[hop2];
        }
        if (legcnt == cmplen) continue;
        cmp4 = 0;
        while (cmp4 < cmplen && !(cmpdeps[cmp4] == rdep1 && cmparrs[cmp4] == rarr2)) cmp4++;
        if (cmp4 < cmplen) continue;

        if (cmpcnt == maxperm) {
          warn(0,"limiting compounds for rid %u by %u",rid,cmpcnt);
          break;
        }
        if (rdeparr[rdep1 * rportcnt + rarr2] != hi32) continue;
        rdeparr[rdep1 * rportcnt + rarr2] = chop;

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
        hopdur[chop] = (ub4)min(midur,hi32);
        chop++;
        cmpcnt++;
      }
      if (cmpcnt == maxperm) break;
    }
  }

  info(0,"%u of %u estimated compound hops",chop,newhopcnt);

  net->chopcnt = chop;
  net->choporg = choporg;
  net->portsbyhop = portsbyhop;
  net->hopdist = hopdist;
  net->hopdur = hopdur;

  return 0;
}
