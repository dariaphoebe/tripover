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
#include "net.h"
#include "compound.h"

#undef hdrstop

static const ub4 cmp_maxperm = 256;

void inicompound(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

// add compound hops
int compound(gnet *net)
{
  ub4 rportcnt,rport2,portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 hop,chop,chopcnt,newhopcnt = 0,newcnt;
  ub4 rid,rrid,ridcnt = net->ridcnt;
  ub4 chain,chaincnt = net->chaincnt;

  ub4 hichainlen = net->hichainlen;
  ub4 hiportlen;
  struct hop *hp,*hp1,*hp2,*hops = net->hops;
  struct port *pdep,*parr,*ports = net->ports;
  struct route *rp,*routes = net->routes;
  struct chain *cp,*chains = net->chains;
  struct chainhop *chp,*chainhops = net->chainhops;
  ub8 *crp,*chainrhops = net->chainrhops;
  ub4 maxperm = min(cmp_maxperm,Chainlen);
  ub4 maxperm2 = maxperm * maxperm;

  ub4 dep,arr,deparr,da,prvda,rdep,rarr;
  int docompound;

  net->chopcnt = hopcnt;

  if (hopcnt == 0) return info(0,"skip compound on %u hop\as",hopcnt);

  net->hopcdur = alloc(hopcnt,ub4,0xff,"net hopcdur",hopcnt); // fallback

  if (portcnt < 3) return info(0,"skip compound on %u port\as",portcnt);
  if (hopcnt < 2) return info(0,"skip compound on %u hop\as",hopcnt);
  if (ridcnt == 0) return info(0,"skip compound on no rids for %u hop\as",hopcnt);

  docompound = dorun(FLN,Runcompound,1);

  if (docompound == 0) return info0(0,"compound not enabled");

  info(0,"compounding %u ports %u hops max chain %u",portcnt,hopcnt,hichainlen);

  ub4 *orgportsbyhop = net->portsbyhop;
  ub4 *orghopdist = net->hopdist;

  ub4 *orghopdur = net->hopdur;
  ub4 midur,dur,sumdur,durdif;

  ub4 hop1,hop2,rhop1,rhop2,dep1,arr2;
  ub4 dist1,dist2,dist = 0,dirdist;
  ub4 tdep,tarr,tdep1,tarr2;
  ub4 ci,ci1,ci2,pchlen,cmpcnt;

  ub4 pchain[Chainlen];
  ub4 pdeps[Chainlen];
  ub4 parrs[Chainlen];
  ub4 ptdep[Chainlen];
  ub4 ptarr[Chainlen];
  ub4 pdist[Chainlen];

  error_zp(orghopdist,hopcnt);

  ub4 cnt,cmphopcnt = 0;

  error_z(chaincnt,ridcnt);

  error_zp(routes,0);
  error_zp(chains,0);

  struct eta eta;
  int warnlim;

  ub8 cumfevcnt = 0,cumcfevcnt = 0;
  ub4 cumfhops = 0;

  ub4 hicnt = 0,hirid = 0;

  for (rid = 0; rid < ridcnt; rid++) {
    rp = routes + rid;
    if (rp->hopcnt > hicnt) { hicnt = rp->hopcnt; hirid = rid; }
  }
  rp = routes + hirid;
  info(0,"r.rid %u.%u has %u hops for len %u",rp->rrid,hirid,hicnt,rp->hichainlen);
  hiportlen = max(hichainlen,hicnt) + 10;
  ub4 hiport2 = hiportlen * hiportlen;

  ub4 *port2rport = alloc(portcnt,ub4,0,"cmp rportmap",portcnt);
  ub4 *rport2port = alloc(hiportlen,ub4,0,"cmp rportmap",hiportlen);

  ub4 *duphops = alloc(hiport2,ub4,0xff,"cmp duphops",hiportlen);
  ub4 *cduphops = alloc(hiport2,ub4,0xff,"cmp cduphops",hiportlen);

  ub4 *rport2hop = alloc(hiport2,ub4,0xff,"cmp rport2hop",hiportlen);
  ub4 *rhopcdur = alloc(hiport2,ub4,0,"cmp hopcdur",newhopcnt);
  ub4 *hopccnt = alloc(hiport2,ub4,0,"cmp hopccnt",newhopcnt);

  ub4 *hoplodur = alloc(hiport2,ub4,0,"cmp hoplodur",newhopcnt);
  ub4 *hophidur = alloc(hiport2,ub4,0,"cmp hophidur",newhopcnt);

  ub4 port2 = portcnt * portcnt;
  ub4 *duprids = alloc(port2,ub4,0xff,"cmp duprids",portcnt);

  // pass 1: count
  for (rid = 0; rid < ridcnt; rid++) {
    if (progress(&eta,"compound rid %u of %u for %u chains pass 1",rid,ridcnt,chaincnt)) return 1;
    rp = routes + rid;
    rrid = rp->rrid;

    nsethi(port2rport,portcnt);
    rportcnt = 0;
    for (hop = 0; hop < hopcnt; hop++) {
      hp = hops + hop;
      if (hp->rid != rid) continue;
      dep = hp->dep; arr = hp->arr;
      if (dep == arr) continue;

      deparr = dep * portcnt + arr;
      if (duprids[deparr] == rid) {
        warn(Iter,"duplicate hop %u %u-%u for rid %u",hop,dep,arr,rid);
        continue;
      }
      duprids[deparr] = rid;

      if (hp->reserve) {
        cumfevcnt += hp->tp.evcnt;
        cumfhops++;
      }
      if (port2rport[dep] == hi32) {
        error_ge(rportcnt,hiportlen);
        port2rport[dep] = rportcnt;
        rport2port[rportcnt++] = dep;
      }
      if (port2rport[arr] == hi32) {
        error_ge_cc(rportcnt,hiportlen,"rrid %u len %u cnt %u",rrid,rp->hichainlen,rp->hopcnt);
        port2rport[arr] = rportcnt;
        rport2port[rportcnt++] = arr;
      }
    }
    warncc(rportcnt >= hiportlen,0,"r.rid %u.%u len %u above %u",rrid,rid,rportcnt,hiportlen);
    rportcnt = min(rportcnt,hiportlen);
    rport2 = rportcnt * rportcnt;
    nsethi(duphops,rport2);
    newcnt = 0;
    for (rdep = 0; rdep < rportcnt; rdep++) duphops[rdep * rportcnt + rdep] = 0;

    warnlim = 1;
    for (chain = 0; chain < chaincnt; chain++) {
      cp = chains + chain;
      cnt = cp->hopcnt;
      if (cp->rid != rid || cnt < 3) continue;

      pchlen = 0;
      for (ci = 0; ci < cnt; ci++) {
        chp = chainhops + cp->hopofs + ci;
        hop = chp->hop;
        error_ge(hop,hopcnt);

        if (pchlen == maxperm) {
          warncc(warnlim,0,"limiting rid %u chain to %u",rid,maxperm);
          warnlim = 0;
          break;
        }

        dep = orgportsbyhop[hop * 2];
        arr = orgportsbyhop[hop * 2 + 1];
        error_ge(dep,portcnt);
        error_ge(arr,portcnt);
        if (dep == arr) continue;

        rdep = port2rport[dep];
        rarr = port2rport[arr];
        error_ge(rdep,rportcnt);
        error_ge(rarr,rportcnt);

        pdeps[pchlen] = rdep;
        parrs[pchlen] = rarr;
        tdep = chp->tdep;
        tarr = chp->tarr;
        error_lt(tarr,tdep);
        pchlen++;
      }
      if (pchlen < 3) continue;

      // generate all not yet existing compounds
      memcpy(cduphops,duphops,rport2 * sizeof(*duphops));
      cmpcnt = 0;
      for (ci1 = 0; ci1 < pchlen - 1; ci1++) {
        dep1 = pdeps[ci1];
        for (ci2 = ci1 + 1; ci2 < pchlen; ci2++) {
          arr2 = parrs[ci2];
          if (dep1 == arr2) continue;
          deparr = dep1 * rportcnt + arr2;
          if (cduphops[deparr] != hi32) continue;
          duphops[deparr] = 0;
          cmpcnt++;
          if (cmpcnt >= maxperm2) break;
        } // each c2
        if (cmpcnt >= maxperm2) {
          warning(0,"limiting compound on rid %u to %u combis",rid,cmpcnt);
          break;
        }
      } // each c1
      newcnt += cmpcnt;
    } // each chain
    cmphopcnt += newcnt;
    vrb0(0,"rid %u len %u cmp %u",rid,rportcnt,newcnt);
  } // each rid
  info(0,"\ah%u compound hops added to \ah%u",cmphopcnt,hopcnt);

  if (cmphopcnt == 0) return 0;

  info(0,"compound %u chains in %u rids pass 2",chaincnt,ridcnt);

  newhopcnt = cmphopcnt + hopcnt;

  chop = hopcnt;

  ub4 *portsbyhop = alloc(newhopcnt * 2,ub4,0,"cmp portsbyhop",newhopcnt);
  memcpy(portsbyhop,orgportsbyhop,hopcnt * 2 * sizeof(ub4));
  afree(orgportsbyhop,"net portsbyhop");
  net->portsbyhop = portsbyhop;

  ub4 *hopdist = alloc(newhopcnt,ub4,0,"cmp hopdist",newhopcnt);
  memcpy(hopdist,orghopdist,hopcnt * sizeof(ub4));
  afree(orghopdist,"net hopdist");
  net->hopdist = hopdist;

  ub4 *hopdur = alloc(newhopcnt,ub4,0,"cmp hopdur",newhopcnt);
  memcpy(hopdur,orghopdur,hopcnt * sizeof(ub4));
  afree(orghopdur,"net hopdur");
  net->hopdur = hopdur;

  ub4 *hopcdur = alloc(newhopcnt,ub4,0,"cmp hopcdur",newhopcnt);

  ub4 *choporg = alloc(newhopcnt * 2,ub4,0,"cmp choporg",newhopcnt);

  ub8 cumchainlen = 0,pchaincnt = 0;
  ub4 eqdurs = 0,aeqdurs = 0;
  ub4 cdist;

  nsethi(duprids,port2);

  // pass 2
  for (rid = 0; rid < ridcnt; rid++) {
    if (progress(&eta,"compound rid %u of %u for %u chains pass 2",rid,ridcnt,chaincnt)) return 1;

    rp = routes + rid;

    nsethi(port2rport,portcnt);
    rportcnt = 0;
    for (hop = 0; hop < hopcnt; hop++) {
      hp = hops + hop;
      if (hp->rid != rid) continue;

      dep = hp->dep; arr = hp->arr;
      if (dep == arr) continue;

      deparr = dep * portcnt + arr;
      if (duprids[deparr] == rid) continue;
      duprids[deparr] = rid;

      if (port2rport[dep] == hi32) { port2rport[dep] = rportcnt; rport2port[rportcnt++] = dep; }
      if (port2rport[arr] == hi32) { port2rport[arr] = rportcnt; rport2port[rportcnt++] = arr; }
    }
    rportcnt = min(rportcnt,hiportlen);
    rport2 = rportcnt * rportcnt;
    nsethi(duphops,rport2);
    nclear(rhopcdur,rport2);
    nclear(hopccnt,rport2);

    for (chain = 0; chain < chaincnt; chain++) {
      cp = chains + chain;
      cnt = cp->hopcnt;
      if (cp->rid != rid || cnt < 3) continue;

      pchlen = 0; cdist = 0;
      for (ci = 0; ci < cnt; ci++) {
        chp = chainhops + cp->hopofs + ci;
        hop = chp->hop;

        dep = portsbyhop[hop * 2];
        arr = portsbyhop[hop * 2 + 1];
        if (dep == arr) continue;

        pdep = ports + dep;
        parr = ports + arr;

        rdep = port2rport[dep];
        rarr = port2rport[arr];
        error_ge(rdep,rportcnt);
        error_ge(rarr,rportcnt);

        for (ci2 = 0; ci2 < pchlen; ci2++) {
          hop2 = pchain[ci2];
          if (hop != hop2) continue;
          error(Exit,"rid %u chain %u hop %u pos %u equals pos %u %s to %s",rid,chain,hop,pchlen,ci2,pdep->name,parr->name);
        }
        dist = hopdist[hop];
        hp = hops + hop;
        pchain[pchlen] = hop;
        pdeps[pchlen] = rdep;
        parrs[pchlen] = rarr;
        pdist[pchlen] = cdist;
        ptdep[pchlen] = chp->tdep;
        ptarr[pchlen] = chp->tarr;
        cdist += max(dist,1);
        pchlen++;
        if (pchlen == maxperm) { warning(0,"limiting rid %u chain to %u",rid,maxperm); break; }
      }
      if (pchlen < 3) continue;

      pchaincnt++;
      cumchainlen += pchlen;

      // generate all not yet existing compounds
      // note that some chains visit ports more than once
      memcpy(cduphops,duphops,rport2 * sizeof(*duphops));
      cmpcnt = 0;
      for (ci1 = 0; ci1 < pchlen - 1; ci1++) {
        dep1 = pdeps[ci1];
        for (ci2 = ci1 + 1; ci2 < pchlen; ci2++) {
          arr2 = parrs[ci2];
          if (dep1 == arr2) continue;
          deparr = dep1 * rportcnt + arr2;
          prvda = cduphops[deparr];
          if (prvda != hi32) {  // existing: accumulate and range duration
            tdep1 = ptdep[ci1];
            tarr2 = ptarr[ci2];
            error_lt(tarr2,tdep1);
            dur = tarr2 - tdep1;
            rhopcdur[prvda] += dur;
            hoplodur[prvda] = min(hoplodur[prvda],dur);
            hophidur[prvda] = max(hophidur[prvda],dur);

            hopccnt[prvda]++;
            continue;
          }

          if (chop >= newhopcnt) {
            warn(0,"limiting compound to %u hops",chop - hopcnt);
            break;
          }

          duphops[deparr] = deparr;
          rport2hop[deparr] = chop;

          // generate compound
          hop1 = pchain[ci1];
          hop2 = pchain[ci2];
          dist1 = pdist[ci1];
          dist2 = pdist[ci2];
          tdep1 = ptdep[ci1];
          tarr2 = ptarr[ci2];

          infocc(dist2 == 0,0,"chop %u dist %u+%u",chop,dist1,dist2);

          error_eq(hop1,hop2);

          dep = rport2port[dep1];
          arr = rport2port[arr2];
          portsbyhop[chop * 2] = dep;
          portsbyhop[chop * 2 + 1] = arr;
          choporg[chop * 2] = hop1;
          choporg[chop * 2 + 1] = hop2;
          hp1 = hops + hop1;
          hp2 = hops + hop2;
          if (hp1->reserve) {
            cumcfevcnt += hp1->tp.evcnt;
            cumfhops++;
          }
          rhop1 = hp1->rhop;
          rhop2 = hp2->rhop;
          crp = chainrhops + cp->rhopofs;

          error_lt(dist2,dist1); // todo ?

          dist = dist2 - dist1 + hopdist[hop2];

          pdep = ports + dep;
          parr = ports + arr;
          dirdist = fgeodist(pdep,parr);
          hopdist[chop] = max(dist,dirdist);

          noexit error_lt(tarr2,tdep1); // todo: day wrap ?

          error_ne(crp[rhop1] >> 32,tdep1);
          error_ne(crp[rhop2] & hi32,tarr2);

          if (tarr2 >= tdep1) midur = tarr2 - tdep1;
          else midur = hi32;
          hopdur[chop] = midur;

          // first entry
          rhopcdur[deparr] = midur;
          hoplodur[deparr] = midur;
          hophidur[deparr] = midur;
          hopccnt[deparr] = 1;

          chop++;
          cmpcnt++;
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

    for (deparr = 0; deparr < rport2; deparr++) {
      da = duphops[deparr];
      if (da == hi32) continue;
      hop = rport2hop[da];
      error_ge(hop,chop);

      cnt = hopccnt[da];
      error_z(cnt,hop);

      error_gt(hoplodur[da],hophidur[da],hop);
      durdif = hophidur[da] - hoplodur[da];
      sumdur = rhopcdur[da];
      if (durdif == 0) {
        dur = sumdur / cnt;
        warncc(dur > 1440 * 2,Iter,"chop %u dur %u",hop,dur);
        eqdurs++;
      } else if (durdif < 10) {
        dur = sumdur / cnt;
        warncc(dur > 1440 * 2,Iter,"chop %u dur %u",hop,dur);
        aeqdurs++;
      } else { // possible if loop in route
        hop1 = choporg[hop * 2];
        hop2 = choporg[hop * 2 + 1];
        infovrb(durdif > 30,Notty|Iter,"chop %u %u-%u dur %u-%u rid %u %s",hop,hop1,hop2,hoplodur[da],hophidur[da],rid,rp->name);
        dur = hi32;
      }
      hopcdur[hop] = dur;
      hopdur[hop] = min(hopdur[hop],dur);
    }

  } // each rid
  chopcnt = chop;

  afree(duprids,"cmp duprids");

  info(0,"\ah%u compound hops, \ah%u with constant duration, \ah%u within 10 min",chop - hopcnt,eqdurs,aeqdurs);
  info(0,"avg chain len %u",(ub4)(cumchainlen / chaincnt));

  net->chopcnt = chopcnt;
  net->choporg = choporg;
  net->hopcdur = hopcdur;

  // check if distance valid ( minimum 1 unit)
#if 1
  for (hop = 0; hop < chopcnt; hop++) {
    dep = portsbyhop[hop * 2];
    arr = portsbyhop[hop * 2 + 1];
    if (dep == arr) continue;
    dist = hopdist[hop];
    pdep = ports + dep;
    parr = ports + arr;
    if (hop < hopcnt) {
      infocc(dist == 0,0,"hop %u %u-%u \ag%u %s to %s",hop,dep,arr,dist,pdep->name,parr->name);
    } else {
      hop1 = choporg[hop * 2];
      hop2 = choporg[hop * 2 + 1];
      infocc(dist == 0,0,"chop %u = %u-%u %u-%u \ag%u %s to %s",hop,hop1,hop2,dep,arr,dist,pdep->name,parr->name);
    }
  }
#endif

  // allocate fare entries here, as they are for both plain and compound hops on reserved routes
  info(0,"\ah%lu + \ah%lu fare entries for %u hops",cumfevcnt,cumcfevcnt,cumfhops);

  cumfevcnt += cumcfevcnt;
  net->fareposcnt = cumfevcnt;

  ub4 *fhopofs = NULL;

  if (cumfhops) fhopofs = net->fhopofs = alloc(chopcnt,ub4,0xff,"fare fhopofs",cumfhops);

  ub4 ofs = 0;
  ub4 h1ndx,h2ndx,hopndx,h;
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    if (hp->reserve) { fhopofs[hop] = ofs; ofs += hp->tp.evcnt; }
  }
  for (chop = hopcnt; chop < chopcnt; chop++) {
    hop1 = choporg[chop * 2];
    hop2 = choporg[chop * 2 + 1];
    hp = hops + hop1;
    if (hp->reserve && fhopofs) { fhopofs[chop] = ofs; ofs += hp->tp.evcnt; }
    rid = hp->rid;
    rp = routes + rid;
    hopndx = 0; h1ndx = h2ndx = hi32; 
    while (hopndx < min(rp->hopcnt,Chainlen) && (h1ndx == hi32 || h2ndx == hi32)) {
      h = rp->hops[hopndx];
      if (h == hop1) h1ndx = hopndx;
      else if (h == hop2) h2ndx = hopndx;
      hopndx++;
    }
    if (h1ndx == hi32 || h2ndx == hi32) {
      vrb0(0,"rid %u hop %u-%u not found at %u-%u chop %u",rid,hop1,hop2,h1ndx,h2ndx,chop);
      continue;
    }
    rp->hop2chop[h1ndx * Chainlen + h2ndx] = chop;
  }

  if (cumfhops) net->fareposbase = mkblock(&net->faremem,cumfevcnt * Faregrp,ub2,Init0,"fare entries for %u reserved hops",cumfhops);

  return 0;
}
