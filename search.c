// search.c - core trip planning

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* perform the actual trip search.
   work in progress:
   initial departure-time only
   a single trip is searched for, aiming at lowest overall duration
   heuristics on distance need rework
   heuristics on duration act as branch-and-bound exhaustive search on event times only
   precomputed net needs reworked heuristics on time
   dynamic part of routing ( thru topnet ) is branch-and-bound exhaustive search on event times
   cost todo
 */

#include <string.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "time.h"
#include "util.h"
#include "os.h"

static ub4 msgfile;
#include "msg.h"

#include "net.h"

#include "search.h"

// time limit in sec for searches
static const ub4 Timelimit = 3;

// emulate walk links as frequency-based
static const ub4 walkiv_min = 10;

void inisearch(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

enum evfld { fldtime,fldtid,flddt,flddur,fldcost,fldcnt };

static ub4 timefld(ub4 ndx) { return ndx * fldcnt + fldtime; }
static ub4 tidfld(ub4 ndx) { return ndx * fldcnt + fldtid; }
static ub4 dtfld(ub4 ndx) { return ndx * fldcnt + flddt; }
static ub4 durfld(ub4 ndx) { return ndx * fldcnt + flddur; }
static ub4 costfld(ub4 ndx) { return ndx * fldcnt + fldcost; }

static void inisrc(search *src,const char *desc,ub4 arg)
{
  ub4 i,leg,t;
  ub4 *ev;
  struct trip * stp;

  fmtstring(src->desc,"%s %u",desc,arg);

  src->costlim = 0;
  src->lodist = hi32;
  src->lodt = hi32;

  for (t = 0; t < Elemcnt(src->trips); t++) {
    stp = src->trips + t;
    stp->cnt = stp->len = 0;
    for (i = 0; i < Nxleg; i++) {
      stp->trip[i * 2] = stp->trip[i * 2 + 1] = hi32;
      stp->port[i] = hi32;
    }
  }
  if (src->evpool == NULL) ev = src->evpool = alloc(Nxleg * Maxevs * fldcnt,ub4,0,"src events",Maxevs);
  else ev = src->evpool;
  for (leg = 0; leg < Nxleg; leg++) {
    src->depevs[leg] = ev;
    ev += Maxevs * fldcnt;
  }
}

static ub4 mkdepevs(search *src,lnet *net,ub4 hop,ub4 midur)
{
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 dcnt = 0,lodev,gencnt;
  ub4 leg,gndx,dmax = Maxevs;
  ub4 t,prvt,dur,lodur,*dev;
  ub8 x,*ev;
  ub4 tdep1,tarr2,ci,chcnt;
  ub4 tid;
  struct timepat *tp;
  struct chainhop *chp,*chainhops = net->chainhops;
  struct chain *cp,*chains = net->chains;
  struct hop *hp1,*hops = net->hops;
  ub4 *choporg = net->choporg;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 chaincnt = net->chaincnt;
  ub4 *hopcdur = net->hopcdur;
  ub4 *p2ghop = net->p2ghop;
  ub8 *events = net->events;
  ub4 hop1,hop2,ghop1,ghop2;

  src->dcnts[0] = 0;

  for (leg = 0; leg < Nxleg; leg++) src->devcurs[leg] = hi32;

  if (hop < hopcnt) {
    hop1 = hop; hop2 = hi32;
  } else if (hop < chopcnt) {
    hop1 = choporg[hop * 2];
    hop2 = choporg[hop * 2 + 1];
  } else return 1;

  error_ge(hop1,hopcnt);
  hp1 = hops + hop1;
  tp = &hp1->tp;

  gencnt = tp->genevcnt;

  if (gencnt == 0) return 0;

  ub4 gt0 = tp->gt0;


  ub4 t0 = tp->t0;
  ub4 t1 = tp->t1;

  if (t0 == t1) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp1->gid,t0,t1,deptmin,deptmax);
//  else if (ht0 > deptmax) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,ht0,ht1,deptmin,deptmax);
//  else if (ht1 <= deptmin) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,ht0,ht1,deptmin,deptmax);
  if (t0 + gt0 > deptmax) return info(Iter,"hop %u tt range \aD%u - \aD%u, dep window \aD%u - \aD%u",hp1->gid,t0 + gt0,t1 + gt0,deptmin,deptmax);
  else if (t1 + gt0 <= deptmin) return info(Iter,"hop %u tt range \aD%u - \aD%u, dep window \aD%u - \aD%u",hp1->gid,t0 + gt0,t1 + gt0,deptmin,deptmax);

  ev = events + tp->evofs;
  dev = src->depevs[0];
  prvt = 0;
  lodur = hi32; lodev = 0;
  for (gndx = 0; gndx < gencnt; gndx++) {
    t = (ub4)ev[gndx * 2];
    if (t < prvt) {
      warn(0,"t \ad%u before \ad%u",t+gt0,prvt+gt0);
      continue;
    }
    prvt = t;

    if (t + gt0 < deptmin) {
      continue;
    } else if (t + gt0 >= deptmax) {
      vrb0(Iter,"t %u gt0 %u deptmax %u",t,gt0,deptmax);
      break;
    }
    x = ev[gndx * 2 + 1];

    if (hop >= hopcnt) {
      dur = hopcdur[hop];
      if (dur == hi32) {
        tid = x & hi24;
        error_ge(tid,chaincnt);
        cp = chains + tid;
        chcnt = cp->hopcnt;
        chp = chainhops + cp->hopofs;
        ci = 0;
        tdep1 = tarr2 = hi32;
        ghop1 = p2ghop[hop1];
        ghop2 = p2ghop[hop2];
        while (ci < chcnt && (tdep1 == hi32 || tarr2 == hi32) ) {
          if (chp[ci].hop == ghop1) { tdep1 = chp[ci].tdep; }
          else if (chp[ci].hop == ghop2) { tarr2 = chp[ci].tarr; }
          ci++;
        }
        if (tdep1 != hi32 && tarr2 != hi32) {
          if (tarr2 >= tdep1) {
            dur = tarr2 - tdep1;
          }
          else warn(0,"chop %u-%u tdep %u-%u",ghop1,ghop2,tdep1,tarr2);
        }
      }
    } else dur = (ub4)(x >> 32); // from event
    if (dur == hi32) dur = midur;
    error_eq(dur,hi32);
    dev[timefld(dcnt)] = t + gt0;
    dev[tidfld(dcnt)] = (ub4)x & hi24; // tid+dayid
    dev[dtfld(dcnt)] = dur;
    dev[durfld(dcnt)] = dur;
    dev[costfld(dcnt)] = dur;
    if (dur < lodur) { lodur = dur; lodev = dcnt; }
    dcnt++;
    if (dcnt >= dmax) {
      warning(0,"exceeding %u dep event",dcnt);
      break;
    }
  }
  src->dcnts[0] = dcnt;
  src->duraccs[0] = tp->duracc;
  if (dcnt == 0) return 0;

  src->hop1s[0] = hop1;
  src->hop2s[0] = hop2;
  src->parts[0] = net->part;

  src->dtcurs[0] = lodur;
  src->devcurs[0] = lodev;
  infocc(src->varcnt < 20,Iter,"%u dep events for leg 0",dcnt);
  return dcnt;
}

static ub4 nxtevs(search *src,lnet *net,ub4 leg,ub4 hop,ub4 midur,ub4 dthi)
{
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 aleg;
  ub4 dcnt = 0,gencnt;
  ub4 gndx,agndx,prvgndx,dmax = Maxevs;
  ub4 adndx,adcnt;
  ub4 rt,t,tid,atid,at,dur,atarr,adur,lodt,lodev,loadev,dt,adt;
  ub4 *dev,*adev;
  ub4 cost,acost;
  ub4 ttmax = 120,ttmin = 5; // todo
  ub8 x,*ev;
  ub4 tdep1,tarr2,ci,chcnt;
  struct timepat *tp;
  struct chainhop *chp,*chainhops = net->chainhops;
  struct chain *cp,*chains = net->chains;
  struct hop *hp1,*hops = net->hops;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 chaincnt = net->chaincnt;
  ub4 *choporg = net->choporg;
  ub4 *hopcdur = net->hopcdur;
  ub4 *p2ghop = net->p2ghop;
  ub8 *events = net->events;
  ub4 hop1,hop2,ghop1,ghop2,gid;
  ub4 costperstop = src->costperstop;

  error_z(leg,0);

  src->dcnts[leg] = 0;

  if (leg < Nxleg - 1) {
    src->dtcurs[leg] = src->dtcurs[leg+1] = hi32;
    src->devcurs[leg] = src->devcurs[leg+1] = hi32;
  }

  if (hop < hopcnt) {
    hop1 = hop; hop2 = hi32;
  } else if (hop < chopcnt) {
    hop1 = choporg[hop * 2];
    hop2 = choporg[hop * 2 + 1];
  } else return warn(Iter,"walk link not expected for %u",hop);

  error_ge(hop1,hopcnt);
  hp1 = hops + hop1;
  tp = &hp1->tp;
  gid = hp1->gid;

  gencnt = tp->genevcnt;

  if (gencnt == 0) return 0;

  ub4 gt0 = tp->gt0;


  ub4 t0 = tp->t0;
  ub4 t1 = tp->t1;

  aleg = leg - 1;
  adcnt = src->dcnts[aleg];
  if (adcnt == 0) return 0;

  if (t0 == hi32) return warn(0,"hop %u tt range %u-%u, dep window %u-%u",gid,t0,t1,deptmin,deptmax);
  else if (t1 == hi32) return warn(0,"hop %u tt range %u-%u, dep window %u-%u",gid,t0,t1,deptmin,deptmax);
  else if (gt0 == hi32) return warn(0,"hop %u tt range %u-%u, dep window %u-%u",gid,t0,t1,deptmin,deptmax);

  if (t0 == t1) return info(0,"hop %u tt range %u-%u, dep window %u-%u",gid,t0,t1,deptmin,deptmax);

  if (t0 + gt0 > deptmax + src->dtlos[leg]) return info(Iter,"hop %u tt range \aD%u - \aD%u, dep window \aD%u - \aD%u",gid,t0 + gt0,t1 + gt0,deptmin,deptmax);
  else if (t1 + gt0 <= deptmin + src->dtlos[leg]) return info(Iter,"hop %u tt range \aD%u - \aD%u, dep window \aD%u - \aD%u",gid,t0 + gt0,t1 + gt0,deptmin,deptmax);

  src->duraccs[leg] = tp->duracc;

  ev = events + tp->evofs;

  dev = src->depevs[leg];
  adev = src->depevs[aleg];

  lodt = hi32; lodev = loadev = hi32;
  prvgndx = agndx = adndx = 0;
  while (adndx < adcnt && prvgndx < gencnt && dcnt < dmax) {
    at = adev[timefld(adndx)];
    adt = adev[dtfld(adndx)];
    adur = adev[durfld(adndx)];
    atid = adev[tidfld(adndx)];
    acost = adev[costfld(adndx)];
    atarr = at + adur;

    warncc(adur > hi16,0,"adur %u",adur);

    // search first candidate departure
    gndx = prvgndx;
    while (gndx < gencnt) {
      rt = (ub4)ev[gndx * 2];
      if (rt + gt0 >= atarr) break;
      gndx++;
    }
    prvgndx = gndx;

    while (gndx < gencnt) {
      rt = (ub4)ev[gndx * 2];
      x = ev[gndx * 2 + 1];
      tid = x & hi24;
      gndx++;
      t = rt + gt0;

      // below min transfer time, not same trip
      if (tid != atid && t <= atarr + tp->duracc + ttmin) {
        src->stat_nxt0++; continue;

      } else if (t < atarr) {
        warn(0,"t \ad%u before \ad%u",t,atarr);
        continue;

      // above max transfer time
      } else if (t > atarr + ttmax) {
//        info(Iter,"t %u gt0 %u deptmax %u",t,gt0,deptmax);
        src->stat_nxt3++;
        break;
      }

      // lookup current chain to get compound duration
      // many have constant end-to-end time, but account for variation
      if (hop >= hopcnt) {
        dur = hopcdur[hop];
        if (dur == hi32) { // no constant dur, search in chain
          error_ge(tid,chaincnt);
          cp = chains + tid;
          chcnt = cp->hopcnt;
          chp = chainhops + cp->hopofs;
          ci = 0;
          tdep1 = tarr2 = hi32;
          ghop1 = p2ghop[hop1];
          ghop2 = p2ghop[hop2];
          while (ci < chcnt && (tdep1 == hi32 || tarr2 == hi32) ) {
            if (chp[ci].hop == ghop1) { tdep1 = chp[ci].tdep; }
            else if (chp[ci].hop == ghop2) { tarr2 = chp[ci].tarr; }
            ci++;
          }
          if (tdep1 != hi32 && tarr2 != hi32) {
            if (tarr2 >= tdep1) dur = tarr2 - tdep1;
            else warn(Iter,"chop %u-%u tdep %u-%u",ghop1,ghop2,tdep1,tarr2); // todo
          }
        }
      } else dur = (ub4)(x >> 32);  // plain hops have duration in event
      if (dur == hi32) dur = midur;
      error_eq(dur,hi32);

      dt = adt + dur + t - atarr;   // accumulate total trip time
      if (dt >= dthi) { src->stat_nxtlim++; continue; }

      // currently, cost is duration plus transfer cost
      cost = acost + dur + t - atarr + costperstop;

      dev[timefld(dcnt)] = t;
      dev[tidfld(dcnt)] = tid;
      dev[dtfld(dcnt)] = dt;
      dev[durfld(dcnt)] = dur;
      dev[costfld(dcnt)] = cost;

      if (cost < lodt) { lodt = cost; lodev = dcnt; loadev = adndx; }

      dcnt++;
    }
    adndx++;
  }
  src->dcnts[leg] = dcnt;
  if (dcnt == 0) return 0;

  warncc(dcnt >= dmax,Iter,"exceeding %u dep event",dcnt);

  src->hop1s[leg] = hop1;
  src->hop2s[leg] = hop2;
  src->parts[leg] = net->part;

  src->dtcurs[leg] = lodt;
  src->devcurs[leg] = lodev;
  src->devcurs[aleg] = loadev;
  infocc(src->varcnt < 20,Iter,"%u dep events for leg %u, locost %u",dcnt,leg,lodt);
  return dcnt;
}

// forward existing evs, e.g. walk link after nonwalk
static ub4 fwdevs(search *src,lnet *net,ub4 leg,ub4 hop1,ub4 hop2,ub4 midur,ub4 dthi)
{
  ub4 aleg;
  ub4 dcnt = 0;
  ub4 dmax = Maxevs;
  ub4 adndx,adcnt;
  ub4 at,adur,lodt,lodev,loadev,dt,adjdt,adt,*dev,*adev;
  ub4 costperstop = src->costperstop;

  error_z(leg,hop1);
  error_ge(midur,hi16);

  src->dcnts[leg] = 0;

  if (leg < Nxleg - 1) {
    src->dtcurs[leg] = src->dtcurs[leg+1] = hi32;
    src->devcurs[leg] = src->devcurs[leg+1] = hi32;
  }

  aleg = leg - 1;
  adcnt = src->dcnts[aleg];
  if (adcnt == 0) return 0;

  dev = src->depevs[leg];
  adev = src->depevs[aleg];

  lodt = hi32; lodev = loadev = hi32;
  adndx = 0;
  while (adndx < adcnt && dcnt < dmax) {
    at = adev[timefld(adndx)];
    adt = adev[dtfld(adndx)];
    adur = adev[durfld(adndx)];

    dt = adt + 5 + midur;
    if (dt >= dthi) { adndx++; continue; }

    dev[timefld(dcnt)] = at + adur + 5;
    dev[tidfld(dcnt)] = hi32;
    dev[dtfld(dcnt)] = dt;
    dev[durfld(dcnt)] = midur;

    adjdt = dt + costperstop;
    if (adjdt < lodt) { lodt = adjdt; lodev = dcnt; loadev = adndx; }
    dcnt++;

    adndx++;
  }
  src->dcnts[leg] = dcnt;
  if (dcnt == 0) return 0;

  src->hop1s[leg] = hop1;
  src->hop2s[leg] = hop2;
  src->parts[leg] = net->part;

  src->dtcurs[leg] = lodt;
  src->devcurs[leg] = lodev;
  src->devcurs[aleg] = loadev;
  src->duraccs[leg] = 1;

  infocc(src->varcnt < 20,Iter,"%u dep events for leg %u",dcnt,leg);

  return dcnt;
}

// fill events with frequency, e.g. walk link before nonwalk or 'every x min' metro
// leg0 only
static ub4 frqevs(search *src,lnet *net,ub4 leg,ub4 hop1,ub4 hop2,ub4 midur,ub4 iv_min,ub4 dthi)
{
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 dcnt = 0;
  ub4 dmax = Maxevs;
  ub4 t,lodt,lodev,dt,*dev;

  error_nz(leg,hop1);

  src->dcnts[leg] = 0;

  if (leg < Nxleg - 1) {
    src->dtcurs[leg] = src->dtcurs[leg+1] = hi32;
    src->devcurs[leg] = src->devcurs[leg+1] = hi32;
  }

  if (midur >= dthi) return 0;

  dev = src->depevs[leg];

  t = 0;
  lodt = hi32; lodev = hi32;
  while (t + deptmin < deptmax && dcnt < dmax) {

    dt = midur;

    dev[timefld(dcnt)] = t + deptmin;
    dev[tidfld(dcnt)] = hi32;
    dev[dtfld(dcnt)] = midur;
    dev[durfld(dcnt)] = midur;

    if (dt < lodt) { lodt = dt; lodev = dcnt; }
    t += iv_min;
    dcnt++;
  }
  src->dcnts[leg] = dcnt;
  if (dcnt == 0) return 0;

  src->hop1s[leg] = hop1;
  src->hop2s[leg] = hop2;
  src->parts[leg] = net->part;

  src->dtcurs[leg] = lodt;
  src->devcurs[leg] = lodev;
  src->duraccs[leg] = 0;

  infocc(src->varcnt < 20,Iter,"%u freq events for leg %u",dcnt,leg);
  return dcnt;
}

// rescan back given the latest arrival to find best start
static ub4 prvevs(search *src,ub4 leg)
{
  ub4 aleg;
  ub4 dcnt,adndx,adcnt;
  ub4 t,at,duracc,adur,lodt,lodev,loadev,dt,*dev,*adev;
  ub4 ttmin = 5,ttmax = 120; // todo

//  if (src->histop == 8) return src->dtcurs[leg]; // xxx debug

  error_z(leg,0);
  aleg = leg - 1;

  adcnt = src->dcnts[aleg];
  error_z(adcnt,leg);

  dcnt = src->dcnts[leg];
  error_z(dcnt,leg);

  duracc = src->duraccs[aleg];

  dev = src->depevs[leg];
  adev = src->depevs[aleg];

  lodev = src->devcurs[leg];
  error_ge(lodev,dcnt);

  t = dev[timefld(lodev)];
  dt = dev[dtfld(lodev)];

  at = 0;
  lodt = hi32; loadev = hi32;
  adndx = 0;
  while (adndx < adcnt) {
    at = adev[timefld(adndx)];
    adur = adev[durfld(adndx)];

    if (at + adur + ttmax < t) { adndx++; continue; }
    else if (at + adur + ttmin + duracc >= t) break;

    dt = t - at;

    if (dt < lodt) { lodt = dt; loadev = adndx; }
    adndx++;
  }
  if (loadev == hi32) warn(Iter,"no preceding dep at leg %u between \ad%u and \ad%u",aleg,at,t); // todo
  else src->devcurs[aleg] = loadev;
  return min(dt,lodt);
}

static ub4 getevs(search *src,gnet *gnet,ub4 nleg)
{
  ub4 dcnt = 0;
  ub4 lodev,l,part;
  ub4 t,tid,at,dur,dt,*dev;
  ub4 *lodevs = src->devcurs;
  lnet *net;
  ub4 rtid,*tid2rtid;
  ub4 hop1,hop2,hopcnt,chopcnt,tidcnt;

  error_z(nleg,0);

  dt = hi32;
  at = 0;
  for (l = 0; l < nleg; l++) {
    dev = src->depevs[l];
    dcnt = src->dcnts[l];

    if (dcnt == 0) return warn(0,"no events for leg %u",l);

    part = src->parts[l];
    error_ge(part,gnet->partcnt);
    net = getnet(part);
    hopcnt = net->hopcnt;
    chopcnt = net->chopcnt;

    tid2rtid = net->tid2rtid;
    tidcnt = net->chaincnt;

    hop1 = src->hop1s[l];
    hop2 = src->hop2s[l];
    warncc(hop1 >= hopcnt && hop1 < chopcnt,0,"hop %u is compound",hop1);
    if (hop2 != hi32) {
      warncc(hop2 >= hopcnt && hop1 < chopcnt,0,"hop %u is compound",hop2);
    }
    lodev = lodevs[l];
    error_eq(lodev,hi32);

    t = dev[timefld(lodev)];
    tid = dev[tidfld(lodev)];
    dt = dev[dtfld(lodev)];
    dur = dev[durfld(lodev)];

    src->curdts[l] = dt;
    src->curdurs[l] = dur;
    src->curts[l] = t;
    error_z(t,l);
    src->curtids[l] = tid;
    if (at > t) warn(Iter,"prvtdep \ad%u after tdep \ad%u",at,t);  // todo
    at = t;

    if (tid == hi32) {
      if (hop1 >= hopcnt) info(0,"whop %u part %u ev %u of %u leg %u t \ad%u dt %u",hop1,part,lodev,dcnt,l,t,dt);
      else info(0,"hop %u part %u ev %u of %u leg %u t \ad%u dt %u no tid",hop1,part,lodev,dcnt,l,t,dt);
    } else {
      warncc(hop1 >= hopcnt,0,"walk link with tid %u",tid);
      error_ge(tid,tidcnt);
      rtid = tid2rtid[tid];
      vrb0(0,"hop %u-%u part %u ev %u of %u leg %u t \ad%u tid %u rtid %u dt %u",hop1,hop2,part,lodev,dcnt,l,t,tid,rtid,dt);
    }
  }
  src->curdt = dt;
  src->curt = src->curts[0];

  return dcnt;
}

static ub4 addevs(ub4 callee,search *src,lnet *net, ub4 *legs,ub4 nleg,ub4 nxleg,ub4 dthi,ub4 *pdtcur)
{
  ub4 whopcnt = net->whopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 midur,*hopdur = net->hopdur;
  ub4 leg,leg0,cnt = 0;
  ub4 hop;
  ub4 dtcur;
  ub4 stopcost,costperstop = src->costperstop;

  enter(callee);

  if (nxleg == 0) {
    hop = legs[0];
    error_ge(hop,whopcnt);
    midur = hopdur[hop];
    if (hop < chopcnt) cnt = mkdepevs(src,net,hop,midur);
    else cnt = frqevs(src,net,0,hop,hi32,midur,walkiv_min,dthi);    // walk links

    dtcur = src->dtcurs[0];
    *pdtcur = dtcur;
    if (cnt == 0 || nleg == 1) { leave(callee); return cnt; }
    leg0 = 1;
  } else leg0 = 0;

  for (leg = leg0; leg < nleg; leg++) {
    hop = legs[leg];
    error_ge(hop,whopcnt);
    midur = hopdur[hop];
    stopcost = costperstop * (leg + nxleg); // todo use ?
    if (hop < chopcnt) {
      cnt = nxtevs(src,net,leg + nxleg,hop,midur,dthi);
    } else { // walk links
      cnt = fwdevs(src,net,leg + nxleg,hop,hi32,midur,hi32);
    }
    dtcur = src->dtcurs[nleg + nxleg - 1];
    *pdtcur = dtcur;
    if (cnt == 0) break;
  }
  leave(callee);
  return cnt;
}

// intra-partition search for given transfers
static ub4 srclocal(ub4 callee,gnet *gnet,lnet *net,ub4 part,ub4 dep,ub4 arr,ub4 stop,search *src,const char *desc)
{
  ub2 *cnts,cnt;
  ub4 *ofss,ofs;
  ub4 *lst;
  block *lstblk;
  ub4 *lodists,lodist;
  ub4 nleg = stop + 1;
  ub4 histop = net->histop;
  ub4 deptmin,deptmax;
  ub4 evcnt;
  ub4 cost,dist = 0,leg,l;
  ub4 lodt,dtcur,dthi,sumdt;
  ub4 *vp;
  ub4 costlim = src->costlim;
  ub4 *hopdist;
  ub4 v0 = 0;
  ub4 portcnt = net->portcnt;
  ub4 whopcnt = net->whopcnt;
  struct trip *stp;
  ub4 ln = callee & 0xffff;

  if (stop > histop) { info(0,"%s: net part %u only has %u-stop connections, skip %u",desc,part,histop,stop); return 0; }

  deptmin = src->deptmin;
  deptmax = src->deptmax;

  cnts = net->concnt[stop];
  ofss = net->conofs[stop];
  lstblk = &net->conlst[stop];
  lodists = net->lodist[stop];
  hopdist = net->hopdist;

  ub4 da = dep * portcnt + arr;

  cnt = cnts[da];
  if (cnt == 0) { src->locnocnt++; vrb0(0,"no %u-stop connection %u-%u",stop,dep,arr); return 0; }

  ofs = ofss[da];
  lst = blkdata(lstblk,0,ub4);
  error_ge(ofs,net->lstlen[stop]);
  vp = lst + ofs * nleg;
  cost = hi32;

  lodist = src->lodist;
  if (lodist == 0) lodist = hi32;

  lodt = hi32;

  do {

    // distance-only
    dist = 0;
    for (leg = 0; leg < nleg; leg++) {
      l = vp[leg];
      error_ge(l,whopcnt);
      dist += hopdist[l];
    }
//    infovrb(dist == 0,0,"dist %u for var %u",dist,v0);
    if (dist < lodist) {
//      if (dist == 0) warning(0,"dist 0 for len %u",nleg);
      lodist = dist;
      src->lostop = stop;
      stp = src->trips + 1;
      for (leg = 0; leg < nleg; leg++) {
        stp->trip[leg * 2 + 1] = vp[leg];
        stp->trip[leg * 2] = part;
        stp->t[leg] = 0;
        fmtstring(stp->desc,"shortest route-only, distance %u",dist / 2);
      }
      stp->cnt = 1;
      stp->len = nleg;
      if (dist && dist == lodists[da]) info(0,"%u-stop found lodist %u at var %u %s:%u",stop,dist,v0,desc,ln);
    }

    // time
    dtcur = dthi = hi32;
    evcnt = addevs(caller,src,net,vp,nleg,0,dthi,&dtcur);
    infocc(evcnt,0,"%u event\as",evcnt);

    stp = src->trips;
    if (evcnt && (dtcur < lodt || stp->cnt == 0)) {
      lodt = dtcur;
      if (nleg > 2) {
        for (leg = nleg - 2; leg; leg--) {
          dtcur = prvevs(src,leg);
          if (dtcur == 0) break;
        }
      }
      if (dtcur == 0) { src->stat_noprv++; v0++; vp += nleg; continue; }

      evcnt = getevs(src,gnet,nleg);
      error_z(evcnt,nleg);
      for (leg = 0; leg < nleg; leg++) {
        stp->trip[leg * 2 + 1] = vp[leg];
        stp->trip[leg * 2] = part;

        error_z(src->curts[leg],leg);
        stp->t[leg] = src->curts[leg];
        stp->dur[leg] = src->curdurs[leg];
        stp->tid[leg] = src->curtids[leg];
        info(0,"  leg %u dep \ad%u",leg,src->curts[leg]);
      }
      leg = nleg - 1;
      sumdt = src->curts[leg] - src->curts[0] + src->curdurs[leg];
      fmtstring(stp->desc,"fastest, time %u distance %u",sumdt,dist / 2);
      stp->cnt = 1;
      stp->len = nleg;
      if (leg) {
        leg--;
        src->lodt = src->curdts[leg];
        src->lot = src->curts[leg];
        src->lotid = src->curtids[leg];
      }
        src->lodist = dist;
    }

    v0++;
    vp += nleg;
    src->locvarcnt++;
  } while (v0 < cnt && cost > costlim && globs.sigint == 0);

  if (src->trips[1].cnt == 0) info(0,"no time for trip %u-%u on \ad%u-\ad%u",dep,arr,deptmin,deptmax);
  if (src->trips[1].cnt == 0 && src->trips[0].cnt == 0) return info(0,"no route for trip %u-%u",dep,arr);

  src->locsrccnt++;

  src->lodist = lodist;
  info(0,"%s: found %u-stop conn %u-%u",desc,stop,dep,arr);

  return 1;
}

// within single part
static ub4 srcglocal(struct gnetwork *gnet,ub4 part,ub4 gdep,ub4 garr,ub4 stop,search *src)
{
  ub4 dep,arr;
  struct network *net = getnet(part);
  ub4 portcnt = net->portcnt;
  ub4 gportcnt = gnet->portcnt;

  error_ge(gdep,gportcnt);

  dep = net->g2pport[gdep];
  arr = net->g2pport[garr];
  error_ge(dep,portcnt);
  error_ge(arr,portcnt);
  return srclocal(caller,gnet,net,part,dep,arr,stop,src,"local");
}

#define Distbins 256
#define Percbins 128

// core interpart search loop
static ub4 srcxpart2(gnet *gnet,lnet *tnet,ub4 dpart,ub4 apart,ub4 gdep,ub4 garr,ub4 gdmid,ub4 gamid,search *src)
{
  ub4 tpart = tnet->part;
  ub4 tportcnt,dportcnt,aportcnt;
  ub4 whopcnt,twhopcnt,awhopcnt;
  struct network *dnet,*anet;
  ub4 dep,arr,dmid,amid,depmid,tdepmid,tdmid,tamid,amidarr;
  ub2 *cnts,*tcnts,*acnts,cnt,tcnt,acnt,var,tvar,avar;
  ub4 *ofss,*tofss,*aofss,ofs,tofs,aofs;
  ub4 *vp,*tvp,*avp;
  ub4 dstop,tstop,astop,histop;
  ub4 nleg,ntleg,naleg,l,leg,tleg,aleg,triplen;
  block *lstblk,*tlstblk,*alstblk;
  ub4 *lst,*tlst,*alst;
  ub4 *lodists,*tlodists,*alodists,lodist,tlodist,alodist;
  ub4 *hopdist,*thopdist,*ahopdist;
  ub4 dist,distrange,distiv,iv,pct;
  ub4 dt,dtcur,dthi,dtndx,dtndx2,tdep,tdur,tid;
  ub4 varcnt = 0;
  ub4 distbins[Distbins];
  ub4 distsums[Distbins];
  ub4 distlims[Percbins];
  ub4 *topdts;
  ub4 topdt1 = Topdts - 1;
  ub4 evcnt;

  ub4 *choporg;
  ub4 *hopdur;
  struct trip *stp;

  ub4 dvarcnt,tvarcnt,avarcnt,dvarxcnt,tvarxcnt,avarxcnt,totvarcnt;

  dvarcnt = tvarcnt = avarcnt = dvarxcnt = tvarxcnt = avarxcnt = 0;
  totvarcnt = src->varcnt;

  aclear(distbins);
  aclear(distsums);
  topdts = src->topdts;

  dnet = getnet(dpart);
  anet = getnet(apart);

  tportcnt = tnet->portcnt;
  dportcnt = dnet->portcnt;
  aportcnt = anet->portcnt;

  whopcnt = dnet->whopcnt;
  twhopcnt = tnet->whopcnt;
  awhopcnt = anet->whopcnt;

  choporg = dnet->choporg;
  hopdur = dnet->hopdur;

  error_zp(choporg,0);
  error_zp(hopdur,0);

  histop = src->histop;

  dep = dnet->g2pport[gdep];
  dmid = dnet->g2pport[gdmid];
  depmid = dep * dportcnt + dmid;

  tdmid = tnet->g2pport[gdmid];
  tamid = tnet->g2pport[gamid];
  tdepmid = tdmid * tportcnt + tamid;

  amid = anet->g2pport[gamid];
  arr = anet->g2pport[garr];
  amidarr = amid * aportcnt + arr;

  distrange = max(src->geodist,1) * 10;

  // todo: verification
  tcnt = 0;
  for (tstop = 0; tstop <= min(tnet->histop,histop); tstop++) {
    tcnts = tnet->concnt[tstop];
    tcnt = tcnts[tdepmid];
    if (tcnt) {
      vrb0(0,"histop %u net %u dist %u top con at %u stop",histop,dnet->histop,src->geodist,tstop);
      break;
    }
  }
  if (tcnt == 0) {
    info(Iter,"no conn for top %u-%u",tdmid,tamid);
    return 0;
  }

  for (pct = 0; pct < Percbins; pct++) distlims[pct] = pct * 5;

  for (dstop = 0; dstop <= min(dnet->histop,histop); dstop++) {

    nleg = dstop + 1;
    cnts = dnet->concnt[dstop];
    cnt = cnts[depmid];
    if (cnt == 0) continue;
    dvarcnt += cnt;

    ofss = dnet->conofs[dstop];
    lstblk = &dnet->conlst[dstop];
    lodists = dnet->lodist[dstop];
    hopdist = dnet->hopdist;

    ofs = ofss[depmid];
    lodist = lodists[depmid];
    lst = blkdata(lstblk,0,ub4);
    error_ge(ofs,dnet->lstlen[dstop]);
    vp = lst + ofs * nleg;

    for (var = 0; var < cnt; var++) {

      if (globs.sigint) return 0;

      dist = 0;
      for (leg = 0; leg < nleg; leg++) {
        l = vp[leg];
        error_ge(l,whopcnt);
        dist += hopdist[l];
      }
      dist = max(dist,lodist);
      distiv = (dist - lodist) * Distbins / distrange;
      if (totvarcnt > 5 && distiv >= Distbins) { vp += nleg; continue; }

      if (varcnt > 50 && distiv < Distbins) {
        pct = distsums[distiv] * Percbins / varcnt;
        if (pct >= Percbins || distsums[distiv] > distlims[pct]) { vp += nleg; continue; }
      }

      dvarxcnt++;

      dtcur = hi32;
      dthi = topdts[topdt1];

      evcnt = addevs(caller,src,dnet,vp,nleg,0,dthi,&dtcur);

      if (evcnt == 0 || dtcur >= topdts[topdt1]) {
//        info(0,"evcnt %u dtcur %u leg %u",evcnt,dtcur,leg);
        vp += nleg;
        continue;
      }

      for (tstop = 0; tstop <= min(tnet->histop,histop - dstop); tstop++) {
        ntleg = tstop + 1;
        tcnts = tnet->concnt[tstop];
        tcnt = tcnts[tdepmid];
        if (tcnt == 0) continue;
        tvarcnt += tcnt;

        tofss = tnet->conofs[tstop];
        tlstblk = &tnet->conlst[tstop];
        tlodists = tnet->lodist[tstop];
        thopdist = tnet->hopdist;

        tofs = tofss[tdepmid];
        tlodist = tlodists[tdepmid];
        tlst = blkdata(tlstblk,0,ub4);
        if (tofs >= tnet->lstlen[tstop]) {
          info(Iter,"ofs %u stop %u part %u",tofs,tstop,tpart);
        }
        bound(tlstblk,tofs * ntleg,ub4);
        error_ge(tofs,tnet->lstlen[tstop]);
        tvp = tlst + tofs * ntleg;

        for (tvar = 0; tvar < tcnt; tvar++) {

          for (tleg = 0; tleg < ntleg; tleg++) {
            l = tvp[tleg];
            error_ge(l,twhopcnt);
            dist += thopdist[l];
          }
          dist = max(dist,tlodist + lodist);
          noexit error_lt(dist,tlodist + lodist); // todo
          distiv = (dist - lodist - tlodist) * Distbins / distrange;
          if (totvarcnt > 5 && distiv >= Distbins) { tvp += ntleg; continue; }

          if (distiv < Distbins && varcnt > 50) {
            pct = distsums[distiv] * Percbins / varcnt;
            if (pct >= Percbins || distsums[distiv] > distlims[pct]) { tvp += ntleg; continue; }
          }

          dtcur = hi32;

          evcnt = addevs(caller,src,tnet,tvp,ntleg,nleg,dthi,&dtcur);
          infocc(evcnt,Iter,"%u event\as",evcnt);

          if (evcnt == 0 || dtcur >= topdts[topdt1]) {
            tvp += ntleg; continue;
          }

          tvarxcnt++;

          for (astop = 0; astop <= min(anet->histop,histop - dstop - tstop); astop++) {
            naleg = astop + 1;
            acnts = anet->concnt[astop];
            acnt = acnts[amidarr];
            if (acnt == 0) continue;
            avarcnt += acnt;

            aofss = anet->conofs[astop];
            alstblk = anet->conlst + astop;
            alodists = anet->lodist[astop];
            ahopdist = anet->hopdist;

            aofs = aofss[amidarr];
            alodist = alodists[amidarr];
            alst = blkdata(alstblk,0,ub4);
            error_ge(aofs,anet->lstlen[astop]);
            bound(alstblk,aofs * naleg,ub4);
            avp = alst + aofs * naleg;
            bound(alstblk,(aofs + acnt) * naleg,ub4);

            for (avar = 0; avar < acnt; avar++) {

              for (aleg = 0; aleg < naleg; aleg++) {
                l = avp[aleg];
                if (l == hi32) {
                  error(0,"arr part %u port %u-%u stop %u var %u leg %u %s",apart,amid,arr,astop,avar,aleg,alstblk->desc);
                  break;
                }
                error_ge(l,awhopcnt);
                dist += ahopdist[l];
              }
              dist = max(dist,lodist + tlodist + alodist);
//              noexit error_lt(dist,alodist + tlodist + lodist);  todo
              distiv = (dist - lodist - tlodist - alodist) * Distbins / distrange;
              if (totvarcnt > 5 && distiv >= Distbins) { avp += naleg; continue; }

              if (distiv < Distbins && varcnt > 50) {
                pct = distsums[distiv] * Percbins / varcnt;
                if (pct >= Percbins || distsums[distiv] > distlims[pct]) { avp += naleg; continue; }
              }

              avarxcnt++;
              distiv = min(distiv,Distbins-1);
              distbins[distiv]++;
              for (iv = 0; iv <= distiv; iv++) distsums[iv]++;

              dtcur = hi32;

              evcnt = addevs(caller,src,anet,avp,naleg,nleg + ntleg,dthi,&dtcur);
              infocc(evcnt,Iter,"%u event\as",evcnt);

              if (evcnt == 0 || dtcur >= topdts[topdt1]) { avp += naleg; continue; }

              triplen = nleg + ntleg + naleg;
              error_ge(triplen,Nxleg);

              dt = dtcur;
              if (triplen > 2) {
                for (aleg = triplen - 2; aleg; aleg--) {
                  dt = prvevs(src,aleg);
                  if (dt == 0) break;
                }
              }
              if (dt == 0) { avp += naleg; continue; }

              evcnt = getevs(src,gnet,triplen);
              if (evcnt == 0) { avp += naleg; continue; }

              dtndx = 0;
              while (dtndx < Topdts && dtcur >= topdts[dtndx]) dtndx++;
              if (dtndx == topdt1) topdts[dtndx] = dtcur;
              else if (dtndx < topdt1) {
                for (dtndx2 = topdt1; dtndx2 > dtndx; dtndx2--) topdts[dtndx2] = topdts[dtndx2-1];
                info(Iter,"put dt %u at pos %u",dtcur,dtndx);
                topdts[dtndx] = dtcur;
              }
              dthi = topdts[topdt1];
              varcnt++;

              stp = src->trips;

              if (stp->cnt == 0 || src->curdt < src->lodt) {
                for (leg = 0; leg < nleg; leg++) {
                  stp->trip[leg * 2 + 1] = vp[leg];
                  stp->trip[leg * 2] = dpart;
                }
                for (tleg = 0; tleg < ntleg; tleg++) {
                  stp->trip[leg * 2 + 1] = tvp[tleg];
                  stp->trip[leg * 2] = tpart;
                  leg++;
                }
                for (aleg = 0; aleg < naleg; aleg++) {
                  stp->trip[leg * 2 + 1] = avp[aleg];
                  stp->trip[leg * 2] = apart;
                  leg++;
                }
                info(0,"store trip \aV%u%p dur %u dep \ad%u",triplen,(void *)stp->trip,src->curdt,src->curt);
                for (leg = 0; leg < triplen; leg++) {
                  error_z(src->curts[leg],leg);
                  tdep = src->curts[leg];
                  tdur = src->curdurs[leg];
                  tid = src->curtids[leg];
                  stp->t[leg] = tdep;
                  stp->dur[leg] = tdur;
                  stp->tid[leg] = tid;
                  info(0,"  leg %u dep \ad%u arr \ad%u tid %u",leg,tdep,tdep + tdur,tid);
                }
                stp->cnt = 1;
                stp->len = triplen;
                leg--;
                src->lodt = src->curdts[leg];
                src->lot = src->curts[leg];
                src->lotid = src->curtids[leg];
                src->lodist = dist;
              }
              avp += naleg;

            } // avar
          } // each astop
          tvp += ntleg;
        } // each tvar
      } // each tstop
      vp += nleg;
    } // each depvar
  } // each dstop

  src->dvarcnt += dvarcnt;
  src->dvarxcnt += dvarxcnt;
  src->tvarcnt += tvarcnt;
  src->tvarxcnt += tvarxcnt;
  src->avarcnt += avarcnt;
  src->avarxcnt += avarxcnt;
  src->varcnt += varcnt;

  if (varcnt) info(0,"%u of %u depvars %u of %u midvars %u of %u arrvars %u total",dvarxcnt,dvarcnt,tvarxcnt,tvarcnt,avarxcnt,avarcnt,varcnt);

  return varcnt;
}

// special case of core search loop : single node at top
static ub4 srcxpart2t(gnet *gnet,ub4 dpart,ub4 apart,ub4 gdep,ub4 garr,ub4 gamid,search *src)
{
  ub4 dportcnt,aportcnt;
  ub4 whopcnt,awhopcnt;
  struct network *dnet,*anet;
  ub4 dep,arr,dmid,amid,depmid,amidarr;
  ub2 *cnts,*acnts,cnt,acnt,var,avar;
  ub4 *ofss,*aofss,ofs,aofs;
  ub4 *vp,*avp;
  ub4 dstop,astop,histop;
  ub4 nleg,ntleg,naleg,l,leg,aleg,triplen;
  block *lstblk,*alstblk;
  ub4 *lst,*alst;
  ub4 *lodists,*alodists,lodist,alodist;
  ub4 *hopdist,*ahopdist;
  ub4 dist,distrange,distiv,iv,pct;
  ub4 dt,dtcur,dthi,dtndx,dtndx2;
  ub4 varcnt = 0;
  ub4 distbins[Distbins];
  ub4 distsums[Distbins];
  ub4 distlims[Percbins];
  ub4 *topdts;
  ub4 topdt1 = Topdts - 1;
  ub4 evcnt;

  ub4 *choporg;
  ub4 *hopdur;
  struct trip *stp;

  ub4 dvarcnt,tvarcnt,avarcnt,dvarxcnt,tvarxcnt,avarxcnt,totvarcnt;

  dvarcnt = tvarcnt = avarcnt = dvarxcnt = tvarxcnt = avarxcnt = 0;
  totvarcnt = src->varcnt;

  aclear(distbins);
  aclear(distsums);
  topdts = src->topdts;

  dnet = getnet(dpart);
  anet = getnet(apart);

  dportcnt = dnet->portcnt;
  aportcnt = anet->portcnt;

  whopcnt = dnet->whopcnt;
  awhopcnt = anet->whopcnt;

  choporg = dnet->choporg;
  hopdur = dnet->hopdur;

  error_zp(choporg,0);
  error_zp(hopdur,0);

  histop = src->histop;

  dep = dnet->g2pport[gdep];
  dmid = dnet->g2pport[gamid];
  depmid = dep * dportcnt + dmid;

  amid = anet->g2pport[gamid];
  arr = anet->g2pport[garr];
  amidarr = amid * aportcnt + arr;

  distrange = max(src->geodist,1) * 10;

  for (pct = 0; pct < Percbins; pct++) distlims[pct] = pct * 5;

  for (dstop = 0; dstop <= min(dnet->histop,histop); dstop++) {

    nleg = dstop + 1;
    cnts = dnet->concnt[dstop];
    cnt = cnts[depmid];
    if (cnt == 0) continue;
    dvarcnt += cnt;

    ofss = dnet->conofs[dstop];
    lstblk = &dnet->conlst[dstop];
    lodists = dnet->lodist[dstop];
    hopdist = dnet->hopdist;

    ofs = ofss[depmid];
    lodist = lodists[depmid];
    lst = blkdata(lstblk,0,ub4);
    error_ge(ofs,dnet->lstlen[dstop]);
    vp = lst + ofs * nleg;

    for (var = 0; var < cnt; var++) {

      if (globs.sigint) return 0;

      dist = 0;
      for (leg = 0; leg < nleg; leg++) {
        l = vp[leg];
        error_ge(l,whopcnt);
        dist += hopdist[l];
      }
      dist = max(dist,lodist);
      distiv = (dist - lodist) * Distbins / distrange;
      if (totvarcnt > 5 && distiv >= Distbins) { vp += nleg; continue; }

      if (varcnt > 50 && distiv < Distbins) {
        pct = distsums[distiv] * Percbins / varcnt;
        if (pct >= Percbins || distsums[distiv] > distlims[pct]) { vp += nleg; continue; }
      }

      dvarxcnt++;

      dtcur = hi32;
      dthi = topdts[topdt1];

      evcnt = addevs(caller,src,dnet,vp,nleg,0,dthi,&dtcur);

      if (evcnt == 0 || dtcur >= topdts[topdt1]) {
//        info(0,"evcnt %u dtcur %u leg %u",evcnt,dtcur,leg);
        vp += nleg;
        continue;
      }

      ntleg = 0;

          for (astop = 0; astop <= min(anet->histop,histop - dstop); astop++) {
            naleg = astop + 1;
            acnts = anet->concnt[astop];
            acnt = acnts[amidarr];
            if (acnt == 0) continue;
            avarcnt += acnt;

            aofss = anet->conofs[astop];
            alstblk = anet->conlst + astop;
            alodists = anet->lodist[astop];
            ahopdist = anet->hopdist;

            aofs = aofss[amidarr];
            alodist = alodists[amidarr];
            alst = blkdata(alstblk,0,ub4);
            error_ge(aofs,anet->lstlen[astop]);
            bound(alstblk,aofs * naleg,ub4);
            avp = alst + aofs * naleg;
            bound(alstblk,(aofs + acnt) * naleg,ub4);

            for (avar = 0; avar < acnt; avar++) {

              for (aleg = 0; aleg < naleg; aleg++) {
                l = avp[aleg];
                if (l == hi32) {
                  error(0,"arr part %u port %u-%u stop %u var %u leg %u %s",apart,amid,arr,astop,avar,aleg,alstblk->desc);
                  break;
                }
                error_ge(l,awhopcnt);
                dist += ahopdist[l];
              }
              dist = max(dist,lodist + alodist);
//              noexit error_lt(dist,alodist + tlodist + lodist);  todo
              distiv = (dist - lodist - alodist) * Distbins / distrange;
              if (totvarcnt > 5 && distiv >= Distbins) { avp += naleg; continue; }

              if (distiv < Distbins && varcnt > 50) {
                pct = distsums[distiv] * Percbins / varcnt;
                if (pct >= Percbins || distsums[distiv] > distlims[pct]) { avp += naleg; continue; }
              }

              avarxcnt++;
              distiv = min(distiv,Distbins-1);
              distbins[distiv]++;
              for (iv = 0; iv <= distiv; iv++) distsums[iv]++;

              dtcur = hi32;

              evcnt = addevs(caller,src,anet,avp,naleg,nleg + ntleg,dthi,&dtcur);
              infocc(evcnt,Iter,"%u event\as",evcnt);

              if (evcnt == 0 || dtcur >= topdts[topdt1]) { avp += naleg; continue; }

              triplen = nleg + ntleg + naleg;
              error_ge(triplen,Nxleg);

              dt = dtcur;
              if (triplen > 2) {
                for (aleg = triplen - 2; aleg; aleg--) {
                  dt = prvevs(src,aleg);
                  if (dt == 0) break;
                }
              }
              if (dt == 0) { avp += naleg; continue; }

              evcnt = getevs(src,gnet,triplen);
              if (evcnt == 0) { avp += naleg; continue; }

              dtndx = 0;
              while (dtndx < Topdts && dtcur >= topdts[dtndx]) dtndx++;
              if (dtndx == topdt1) topdts[dtndx] = dtcur;
              else if (dtndx < topdt1) {
                for (dtndx2 = topdt1; dtndx2 > dtndx; dtndx2--) topdts[dtndx2] = topdts[dtndx2-1];
                info(Iter,"put dt %u at pos %u",dtcur,dtndx);
                topdts[dtndx] = dtcur;
              }
              dthi = topdts[topdt1];
              varcnt++;

              stp = src->trips;
              if (stp->cnt == 0 || src->curdt < src->lodt) {
                for (leg = 0; leg < nleg; leg++) {
                  stp->trip[leg * 2 + 1] = vp[leg];
                  stp->trip[leg * 2] = dpart;
                }
                for (aleg = 0; aleg < naleg; aleg++) {
                  stp->trip[leg * 2 + 1] = avp[aleg];
                  stp->trip[leg * 2] = apart;
                  leg++;
                }
                info(0,"store trip \aV%u%p dur %u dep \ad%u tid %x",triplen,(void *)stp->trip,src->curdt,src->curt,src->lotid);
                for (leg = 0; leg < triplen; leg++) {
                  error_z(src->curts[leg],leg);
                  stp->t[leg] = src->curts[leg];
                  stp->dur[leg] = src->curdurs[leg];
                  stp->tid[leg] = src->curtids[leg];
                  info(0,"  leg %u dep \ad%u",leg,src->curts[leg]);
                }
                stp->cnt = 1;
                stp->len = triplen;
                leg--;
                src->lodt = src->curdt;
                src->lot = src->curts[leg];
                src->lotid = src->curtids[leg];
                src->lodist = dist;
              }
              avp += naleg;

            } // avar
          } // each astop

      vp += nleg;
    } // each depvar
  } // each dstop

  src->dvarcnt += dvarcnt;
  src->dvarxcnt += dvarxcnt;
  src->tvarcnt += tvarcnt;
  src->tvarxcnt += tvarxcnt;
  src->avarcnt += avarcnt;
  src->avarxcnt += avarxcnt;
  src->varcnt += varcnt;

  if (varcnt) info(0,"%u of %u depvars %u of %u midvars %u of %u arrvars %u total",dvarxcnt,dvarcnt,tvarxcnt,tvarcnt,avarxcnt,avarcnt,varcnt);

  return varcnt;
}

// main loop of interpart search : dep,top,arr each separate parts 
static ub4 srcxpart(struct gnetwork *gnet,ub4 gdep,ub4 garr,search *src,char *ref)
{
  struct network *tnet;
  ub1 *portparts = gnet->portparts;
  ub4 partcnt = gnet->partcnt;
  ub4 deparr;

  ub4 conn = 0;
  ub4 tpart,dpart,apart;

  ub4 tdmid,tamid,gamid,gdmid;
  ub4 tportcnt;
  ub4 *tp2g;
  ub4 stats[8];
  ub4 xpartcnt = 0,estvar;
  ub4 iv,niv = Elemcnt(stats);

  struct eta eta;

  ub2 *xmap,*xamap,*xmapdbase,*xmapabase,xm,xam;
  ub1 *tmap;
  block *xpartdmap = &gnet->xpartdmap;
  block *xpartamap = &gnet->xpartamap;

  if (partcnt == 1) { error(0,"interpart search called without partitions, ref %s",ref); return 0; }

  aclear(stats);

  memset(src->topdts,0xff,sizeof(src->topdts));

  tpart = gnet->tpart;
  tnet = getnet(tpart);
  tportcnt = tnet->portcnt;
  tp2g = tnet->p2gport;

  xmapdbase = blkdata(xpartdmap,0,ub2);
  xmapabase = blkdata(xpartamap,0,ub2);

  xmap = xmapdbase + gdep * tportcnt;
  xamap = xmapabase + garr * tportcnt;
  tmap = tnet->conmask;

/* foreach (gdep,gdtmid) from xmap
     foreach (garr,gatmid) from xmap
       foreach alt(gdep,gdtmid)
         trip = ...
         foreach alt(garr,gatmid)
           trip .= ...
           foreach alt(gdtmid,gatmid)
             trip .= ...
 */

  for (tdmid = 0; tdmid < tportcnt; tdmid++) {
    xm = xmap[tdmid];
    if (xm == 0) continue;
    stats[0]++;
    gdmid = tp2g[tdmid];
    for (tamid = 0; tamid < tportcnt; tamid++) {
//      if (tdmid == tamid) continue;

      xam = xamap[tamid];
      if (xam == 0) continue;

      stats[1]++;
      deparr = tdmid * tportcnt + tamid;
      if (tmap[deparr] == 0) continue;
      vrb0(0,"conn %x for top %u-%u",tmap[deparr],tdmid,tamid);
      stats[3]++;
      gamid = tp2g[tamid];

      // assess trips gdep-gdmid-gamid-garr, with gdmid-gamid in top

      for (dpart = 0; dpart < tpart; dpart++) { // foreach part with dep as member
        if (portparts[gdep * partcnt + dpart] == 0) continue;
        stats[4]++;
        if (portparts[gdmid * partcnt + dpart] == 0) continue;
        stats[5]++;

        for (apart = 0; apart < tpart; apart++) { // foreach part with arr as member
          if (portparts[garr * partcnt + apart] == 0) continue;
          stats[6]++;
          if (portparts[gamid * partcnt + apart] == 0) continue;

          xpartcnt++;
          stats[7] = xpartcnt;
          estvar = xpartcnt * tportcnt / max(tdmid,1);

          if (progress(&eta,"step %u of %u, %u vars",xpartcnt - 1,estvar,src->varcnt)) return 0;
          if (xpartcnt == 1) eta.limit = src->querytlim;

          if (gdmid == gamid) conn += srcxpart2t(gnet,dpart,apart,gdep,garr,gdmid,src);
          else conn += srcxpart2(gnet,tnet,dpart,apart,gdep,garr,gdmid,gamid,src);
        }
      }
    }
  }

  for (iv = 0; iv < niv; iv++) info(0,"stats %u %u",iv,stats[iv]);

  return conn;
}

// special case of main interpart search : dep in top 
static ub4 srcxdpart(struct gnetwork *gnet,ub4 gdep,ub4 garr,search *src,char *ref)
{
  struct network *tnet;
  ub1 *portparts = gnet->portparts;
  ub4 partcnt = gnet->partcnt;
  ub4 tpart = gnet->tpart;
  ub4 deparr;

  ub4 conn = 0;
  ub4 dpart,apart;

  ub4 tdmid,tamid,gamid,gdmid;
  ub4 tportcnt;
  ub4 *t2g,*g2t;
  ub4 stats[8];
  ub4 xpartcnt = 0,estvar;
  ub4 iv,niv = Elemcnt(stats);

  struct eta eta;

  ub2 *xamap,*xmapabase,xam;
  ub1 *tmap;
  block *xpartamap = &gnet->xpartamap;

  if (partcnt == 1) { error(0,"interpart search called without partitions, ref %s",ref); return 0; }

  aclear(stats);

  memset(src->topdts,0xff,sizeof(src->topdts));

  tnet = getnet(tpart);
  tportcnt = tnet->portcnt;
  t2g = tnet->p2gport;
  g2t = tnet->g2pport;

  xmapabase = blkdata(xpartamap,0,ub2);

  xamap = xmapabase + garr * tportcnt;
  tmap = tnet->conmask;

  gdmid = gdep;
  tdmid = g2t[gdmid];
  error_eq(tdmid,hi32);

  for (tamid = 0; tamid < tportcnt; tamid++) {
//      if (tdmid == tamid) continue;

    xam = xamap[tamid];
    if (xam == 0) continue;

    stats[1]++;
    deparr = tdmid * tportcnt + tamid;
    if (tmap[deparr] == 0) continue;
    vrb0(0,"conn %x for top %u-%u",tmap[deparr],tdmid,tamid);
    stats[3]++;
    gamid = t2g[tamid];

    // assess trips gdmid-gamid-garr, with gdmid-gamid in top

    dpart = tpart;

    for (apart = 0; apart < tpart; apart++) { // foreach part with arr as member
      if (portparts[garr * partcnt + apart] == 0) continue;
      stats[6]++;
      if (portparts[gamid * partcnt + apart] == 0) continue;

      xpartcnt++;
      stats[7] = xpartcnt;
      estvar = xpartcnt * tportcnt / max(tdmid,1);

      if (progress(&eta,"step %u of %u, %u vars",xpartcnt - 1,estvar,src->varcnt)) return 0;
      if (xpartcnt == 1) eta.limit = src->querytlim;

      conn += srcxpart2t(gnet,dpart,apart,gdep,garr,gamid,src);
    }
  }

  for (iv = 0; iv < niv; iv++) info(0,"stats %u %u",iv,stats[iv]);

  return conn;
}

// special case interpart search : arr in top
static ub4 srcxapart(struct gnetwork *gnet,ub4 gdep,ub4 garr,search *src,char *ref)
{
  struct network *tnet;
  ub1 *portparts = gnet->portparts;
  ub4 partcnt = gnet->partcnt;


  ub4 conn = 0;
  ub4 tpart,dpart,apart;

  ub4 tdmid,tamid,gdmid;
  ub4 tportcnt;
  ub4 *t2g,*g2t;
  ub4 stats[8];
  ub4 xpartcnt = 0,estvar;
  ub4 iv,niv = Elemcnt(stats);

  struct eta eta;

  ub2 xm,*xmap,*xmapdbase;
  block *xpartdmap = &gnet->xpartdmap;

  if (partcnt == 1) { error(0,"interpart search called without partitions, ref %s",ref); return 0; }

  aclear(stats);

  memset(src->topdts,0xff,sizeof(src->topdts));

  tpart = gnet->tpart;
  tnet = getnet(tpart);
  tportcnt = tnet->portcnt;
  t2g = tnet->p2gport;
  g2t = tnet->g2pport;

  xmapdbase = blkdata(xpartdmap,0,ub2);

  xmap = xmapdbase + gdep * tportcnt;

  tamid = g2t[garr];
  error_eq(tamid,hi32);

  for (tdmid = 0; tdmid < tportcnt; tdmid++) {
    xm = xmap[tdmid];
    if (xm == 0) continue;
    stats[0]++;
    gdmid = t2g[tdmid];

    // assess trips gdep-gdmid-gamid-garr, with gdmid-gamid in top

    for (dpart = 0; dpart < tpart; dpart++) { // foreach part with dep as member
      if (portparts[gdep * partcnt + dpart] == 0) continue;
      stats[4]++;
      if (portparts[gdmid * partcnt + dpart] == 0) continue;
      stats[5]++;

      apart = tpart;

      xpartcnt++;
      stats[7] = xpartcnt;
      estvar = xpartcnt * tportcnt / max(tdmid,1);

      if (progress(&eta,"step %u of %u, %u vars",xpartcnt - 1,estvar,src->varcnt)) return 0;
      if (xpartcnt == 1) eta.limit = src->querytlim;

      conn += srcxpart2t(gnet,dpart,apart,gdep,garr,gdmid,src);
    }
  }

  for (iv = 0; iv < niv; iv++) info(0,"stats %u %u",iv,stats[iv]);

  return conn;
}

// toplevel: choose transfer count and partitions
static ub4 dosrc(struct gnetwork *gnet,ub4 nstoplo,ub4 nstophi,search *src,char *ref)
{
  ub4 gportcnt = gnet->portcnt;
  ub1 *portparts = gnet->portparts;
  ub4 part,partcnt = gnet->partcnt;
  ub4 tpart = gnet->tpart;
  ub4 gdep = src->dep;
  ub4 garr = src->arr;
  ub4 stop;

  ub4 conn = 0;

  if (gportcnt == 0) { error(0,"search without ports, ref %s",ref); return 0; }

  if (partcnt == 0) { error(0,"search called without partitions, ref %s",ref); return 0; }
  else if (partcnt == 1) {
    if (portparts[gdep] == 0) return info(0,"port %u not in partmap",gdep);
    if (portparts[garr] == 0) return info(0,"port %u not in partmap",garr);
    for (stop = nstoplo; stop < nstophi; stop++) {
      info(0,"search %u stops",stop);
      conn = srcglocal(gnet,0,gdep,garr,stop,src);
      if (conn) { info(0,"found trip at %u stops",stop); return conn; }
    }
    return 0;
  }

  for (part = 0; part < partcnt; part++) {
    if (portparts[gdep * partcnt + part] == 0 || portparts[garr * partcnt + part] == 0) continue;
    info(0,"dep %u and arr %u share part %u",gdep,garr,part);
    for (stop = nstoplo; stop < nstophi; stop++) {
      info(0,"search %u stops in part %u",stop,part);
      conn = srcglocal(gnet,part,gdep,garr,stop,src);
      if (conn) { info(0,"found trip at %u stops in part %u ",stop,part); return conn; }
    }
    if (nstophi < 2) return 0;
  }

  // no shared parts, or intrapart had no result
  if (portparts[gdep * partcnt + tpart]) {
    info0(0,"search interpart: dep in top");
    conn = srcxdpart(gnet,gdep,garr,src,ref);
  } else if (portparts[garr * partcnt + tpart]) {
    info0(0,"search interpart: arr in top");
    conn = srcxapart(gnet,gdep,garr,src,ref);
  }
  if (conn) return conn;

  info0(0,"search interpart");
  conn = srcxpart(gnet,gdep,garr,src,ref);
  return conn;
}

static int sametrip(struct trip *tp1,struct trip *tp2)
{
  ub4 len = tp1->len;

  if (len != tp2->len) return 0;
  if (memcmp(tp1->trip,tp2->trip,len * 2 * sizeof(ub4))) return 0;
  if (tp1->t[0] && tp2->t[0] && memcmp(tp1->t,tp2->t,len * sizeof(tp1->t[0]))) return 0;
  return 1;
}

// handle criteria and reporting
int plantrip(search *src,char *ref,ub4 dep,ub4 arr,ub4 nstoplo,ub4 nstophi)
{

  ub4 nleg,t,t2;
  ub4 conn;
  struct gnetwork *net = getgnet();
  ub4 portcnt = net->portcnt;
  struct port *parr,*pdep,*ports = net->ports;
  ub4 deptmin,depttmin,deptmax;
  ub8 t0,dt;
  ub4 utcofs;
  ub4 resmax = sizeof(src->resbuf);
  struct trip *stp,*stp2;
  int same;

  if (dep >= portcnt) return error(0,"departure %u not in %u portlist",dep,portcnt);
  if (arr >= portcnt) return error(0,"arrival %u not in %u portlist",arr,portcnt);
  pdep = ports + dep;
  if (dep == arr) return error(0,"departure %u equal to arrival: %s",arr,pdep->name);

  if (nstophi >= Nxstop) { warning(0,"limiting max %u-stop to %u", nstophi,Nxstop); nstophi = Nxstop - 1; }
  if (nstoplo > nstophi) { warning(0,"setting min %u-stop to %u", nstoplo,nstophi); nstoplo = nstophi; }

  info(0,"search geo dep %u arr %u between %u and %u stops, ref %s",dep,arr,nstoplo,nstophi,ref);

  inisrc(src,"src",0);
  src->dep = dep;
  src->arr = arr;

  pdep = ports + dep;
  parr = ports + arr;

  src->costlim = 0;
  src->costperstop = 15;
  src->lodist = hi32;

  src->geodist = fgeodist(pdep,parr);

  utcofs = utc12ofs(src->utcofs12);

  if (src->deptmin_cd == 0) deptmin = net->t0;
  else deptmin = yymmdd2min(src->deptmin_cd,utcofs);
  depttmin = hhmm2min(src->depttmin_cd);
  deptmin += depttmin;
  deptmax = deptmin + src->tspan * 60 * 24;

  src->deptmin = deptmin;
  src->deptmax = deptmax;
  src->histop = nstophi;

  t0 = gettime_usec();
  src->querytlim = t0 + 1000 * 1000 * Timelimit;

  info(CC,"search dep %u arr %u on \ad%u-\ad%u %s to %s geodist %u",dep,arr,deptmin,deptmax,pdep->name,parr->name,src->geodist);
  conn = dosrc(net,nstoplo,nstophi,src,ref);
  info(0,"searched %u local variants in %u local searches %u noloc",src->avarxcnt,src->locsrccnt,src->locnocnt);
  info(0,"%u of %u depvars %u of %u midvars %u of %u arrvars %u stored",src->dvarxcnt,src->dvarcnt,src->tvarxcnt,src->tvarcnt,src->avarxcnt,src->avarcnt,src->varcnt);

  dt = gettime_usec() - t0;

  if (dt > 2000000) info(0,"search took %u sec",(ub4)(dt / 1000000));
  if (dt > 2000) info(0,"search took \a.%u usec",(ub4)(dt / 1000));
  else info(0,"search took %u usec",(ub4)dt);

  if (conn == 0) {
    if (src->avarxcnt) return info(0,"no time found for %u stop\as",nstophi);
    else return info(0,"no route found for %u stop\as",nstophi);
  }
  nleg = src->lostop + 1;
  info(0,"found %u-%u in %u/%u legs",dep,arr,src->trips[0].len,nleg);

  for (t = 0; t < Elemcnt(src->trips); t++) {
    stp = src->trips + t;
    if (stp->cnt == 0) continue;
    same = 0;
    for (t2 = 0; t2 < t; t2++) {
      stp2 = src->trips + t2;
      same = sametrip(stp,stp2);
      if (same) break;
    }
    if (same) continue;
    if (gtriptoports(net,stp,src->resbuf,resmax,&src->reslen,utcofs)) return 1;
  }
  return 0;
}
