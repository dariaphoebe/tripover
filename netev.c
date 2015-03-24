// netev.c - time events for prepared net

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <stddef.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"

static ub4 msgfile;
#include "msg.h"

#include "math.h"
#include "time.h"
#include "util.h"
#include "net.h"
#include "netev.h"

static const ub4 subsamples = 64;
static const ub4 maxscnt = 1024 * 32;

static int vrbena;

void ininetev(void)
{

  msgfile = setmsgfile(__FILE__);
  iniassert();
  vrbena = (getmsglvl() >= Vrb);
}

// cleanup workspace after use
int rmsubevs(lnet *net)
{
  afree(net->sevents,"time subevs");
  afree(net->shopdur,"net shopdur");
  afree(net->sevcnts,"net sevcnts");
  return 0;
}

// prepare a subset of events for all plain and compound hops
// use random sampling in a given timebox
// estdur() only uses these
int mksubevs(lnet *net)
{
  struct hop *hp,*hp1,*hp2,*hops = net->hops;
  struct port *pdep,*parr,*ports = net->ports;
  ub4 *portsbyhop = net->portsbyhop;
  struct timepat *tp,*tp1,*tp2;
  struct chain *cp,*chains = net->chains;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 whopcnt = net->whopcnt;
  ub4 *hopdur = net->hopdur;
  ub4 *choporg = net->choporg;
  ub8 *crp,*chainrhops = net->chainrhops;
  ub4 e,i,r,rr,s,evcnt,sevcnt,scnt,hiscnt;
  ub4 cnt1,cnt2;
  ub4 hop,h1,h2,rh1,rh2;
  ub4 dep,arr;
  ub4 rid;
  ub4 t,dur,sumdur;
  ub4 tdep1,tarr2;
  ub4 tid;
  ub8 rtdur,*ev,*events = net->events;
  ub8 *sev,*sevents;

  ub4 st0_cd = globs.netvars[Net_tpat0];
  ub4 st1_cd = globs.netvars[Net_tpat1];
  ub4 st0 = cd2day(st0_cd) * 1440;
  ub4 st1 = cd2day(st1_cd) * 1440;
  ub4 gt0,gt01,nt0,nt1;

  ub4 dtbins[60 * 12];
  ub4 dthibin = Elemcnt(dtbins) - 1;

  aclear(dtbins);

  error_ovf(maxscnt,ub2);

  nt0 = net->t0; nt1 = net->t1;
  if (nt0 == 0 || nt1 == 0) return error0(0,"no overall time period");
  else if (nt1 <= nt0)  return error(0,"no overall time period at \ad%u",nt0);
  st0 = min(st0,nt1);
  st0 = max(st0,nt0);
  st1 = min(st0,nt1);
  st1 = max(st0,nt0);
  if (st1 == st0) {
    if (st1 < nt1) st1 = nt1;
    else st0 = nt0;
    warn(0,"no sampling period: adjusted to \ad%u - \ad%u",st0,st1);
  }

  info(0,"preparing %u sampling events in period \ad%u-\ad%u",subsamples,st0,st1);

  sevcnt = chopcnt * subsamples;
  sevents = alloc(sevcnt,ub8,0xff,"time subevs",sevcnt);

  ub4 *shopdur = alloc(whopcnt,ub4,0,"net shopdur",whopcnt);
  ub4 *sevcnts = alloc(chopcnt,ub4,0,"net sevcnts",chopcnt);

  if (whopcnt > chopcnt) memcopy(shopdur + chopcnt,hopdur + chopcnt,(whopcnt - chopcnt) * sizeof(*shopdur));

  hiscnt = 0;

  ub8 sumevcnt = 0,sumsevcnt = 0;

  struct eta eta;

  for (hop = 0; hop < hopcnt; hop++) {

    if (progress(&eta,"pass 1 hop %u of %u, \ah%lu from \ah%lu events",hop,chopcnt,sumsevcnt,sumevcnt)) return 1;

    sev = sevents + hop * subsamples;
    hp = hops + hop;
    tp = &hp->tp;
    evcnt = tp->genevcnt;
    if (evcnt == 0) continue;
    sumevcnt += evcnt;
    ev = events + tp->evofs;
    gt0 = tp->gt0;
    scnt = 0;
    for (e = 0; e < evcnt; e++) { // count #evs within box
      rtdur = ev[e * 2];
      t = (ub4)(rtdur & hi32) + gt0;
      dur = (ub4)(rtdur >> 32);
      warncc(t == hi32,0,"hop %u ev %u t hi",hop,e);
      warncc(dur == hi32,0,"hop %u ev %u dur hi",hop,e);
//    info(0,"t \ad%u",t);
//    info(0,"s0 \ad%u s1 \ad%u",st0,st1);
      if (t < st0) continue;
      else if (t >= st1) break;
      scnt++;
      if (scnt >= maxscnt) break;
    }
    tp->sevcnt = scnt;
    sumsevcnt += scnt;
    hiscnt = max(hiscnt,scnt);
  }
  if (hiscnt < 2) return error(0,"no events to sample from \ah%lu",sumevcnt);
  else if (hiscnt == maxscnt) warn(0,"limiting event samples to \ah%u",hiscnt);

  info(0,"sampling \ah%lu events from \ah%lu in %u box",sumsevcnt,sumevcnt,hiscnt);

  ub4 hiscnt1 = hiscnt + 1;
  ub4 *rndset,*rndsets = alloc(hiscnt1 * hiscnt1,ub4,0,"time randoms",hiscnt);
  ub8 *cev = alloc(hiscnt,ub8,0xff,"time cevs",hiscnt);
  ub4 *rnduniq = alloc(hiscnt,ub4,0,"time rndtmp",hiscnt);

//  error_ge(subsamples,hiscnt);

  // determine random subsample patterns once per #events
  if (progress(&eta,"pass 1 %u of %u %u",0,hiscnt,0)) return 1;
  for (i = subsamples; i < hiscnt; i++) {
    if (progress(&eta,"pass 1 %u of %u %u",i,hiscnt,0)) return 1;

    rndset = rndsets + hiscnt * i;
    r = 0;
    nclear(rnduniq,hiscnt);
    while (r < i) {
      rr = rnd(i);
      if (rnduniq[rr]) continue;
      rnduniq[rr] = 1;
      error_ge(r + hiscnt * i,hiscnt1 * hiscnt1);
      rndset[r++] = rr;
    }
  }

  sumsevcnt = 0;

  // plain hops
  for (hop = 0; hop < hopcnt; hop++) {

    if (progress(&eta,"pass 2 hop %u of %u, \ah%lu from \ah%lu events",hop,chopcnt,sumsevcnt,sumevcnt)) return 1;

    sev = sevents + hop * subsamples;
    hp = hops + hop;
    tp = &hp->tp;
    evcnt = tp->genevcnt;
    if (evcnt == 0) continue;
    ev = events + tp->evofs;
    gt0 = tp->gt0;
    scnt = tp->sevcnt;
    sumsevcnt += scnt;
    if (scnt == 0) continue;

    e = 0; t = 0;
    while (e < evcnt) {
      rtdur = ev[e * 2];
      t = (ub4)(rtdur & hi32) + gt0;
      dur = (ub4)(rtdur >> 32);
      warncc(t == hi32,0,"hop %u ev %u hi",hop,e);
      if (t >= st0) break;
      e++;
    }
    s = 0; sumdur = 0;
    error_gt(scnt,hiscnt,hop);
    if (scnt > subsamples) {
      rndset = rndsets + hiscnt * scnt;
      for (r = 0; r < subsamples; r++) {
        rr = rndset[r];
        error_ge(rr+e,evcnt);
        rtdur = ev[(e+rr) * 2];
        t = (ub4)(rtdur & hi32) + gt0;
        dur = (ub4)(rtdur >> 32);

      if (dur > 6000) {
        dep = portsbyhop[hop * 2];
        arr = portsbyhop[hop * 2 + 1];
        pdep = ports + dep;
        parr = ports + arr;
        info(Notty,"hop %u dur \ax%u %s to %s t \ad%u",hop,dur,pdep->name,parr->name,t);
      }

        dur &= hi16; // todo
        sev[r] = ((ub8)t << 32) | dur;
        sumdur += dur;
        dtbins[min(dur,dthibin)]++;
        scnt = subsamples;
      }
      sort8(sev,scnt,FLN,"subevs");
    } else {
      while (s < scnt && e < evcnt) {
        rtdur = ev[e * 2];
        t = (ub4)(rtdur & hi32) + gt0;
        dur = (ub4)(rtdur >> 32);
        sev[s++] = ((ub8)t << 32) | dur;
        e++;
        sumdur += dur;
        dtbins[min(dur,dthibin)]++;
      }
      warncc(s != scnt,0,"hop %u s %u scnt %u",hop,s,scnt);
    }
    shopdur[hop] = sumdur / scnt;
    sevcnts[hop] = scnt;
  }

  // compound hops
  for (hop = hopcnt; hop < chopcnt; hop++) {

    if (progress(&eta,"pass 2 hop %u of %u, \ah%lu from \ah%lu events",hop,chopcnt,sumsevcnt,sumevcnt)) return 1;

    h1 = choporg[hop * 2];
    h2 = choporg[hop * 2 + 1];

    hp1 = hops + h1;
    hp2 = hops + h2;

    tp1 = &hp1->tp;
    tp2 = &hp2->tp;

    cnt1 = tp1->genevcnt;
    cnt2 = tp2->genevcnt;
    if (cnt1 == 0 || cnt2 == 0) continue;

    sev = sevents + hop * subsamples;

    rid = hp1->rid;

    rh1 = hp1->rhop;
    rh2 = hp2->rhop;
    if (rh1 >= Chainlen || rh2 >= Chainlen) continue;

    rid = hp1->rid;
    ev = events + tp1->evofs;

    gt01 = tp1->gt0;

    e = 0; t = 0;
    while (e < cnt1) {
      rtdur = ev[e * 2];
      t = (ub4)(rtdur & hi32) + gt01;
      if (t >= st0) break;
      e++;
    }
    scnt = 0;
    sumdur = 0;

    // count and copy
    while (e < cnt1 && t < st1) {
      rtdur = (ub4)ev[e * 2];
      t = (ub4)(rtdur & hi32) + gt01;
      tid = ev[e * 2 + 1] & hi24;
      cp = chains + tid;
      if (cp->hopcnt < 3) { e++; continue; }

      error_ne(cp->rid,rid);

      crp = chainrhops + cp->rhopofs;
      tdep1 = (ub4)(crp[rh1] >> 32);
      tarr2 = crp[rh2] & hi32;
      if (tdep1 == hi32 || tarr2 == hi32) { e++; continue; }
      else if (tarr2 < tdep1) { // todo: assume reversed order for circular routes
        vrb0(0,"chop %u-%u dep %u arr %u",h1,h2,tdep1,tarr2);
        tdep1 = (ub4)(crp[rh2] >> 32);
        tarr2 = crp[rh1] & hi32;
        warncc(tarr2 < tdep1,0,"chop %u-%u dep %u arr %u",h1,h2,tdep1,tarr2);
      }
      dur = tarr2 - tdep1;

      if (dur > 6000) {
        dep = portsbyhop[hop * 2];
        arr = portsbyhop[hop * 2 + 1];
        pdep = ports + dep;
        parr = ports + arr;
        info(Notty,"chop %u-%u %s to %s td %u ta %u",h1,h2,pdep->name,parr->name,tdep1,tarr2);
      }

      cev[scnt] = ((ub8)t << 32) | dur;
      sumdur += dur;
      dtbins[min(dur,dthibin)]++;
      scnt++;
      if (scnt >= tp1->sevcnt) break; // todo
      error_gt(scnt,tp1->sevcnt,hop);
      e++;
    }
    if (scnt == 0) continue;

    shopdur[hop] = sumdur / scnt;

    if (scnt > subsamples) {
      rndset = rndsets + hiscnt * scnt;
      for (r = 0; r < subsamples; r++) {
        rr = rndset[r];
        warncc(rr >= scnt,0,"rr %u scnt %u",rr,scnt);
        rtdur = cev[rr];
        sev[r] = rtdur;
      }
      scnt = subsamples;
      sort8(sev,scnt,FLN,"subevs");
    } else memcopy(sev,cev,scnt * sizeof(*sev));
    sevcnts[hop] = scnt;
    sumsevcnt += scnt;
  }
  afree(rndsets,"time randoms");

  ub8 sumcnt = 0;
  sumevcnt = 0;
  for (i = 0; i <= dthibin; i++) sumevcnt += dtbins[i];
  for (i = 0; i <= dthibin; i++) {
    scnt = dtbins[i];
    sumcnt += scnt;
    infocc(scnt,0,"dur %u cnt %u %lu%%",i,scnt,sumcnt * 100 / sumevcnt);
  }

  ub4 noevcnt = 0;

  // verify
  for (hop = 0; hop < chopcnt; hop++) {
    scnt = sevcnts[hop];
    sev = sevents + hop * subsamples;

    if (scnt == 0) noevcnt++;

#if 1
    for (s = 0; s < scnt; s++) {
      rtdur = sev[s];
      dur = rtdur & hi32;
      if (dur > 6000) {
        t = (ub4)(rtdur >> 32);
        dep = portsbyhop[hop * 2];
        arr = portsbyhop[hop * 2 + 1];
        pdep = ports + dep;
        parr = ports + arr;
        info(0,"hop %u %s to %s dur %u t %u",hop,pdep->name,parr->name,dur,t);
      }
    }
#endif
  }

  infocc(noevcnt,0,"%u of %u hops without sample events",noevcnt,chopcnt);

  for (hop = 0; hop < whopcnt; hop++) {
    warncc(shopdur[hop] == hi32,0,"hop %u sdur %u",hop,shopdur[hop]);
  }

  net->sevents = sevents;
  net->sevcnts = sevcnts;
  net->shopdur = shopdur;

  return 0;
}

// get an average total duration between two hops
// based on a prepared handful of random samples
static ub4 estdur2(lnet *net,ub4 hop1,ub4 hop2,ub4 ttmin,ub4 ttmax)
{
  ub4 chopcnt = net->chopcnt;
  ub8 tdur1,tdur2,*sev1,*sev2,*sevents = net->sevents;
  ub4 scnt1,scnt2,*sevcnts = net->sevcnts;
  ub4 e1,e2,prve1,prve2,i;
  ub4 t1,t2,dur1,dur2,avgdt,dt,lodt;
  ub4 dtcnt2,dtcnt = 0,dtbcnt = 0;
  ub8 dtsum = 0,dtbsum = 0;

static ub4 dtbins[60 * 12];
static ub4 dthibin = Elemcnt(dtbins) - 1;
static ub4 statcnt;
static ub4 stat_nocnt;

  error_zp(sevents,0);

  error_ge(hop1,chopcnt);
  error_ge(hop2,chopcnt);

  sev1 = sevents + hop1 * subsamples;
  sev2 = sevents + hop2 * subsamples;
  scnt1 = sevcnts[hop1];
  scnt2 = sevcnts[hop2];

  error_gt(scnt1,subsamples,hop1);
  error_gt(scnt2,subsamples,hop2);

  if (scnt1 == 0 || scnt2 == 0) {
//    info(0,"hop %u-%u evs %u+%u",hop1,hop2,scnt1,scnt2);
    stat_nocnt++;
    return hi32;
  }

  // forward
  e2 = prve2 = 0;
  for (e1 = 0; e1 < scnt1; e1++) {
    tdur1 = sev1[e1];
    dur1 = tdur1 & hi32;
    t1 = (ub4)(tdur1 >> 32);
    e2 = prve2;
    while (e2 < scnt2) {
      tdur2 = sev2[e2];
      t2 = (ub4)(tdur2 >> 32);
      if (t2 >= t1 + dur1 + ttmin) break;
      e2++;
    }
    prve2 = e2;
    if (e2 == scnt2) break;

    lodt = hi32;
    dtcnt2 = 0;

    while (e2 < scnt2) { // search for the single best xfer
      tdur2 = sev2[e2++];
      t2 = (ub4)(tdur2 >> 32);

      if (dtcnt2 && t2 - t1 + dur1 > ttmax) break;

      dur2 = tdur2 & hi32;
      dur2 &= hi16; // todo

      dt = (t2 - t1) + dur2;
      if (dt < lodt) { lodt = dt; dtcnt2++; }
    }
    if (dtcnt2 == 0) continue;

    dtsum += lodt;
    dtcnt++;
  }

  // backward
  e1 = prve1 = 0;
  for (e2 = 0; e2 < scnt2; e2++) {
    tdur2 = sev2[e2];
    t2 = (ub4)(tdur2 >> 32);
    dur2 = tdur2 & hi32;
    e1 = prve1;
    while (e1 < scnt1) {
      tdur1 = sev1[e1];
      t1 = (ub4)(tdur1 >> 32);
      dur1 = tdur1 & hi32;
      if (t1 + dur1 + ttmin >= t2) break;
      e1++;
    }
    prve1 = e1;

    lodt = hi32;
    dtcnt2 = 0;

    while (e1) {
      tdur1 = sev1[--e1];
      t1 = (ub4)(tdur1 >> 32);
      dur1 = tdur1 & hi32;
      dur1 &= hi16;

      if (dtcnt2 && t2 - t1 + dur1 > ttmax) break;

      dt = (t2 - t1) + dur2;
      if (dt < lodt) { lodt = dt; dtcnt2++; }
    }
    if (dtcnt2 == 0) continue;

    dtbsum += lodt;
    dtbcnt++;
  }

  if (dtcnt == 0 && dtbcnt == 0) return hi32;
  else if (dtcnt == 0) avgdt = (ub4)(dtbsum / dtbcnt);
  else if (dtbcnt == 0) avgdt = (ub4)(dtsum / dtcnt);
  else avgdt = (ub4)min(dtsum / dtcnt,dtbsum / dtbcnt);

  if (vrbena == 0) return avgdt;

  dtbins[min(avgdt,dthibin)]++;

  if (++statcnt & hi24) return avgdt;

  ub8 sumcnt = 0,sumscnt = 0;
  ub4 scnt;
  for (i = 0; i <= dthibin; i++) sumscnt += dtbins[i];

  for (i = 0; i <= dthibin; i++) {
    scnt = dtbins[i];
    sumcnt += scnt;
    infocc(scnt,Notty,"dur %u cnt %u %lu%%",i,scnt,sumcnt * 100 / sumscnt);
  }

  return avgdt;
}

// get an average total duration between 3 hops
// based on a prepared handful of random samples
static ub4 estdur3(lnet *net,ub4 hop1,ub4 hop2,ub4 hop3,ub4 ttmin,ub4 ttmax)
{
  ub8 tdur1,tdur2,tdur3,*sev1,*sev2,*sev3,*sevents = net->sevents;
  ub4 scnt1,scnt2,scnt3,*sevcnts = net->sevcnts;
  ub4 e1,e2,e3,i;
  ub4 t1,t2,t3,dur1,dur2,dur3,avgdt,dt;
  ub4 dtcnt = 0,dtbcnt = 0;
  ub8 dtsum = 0,dtbsum = 0;

static ub4 dtbins[60 * 12];
static ub4 dthibin = Elemcnt(dtbins) - 1;
static ub4 statcnt;

  error_zp(sevents,0);

  sev1 = sevents + hop1 * subsamples;
  sev2 = sevents + hop2 * subsamples;
  sev3 = sevents + hop3 * subsamples;
  scnt1 = sevcnts[hop1];
  scnt2 = sevcnts[hop2];
  scnt3 = sevcnts[hop3];

  if (scnt1 == 0 || scnt2 == 0 || scnt3 == 0) return hi32;

  // forward
  e2 = e3 = 0;
  tdur2 = sev2[e2];
  t2 = (ub4)(tdur2 >> 32);
  tdur3 = sev2[e3];
  t3 = (ub4)(tdur3 >> 32);
  for (e1 = 0; e1 < scnt1; e1++) {
    tdur1 = sev1[e1];
    t1 = (ub4)(tdur1 >> 32);
    dur1 = tdur1 & hi32;

    while (e2 < scnt2 && t2 < t1 + dur1 + ttmin) {
      tdur2 = sev2[e2++];
      t2 = (ub4)(tdur2 >> 32);
    }
    if (e2 == scnt2) break;
    else if (t2 - t1 + dur1 > ttmax) continue;
    dur2 = tdur2 & hi32;

    while (e3 < scnt3 && t3 < t2 + dur2 + ttmin) {
      tdur3 = sev2[e3++];
      t3 = (ub4)(tdur3 >> 32);
    }
    if (e3 == scnt3) break;
    else if (t3 - t2 + dur2 > ttmax) continue;
    dur3 = tdur3 & hi32;

    dt = (t3 - t1) + dur3;
    dtsum += dt;
    dtcnt++;
  }

  // backward
  e1 = e2 = 0;
  tdur1 = sev1[e1];
  t1 = (ub4)(tdur1 >> 32);
  dur1 = tdur1 & hi32;
  tdur2 = sev2[e2];
  t2 = (ub4)(tdur2 >> 32);
  dur2 = tdur2 & hi32;

  for (e3 = 0; e3 < scnt3; e3++) {
    tdur3 = sev3[e3];
    t3 = (ub4)(tdur3 >> 32);

    while (e2 < scnt2 && t2 + dur2 + ttmin < t3) {
      tdur2 = sev2[e2++];
      t2 = (ub4)(tdur2 >> 32);
      dur2 = tdur2 & hi32;
    }
    if (e2 == scnt2 || t2 + dur2 + ttmax > t3) break;
    else if (t3 - t2 + dur2 > ttmax) continue;

    while (e1 < scnt1 && t1 + dur1 + ttmin < t2) {
      tdur1 = sev1[e1++];
      t1 = (ub4)(tdur1 >> 32);
      dur1 = tdur1 & hi32;
    }
    if (e1 == scnt1 || t1 + dur1 + ttmax > t2) break;
    else if (t2 - t1 + dur1 > ttmax) continue;
    dur3 = tdur3 & hi32;

    dt = t3 - t1 + dur3;
    dtbsum += dt;
    dtbcnt++;
  }

  if (dtcnt == 0 && dtbcnt == 0) return hi32;
  else if (dtcnt == 0) avgdt = (ub4)(dtbsum / dtbcnt);
  else if (dtbcnt == 0) avgdt = (ub4)(dtsum / dtcnt);
  else avgdt = (ub4)min(dtsum / dtcnt,dtbsum / dtbcnt);

  if (vrbena == 0) return avgdt;

  dtbins[min(avgdt,dthibin)]++;

  if (++statcnt & hi24) return avgdt;

  ub8 sumcnt = 0,sumscnt = 0;
  ub4 scnt;
  for (i = 0; i <= dthibin; i++) sumscnt += dtbins[i];

  for (i = 0; i <= dthibin; i++) {
    scnt = dtbins[i];
    sumcnt += scnt;
    infocc(scnt,Notty,"dur %u cnt %u %lu%%",i,scnt,sumcnt * 100 / sumscnt);
  }

  return avgdt;
}

// first step of estimated average total trip duration
// stores arrival times of last hop
ub4 prepestdur(lnet *net,ub4 *trip,ub4 len)
{
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 *shopdur = net->shopdur;
  ub4 avgdur;
  ub4 h,h1,h2;
  struct hop *hp,*hops = net->hops;
  ub4 ttmax = globs.netvars[Net_tpatmaxtt];
  ub4 ttmin = globs.netvars[Net_tpatmintt];

  if (len == 1) {  // basic net1 case
    h = *trip;
    if (h < hopcnt) {
      hp = hops + h;
      avgdur = hp->tp.avgdur;
      return avgdur;
    } else return shopdur[h];

  } else if (len == 2) {
    h1 = trip[0];
    h2 = trip[1];
    if (h1 < chopcnt && h2 < chopcnt) return estdur2(net,h1,h2,ttmin,ttmax);
    else return shopdur[h1] + shopdur[h2];
  }
  // rest todo
  return 1000;
}

// estimate average total trip duration, using arrival times of preceding triplet
ub4 estdur(lnet *net,ub4 *trip1,ub4 len1,ub4 *trip2,ub4 len2)
{
  ub4 chopcnt = net->chopcnt;
  ub4 *shopdur = net->shopdur;
  ub4 h1,h2,h3;

  ub4 ttmax = globs.netvars[Net_tpatmaxtt];
  ub4 ttmin = globs.netvars[Net_tpatmintt];

  if (len1 == 1 && len2 == 1) { // basic net1 case
    h1 = *trip1; h2 = *trip2;
    if (h1 < chopcnt && h2 < chopcnt) return estdur2(net,h1,h2,ttmin,ttmax);
    else return shopdur[h1] + shopdur[h2];

  } else if (len1 + len2 == 3) {
    if (len1 == 2 && len2 == 1) { h1 = trip1[0]; h2 = trip1[1]; h3 = trip2[0]; }
    else { h1 = trip1[0]; h2 = trip2[0]; h3 = trip2[1]; }

    if (h1 < chopcnt && h2 < chopcnt && h3 < chopcnt) return estdur3(net,h1,h2,h3,ttmin,ttmax);
    else if (h1 < chopcnt && h2 < chopcnt) {
      return estdur2(net,h1,h2,ttmin,ttmax) + shopdur[h3];
    } else if (h1 < chopcnt && h3 < chopcnt) {
      return estdur2(net,h1,h3,ttmin + shopdur[h2],ttmax);
    } else if (h2 < chopcnt && h3 < chopcnt) {
      return estdur2(net,h2,h3,ttmin,ttmax) + shopdur[h1];
    } else return shopdur[h1] + shopdur[h2] + shopdur[h3];
  }  

// todo
  return 1000;
}

ub4 estdur_3(lnet *net,ub4 h1,ub4 h2,ub4 h3)
{
  ub4 chopcnt = net->chopcnt;
  ub4 *shopdur = net->shopdur;

  ub4 ttmax = globs.netvars[Net_tpatmaxtt];
  ub4 ttmin = globs.netvars[Net_tpatmintt];

  if (h1 < chopcnt && h2 < chopcnt && h3 < chopcnt) return estdur3(net,h1,h2,h3,ttmin,ttmax);
  else if (h1 < chopcnt && h2 < chopcnt) {
    return estdur2(net,h1,h2,ttmin,ttmax) + shopdur[h3];
  } else if (h1 < chopcnt && h3 < chopcnt) {
    return estdur2(net,h1,h3,ttmin + shopdur[h2],ttmax);
  } else if (h2 < chopcnt && h3 < chopcnt) {
    return estdur2(net,h2,h3,ttmin,ttmax) + shopdur[h1];
  } else return shopdur[h1] + shopdur[h2] + shopdur[h3];
}

ub4 estdur_2(lnet *net,ub4 h1,ub4 h2)
{
  ub4 chopcnt = net->chopcnt;
  ub4 *shopdur = net->shopdur;

  ub4 ttmax = globs.netvars[Net_tpatmaxtt];
  ub4 ttmin = globs.netvars[Net_tpatmintt];

  if (h1 < chopcnt && h2 < chopcnt) return estdur2(net,h1,h2,ttmin,ttmax);
  else return shopdur[h1] + shopdur[h2];
}
