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

#include "bitfields.h"
#include "net.h"

#include "search.h"

void inisearch(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

static const ub4 fldcnt = 4;
static const ub4 walkfreq = 5;

static void inisrc(search *src,const char *desc,ub4 arg)
{
  ub4 i,leg;
  ub4 *ev;

  fmtstring(src->desc,"%s %u",desc,arg);

  src->costlim = 0;
  src->lodist = hi32;
  src->lodt = hi32;

  src->trip.cnt = src->trip.len = 0;
  for (i = 0; i < Nxleg; i++) {
    src->trip.trip[i * 2] = src->trip.trip[i * 2 + 1] = hi32;
    src->trip.port[i] = hi32;
  }
  ev = src->evpool = alloc(Nxleg * Maxevs * fldcnt,ub4,0,"src events",Maxevs);
  for (leg = 0; leg < Nxleg; leg++) {
    src->depevs[leg] = ev;
    ev += Maxevs * fldcnt;
  }
}

static ub4 timefld(ub4 ndx) { return ndx * fldcnt; }
static ub4 tidfld(ub4 ndx) { return ndx * fldcnt + 1; }
static ub4 dtfld(ub4 ndx) { return ndx * fldcnt + 2; }
static ub4 durfld(ub4 ndx) { return ndx * fldcnt + 3; }

static ub4 mkdepevs(search *src,ub8 *events,ub2 *evmaps,struct timepat *tp,struct hop *hp,ub4 midur)
{
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 gt0 = tp->gt0;
  ub4 ht0 = tp->ht0;
  ub4 ht1 = tp->ht1;
  ub4 t0 = tp->t0;
  ub4 t1 = tp->t1;
  ub4 dcnt = 0,lodev,gencnt = tp->genevcnt;
  ub4 leg,gndx,agndx,dmax = Maxevs;
  ub4 t,dur,lodur,*dev,*adev;
  ub8 x,*ev,*aev;
  ub2 *map;

  src->dcnts[0] = 0;

  for (leg = 0; leg < Nxleg; leg++) src->devcurs[leg] = hi32;

  if (gencnt == 0) return 0;

  if (t0 == t1) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,t0,t1,deptmin,deptmax);
//  else if (ht0 > deptmax) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,ht0,ht1,deptmin,deptmax);
//  else if (ht1 <= deptmin) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,ht0,ht1,deptmin,deptmax);
  if (t0 + gt0 > deptmax) return info(Iter,"hop %u tt range \aD%u - \aD%u, dep window \aD%u - \aD%u",hp->gid,t0 + gt0,t1 + gt0,deptmin,deptmax);
  else if (t1 + gt0 <= deptmin) return info(Iter,"hop %u tt range \aD%u - \aD%u, dep window \aD%u - \aD%u",hp->gid,t0 + gt0,t1 + gt0,deptmin,deptmax);

  ev = events + tp->evofs;
  map = evmaps + tp->dayofs;
  dev = src->depevs[0];
  t = 0;
  lodur = hi32; lodev = 0;
  for (gndx = 0; gndx < gencnt * 2; gndx++) {
    t = (ub4)ev[gndx * 2];
    if (t + gt0 < deptmin) {
//      vrb0(Iter,"t %u gt0 %u deptmin %u",t,gt0,deptmin);
      continue;
    } else if (t + gt0 >= deptmax) {
      vrb0(Iter,"t %u gt0 %u deptmax %u",t,gt0,deptmax);
      break;
    }
    x = ev[gndx * 2 + 1];
    if (midur != hi32) dur = midur;
    else dur = (x >> 32);
    dev[timefld(dcnt)] = t + gt0;
    dev[tidfld(dcnt)] = (ub4)x; // tid+dayid
    dev[dtfld(dcnt)] = dur;
    dev[durfld(dcnt)] = dur;
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

  src->dtcurs[0] = lodur;
  src->devcurs[0] = lodev;
  infocc(src->varcnt < 20,Iter,"%u dep events",dcnt);
  return dcnt;
}

static ub4 nxtevs(search *src,ub8 *events,ub2 *evmaps,ub4 leg,struct timepat *tp,struct hop *hp,ub4 midur,ub4 dthi)
{
  struct timepat *atp;
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 gt0 = tp->gt0;
  ub4 ht0 = tp->ht0;
  ub4 ht1 = tp->ht1;
  ub4 t0 = tp->t0;
  ub4 t1 = tp->t1;
  ub4 aleg;
  ub4 dcnt = 0,gencnt = tp->genevcnt;
  ub4 gndx,agndx,dmax = Maxevs;
  ub4 adndx,adcnt;
  ub4 rt,t,tid,at,dur,adur,lodt,lodev,loadev,dt,adt,*dev,*adev;
  ub4 ttmax = 120; // todo
  ub8 x,*ev,*aev;
  ub2 *map;

  error_z(leg,0);

  src->dcnts[leg] = 0;

  if (leg < Nxleg - 1) {
    src->dtcurs[leg] = src->dtcurs[leg+1] = hi32;
    src->devcurs[leg] = src->devcurs[leg+1] = hi32;
  }

  if (gencnt == 0) return 0;

  aleg = leg - 1;
  adcnt = src->dcnts[aleg];
  if (adcnt == 0) return 0;

  if (t0 == hi32) return warn(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,t0,t1,deptmin,deptmax);
  else if (t1 == hi32) return warn(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,t0,t1,deptmin,deptmax);
  else if (gt0 == hi32) return warn(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,t0,t1,deptmin,deptmax);

  if (t0 == t1) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,t0,t1,deptmin,deptmax);

  if (t0 + gt0 > deptmax + src->dtlos[leg]) return info(Iter,"hop %u tt range \aD%u - \aD%u, dep window \aD%u - \aD%u",hp->gid,t0 + gt0,t1 + gt0,deptmin,deptmax);
  else if (t1 + gt0 <= deptmin + src->dtlos[leg]) return info(Iter,"hop %u tt range \aD%u - \aD%u, dep window \aD%u - \aD%u",hp->gid,t0 + gt0,t1 + gt0,deptmin,deptmax);

  src->duraccs[leg] = tp->duracc;

  ev = events + tp->evofs;
  map = evmaps + tp->dayofs;

//  atp = src->tps[aleg];
//  aev = events + atp->evofs;

  dev = src->depevs[leg];
  adev = src->depevs[aleg];

  t = at = 0;
  lodt = hi32; lodev = loadev = hi32;
  gndx = agndx = adndx = 0;
  while (gndx < gencnt && adndx < adcnt && dcnt < dmax) {
    at = adev[timefld(adndx)];
    adt = adev[dtfld(adndx)];
    adur = adev[durfld(adndx)];

    // todo: first check exact connection. if tid equal, do not count as transfer
    //       else apply min transfer time + accuracy
    while (gndx < gencnt && t < at + ttmax) {
      rt = (ub4)ev[gndx * 2];
      t = rt + gt0;
      if (t <= at + adur + tp->duracc) { gndx++; continue; }
      else if (t < deptmin) {
//        info(Iter,"t %u gt0 %u deptmin %u",t,gt0,deptmin);
        gndx++;
        continue;
      } else if (t >= min(deptmax,at + adur + ttmax)) {
//        info(Iter,"t %u gt0 %u deptmax %u",t,gt0,deptmax);
        break;
      }
      x = ev[gndx * 2 + 1];
      gndx++;
      if (midur != hi32) dur = midur; // todo compound
      else dur = (x >> 32);
      dt = dur + t + adt - at; // accumulate total trip time
      if (dt >= dthi) { gndx++; continue; }
      tid = x & hi24;

      dev[timefld(dcnt)] = t;
      dev[tidfld(dcnt)] = tid;
      dev[dtfld(dcnt)] = dt;
      dev[durfld(dcnt)] = dur;

      if (dt < lodt) { lodt = dt; lodev = dcnt; loadev = adndx; }
      dcnt++;
      if (dcnt >= dmax) {
        warning(0,"exceeding %u dep event",dcnt);
        break;
      }
    }
    adndx++;
  }
  src->dcnts[leg] = dcnt;
  if (dcnt == 0) return 0;

  src->dtcurs[leg] = lodt;
  src->devcurs[leg] = lodev;
  src->devcurs[aleg] = loadev;
  infocc(src->varcnt < 20,Iter,"%u dep events for leg %u",dcnt,leg);
  return dcnt;
}

// forward existing evs, e.g. walk link after nonwalk
static ub4 fwdevs(search *src,ub4 leg,ub4 hop,ub4 midur,ub4 dthi)
{
  struct timepat *atp;
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 aleg;
  ub4 dcnt = 0;
  ub4 gndx,agndx,dmax = Maxevs;
  ub4 adndx,adcnt;
  ub4 rt,t,tid,atid,at,dur,adur,lodt,lodev,loadev,dt,adt,*dev,*adev;
  ub8 x,*ev,*aev;
  ub2 *map;

  error_z(leg,hop);

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

  t = at = 0;
  lodt = hi32; lodev = loadev = hi32;
  adndx = 0;
  while (adndx < adcnt && dcnt < dmax) {
    at = adev[timefld(adndx)];
    adt = adev[dtfld(adndx)];
    adur = adev[durfld(adndx)];
    atid = adev[durfld(adndx)];

    dt = adt + midur;
    if (dt >= dthi) { adndx++; continue; }

    dev[timefld(dcnt)] = at + midur;
    dev[tidfld(dcnt)] = atid;
    dev[dtfld(dcnt)] = dt;
    dev[durfld(dcnt)] = midur;

    if (dt < lodt) { lodt = dt; lodev = dcnt; loadev = adndx; }
    dcnt++;

    adndx++;
  }
  src->dcnts[leg] = dcnt;
  if (dcnt == 0) return 0;

  src->dtcurs[leg] = lodt;
  src->devcurs[leg] = lodev;
  src->devcurs[aleg] = loadev;
  src->duraccs[leg] = 0;
  infocc(src->varcnt < 20,Iter,"%u dep events for leg %u",dcnt,leg);
  return dcnt;
}

// fill events with frequency, e.g. walk link before nonwalk or 'every x min' metro
// leg0 only
static ub4 frqevs(search *src,ub4 leg,ub4 hop,ub4 midur,ub4 freq,ub4 dthi)
{
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 dcnt = 0;
  ub4 gndx,agndx,dmax = Maxevs;
  ub4 adndx,adcnt;
  ub4 rt,t,tid,atid,at,dur,adur,lodt,lodev,loadev,dt,adt,*dev,*adev;
  ub8 x,*ev,*aev;
  ub2 *map;

  error_nz(leg,hop);

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
    t += freq;
    dcnt++;
  }
  src->dcnts[leg] = dcnt;
  if (dcnt == 0) return 0;

  src->dtcurs[leg] = lodt;
  src->devcurs[leg] = lodev;
  src->duraccs[leg] = 0;
  infocc(src->varcnt < 20,Iter,"%u dep events for leg %u",dcnt,leg);
  return dcnt;
}

// rescan back given the latest arrival to find best start
static ub4 prvevs(search *src,ub4 leg)
{
  struct timepat *atp;
  ub4 aleg;
  ub4 gndx,agndx;
  ub4 dcnt,adndx,adcnt;
  ub4 rt,t,tid,at,dur,duracc,adur,lodt,lodev,loadev,dt,adt,*dev,*adev;
  ub4 ttmax = 120; // todo
  ub8 x,*ev,*aev;
  ub2 *map;

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
  dur = dev[durfld(lodev)];

  at = 0;
  lodt = hi32; loadev = hi32;
  adndx = 0;
  while (adndx < adcnt) {
    at = adev[timefld(adndx)];
    adt = adev[dtfld(adndx)];
    adur = adev[durfld(adndx)];

    if (at + adur + ttmax < t) { adndx++; continue; }
    else if (at + adur + duracc >= t) break;

    dt = t - at;

    if (dt < lodt) { lodt = dt; loadev = adndx; }
    adndx++;
  }
  if (loadev == hi32) return warn(0,"no preceding dep at leg %u between \ad%u and \ad%u",aleg,at,t); // todo
  src->devcurs[aleg] = loadev;
  return dt;
}

static ub4 getevs(search *src,ub4 nleg)
{
  struct timepat *atp;
  ub4 dcnt = 0;
  ub4 ndx,lodev,l;
  ub4 t,tid,at,dur,dt,lodt,adt,*dev,*adev;
  ub8 x,*ev,*aev;
  ub2 *map;
  ub4 *lodevs = src->devcurs;

  error_z(nleg,0);

  dt = hi32;
  at = 0;
  for (l = 0; l < nleg; l++) {
    dev = src->depevs[l];
    dcnt = src->dcnts[l];

    if (dcnt == 0) return warn(0,"no events for leg %u",l);

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
    error_ge(at,t);
    at = t;
    info(0,"ev %u of %u leg %u t \ad%u tid %x dt %u",lodev,dcnt,l,t,tid,dt);
  }
  src->curdt = dt;
  src->curt = src->curts[0];

  return dcnt;
}

static ub4 addevs(search *src,ub8 *events,ub2 *evmaps,ub4 leg,struct timepat *tp,struct hop *hp,ub4 midur,ub4 dthi)
{
  ub4 cnt;

  if (leg) cnt = nxtevs(src,events,evmaps,leg,tp,hp,midur,dthi);
  else cnt = mkdepevs(src,events,evmaps,tp,hp,midur);
  return cnt;
}

// intra-partition search for given transfers
static ub4 srclocal(ub4 callee,struct network *net,ub4 part,ub4 dep,ub4 arr,ub4 stop,search *src,const char *desc)
{
  struct hop *hp,*hops = net->hops;
  struct timepat *tp;
  ub2 *cnts,cnt;
  ub4 *ofss,ofs;
  ub4 *lst;
  block *lstblk;
  ub4 *lodists,lodist;
  ub4 nleg = stop + 1;
  ub4 histop = net->histop;

  ub4 deptmin,deptmax;
  ub8 *events = net->events;
  ub2 *evmaps = net->evmaps;
  ub4 evcnt;

  ub4 cost,dist = 0,leg,l,l1,l2;
  ub4 lodt,dtcur;
  ub4 *vp;

  ub4 costlim = src->costlim;
  ub4 *hopdist;
  ub4 v0 = 0;

  ub4 portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 whopcnt = net->whopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 *choporg = net->choporg;
  ub4 midur,*hopdur = net->hopdur;

  ub4 ln = callee & 0xffff;

  if (stop > histop) { vrb0(0,"%s: net part %u only has %u-stop connections, skip %u",desc,part,histop,stop); return 0; }

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
    for (leg = 0; leg < nleg; leg++) {
      l = vp[leg];
      error_ge(l,chopcnt);
      dist += hopdist[l];
    }
//    infovrb(dist == 0,0,"dist %u for var %u",dist,v0);
    if (dist < lodist) {
//      if (dist == 0) warning(0,"dist 0 for len %u",nleg);
      lodist = dist;
      src->lostop = stop;
#if 0
      for (leg = 0; leg < nleg; leg++) {
        src->trip.trip[leg * 2 + 1] = vp[leg];
        src->trip.trip[leg * 2] = part;
      }
#endif
      if (dist && dist == lodists[da]) info(0,"%u-stop found lodist %u at var %u %s:%u",stop,dist,v0,desc,ln);
    }

    // time
    evcnt = 0; dtcur = hi32;
    for (leg = 0; leg < nleg; leg++) {
      l = vp[leg];
      midur = hopdur[l];
      if (l < hopcnt) {
        hp = hops + l;
        tp = &hp->tp;
        evcnt = addevs(src,events,evmaps,leg,tp,hp,midur,hi32);
      } else if (l < whopcnt) {
        if (leg) evcnt = fwdevs(src,leg,l,midur,hi32);
        else evcnt = frqevs(src,leg,l,midur,walkfreq,hi32);
      } else {
        l1 = choporg[l * 2];
        l2 = choporg[l * 2 + 1];
        hp = hops + l1;
        tp = &hp->tp;
        evcnt = addevs(src,events,evmaps,leg,tp,hp,midur,hi32);
      }
      dtcur = src->dtcurs[leg];
      if (evcnt == 0) break;
    }
    if (evcnt && (dtcur < lodt || src->trip.cnt == 0)) {
      lodt = dtcur;
      if (nleg > 2) {
        for (leg = nleg - 2; leg; leg--) {
          dtcur = prvevs(src,leg);
          if (dtcur == 0) break;
        }
      }
      if (dtcur == 0) { v0++; vp += nleg; continue; }

      evcnt = getevs(src,nleg);
      error_z(evcnt,nleg);
      for (leg = 0; leg < nleg; leg++) {
        src->trip.trip[leg * 2 + 1] = vp[leg];
        src->trip.trip[leg * 2] = part;

        error_z(src->curts[leg],leg);
        src->trip.t[leg] = src->curts[leg];
        src->trip.dur[leg] = src->curdurs[leg];
        src->trip.tid[leg] = src->curtids[leg];
        info(0,"  leg %u dep \ad%u",leg,src->curts[leg]);
      }
      src->trip.cnt = 1;
      src->trip.len = nleg;
      leg--;
      src->lodt = src->curdt;
      src->lot = src->curts[leg];
      src->lotid = src->curtids[leg];
      src->lodist = dist;
    }

    v0++;
    vp += nleg;
    src->locvarcnt++;
  } while (v0 < cnt && cost > costlim && globs.sigint == 0);

  if (src->trip.cnt == 0) return info(0,"no time for trip %u-%u on \ad%u-\ad%u",dep,arr,deptmin,deptmax);

  src->locsrccnt++;

  src->lodist = lodist;
//  src->part = part;
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
  return srclocal(caller,net,part,dep,arr,stop,src,"local");
}

#define Distbins 256
#define Percbins 128

static ub4 srcxpart2(struct network *tnet,ub4 dpart,ub4 apart,ub4 gdep,ub4 garr,ub4 gdmid,ub4 gamid,search *src)
{
  ub4 tpart = tnet->part;
  ub4 tportcnt,dportcnt,aportcnt;
  ub4 hopcnt,thopcnt,ahopcnt,chopcnt,tchopcnt,achopcnt,whopcnt,twhopcnt,awhopcnt;
  struct network *dnet,*anet;
  ub4 dep,arr,dmid,amid,depmid,tdepmid,tdmid,tamid,adepmid,amidarr;
  ub2 *cnts,*tcnts,*acnts,cnt,tcnt,acnt,var,tvar,avar;
  ub4 *ofss,*tofss,*aofss,ofs,tofs,aofs;
  ub4 *vp,*tvp,*avp;
  ub4 dstop,tstop,astop,histop;
  ub4 nleg,ntleg,naleg,l,l1,l2,leg,tleg,aleg,triplen;
  block *lstblk,*tlstblk,*alstblk;
  ub4 *lst,*tlst,*alst;
  ub4 *lodists,*tlodists,*alodists,lodist,tlodist,alodist;
  ub4 *hopdist,*thopdist,*ahopdist;
  ub4 dist,distrange,distiv,iv,pct;
  ub4 dt,dtcur,dthi,dtndx,dtndx2;
  ub4 varcnt = 0;
  ub4 distbins[Distbins];
  ub4 distsums[Distbins];
  ub4 distlims[Percbins];
  ub4 *topdts;
  ub4 topdt1 = Topdts - 1;
  ub8 *events,*tevents,*aevents;
  ub2 *evmaps,*tevmaps,*aevmaps;
  ub4 evcnt;
  struct hop *hp,*hops,*thops,*ahops;
  struct timepat *tp;
  ub4 *choporg,*tchoporg,*achoporg;
  ub4 midur,*hopdur,*thopdur,*ahopdur;

  ub4 *trip;

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

  hopcnt = dnet->hopcnt;
  thopcnt = tnet->hopcnt;
  ahopcnt = anet->hopcnt;
  whopcnt = dnet->whopcnt;
  twhopcnt = tnet->whopcnt;
  awhopcnt = anet->whopcnt;
  chopcnt = dnet->chopcnt;
  tchopcnt = tnet->chopcnt;
  achopcnt = anet->chopcnt;

  choporg = dnet->choporg;
  tchoporg = tnet->choporg;
  achoporg = anet->choporg;
  hopdur = dnet->hopdur;
  thopdur = tnet->hopdur;
  ahopdur = anet->hopdur;

  error_zp(choporg,0);
  error_zp(hopdur,0);

  histop = src->histop;
  trip = src->trip.trip;

  dep = dnet->g2pport[gdep];
  dmid = dnet->g2pport[gdmid];
  depmid = dep * dportcnt + dmid;

  tdmid = tnet->g2pport[gdmid];
  tamid = tnet->g2pport[gamid];
  tdepmid = tdmid * tportcnt + tamid;

  amid = anet->g2pport[gamid];
  arr = anet->g2pport[garr];
  amidarr = amid * aportcnt + arr;

  events = dnet->events;
  evmaps = dnet->evmaps;
  hops = dnet->hops;

  tevents = tnet->events;
  tevmaps = tnet->evmaps;
  thops = tnet->hops;

  aevents = anet->events;
  aevmaps = anet->evmaps;
  ahops = anet->hops;

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
    warn(0,"no conn for top %u-%u",tdmid,tamid);
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
        error_ge(l,chopcnt);
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
      evcnt = 0;
      for (leg = 0; leg < nleg; leg++) {
        l = vp[leg];

        midur = hopdur[l];
        if (l < hopcnt) {
          hp = hops + l;
          tp = &hp->tp;
          evcnt = addevs(src,events,evmaps,leg,tp,hp,midur,dthi);
        } else if (l < whopcnt) {
          if (leg) evcnt = fwdevs(src,leg,l,midur,dthi);
          else evcnt = frqevs(src,leg,l,midur,walkfreq,dthi);
        } else {
          l1 = choporg[l * 2];
          l2 = choporg[l * 2 + 1];
          hp = hops + l1;
          tp = &hp->tp;
          evcnt = addevs(src,events,evmaps,leg,tp,hp,midur,dthi);
        }
        if (evcnt == 0) break;
        dtcur = src->dtcurs[leg];
        if (dtcur >= topdts[topdt1]) break;
      }
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
        error_ge(tofs,tnet->lstlen[tstop]);
        tvp = tlst + tofs * ntleg;

        for (tvar = 0; tvar < tcnt; tvar++) {

          for (tleg = 0; tleg < ntleg; tleg++) {
            l = tvp[tleg];
            error_ge(l,tchopcnt);
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
          for (tleg = 0; tleg < ntleg; tleg++) {
            l = tvp[tleg];
            midur = thopdur[l];

            if (l < thopcnt) {
              hp = thops + l;
              tp = &hp->tp;
              evcnt = addevs(src,tevents,tevmaps,tleg + nleg,tp,hp,midur,dthi);
            } else if (l < twhopcnt) {
              if (tleg) evcnt = fwdevs(src,tleg,l,midur,dthi);
              else evcnt = frqevs(src,tleg,l,midur,walkfreq,dthi);
            } else {
              l1 = tchoporg[l * 2];
              l2 = tchoporg[l * 2 + 1];
              hp = thops + l1;
              tp = &hp->tp;
              evcnt = addevs(src,tevents,tevmaps,tleg + nleg,tp,hp,midur,dthi);
            }
            if (evcnt == 0) break;
            dtcur = src->dtcurs[leg];
            if (dtcur >= topdts[topdt1]) break;
          }
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
                  error(Exit,"arr part %u port %u-%u stop %u var %u leg %u at %p %s",apart,amid,arr,astop,avar,aleg,avp,alstblk->desc);
                  break;
                }
                error_ge(l,achopcnt);
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

              evcnt = 0;
              dtcur = hi32;
              for (aleg = 0; aleg < naleg; aleg++) {
                l = avp[aleg];
                midur = ahopdur[l];

                if (l < ahopcnt) {
                  hp = ahops + l;
                  tp = &hp->tp;
                  evcnt = addevs(src,aevents,aevmaps,aleg + nleg + ntleg,tp,hp,midur,dthi);
                } else if (l < awhopcnt) {
                  if (aleg) evcnt = fwdevs(src,aleg,l,midur,dthi);
                  else evcnt = frqevs(src,aleg,l,midur,walkfreq,dthi);
                } else {
                  l1 = achoporg[l * 2];
                  l2 = achoporg[l * 2 + 1];
                  hp = ahops + l1;
                  tp = &hp->tp;
                  evcnt = addevs(src,aevents,aevmaps,aleg + nleg + ntleg,tp,hp,midur,dthi);
                }
                if (evcnt == 0) break;
                dtcur = src->dtcurs[aleg + nleg + ntleg];
                if (dtcur >= topdts[topdt1]) break;
              }
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

              evcnt = getevs(src,triplen);
              if (evcnt == 0) { avp += naleg; continue; }

              dtndx = 0;
              while (dtndx < Topdts && dtcur >= topdts[dtndx]) dtndx++;
              if (dtndx == topdt1) topdts[dtndx] = dtcur;
              else if (dtndx < topdt1) {
                for (dtndx2 = topdt1; dtndx2 > dtndx; dtndx2--) topdts[dtndx2] = topdts[dtndx2-1];
                info(0,"put dt %u at pos %u",dtcur,dtndx);
                topdts[dtndx] = dtcur;
              }
              dthi = topdts[topdt1];
              varcnt++;

              if (src->trip.cnt == 0 || src->curdt < src->lodt) {
                for (leg = 0; leg < nleg; leg++) {
                  trip[leg * 2 + 1] = vp[leg];
                  trip[leg * 2] = dpart;
                }
                for (tleg = 0; tleg < ntleg; tleg++) {
                  trip[leg * 2 + 1] = tvp[tleg];
                  trip[leg * 2] = tpart;
                  leg++;
                }
                for (aleg = 0; aleg < naleg; aleg++) {
                  trip[leg * 2 + 1] = avp[aleg];
                  trip[leg * 2] = apart;
                  leg++;
                }
                info(0,"store trip \aV%u%p dur %u dep \ad%u tid %x",triplen,trip,src->curdt,src->curt,src->lotid);
                for (leg = 0; leg < triplen; leg++) {
                  error_z(src->curts[leg],leg);
                  src->trip.t[leg] = src->curts[leg];
                  src->trip.dur[leg] = src->curdurs[leg];
                  src->trip.tid[leg] = src->curtids[leg];
                  info(0,"  leg %u dep \ad%u",leg,src->curts[leg]);
                }
                src->trip.cnt = 1;
                src->trip.len = triplen;
                leg--;
                src->lodt = src->curdt;
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

static ub4 srcxpart(struct gnetwork *gnet,ub4 gdep,ub4 garr,search *src,char *ref)
{
  struct network *net,*tnet,*anet;
  ub1 *portparts = gnet->portparts;
  ub4 partcnt = gnet->partcnt;
  ub4 dep,arr,deparr;
  ub4 dstop,astop,tstop,dlostop,dhistop;
  ub4 conn = 0;
  ub4 part,tpart,dpart,apart;
  ub4 ti,ati,tcnt,tacnt;
  ub4 dtmid,gdtmid,tdmid,tamid,atmid,gatmid,gamid,xamid,xmid,gdmid;
  ub4 portcnt,tportcnt,aportcnt;
  ub4 *tp2g;
  ub4 stats[8];
  ub4 xpartcnt = 0,estvar;
  ub4 iv,niv = Elemcnt(stats);
  int rv;
  struct eta eta;

  ub2 *xmap,*xamap,*xmapdbase,*xmapabase,stopset,xm,xam;
  ub1 *tmap;
  block *xpartdmap = &gnet->xpartdmap;
  block *xpartamap = &gnet->xpartamap;

  if (partcnt == 1) { error(0,"interpart search called without partitions, ref %s",ref); return 0; }

  aclear(stats);

  memset(src->topdts,0xff,sizeof(src->topdts));

  tpart = partcnt - 1;
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
      if (tdmid == tamid) continue;

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

          conn += srcxpart2(tnet,dpart,apart,gdep,garr,gdmid,gamid,src);
        }
      }
    }
  }

  for (iv = 0; iv < niv; iv++) info(0,"stats %u %u",iv,stats[iv]);

  return conn;
}

static ub4 dosrc(struct gnetwork *gnet,ub4 nstoplo,ub4 nstophi,search *src,char *ref)
{
  ub4 gportcnt = gnet->portcnt;
  ub1 *portparts = gnet->portparts;
  ub4 part,partcnt = gnet->partcnt;
  ub4 gdep = src->dep;
  ub4 garr = src->arr;
  ub4 stop;

  ub4 conn = 0;

  if (partcnt == 0) { error(0,"search called without partitions, ref %s",ref); return 0; }
  else if (gportcnt == 0) { error(0,"search without ports, ref %s",ref); return 0; }

  for (part = 0; part < partcnt; part++) {
    if (partcnt > 1 && (portparts[gdep * partcnt + part] == 0 || portparts[garr * partcnt + part] == 0)) continue;
    info(0,"dep %u and arr %u share part %u",gdep,garr,part);
    for (stop = nstoplo; stop < nstophi; stop++) {
      conn = srcglocal(gnet,part,gdep,garr,stop,src);
      if (conn) return conn;
    }
    if (conn) return conn;
  }

  conn = srcxpart(gnet,gdep,garr,src,ref);

  return conn;
}

// work in progress
int plantrip(search *src,char *ref,ub4 dep,ub4 arr,ub4 nstoplo,ub4 nstophi)
{
  ub4 port;
  ub4 stop,nleg,portno;
  ub4 conn;
  struct gnetwork *net = getgnet();
  ub4 portcnt = net->portcnt;
  struct port *parr,*pdep,*pp,*ports = net->ports;
  ub4 deptmin,deptmax;
  ub8 t0,dt;
  ub4 reslen;

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
  src->lodist = hi32;

  src->geodist = fgeodist(pdep,parr);

  deptmin = yymmdd2min(src->deptmin_cd,0);
  deptmax = deptmin + src->tspan * 60 * 24;

  src->deptmin = deptmin;
  src->deptmax = deptmax;
  src->histop = nstophi;

  t0 = gettime_usec();

  info(CC,"search dep %u arr %u on \ad%u-\ad%u %s to %s geodist %u",dep,arr,deptmin,deptmax,pdep->name,parr->name,src->geodist);
  conn = dosrc(net,nstoplo,nstophi,src,ref);
  info(0,"searched %u local variants in %u local searches %u noloc",src->avarxcnt,src->locsrccnt,src->locnocnt);
  info(0,"%u of %u depvars %u of %u midvars %u of %u arrvars %u stored",src->dvarxcnt,src->dvarcnt,src->tvarxcnt,src->tvarcnt,src->avarxcnt,src->avarcnt,src->varcnt);

  dt = gettime_usec() - t0;
  info(0,"search took \a.%u usec",(ub4)dt);

  if (conn == 0) {
    if (src->avarxcnt) return info(0,"no time found for %u stop\as",nstophi);
    else return info(0,"no route found for %u stop\as",nstophi);
  }
  nleg = src->lostop + 1;
  info(0,"found %u-%u in %u/%u legs",dep,arr,src->trip.len,nleg);

  reslen = sizeof(src->resbuf);
  if (gtriptoports(net,&src->trip,src->resbuf,&reslen)) {
    src->trip.cnt = 0;
    return 1;
  }
  src->reslen = reslen;
  return 0;
}
