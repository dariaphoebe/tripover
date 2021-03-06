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
   heuristics on duration act as branch-and-bound exhaustive search on event times only
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

// time limit in msec for searches
static const ub8 Timelimit = 3000;

static const ub4 altlimit = 1024 * 4;

// bias for 'fewer transfer' search
static const ub4 stopcostfac = 8;

// emulate walk after walk links as frequency-based
static const ub4 walkiv_min = 5;

static int vrbena;

void inisearch(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
  vrbena = (getmsglvl() >= Vrb);
}

enum evfld { timefld,tidfld,dtfld,durfld,costfld,prvfld,farefld,sdafld,flnfld,fldcnt };

static const ub4 evmagic1 = 0xbdc90b28;
static const ub4 evmagic2 = 0x695c4b78;

static void inisrc(search *src,const char *desc,ub4 arg)
{
  ub4 i,leg,t,legevs;
  ub4 *ev,*evbase;
  struct trip * stp;

  fmtstring(src->desc,"%s %u",desc,arg);

  src->locost = hi32;
  src->lodist = hi32;
  src->lodt = hi32;
  src->reslen = 0;

  for (t = 0; t < Elemcnt(src->trips); t++) {
    stp = src->trips + t;
    stp->cnt = stp->len = 0;
    for (i = 0; i < Nxleg; i++) {
      stp->trip[i * 2] = stp->trip[i * 2 + 1] = hi32;
      stp->port[i] = hi32;
    }
  }
  legevs = Maxevs * fldcnt;
  if (src->evpool == NULL) {
    evbase = src->evpool = alloc(Nxleg * (Maxevs + 2) * fldcnt,ub4,0,"src events",Maxevs);
  } else evbase = src->evpool;
  ev = evbase;
  for (leg = 0; leg < Nxleg; leg++) { // add a redzone around each leg
    ev = evbase + leg * (Maxevs + 2) * fldcnt;
    *ev = evmagic1;
    src->depevs[leg] = ev + fldcnt;
    ev += (Maxevs + 1) * fldcnt;
    *ev = evmagic2;
  }
  if (ev >= evbase + Nxleg * (Maxevs + 2) * fldcnt) error(Exit,"ev %p above %p",ev,evbase + Nxleg * (Maxevs + 2) * fldcnt);
}

static void timelimit(search *src,ub4 limit)
{
  src->querytlim = min(src->querytlim,src->queryt0 + 1000UL * limit);
  src->tlim = min(src->tlim,limit);
}

static void fmtsum(struct trip *stp,ub4 dt,ub4 nxt,ub4 dist,ub4 fare,ub4 lodt,const char *ref)
{
  ub4 len = (ub4)sizeof(stp->desc);
  ub4 pos = mysnprintf(stp->desc,0,len,"#item\ttime\tdistance\tstops\nfare\tref\n");

  if (nxt == 0) nxt = hi32;
  mysnprintf(stp->desc,pos,len,"sum\t\at%u\t\ag%u\t%u\t%u\t\at%u\t%s-%u\n",dt,dist,stp->len - 1,fare,nxt,ref,lodt);
}

static int chkdev(ub4 *dev,ub4 leg)
{
  ub4 *d1 = dev - fldcnt;
  ub4 *d2 = dev + Maxevs * fldcnt;

  if (*d1 != evmagic1) return errorfln(FLN,0,d1[flnfld],"leg %u magic1 mismatch %x at %p",leg,*d1,d1);
  else if (*d2 != evmagic2) return errorfln(FLN,0,d2[flnfld],"leg %u magic2 mismatch %x at %p",leg,*d2,d2);
  else return 0;
}

// get suitable departure window given ev frequency
// based on leg with lowest frequency around deptime
static ub4 getdepwin(search *src,lnet *net,ub4 *legs,ub4 nleg)
{
  ub4 deptmin = src->deptmin;
  ub4 gencnt,gndx;
  ub4 leg,hop,hop2;
  ub4 t,tf;
  ub4 gt0,t0,t1;
  ub8 x,*ev;
  ub4 lospan4,lospan8,lospan24,lospan48,lospan72;
  ub4 span4,span8,span24,span48,span72;
  struct timepat *tp,*tp1,*tp2;
  struct hop *hp1,*hp2,*hops = net->hops;
  ub4 *choporg = net->choporg;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub8 *events = net->events;
  ub4 hop1;

  lospan4=lospan8=lospan24=lospan48=lospan72 = hi32;

  for (leg = 0; leg < nleg; leg++) {
    hop = legs[leg];
    if (hop < hopcnt) {
      hop1 = hop;
      hop2 = hi32;
      tp2 = NULL;
    } else if (hop < chopcnt) {
      hop1 = choporg[hop * 2];
      hop2 = choporg[hop * 2 + 1];
      hp2 = hops + hop2;
      tp2 = &hp2->tp;
    } else continue;

    error_ge(hop1,hopcnt);

    hp1 = hops + hop1;
    tp1 = &hp1->tp;
    if (hop2 != hi32 && tp2->genevcnt < tp1->genevcnt) tp = tp2;
    else tp = tp1;

    gencnt = tp->genevcnt;
    gt0 = tp->gt0;
    ev = events + tp->evofs;

    if (src->firsthop[leg] == hop) {
      tf = src->first[leg];
      gndx = src->firstndx[leg];
    } else {
      src->firsthop[leg] = hop;
      src->first[leg] = 0;

      tf = 0;

      t0 = tp->t0;
      t1 = tp->t1;

      if (gencnt == 0) break;
      if (t0 == t1) break;
      if (t1 + gt0 <= deptmin) break;

      for (gndx = 0; gndx < gencnt; gndx++) {
        x = ev[gndx * 2];
        tf = (ub4)x + gt0;
        if (tf >= deptmin) break;
      }
      src->first[leg] = tf;
      src->firstndx[leg] = gndx;
    }
    if (tf == 0) return 0;

    t = tf;
    span8 = span24 = span48 = span72 = 0;
    span4 = 1;

    while (gndx < gencnt && t < tf + 4 * 60 && span4 < 13) {
      x = ev[gndx * 2];
      t = (ub4)x + gt0;
      span4++;
    }
    if (span4 == 13) continue;
    lospan4 = min(lospan4,span4);
    span8 = span4;

    while (gndx < gencnt && t < tf + 8 * 60 && span8 < 13) {
      x = ev[gndx * 2];
      t = (ub4)x + gt0;
      span8++;
    }
    if (span8 == 13) continue;
    lospan8 = min(lospan8,span8);
    span24 = span8;

    while (gndx < gencnt && t < tf + 24 * 60 && span24 < 13) {
      x = ev[gndx * 2];
      t = (ub4)x + gt0;
      span24++;
    }
    if (span24 == 13) continue;
    lospan24 = min(lospan24,span24);
    span48 = span24;

    while (gndx < gencnt && t < tf + 48 * 60 && span48 < 13) {
      x = ev[gndx * 2];
      t = (ub4)x + gt0;
      span48++;
    }
    if (span48 == 13) continue;
    lospan48 = min(lospan48,span48);
    span72 = span48;

    while (gndx < gencnt && t < tf + 72 * 60 && span72 < 13) {
      x = ev[gndx * 2];
      t = (ub4)x + gt0;
      span72++;
    }
    lospan72 = min(lospan72,span72);
  }
  if (lospan4 > 12) return 4;
  else if (lospan8 > 12) return 8;
  else if (lospan24 > 12) return 24;
  else if (lospan48 > 12) return 48;
  else if (lospan72 > 12) return 72;
  else return 168;
}

// create list of candidate events for first leg
static ub4 mkdepevs(search *src,lnet *net,ub4 hop,ub4 midur,ub4 costlim)
{
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 deptmid = src->deptmid;
  ub4 dcnt = 0,lodev,lodev2,gencnt;
  ub4 gndx,dmax = Maxevs;
  ub4 t,prvt,dur,lodur,*dev,*devp;
  ub4 srdep,srarr,srda,srda2;
  ub8 x,x1,*ev;
  ub4 tdep1,tarr2;
  ub4 tid;
  struct timepat *tp;
  ub8 *crp,*chainrhops = net->chainrhops;
  ub8 *crpp,*chainrphops = net->chainrphops;
  struct chain *cp,*chains = net->chains;
  struct hop *hp1,*hp2,*hops = net->hops;
  ub4 *choporg = net->choporg;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 chaincnt = net->chaincnt;
  ub8 *events = net->events;
  ub4 hop1,hop2,rh1,rh2,rid;
  ub4 ofs;
  ub2 fare;
  ub2 *farepos,*fareposbase = net->fareposbase;
  ub4 deptbias;
  ub4 cost,locost;

  src->dcnts[0] = src->dcnts[1] = 0;
  src->nleg = 0;

  if (hop < hopcnt) {
    hop1 = hop; hop2 = hi32;
    rh2 = 0;
  } else if (hop < chopcnt) {
    hop1 = choporg[hop * 2];
    hop2 = choporg[hop * 2 + 1];
    hp2 = hops + hop2;
    rh2 = hp2->rhop;
  } else return error(Ret0,"unexpected walk link %u on initial leg",hop);

  error_ge(hop1,hopcnt);
  hp1 = hops + hop1;
  tp = &hp1->tp;
  rid = hp1->rid;
  rh1 = hp1->rhop;

  if (rh1 >= Chainlen || rh2 >= Chainlen) return 0;

  gencnt = tp->genevcnt;

  ub4 gt0 = tp->gt0;

  ub4 t0 = tp->t0;
  ub4 t1 = tp->t1;

  ub4 extracost;

  if (gencnt == 0) return vrb0(Notty,"no events for hop %u at \ad%u - \ad%u",hop,t0 + gt0,t1 + gt0);

  if (t0 == t1) return info(Iter|Notty,"hop %u tt range %u-%u, dep window %u-%u",hp1->gid,t0,t1,deptmin,deptmax);
//  else if (ht0 > deptmax) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,ht0,ht1,deptmin,deptmax);
//  else if (ht1 <= deptmin) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,ht0,ht1,deptmin,deptmax);

  locost = hi32;

  ev = events + tp->evofs;
  dev = src->depevs[0];

  if (chkdev(dev,0)) return 0;

  ub4 gndx0 = 0;
  // if out of departure window, take a single event
  if (t0 + gt0 > deptmax) {
    gencnt = 1;
//    extracost_win = t0 + gt0 - deptmax;
    info(Notty|Iter,"hop %u \ad%u - \ad%u after dep window \ad%u - \ad%u",hp1->gid,t0 + gt0,t1 + gt0,deptmin,deptmax);

  } else if (t1 + gt0 <= deptmin) {
    info(Notty|Iter,"hop %u \ad%u - \ad%u before dep window \ad%u - \ad%u %s",hp1->gid,t0 + gt0,t1 + gt0,deptmin,deptmax,hp1->name);
    gndx0 = gencnt - 1;
//    extracost_win = (deptmin - (t1 + gt0)) * 8;
  }

  if (hp1->reserve && net->fhopofs) {
    ofs = net->fhopofs[hop];
    farepos = fareposbase + ofs * Faregrp;
  } else farepos = NULL;

  prvt = 0;
  lodur = hi32; lodev = lodev2 = hi32;
  for (gndx = gndx0; gndx < gencnt; gndx++) {
    x = ev[gndx * 2];
    t = (ub4)x;
    if (t < prvt) {
      warn(0,"t \ad%u before \ad%u",t+gt0,prvt+gt0);
      continue;
    }
    prvt = t;

    if (t + gt0 < deptmin && gndx0 == 0) continue;
    else if (t + gt0 >= deptmax && gencnt > 1) break;

//    extracost = extracost_win;
    extracost = 0;

    // add minor bias to prefer times around given date/time
    if (t + gt0 < deptmid) {
      deptbias = deptmid - (t + gt0);
      extracost += min(deptbias,1440) / 50;
      if (deptbias > 1440) extracost += (deptbias - 1440) / 200;
    } else {
      deptbias = t + gt0 - deptmid;
      extracost += min(deptbias,1440) / 100;
      if (deptbias > 1440) extracost += (deptbias - 1440) / 400;
    }

    if (farepos) {
      fare = farepos[gndx * Faregrp];
//      info(0,"hop %u ev %u t \ad%u fare %u at %p",hop,gndx,t + gt0,fare,farepos + gndx * Faregrp);
      if (fare == hi16) continue;
    } else fare = 0; // todo: use nonreserved fare rules

    x1 = ev[gndx * 2 + 1];

    if (hop < hopcnt) {
      dur = (ub4)(x >> 32); // from event
      srda = (ub4)(x1 >> 48);
      infocc(dur == hi32,Notty,"hop %u dur hi32",hop);

    } else { // compound: get dur and srarr from chain
      tid = x1 & hi24;
      srda = (ub4)(x1 >> 48);
      srdep = srda >> 8;
      error_ge(tid,chaincnt);
      cp = chains + tid;
      if (cp->hopcnt < 2) continue;
      if (rh1 >= cp->rhopcnt || rh2 >= cp->rhopcnt) continue;
      crp = chainrhops + cp->rhopofs;
      crpp = chainrphops + cp->rhopofs;
      tdep1 = (ub4)(crp[rh1] >> 32);
      tarr2 = crp[rh2] & hi32;
      if (tdep1 == hi32 || tarr2 == hi32) continue; // this trip does not have the compound
      else if (tarr2 < tdep1) {
        tdep1 = (ub4)(crp[rh2] >> 32);
        tarr2 = crp[rh1] & hi32;
        if (tarr2 < tdep1) {
          warn(Notty,"chop %u-%u tdep %u tarr %u",hop1,hop2,tdep1,tarr2);
          continue;
        }
      }
      dur = tarr2 - tdep1;
      infocc(0,Notty,"chop %u-%u dur hi32",hop1,hop2);
      srda2 = (ub4)crpp[rh1];
      if ( (srda2 >> 8) != srdep) warn(0,"srdep %u vs %u",srda2 >> 8,srdep);
      srda2 = (ub4)crpp[rh2];
      srarr = srda2 & 0xff;
      srda = (srdep << 8) | srarr;
    }

    if (dur == hi32) dur = midur;
    error_eq(dur,hi32);

//    extracost = min(extracost,dur / 10);

    cost = dur + extracost;

    if (cost >= costlim) continue;

    devp = dev + dcnt * fldcnt;
    devp[timefld] = t + gt0;
    devp[tidfld] = (ub4)x1 & hi24;
    devp[dtfld] = dur;
    devp[durfld] = dur;
    devp[costfld] = cost;
    devp[farefld] = fare;
    devp[prvfld] = hi32;
    devp[sdafld] = srda;
    devp[flnfld] = FLN;

    if (cost < locost) { locost = cost; lodev = dcnt; }
    else if (cost == locost && lodev2 == hi32) lodev2 = dcnt;  // support 'next trip in <time>'
    dcnt++;
    if (dcnt >= dmax) {
      warning(Notty,"exceeding %u dep event",dcnt);
      break;
    }
  }
  src->dcnts[0] = dcnt;
  src->duraccs[0] = tp->duracc;
  if (dcnt == 0) return 0;

  src->nleg = 1;

  src->hop1s[0] = hop1;
  src->hp1s[0] = hp1;
  src->hop2s[0] = hop2;
  src->parts[0] = net->part;

  src->costcurs[0] = locost;
  src->devcurs[0] = lodev;
  src->devcurs2[0] = lodev2;
  warncc(lodev >= dcnt,Notty,"leg 0 lodev %u cnt %u",lodev,dcnt);
  infocc(vrbena,Notty,"%u dep events for leg 0",dcnt);
  if (chkdev(dev,0)) return 0;
  return dcnt;
}

// create list of candidate events for subsequent legs
static ub4 nxtevs(search *src,lnet *net,ub4 leg,ub4 bstop,ub4 hop,ub4 midur,ub4 costlim)
{
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 aleg,ahop;
  ub4 dndx,dcnt,ddcnt,gencnt;
  ub4 gndx,agndx,prvgndx,dmax = Maxevs;
  ub4 adndx,adcnt,ofs;
  ub4 rt,t,last,at,dur,atarr,adur,dt,adt;
  ub4 srdep,srarr,srda,srda2;
  ub4 tid,atid,lodev,lodev2,loadev;
  ub4 *dev,*adev,*devp,*adevp;
  ub4 cost,curcost,locost,acost;
  ub4 ttmax = src->maxtt;
  ub4 ttmin = src->mintt;
  ub8 x,x1,*ev;
  ub4 tdep1,tarr2;
  struct timepat *tp;
  ub8 *crp,*chainrhops = net->chainrhops;
  ub8 *crpp,*chainrphops = net->chainrphops;
  struct chain *cp,*chains = net->chains;
  struct hop *hp1,*hp2,*ahp,*hops = net->hops;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 chaincnt = net->chaincnt;
  ub4 *choporg = net->choporg;
  ub8 *events = net->events;
  ub4 hop1,hop2,rh1,rh2,gid,rid,rid2;
  ub4 costperstop = src->costperstop;
  ub4 fare,afare;
  ub2 *farepos,*fareposbase = net->fareposbase;
  ub4 ttbiascost;

  error_z(leg,0);
  aleg = leg - 1;

  src->dcnts[leg] = 0;
  if (leg + 1 < Nxleg) src->dcnts[leg+1] = 0;
  if (src->nleg < leg) return error(Ret0,"nleg %u vs %u",src->nleg,leg);

  if (hop < hopcnt) {
    hop1 = hop; hop2 = hi32;
    hp2 = NULL;
    rh2 = 0;
  } else if (hop < chopcnt) {
    hop1 = choporg[hop * 2];
    hop2 = choporg[hop * 2 + 1];
    hp2 = hops + hop2;
    rh2 = hp2->rhop;
  } else return warn(Iter,"walk link not expected for %u",hop);

  error_ge(hop1,hopcnt);
  hp1 = hops + hop1;
  tp = &hp1->tp;
  gid = hp1->gid;
  rid = hp1->rid;
  rh1 = hp1->rhop;

  infocc(rh1 >= Chainlen,Notty,"hop %u -> %u rhop %u",hop,hop1,rh1);
  infocc(hop >= hopcnt && rh2 >= Chainlen,Notty,"hop %u -> %u-%u rhop %u",hop,hop1,hop2,rh2);

  if (hp2) {
    rid2 = hp2->rid;
    warncc(rid2 != rid,0,"hop %u -> %u-%u rid %u vs %u",hop,hop1,hop2,rid,rid2);
  }

  if (rh1 >= Chainlen || rh2 >= Chainlen) return 0;

  gencnt = tp->genevcnt;

  if (gencnt == 0) return 0;

  ub4 gt0 = tp->gt0;

  ub4 t0 = tp->t0;
  ub4 t1 = tp->t1;

  ttmin = max(tp->duracc,ttmin);

  adcnt = src->dcnts[aleg];
  if (adcnt == 0) return 0;

  if (t0 == hi32) return warn(0,"hop %u tt range %u-%u, dep window %u-%u",gid,t0,t1,deptmin,deptmax);
  else if (t1 == hi32) return warn(0,"hop %u tt range %u-%u, dep window %u-%u",gid,t0,t1,deptmin,deptmax);
  else if (gt0 == hi32) return warn(0,"hop %u tt range %u-%u, dep window %u-%u",gid,t0,t1,deptmin,deptmax);

  if (t0 == t1) return info(Notty,"hop %u tt range %u-%u, dep window %u-%u",gid,t0,t1,deptmin,deptmax);

  if (hp1->reserve && net->fhopofs) {
    ofs = net->fhopofs[hop];
    farepos = fareposbase + ofs * Faregrp;
  } else farepos = NULL;

  ahop = src->hop1s[aleg];
  ahp = src->hp1s[aleg];
  if (ahop >= chopcnt) ttmin = 0; // walk arrivals have no transfer time

  // air to any
  else if (ahp && ahp->kind == Airint) ttmin = 90;
  else if (ahp && ahp->kind == Airdom) ttmin = 60;

  // any to air
  if (hp1->kind == Airint) {
    ttmin = 120; ttmax = 24 * 60;
  } else if (hp1->kind == Airdom) {
    ttmin = 90; ttmax = 24 * 60;
  }

  src->duraccs[leg] = tp->duracc;

  ev = events + tp->evofs;

  dev = src->depevs[leg];
  adev = src->depevs[aleg];

  if (chkdev(dev,leg)) return 0;
  if (chkdev(dev,aleg)) return 0;

  locost = hi32; lodev = loadev = lodev2 = hi32;

  prvgndx = agndx = adndx = 0;
  dcnt = dndx = last = 0;
  while (adndx < adcnt && prvgndx < gencnt && dcnt < dmax) {
    adevp = adev + adndx * fldcnt;
    at = adevp[timefld];
    adt = adevp[dtfld];
    adur = adevp[durfld];
    atid = adevp[tidfld];
    acost = adevp[costfld];
    afare = adevp[farefld];
    atarr = at + adur;

//    warncc(adur > hi16,0,"adur %u",adur);

    // search first candidate departure
    // note that the output event may already have been written for last iter
    gndx = prvgndx;
    while (gndx < gencnt) {
      x = ev[gndx * 2];
      rt = (ub4)x;
      if (rt + gt0 >= at) break;
      gndx++;
    }
    prvgndx = gndx;

    ddcnt = 0;

    while (gndx < gencnt && dcnt < dmax) {
      x = ev[gndx * 2];
      rt = (ub4)x;

      x1 = ev[gndx * 2 + 1];
      tid = x1 & hi24;
      gndx++;
      t = rt + gt0;

      // below min transfer time, not same trip
      if ( (tid != atid && t < atarr + ttmin) || t < atarr) continue;

      // early out on txtime only
      cost = acost + t - atarr;
      if (cost >= costlim) break;

      if (t > atarr + ttmax) {
        if ( (ddcnt > 8 && ddcnt != dcnt) || (ddcnt > 64) ) break;
        ddcnt++;
      }

      if (farepos) {
        fare = farepos[gndx * Faregrp];
//      info(0,"hop %u ev %u t \ad%u fare %u at %p",hop,gndx,t + gt0,fare,farepos + gndx * Faregrp);
        if (fare == hi16) continue;
      } else fare = 0; // todo: use nonreserved fare rules

      if (hop < hopcnt) {
        dur = (ub4)(x >> 32); // from event
//        infocc(dur == hi32,Notty,"hop %u dur hi",hop);
        srda = (ub4)(x1 >> 48);

      } else { // compound: get dur from chain
        error_ge(tid,chaincnt);
        cp = chains + tid;

        if (cp->rid != rid) {
          info(Notty,"hop %u -> %u-%u %s tid %u rid %u vs %u",hop,hop1,hop2,hp1->name,tid,rid,cp->rid); // todo?
          info(Notty,"rrid %u cnt %u",cp->rrid,cp->hopcnt); // todo?
          continue;
        }
        if (cp->hopcnt < 2) continue;

        srda = (ub4)(x1 >> 48);
        srdep = (srda >> 8) & 0xff;

        crp = chainrhops + cp->rhopofs;
        crpp = chainrphops + cp->rhopofs;

        if (rh1 >= cp->rhopcnt) {
          info(Notty,"chop %u rh1 %u rhopcnt %u tid %u",hop,rh1,cp->rhopcnt,tid);
          continue;
        }
        if (rh2 >= cp->rhopcnt) {
          info(Notty,"chop %u rh2 %u rhopcnt %u tid %u",hop,rh2,cp->rhopcnt,tid);
          continue;
        }
        tdep1 = (ub4)(crp[rh1] >> 32);
        tarr2 = crp[rh2] & hi32;
        if (tdep1 == hi32 || tarr2 == hi32) continue; // this trip does not have the compound
        else if (tarr2 < tdep1) {
          tdep1 = (ub4)(crp[rh2] >> 32);
          tarr2 = crp[rh1] & hi32;
          if (tarr2 < tdep1) {
            warn(0,"chop %u-%u tdep %u tarr %u",hop1,hop2,tdep1,tarr2);
            continue;
          }
        }
        srda2 = (ub4)crpp[rh1];
        if ( (srda2 >> 8) != srdep) info(Notty|Iter,"srdep %u vs %u",srda2 >> 8,srdep);  // todo
        srda2 = (ub4)crpp[rh2];
        srarr = srda2 & 0xff;
        srda = (srdep << 8) | srarr;

        dur = tarr2 - tdep1;
      }

      if (dur == hi32) dur = midur;
      error_eq(dur,hi32);

      dt = adt + dur + t - atarr;   // accumulate total trip time

      // currently, cost is duration plus transfer cost
      cost = acost + dur + t - atarr;
      if (costperstop) {
        ttbiascost = ( min(adt + dur,1440) * bstop * costperstop) / stopcostfac;
        ttbiascost += bstop * 5;
        curcost = cost + ttbiascost;
      } else curcost = cost;

      if (curcost >= costlim) break;

      if (t > last || dcnt == 0) {  // new entry
        devp = dev + dcnt++ * fldcnt;

        last = t;

      } else { // written in earlier pass or unordered
        dndx = dcnt;    // search matching entry
        do {
          dndx--;
          devp = dev + dndx * fldcnt;
          if (devp[timefld] == t) break;
        } while (dndx);
        if (devp[timefld] == t) {
          if (cost >= devp[costfld]) continue; // overwrite only if better
        } else devp = dev + dcnt++ * fldcnt;
      }

      devp[timefld] = t;
      devp[tidfld] = tid;
      devp[dtfld] = dt;
      devp[durfld] = dur;
      devp[costfld] = cost;
      devp[prvfld] = adndx;
      devp[farefld] = fare + afare;
      devp[flnfld] = FLN;
      devp[sdafld] = srda;

      if (curcost < locost) { locost = curcost; lodev = dndx; loadev = adndx; }
      else if (curcost == locost && lodev2 == hi32) lodev2 = dcnt;

    } // each dep
    adndx++;
  } // each prvarr

  if (chkdev(dev,leg)) return 0;
  if (chkdev(dev,aleg)) return 0;
  src->dcnts[leg] = dcnt;
  if (dcnt == 0) return 0;

  warncc(dcnt >= dmax,Notty,"exceeding %u dep event",dcnt);
  if (lodev >= dcnt) return warn(0,"leg %u lodev %u cnt %u",leg,lodev,dcnt);

  src->nleg = leg + 1;

  src->hop1s[leg] = hop1;
  src->hp1s[leg] = hp1;
  src->hop2s[leg] = hop2;
  src->parts[leg] = net->part;

  src->costcurs[leg] = locost;
  src->devcurs[leg] = lodev;
  src->devcurs2[leg] = lodev2;
  infocc(vrbena,Notty,"%u dep events for leg %u, locost %u",dcnt,leg,locost);
  return dcnt;
}

// forward existing evs, e.g. walk link after nonwalk
static ub4 fwdevs(search *src,lnet *net,ub4 leg,ub4 bstop,ub4 hop1,ub4 hop2,ub4 midur,ub4 costlim)
{
  ub4 aleg;
  ub4 dcnt = 0;
  ub4 dmax = Maxevs;
  ub4 adndx,adcnt;
  ub4 t,at,adur,acost,lodev,lodev2,loadev,dt,adjdt,adt;
  ub4 *dev,*adev,*devp,*adevp;
  ub4 afare;
  ub4 cost,locost,curcost,txbiascost;
  ub4 costperstop = src->costperstop;

  error_z(leg,hop1);
  error_ge(midur,hi16);

  src->dcnts[leg] = 0;
  if (leg + 1 < Nxleg) src->dcnts[leg+1] = 0;

  aleg = leg - 1;
  if (src->nleg < leg) return error(Ret0,"nleg %u vs %u",src->nleg,leg);

  adcnt = src->dcnts[aleg];
  if (adcnt == 0) return 0;

  cost = src->costcurs[aleg];

  txbiascost = (min(midur,1440) * bstop * costperstop) / stopcostfac;
  if (costperstop) txbiascost += bstop * 5;
  curcost = cost + txbiascost;
  if (curcost >= costlim) return 0;

  dev = src->depevs[leg];
  adev = src->depevs[aleg];

  if (chkdev(dev,leg)) return 0;
  if (chkdev(dev,aleg)) return 0;

  locost = hi32; lodev = lodev2 = loadev = hi32;
  adndx = 0;
  while (adndx < adcnt && dcnt < dmax) {
    adevp = adev + adndx * fldcnt;
    at = adevp[timefld];
    adt = adevp[dtfld];
    adur = adevp[durfld];
    acost = adevp[costfld];
    afare = adevp[farefld];

    t = at + adur + 2;
    dt = adt + midur + 2;
    cost = acost + midur + 2;

    if (costperstop) {
      txbiascost = (min(dt,1440) * bstop * costperstop) / stopcostfac;
      txbiascost += bstop * 5;
      curcost = cost + txbiascost;
    } else curcost = cost;

    if (curcost >= costlim) { adndx++; continue; }

    devp = dev + dcnt * fldcnt;
    devp[timefld] = t;
    devp[tidfld] = hi32;
    devp[dtfld] = dt;
    devp[durfld] = midur;
    devp[prvfld] = adndx;
    devp[farefld] = afare;
    devp[sdafld] = hi16;
    devp[flnfld] = FLN;
    adjdt = acost + midur;

    devp[costfld] = cost;

    if (curcost < locost) { locost = curcost; lodev = dcnt; loadev = adndx; }
    else if (curcost == locost && lodev2 == hi32) lodev2 = dcnt;
    dcnt++;

    adndx++;
  }
  if (chkdev(dev,leg)) return 0;
  if (chkdev(dev,aleg)) return 0;
  src->dcnts[leg] = dcnt;
  if (dcnt == 0) return 0;
  if (lodev >= dcnt) return warn(0,"leg %u lodev %u cnt %u",leg,lodev,dcnt);

  src->nleg = leg + 1;

  src->hop1s[leg] = hop1;
  src->hop2s[leg] = hop2;
  src->hp1s[leg] = NULL;
  src->parts[leg] = net->part;

  src->costcurs[leg] = locost;
  src->devcurs[leg] = lodev;
  src->devcurs2[leg] = lodev2;
  src->duraccs[leg] = 1;
  infocc(vrbena,Notty,"%u dep events for leg %u",dcnt,leg);

  return dcnt;
}

// fill events with frequency, e.g. walk link before nonwalk or 'every x min' metro
// leg0 only
static ub4 frqevs(search *src,lnet *net,ub4 leg,ub4 hop1,ub4 nxthop,ub4 midur,ub4 iv_min,ub4 costlim)
{
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 gencnt,gndx;
  ub4 gt0,t0,t1;
  ub8 x,*ev;
  ub4 dcnt = 0,dmax = Maxevs;
  ub4 t,rt,lodt,lodev,dt,*dev,*devp;
  ub4 chop1;

  struct timepat *tp;
  struct hop *hp,*hops = net->hops;
  ub8 *events = net->events;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 *choporg = net->choporg;

  error_nz(leg,hop1);

  src->dcnts[0] = src->dcnts[1] = 0;
  src->nleg = 0;

  if (midur >= costlim) return 0;

  dev = src->depevs[leg];
  if (chkdev(dev,leg)) return 0;

  dt = midur;
  lodt = dt; lodev = 0;

  if (nxthop != hi32) {
    if (nxthop < hopcnt) hp = hops + nxthop;
    else if (nxthop < chopcnt) {
      chop1 = choporg[nxthop * 2];
      hp = hops + chop1;
     } else return error(Notty,"unexpected walk link %u",nxthop);
    tp = &hp->tp;
    gencnt = tp->genevcnt;
    ev = events + tp->evofs;

    gt0 = tp->gt0;
    t0 = tp->t0;
    t1 = tp->t1;

    for (gndx = 0; gndx < gencnt; gndx++) {
      x = ev[gndx * 2];
      rt = (ub4)x;
      gndx++;
      t = rt + gt0;
      t -= min(t,dt);
      if (t < deptmin) continue;
      else if (t > deptmax) break;

      devp = dev + dcnt * fldcnt;
      devp[timefld] = t;
      devp[tidfld] = hi32;
      devp[prvfld] = hi32;
      devp[dtfld] = midur;
      devp[durfld] = midur;
      devp[costfld] = midur;
      devp[farefld] = 0;
      devp[sdafld] = hi16;
      devp[flnfld] = FLN;

      dcnt++;
      if (dcnt >= dmax) break;
    }

  } else {

    t = 0;
    while (t + deptmin < deptmax && dcnt < dmax) {

      devp = dev + dcnt * fldcnt;
      devp[timefld] = t + deptmin;
      devp[tidfld] = hi32;
      devp[prvfld] = hi32;
      devp[dtfld] = midur;
      devp[durfld] = midur;
      devp[costfld] = midur;
      devp[farefld] = 0;
      devp[sdafld] = hi16;
      devp[flnfld] = FLN;

      t += iv_min;
      dcnt++;
    }
  }

  if (chkdev(dev,leg)) return 0;
  src->dcnts[leg] = dcnt;
  if (dcnt == 0) return 0;
  if (lodev >= dcnt) return warn(0,"leg %u lodev %u cnt %u",leg,lodev,dcnt);

  src->nleg = 1;

  src->hop1s[leg] = hop1;
  src->hop2s[leg] = hi32;
  src->hp1s[leg] = NULL;
  src->parts[leg] = net->part;

  src->costcurs[leg] = lodt;
  src->devcurs[leg] = lodev;
  src->duraccs[leg] = 0;

  infocc(vrbena,Notty,"%u freq events for leg %u before %u",dcnt,leg,nxthop);
  return dcnt;
}

static ub4 getevs(search *src,gnet *gnet,ub4 nleg,ub4 topn)
{
  ub4 dcnt = 0;
  ub4 lodev,nxtlodev,l,part,fln;
  ub4 t,tid,at,dur,cost,dt,srdep,srarr,srda,*dev,*devp;
  ub4 *lodevs = src->devcurs;
  ub4 *lodevs2 = src->devcurs2;
  lnet *net;
  ub4 rtid,*tid2rtid;
  ub4 hop1,hop2,hopcnt,chopcnt,tidcnt;
  ub4 fare;

  error_z(nleg,0);

  if (nleg != src->nleg) return error(Ret0,"nleg %u vs %u",nleg,src->nleg);

  dt = hi32;
  at = hi32;
  l = nleg - 1;
  if (topn == 0) {
    lodev = lodevs[l];
    if (lodev == hi32) return warn(0,"no events for leg %u",l);
  } else {
    lodev = lodevs2[l];
    if (lodev == hi32) return 0;
  }
  dcnt = src->dcnts[l];
  if (dcnt == 0) return warn(0,"no events for leg %u",l);
  else if (lodev >= dcnt) return warn(0,"lodev %u cnt %u for leg %u",lodev,dcnt,l);

  do {
    dev = src->depevs[l];
    dcnt = src->dcnts[l];

    if (dcnt == 0) return warn(0,"no events for leg %u",l);

    if (chkdev(dev,l)) return 0;
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

    devp = dev + lodev * fldcnt;
    t = devp[timefld];
    tid = devp[tidfld];
    dt = devp[dtfld];
    dur = devp[durfld];
    cost = devp[costfld];
    srda = devp[sdafld];
    fln = devp[flnfld];

    srdep = srda >> 8;
    srarr = srda & 0xff;
    if (srdep == 0xff) srdep = hi32;
    if (srarr == 0xff) srarr = hi32;

    if (lodev >= dcnt) return errorfln(FLN,Ret0,fln,"lodev %u cnt %u for leg %u hop %u at \ad%u",lodev,dcnt,l,hop1,t);

    nxtlodev = devp[prvfld];
    fare = devp[farefld];

//    infofln2(FLN,0,fln,"leg %u cnt %u at %u hop %u dur %u t \ad%u",l,dcnt,lodev,hop1,dur,t);

    src->curdts[l] = dt;
    src->curdurs[l] = dur;
    src->curts[l] = t;
    if (t == 0) return errorfln(FLN,Ret0,devp[flnfld],"t 0 for lodev %u,%u leg %u",lodev,nxtlodev,l);

    src->curtids[l] = tid;
    src->curfares[l] = fare;
    src->cursdeps[l] = srdep;
    src->cursarrs[l] = srarr;

    if (l == nleg - 1) src->curdt = dt;

    if (t && at < t) warn(Iter,"prvtdep \ad%u after tdep \ad%u",t,at);  // todo
    at = t;

    if (tid == hi32) {
      infocc(vrbena && hop1 < chopcnt,Notty,"hop %u part %u ev %u of %u leg %u t \ad%u dt %u no tid",hop1,part,lodev,dcnt,l,t,dt);
    } else {
      if (hop1 >= chopcnt) return errorfln(FLN,Ret0,fln,"walk link %u with tid %u leg %u pos %u at \aD%u p %p",hop1,tid,l,lodev,t,devp);
      if (tid >= tidcnt) errorfln(FLN,0,fln,"leg %u",l);
      error_ge(tid,tidcnt);
      rtid = tid2rtid[tid];
      vrb0(0,"hop %u-%u part %u ev %u of %u leg %u t \ad%u tid %u rtid %u dt %u",hop1,hop2,part,lodev,dcnt,l,t,tid,rtid,dt);
    }
    lodev = nxtlodev;
  } while (lodev != hi32 && l && l--);

  src->curt = src->curts[0];
  src->curhwin = src->dephwin;
  info(0,"win %u",src->dephwin);

  return dcnt;
}

static ub4 addevs(ub4 callee,search *src,lnet *net, ub4 *legs,ub4 nleg,ub4 nxleg,ub4 costlim,ub4 *pcurcost)
{
  ub4 whopcnt = net->whopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 midur,*hopdur = net->hopdur;
  ub4 leg,bstop = 0,leg0,cnt = 0;
  ub4 hop,nxthop;
  ub4 curcost,whr;

  enter(callee);

  if (src->udeptmax == 0) {
    whr = getdepwin(src,net,legs,nleg);
    if (whr == 0) whr = 24;
    src->deptmax = src->deptmin + whr * 60;
    src->dephwin = whr;
  } else src->deptmax = src->udeptmax;

  if (nxleg == 0) {
    hop = legs[0];
    if (nleg > 1) {
      nxthop = legs[1];
      if (nxthop >= chopcnt) nxthop = hi32;
    } else nxthop = hi32;
    error_ge(hop,whopcnt);
    midur = hopdur[hop];
    if (hop < chopcnt) { cnt = mkdepevs(src,net,hop,midur,costlim); bstop = 1; }
    else cnt = frqevs(src,net,0,hop,nxthop,midur,walkiv_min,costlim);    // walk links
    if (cnt == 0) { leave(callee); return 0; }
    src->totevcnt[0] += cnt;

    curcost = src->costcurs[0];
    *pcurcost = curcost;
    if (nleg == 1) {
      leave(callee);
      src->totevcnt[0] += cnt;
      src->combicnt++;
      return cnt;
    }
    leg0 = 1;
  } else { leg0 = 0; bstop = nxleg - 1; }

  for (leg = leg0; leg < nleg; leg++) {
    hop = legs[leg];
    error_ge(hop,whopcnt);
    midur = hopdur[hop];
    if (hop < chopcnt) bstop++;
  }

  for (leg = leg0; leg < nleg; leg++) {
    hop = legs[leg];
    midur = hopdur[hop];
    if (hop < chopcnt) {
      cnt = nxtevs(src,net,leg + nxleg,bstop,hop,midur,costlim);
    } else { // walk links
      cnt = fwdevs(src,net,leg + nxleg,bstop,hop,hi32,midur,costlim);
    }
    if (cnt == 0) break;
    curcost = src->costcurs[nleg + nxleg - 1];
    *pcurcost = curcost;
    src->totevcnt[leg + nxleg] += cnt;
  }
  leave(callee);
  src->combicnt++;
  if (cnt && src->nleg != nleg + nxleg) return error(Ret0,"nleg %u vs %u",src->nleg,nleg);

  return cnt;
}

// dynamic search for one extra stop
static int srcdyn(gnet *gnet,lnet *net,search *src,ub4 dep,ub4 arr,ub4 stop,int havedist,const char *desc)
{
  struct port *pmid,*ports = net->ports;
  ub4 portcnt = net->portcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 *hopdist = net->hopdist;
  ub4 part = net->part;
  ub4 midstop1,midstop2,mid,depmid,midarr;
  ub4 ofs1,ofs2,stop1,leg1,leg2,nleg1,nleg2,n1,n2,v1,v2;
  ub4 *conofs1,*conofs2;
  ub2 *cnts1,*cnts2;
  block *lstblk1,*lstblk2;
  ub4 *conlst1,*conlst2,*lst1,*lst2,*lst11,*lst22;
  ub4 dtcur,sumdt;
  ub4 fare;
  ub4 curcost,costlim;
  ub4 evcnt;
  ub4 trip[Nxleg];
  struct trip *stp;
  ub4 leg,l,nleg;
  ub4 dist,hdist,dist1,dist2,walkdist1,sumwalkdist1,walkdist2,sumwalkdist2;
  ub4 nethistop = min(net->histop,src->nethistop);
  ub4 nethileg = nethistop + 1;

  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 walklimit = src->walklimit;
  ub4 sumwalklimit = src->sumwalklimit;

  ub4 lodist = src->lodist;
  int havetime = src->trips[0].cnt;
  ub4 tdep0,tnxt;

  costlim = src->locost;

  stp = src->trips;

  stop1 = stop - 1;
  nleg = stop + 1;
  if (nleg > nethileg * 2 || stop1 >= Nstop) return warn(0,"ending %u-stop search on %u-stop precomputed net",stop,nethistop);

  for (midstop1 = 0; midstop1 < min(stop,nethistop); midstop1++) {
    midstop2 = stop1 - midstop1;
    if (midstop2 > nethistop) continue;

    nleg1 = midstop1 + 1;
    nleg2 = midstop2 + 1;
    nleg = nleg1 + nleg2;

    info(0,"dynsrc 1 %u-%u = %u stops costlim %u",nleg1,nleg2,stop,costlim);

    cnts1 = net->concnt[midstop1];
    cnts2 = net->concnt[midstop2];

    if (cnts1 == NULL) return warn(0,"ending %u-stop search on %u-stop precomputed net",stop,nethistop);
    if (cnts2 == NULL) return warn(0,"ending %u-stop search on %u-stop precomputed net",stop,nethistop);

    src->hisrcstop = max(src->hisrcstop,nleg - 1);

    lstblk1 = net->conlst + midstop1;
    lstblk2 = net->conlst + midstop2;

    conlst1 = blkdata(lstblk1,0,ub4);
    conlst2 = blkdata(lstblk2,0,ub4);

    conofs1 = net->conofs[midstop1];
    conofs2 = net->conofs[midstop2];

    for (mid = 0; mid < portcnt; mid++) {
      if (mid == dep || mid == arr) continue;
      depmid = dep * portcnt + mid;
      n1 = cnts1[depmid];
      if (n1 == 0) continue;

      pmid = ports + mid;
      if (pmid->oneroute) continue;

      midarr = mid * portcnt + arr;
      n2 = cnts2[midarr];
      if (n2 == 0) continue;

      if (gettime_usec() > src->querytlim) return havedist | havetime;

//      info(Notty,"mid %u depmid %u midarr %u",mid,n1,n2);

      ofs1 = conofs1[depmid];
      ofs2 = conofs2[midarr];

      lst1 = conlst1 + ofs1 * nleg1;
      lst2 = conlst2 + ofs2 * nleg2;

      for (v1 = 0; v1 < n1; v1++) {
        lst11 = lst1 + v1 * nleg1;

        dist1 = walkdist1 = sumwalkdist1 = 0;
        for (leg1 = 0; leg1 < nleg1; leg1++) {
          leg = lst11[leg1];
          trip[leg1] = leg;
          hdist = hopdist[leg];
          dist1 += hdist;
          if (leg >= chopcnt) {
            walkdist1 += hdist;
            sumwalkdist1 += hdist;
            if (walkdist1 > walklimit) break;
          } else walkdist1 = 0;
        }
        if (walkdist1 > walklimit || sumwalkdist1 > sumwalklimit) continue;

        for (v2 = 0; v2 < n2; v2++) {
          lst22 = lst2 + v2 * nleg2;

          dist2 = dist1;
          walkdist2 = walkdist1;
          sumwalkdist2 = sumwalkdist1;

          for (leg2 = 0; leg2 < nleg2; leg2++) {
            leg = lst22[leg2];
            trip[nleg1 + leg2] = leg;
            hdist = hopdist[leg];
            dist2 += hdist;
            if (leg >= chopcnt) {
              walkdist2 += hdist;
              sumwalkdist2 += hdist;
              if (walkdist2 > walklimit) break;
            } else walkdist2 = 0;
          }
          if (walkdist2 > walklimit || sumwalkdist2 > sumwalklimit) continue;

          dist = dist2;

          if (dist < lodist) { // route-only
            stp = src->trips + 1;
            stp->dist = dist;
            for (l = 0; l < nleg; l++) {
              leg = trip[l];
              stp->trip[l * 2 + 1] = leg;
              stp->trip[l * 2] = part;
              stp->tid[l] = hi32;
              stp->t[l] = 0;
              stp->srdep[l] = hi32;
              stp->srarr[l] = hi32;
            }
            stp->cnt = havedist = 1;
            stp->len = nleg;
            fmtsum(stp,hi32,hi32,dist,0,mid,"d1");
            lodist = src->lodist = dist;
            info(0,"find route-only at dist %u",lodist);
          }

          evcnt = addevs(caller,src,net,trip,nleg,0,costlim,&curcost);
          stp = src->trips;

          if (evcnt == 0 || (curcost >= costlim && havetime)) continue;

          costlim = curcost;

          evcnt = getevs(src,gnet,nleg,0);
          if (evcnt == 0) continue;

          dtcur = src->curdt;
          infocc(evcnt,0,"%u legs %u event\as dtcur %u cost %u",nleg,evcnt,dtcur,curcost);

          for (l = 0; l < nleg; l++) {
            leg = trip[l];
            stp->trip[l * 2 + 1] = leg;
            stp->trip[l * 2] = part;

            stp->t[l] = src->curts[l];
            stp->dur[l] = src->curdurs[l];
            stp->tid[l] = src->curtids[l];
            stp->srdep[l] = src->cursdeps[l];
            stp->srarr[l] = src->cursarrs[l];
            info(0,"  leg %u hop %u rdep %u at \ad%u dt %u evs %u",l,leg,stp->srdep[l],src->curdts[l],src->curts[l],src->dcnts[l]);
          }
          l = nleg - 1;
          sumdt = src->curts[l] - src->curts[0] + src->curdurs[l];
          infocc(sumdt != dtcur,Notty,"sumdt %u dtcur %u",sumdt,dtcur);
          fare = src->curfares[l];
          stp->cnt = havetime = 1;
          stp->len = nleg;
          stp->dt = sumdt;
          stp->dist = dist;
          if (l) {
            l--;  // todo
            src->lot = src->curts[l];
            src->lotid = src->curtids[l];
          }
          if (dist < src->lodist) src->lodist = dist;
          src->locost = curcost;

          tdep0 = src->curts[0];
          evcnt = getevs(src,gnet,nleg,1);
          if (evcnt) tnxt = max(src->curts[0],tdep0) - tdep0;
          else tnxt = hi32;
          fmtsum(stp,sumdt,tnxt,dist,fare,mid,"d1");
        } // each v2
      } // each v1

      if (havetime && stop > 2) {
        timelimit(src,600);
        src->timestop = min(src->timestop,stop);
      }

    } // each mid
  } // each midstop

  if (havetime) {
    src->locsrccnt++;
    info(0,"%s: found %u-stop conn %u-%u in dynsrc1",desc,stop,dep,arr);
    return 1;
  }

  info(0,"no time for %u-stop trip %u-%u on \ad%u-\ad%u",stop,dep,arr,deptmin,deptmax);

  return havedist;
}

// dynamic search for one or more extra stops, using 2 vias
static int srcleg3(gnet *gnet,lnet *net,search *src,ub4 dep,ub4 arr,ub4 nleg1,ub4 nleg2,ub4 nleg3,int havedist,const char *desc)
{
  ub4 portcnt = net->portcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 *hopdist = net->hopdist;
  struct port *pmid1,*pmid2,*ports = net->ports;
  ub4 part = net->part;
  ub4 stop1,stop2,stop3;
  ub4 mid1,mid2,depmid1,mid12,mid2arr;
  ub4 ofs1,ofs2,ofs3,leg1,leg2,leg3,n1,n2,n3,v1,v2,v3;
  ub4 *conofs1,*conofs2,*conofs3;
  ub2 *cnts1,*cnts2,*cnts3;
  block *lstblk1,*lstblk2,*lstblk3;
  ub4 *conlst1,*conlst2,*conlst3,*lst1,*lst2,*lst3,*lst11,*lst22,*lst33;
  ub4 sumdt;
  ub4 evcnt;
  ub4 trip[Nxleg];
  struct trip *stp;
  ub4 leg,l,nleg;
  ub4 dist,hdist,dist1,dist2,dist3,walkdist1,sumwalkdist1,walkdist2,sumwalkdist2,walkdist3,sumwalkdist3;
  ub4 nethistop = min(net->histop,src->nethistop);

  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 walklimit = src->walklimit;
  ub4 sumwalklimit = src->sumwalklimit;
  ub4 tdep0,tnxt;

  ub4 lodist = src->lodist;
  ub4 curcost,costlim = src->locost;
  int havetime = src->trips[0].cnt;
  ub4 fare = 0;
  ub4 altcnt;

  error_z(nleg1,dep);
  error_z(nleg2,dep);
  error_z(nleg3,dep);

  stop1 = nleg1 - 1; stop2 = nleg2 - 1; stop3 = nleg3 - 1;
  if (stop1 > nethistop || stop2 > nethistop || stop3 > nethistop) {
    return info(Notty,"legs %u,%u,%u not in precomputed %u",nleg1,nleg2,nleg3,nethistop);
  }

  if (stop1 >= Nstop || stop2 >= Nstop || stop3 >= Nstop) return 0;

  stp = src->trips;

  nleg = nleg1 + nleg2 + nleg3;
  if (nleg1 + nleg2 + nleg3 >= Nxleg) return 0;
  if (nleg >= Nxleg) return 0;

  cnts1 = net->concnt[stop1];
  cnts2 = net->concnt[stop2];
  cnts3 = net->concnt[stop3];

  if (cnts1 == NULL || cnts2 == NULL || cnts3 == NULL) return 0;

  src->hisrcstop = max(src->hisrcstop,nleg - 1);

  lstblk1 = net->conlst + stop1;
  lstblk2 = net->conlst + stop2;
  lstblk3 = net->conlst + stop3;

  conlst1 = blkdata(lstblk1,0,ub4);
  conlst2 = blkdata(lstblk2,0,ub4);
  conlst3 = blkdata(lstblk3,0,ub4);

  conofs1 = net->conofs[stop1];
  conofs2 = net->conofs[stop2];
  conofs3 = net->conofs[stop3];

  for (mid1 = 0; mid1 < portcnt; mid1++) {
    if (mid1 == dep || mid1 == arr) continue;
    depmid1 = dep * portcnt + mid1;
    n1 = cnts1[depmid1];
    if (n1 == 0) continue;

    pmid1 = ports + mid1;
    if (pmid1->oneroute) continue;

    if (gettime_usec() > src->querytlim) return havetime | havedist;

//    info(Notty,"mid1 %u cnt %u lodist %u",mid1,n1,lodist);

    for (mid2 = 0; mid2 < portcnt; mid2++) {
      if (mid2 == dep || mid2 == mid1 || mid2 == arr) continue;
      mid12 = mid1 * portcnt + mid2;
      n2 = cnts2[mid12];
      if (n2 == 0) continue;

      mid2arr = mid2 * portcnt + arr;
      n3 = cnts3[mid2arr];
      if (n3 == 0) continue;

      pmid2 = ports + mid2;
      if (pmid2->oneroute) continue;

//      info(Notty,"mid %u-%u cnt %u %u %u",mid1,mid2,n1,n2,n3);

      ofs1 = conofs1[depmid1];
      ofs2 = conofs2[mid12];
      ofs3 = conofs3[mid2arr];

      lst1 = conlst1 + ofs1 * nleg1;
      lst2 = conlst2 + ofs2 * nleg2;
      lst3 = conlst3 + ofs3 * nleg3;

      altcnt = 0;

      for (v1 = 0; v1 < n1; v1++) {
        if (altcnt > altlimit) break;
        lst11 = lst1 + v1 * nleg1;

        if (gettime_usec() > src->querytlim) return havetime | havedist;

        dist1 = walkdist1 = sumwalkdist1 = 0;
        for (leg1 = 0; leg1 < nleg1; leg1++) {
          leg = lst11[leg1];
          trip[leg1] = leg;
          hdist = hopdist[leg];
          dist1 += hdist;
          if (leg >= chopcnt) {
            walkdist1 += hdist;
            sumwalkdist1 += hdist;
            if (walkdist1 > walklimit) break;
          } else walkdist1 = 0;
        }
        if (walkdist1 > walklimit || sumwalkdist1 > sumwalklimit) continue;

        if (lodist != hi32 && dist1 > 10 * lodist) continue;

        for (v2 = 0; v2 < n2; v2++) {
          if (altcnt > altlimit) break;

          lst22 = lst2 + v2 * nleg2;

          dist2 = dist1;
          walkdist2 = walkdist1;
          sumwalkdist2 = sumwalkdist1;
          for (leg2 = 0; leg2 < nleg2; leg2++) {
            leg = lst22[leg2];
            trip[nleg1 + leg2] = leg;
            hdist = hopdist[leg];
            dist2 += hdist;
            if (leg >= chopcnt) {
              walkdist2 += hdist;
              sumwalkdist2 += hdist;
              if (walkdist2 > walklimit) break;
            } else walkdist2 = 0;
          }
          if (walkdist2 > walklimit || sumwalkdist2 > sumwalklimit) continue;

          if (lodist != hi32 && dist1 + dist2 > 10 * lodist) continue;

          for (v3 = 0; v3 < n3; v3++) {
            if (altcnt++ > altlimit) break;

            lst33 = lst3 + v3 * nleg3;

            dist3 = dist2;
            walkdist3 = walkdist2;
            sumwalkdist3 = sumwalkdist2;
            for (leg3 = 0; leg3 < nleg3; leg3++) {
              leg = lst33[leg3];
              trip[nleg1 + nleg2 + leg3] = leg;
              hdist = hopdist[leg];
              dist3 += hdist;
              if (leg >= chopcnt) {
                walkdist3 += hdist;
                sumwalkdist3 += hdist;
                if (walkdist3 > walklimit) break;
              } else walkdist3 = 0;
            }
            if (walkdist3 > walklimit || sumwalkdist3 > sumwalklimit) continue;

            if (lodist != hi32 && dist1 + dist2 > 10 * lodist) continue;

            dist = dist3;
            if (dist < lodist) { // route-only
              stp = src->trips + 1;
              stp->dist = dist;
              for (l = 0; l < nleg; l++) {
                leg = trip[l];
                stp->trip[l * 2 + 1] = leg;
                stp->trip[l * 2] = part;
                stp->t[l] = 0;
                stp->tid[l] = hi32;
                stp->srdep[l] = hi32;
                stp->srarr[l] = hi32;
              }
              stp->cnt = havedist = 1;
              stp->len = nleg;
              fmtsum(stp,hi32,hi32,dist,fare,0,"d2");
              info(0,"find route-only at dist %u",dist);
              lodist = src->lodist = dist;
            }

            evcnt = addevs(caller,src,net,trip,nleg,0,costlim,&curcost);
            if (evcnt == 0 || (curcost >= costlim && havetime)) continue;
            infocc(evcnt,0,"%u legs %u event\as dt %u curcost %u costlim %u",nleg,evcnt,src->curdt,curcost,costlim);

            stp = src->trips;

            evcnt = getevs(src,gnet,nleg,0);
            if (evcnt == 0) continue;

            costlim = curcost;

            for (l = 0; l < nleg; l++) {
              leg = trip[l];
              stp->trip[l * 2 + 1] = leg;
              stp->trip[l * 2] = part;

              stp->t[l] = src->curts[l];
              stp->dur[l] = src->curdurs[l];
              stp->tid[l] = src->curtids[l];
              stp->srdep[l] = src->cursdeps[l];
              stp->srarr[l] = src->cursarrs[l];
              info(0,"  leg %u hop %u dep \ad%u dt %u evs %u",l,leg,src->curts[l],src->curdts[l],src->dcnts[l]);
            }
            l = nleg - 1;
            sumdt = src->curts[l] - src->curts[0] + src->curdurs[l];
            stp->cnt = havetime = 1;
            stp->len = nleg;
            stp->dt = sumdt;
            stp->dist = dist;
            if (l) {
              l--; // todo
//              src->lodt = lodt;
              src->lot = src->curts[l];
              src->lotid = src->curtids[l];
            }
            if (dist < src->lodist) src->lodist = dist;
            src->locost = curcost;

            if (nleg > 3) {
              timelimit(src,1000);
              src->timestop = min(src->timestop,nleg - 1);
            }

            tdep0 = src->curts[0];
            evcnt = getevs(src,gnet,nleg,1);
            if (evcnt) tnxt = max(src->curts[0],tdep0) - tdep0;
            else tnxt = hi32;
            fmtsum(stp,sumdt,tnxt,dist,fare,curcost,"d2");
            info(0,"found %u-stop trip %s",nleg-1,stp->desc);
          } // each v3
        } // each v2
      } // each v1
      infocc(altcnt > altlimit / 2,Notty,"%u of max %u alts",altcnt,altlimit)
    } // each mid2
  } // each mid1

  if (havetime) {
    src->locsrccnt++;
    return 1;
  }

  info(0,"no time for %u-stop trip %u-%u on \ad%u-\ad%u %s",nleg-1,dep,arr,deptmin,deptmax,desc);
  src->lodist = lodist;

  return havedist;
}

// dynamic search for one or more extra stops, using 2 vias
static int srcdyn2(gnet *gnet,lnet *net,search *src,ub4 dep,ub4 arr,ub4 stop,int havedist,const char *desc)
{
  ub4 nleg1,nleg2,nleg3;
  ub4 mid1step,mid2step,mid3step;
  ub4 nleg = stop + 1;
  ub4 nethistop = min(net->histop,src->nethistop);
  ub4 netleg,nethileg = nethistop + 1;
  int rv = 0;

  if (stop < 2) return info(0,"skip dyn search for %u-stop",stop);

  info(0,"dynamic search 2 in %u-stops: precomputed to %u",stop,net->histop);

  netleg = nethileg * 3;
  if (nleg > netleg) return info(0,"ending %u-stop search on %u-stop precomputed net",stop,netleg - 1);

  for (mid1step = 0; mid1step <= nethistop; mid1step++) {
    for (mid2step = 0; mid2step <= nethistop; mid2step++) {
      for (mid3step = 0; mid3step <= nethistop; mid3step++) {
        nleg1 = mid1step + 1; nleg2 = mid2step + 1; nleg3 = mid3step + 1;
        nleg = nleg1 + nleg2 + nleg3;
        if (nleg != stop + 1) continue;

        info(0,"searching %u+%u+%u legs costlim %u",nleg1,nleg2,nleg3,src->locost);
        rv |= srcleg3(gnet,net,src,dep,arr,nleg1,nleg2,nleg3,havedist,desc);
        info(0,"now %lu t0 %lu lim %lu dif %lu",gettime_usec() / 1000,src->queryt0 / 1000,src->querytlim / 1000,(gettime_usec() - src->queryt0) / 1000);
        if (gettime_usec() > src->querytlim) {
          info(0,"timeout at %lu msec",(gettime_usec() - src->queryt0) / 1000);
          return rv;
        }
      }
    }
  }
  return rv;
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
  ub4 nethistop = min(net->histop,src->nethistop);
  ub4 nethileg = nethistop + 1;
  ub4 deptmin,deptmax;
  ub4 evcnt;
  ub4 hdist,dist = 0,leg,l;
  ub4 dtcur,sumdt;
  ub4 *vp;
  ub4 curcost,costlim = src->locost;
  ub4 *hopdist;
  ub4 v0;
  ub4 portcnt = net->portcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 whopcnt = net->whopcnt;
  struct trip *stp;
  ub4 ln = callee & 0xffff;
  int rv,havetime = 0,havedist = 0;
  ub4 fare = 0;
  ub4 tdep0,tnxt;
  ub4 walklimit = src->walklimit;
  ub4 sumwalklimit = src->sumwalklimit;
  ub4 walkdist,sumwalkdist;

  lodist = src->lodist;

  if (stop > nethistop) {
    info(Notty,"%s: net part %u has %u-stop connections, request %u",desc,part,src->nethistop,stop);
    info(0,"nleg %u nethileg %u nethistop %u",nleg,nethileg,nethistop);
    if (stop > src->histop) return info(Notty,"%s: stop limit %u reached",desc,src->histop);
    else if (nleg > nethileg * 2) {
      if (src->timestop < 2) timelimit(src,300);
      return srcdyn2(gnet,net,src,dep,arr,stop,havedist,desc);
    } else {
      rv = srcdyn(gnet,net,src,dep,arr,stop,havedist,desc);
      if (rv && stop > 3) timelimit(src,300);
      else if (rv && stop > 2) timelimit(src,600);
      rv |= srcdyn2(gnet,net,src,dep,arr,stop,havedist,desc);
      return rv;
    }
  } else {
    if (src->timestop < 2) timelimit(src,300);
  }

  if (stop >= Nstop) return 0;

  src->hisrcstop = max(src->hisrcstop,stop);

  deptmin = src->deptmin;
  deptmax = src->deptmax;

  info(0,"search in precomputed %u-stop net",stop);

  cnts = net->concnt[stop];
  ofss = net->conofs[stop];
  lstblk = net->conlst + stop;
  lodists = net->lodist[stop];
  hopdist = net->hopdist;

  ub4 da = dep * portcnt + arr;

  cnt = cnts[da];

  if (cnt) {
    ofs = ofss[da];
    lst = blkdata(lstblk,0,ub4);
    bound(lstblk,ofs * nleg,ub4);
    error_ge(ofs,net->lstlen[stop]);
    vp = lst + ofs * nleg;
  } else {
    vp = NULL;
    src->locnocnt++;
    vrb0(0,"no %u-stop connection %u-%u",stop,dep,arr);
  }

  for (v0 = 0; v0 < cnt; v0++) {

    // distance-only
    dist = walkdist = sumwalkdist = 0;
    for (leg = 0; leg < nleg; leg++) {
      l = vp[leg];
      error_ge(l,whopcnt);
      hdist = hopdist[l];
      dist += hdist;
      if (l >= chopcnt) {
        walkdist += hdist;
        sumwalkdist += hdist;
        if (walkdist > walklimit || sumwalkdist > sumwalklimit) break;
      } else walkdist = 0;
    }
    if (walkdist > walklimit || sumwalkdist > sumwalklimit) { vp += nleg; continue; }

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
        stp->tid[leg] = hi32;
        stp->srdep[leg] = hi32;
        stp->srarr[leg] = hi32;
      }
      stp->cnt = havedist = 1;
      stp->len = nleg;
      stp->dist = dist;
      fmtsum(stp,hi32,hi32,dist,fare,0,"s");
      if (dist && lodists && dist == lodists[da]) info(Notty,"%u-stop found lodist %u at var %u %s:%u",stop,dist,v0,desc,ln);
    }

    // time
    dtcur = hi32;
    evcnt = addevs(caller,src,net,vp,nleg,0,costlim,&curcost);
    infocc(evcnt,Notty,"%u event\as curcost %u costlim %u",evcnt,curcost,costlim);

    stp = src->trips;
    if (evcnt == 0 || (costlim < curcost && stp->cnt)) {
      vp += nleg;
      src->locvarcnt++;
      continue;
    }

    costlim = curcost;

    evcnt = getevs(src,gnet,nleg,0);
    if (evcnt == 0) { vp += nleg; continue; }

    for (leg = 0; leg < nleg; leg++) {
      stp->trip[leg * 2 + 1] = vp[leg];
      stp->trip[leg * 2] = part;

      error_z(src->curts[leg],leg);
      stp->t[leg] = src->curts[leg];
      stp->dur[leg] = src->curdurs[leg];
      stp->tid[leg] = src->curtids[leg];
      stp->srdep[leg] = src->cursdeps[leg];
      stp->srarr[leg] = src->cursarrs[leg];
      info(0,"  leg %u %u,%u dep \ad%u dt %u evs %u",leg,src->cursdeps[leg],src->cursarrs[leg],src->curts[leg],src->curdts[leg],src->dcnts[leg]);
    }
    leg = nleg - 1;
    sumdt = src->curts[leg] - src->curts[0] + src->curdurs[leg];
    stp->cnt = havetime = 1;
    stp->len = nleg;
    stp->dt = sumdt;
    stp->dist = dist;
    if (leg) {
      leg--;
      src->lot = src->curts[leg];
      src->lotid = src->curtids[leg];
    }
    if (dist < src->lodist) src->lodist = dist;
    src->locost = curcost;

    tdep0 = src->curts[0];
    evcnt = getevs(src,gnet,nleg,1);
    if (evcnt) tnxt = max(src->curts[0],tdep0) - tdep0;
    else tnxt = hi32;
    fmtsum(stp,sumdt,tnxt,dist,fare,0,"s");

    vp += nleg;
    src->locvarcnt++;
  } // each v0

  if (havetime) {
    src->locsrccnt++;
    src->timestop = min(src->timestop,stop);
    info(0,"%s: found %u-stop conn %u-%u",desc,stop,dep,arr);
    if (stop == 0) timelimit(src,500);
    return 1;
  }

  info(0,"no time for %u-stop trip %u-%u on \ad%u-\ad%u",stop,dep,arr,deptmin,deptmax);

  if (havedist == 0) info(0,"no route for %u-stop trip %u-%u",stop,dep,arr);

  if (stop < nethistop || stop == src->histop) return havedist;
  if (gettime_usec() > src->querytlim) return havedist;

  // if no time, search for new via. if no route, search with one added via

  if (havedist == 0) stop++;
  if (stop == 0) return 0;

  rv = srcdyn(gnet,net,src,dep,arr,stop,havedist,desc);
  return rv;
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
  ub4 dt,dtcur,dthi,dtndx,dtndx2,tdep,tdur,tid,sumdt;
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
    info(Iter|Notty,"no conn for top %u-%u",tdmid,tamid);
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

    distrange = max(lodist,1) * 10;

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
          info(Iter|Notty,"ofs %u stop %u part %u",tofs,tstop,tpart);
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
          dist = max(dist,lodist + tlodist);
          distiv = (dist - lodist - tlodist) * Distbins / distrange;
          if (totvarcnt > 5 && distiv >= Distbins) { tvp += ntleg; continue; }

          if (distiv < Distbins && varcnt > 50) {
            pct = distsums[distiv] * Percbins / varcnt;
            if (pct >= Percbins || distsums[distiv] > distlims[pct]) { tvp += ntleg; continue; }
          }

          dtcur = hi32;

          evcnt = addevs(caller,src,tnet,tvp,ntleg,nleg,dthi,&dtcur);
          infocc(evcnt,Iter|Notty,"%u event\as",evcnt);

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
              infocc(evcnt,Iter|Notty,"%u event\as",evcnt);

              if (evcnt == 0 || dtcur >= topdts[topdt1]) { avp += naleg; continue; }

              triplen = nleg + ntleg + naleg;
              error_ge(triplen,Nxleg);

              dt = dtcur;

              evcnt = getevs(src,gnet,triplen,0);
              if (evcnt == 0) { avp += naleg; continue; }

              dtndx = 0;
              while (dtndx < Topdts && dtcur >= topdts[dtndx]) dtndx++;
              if (dtndx == topdt1) topdts[dtndx] = dtcur;
              else if (dtndx < topdt1) {
                for (dtndx2 = topdt1; dtndx2 > dtndx; dtndx2--) topdts[dtndx2] = topdts[dtndx2-1];
                info(Iter|Notty,"put dt %u at pos %u",dtcur,dtndx);
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
                leg = triplen - 1;
                sumdt = src->curts[leg] - src->curts[0] + src->curdurs[leg];
                stp->len = triplen;
                fmtsum(stp,sumdt,hi32,dist,0,src->curdt,"d1-xp");
                info(0,"store trip \aV%u%p dur %u dep \ad%u",triplen,(void *)stp->trip,src->curdt,src->curt);
                for (leg = 0; leg < triplen; leg++) {
                  error_z(src->curts[leg],leg);
                  tdep = src->curts[leg];
                  tdur = src->curdurs[leg];
                  tid = src->curtids[leg];
                  stp->t[leg] = tdep;
                  stp->dur[leg] = tdur;
                  stp->tid[leg] = tid;
                  stp->srdep[leg] = src->cursdeps[leg];
                  stp->srarr[leg] = src->cursarrs[leg];
                  info(0,"  leg %u dep \ad%u arr \ad%u tid %u",leg,tdep,tdep + tdur,tid);
                }
                stp->cnt = 1;
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

  if (varcnt) info(Notty,"%u of %u depvars %u of %u midvars %u of %u arrvars %u total",dvarxcnt,dvarcnt,tvarxcnt,tvarcnt,avarxcnt,avarcnt,varcnt);

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
  ub4 dt,dtcur,dthi,dtndx,dtndx2,sumdt;
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

    distrange = max(lodist,1) * 10;

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
              infocc(evcnt,Iter|Notty,"%u event\as",evcnt);

              if (evcnt == 0 || dtcur >= topdts[topdt1]) { avp += naleg; continue; }

              triplen = nleg + ntleg + naleg;
              error_ge(triplen,Nxleg);

              dt = dtcur;

              evcnt = getevs(src,gnet,triplen,0);
              if (evcnt == 0) { avp += naleg; continue; }

              dtndx = 0;
              while (dtndx < Topdts && dtcur >= topdts[dtndx]) dtndx++;
              if (dtndx == topdt1) topdts[dtndx] = dtcur;
              else if (dtndx < topdt1) {
                for (dtndx2 = topdt1; dtndx2 > dtndx; dtndx2--) topdts[dtndx2] = topdts[dtndx2-1];
                info(Iter|Notty,"put dt %u at pos %u",dtcur,dtndx);
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
                leg = triplen - 1;
                sumdt = src->curts[leg] - src->curts[0] + src->curdurs[leg];
                stp->len = triplen;
                fmtsum(stp,sumdt,hi32,dist,0,src->curdt,"d1-xpt");
                info(0,"store trip \aV%u%p dur %u dep \ad%u tid %x",triplen,(void *)stp->trip,src->curdt,src->curt,src->lotid);
                for (leg = 0; leg < triplen; leg++) {
                  error_z(src->curts[leg],leg);
                  stp->t[leg] = src->curts[leg];
                  stp->dur[leg] = src->curdurs[leg];
                  stp->tid[leg] = src->curtids[leg];
                  stp->srdep[leg] = src->cursdeps[leg];
                  stp->srarr[leg] = src->cursarrs[leg];
                  info(0,"  leg %u dep \ad%u",leg,src->curts[leg]);
                }
                stp->cnt = 1;
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

  if (varcnt) info(Notty,"%u of %u depvars %u of %u midvars %u of %u arrvars %u total",dvarxcnt,dvarcnt,tvarxcnt,tvarcnt,avarxcnt,avarcnt,varcnt);

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

  for (iv = 0; iv < niv; iv++) info(Notty,"stats %u %u",iv,stats[iv]);

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

  for (iv = 0; iv < niv; iv++) info(Notty,"stats %u %u",iv,stats[iv]);

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

  for (iv = 0; iv < niv; iv++) info(Notty,"stats %u %u",iv,stats[iv]);

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
  ub4 nethistop;
  lnet *net;

  ub4 conn = 0,allconn = 0;

  nsethi(src->firsthop,Nxleg);
  aclear(src->first);

  if (gportcnt == 0) { error(0,"search without ports, ref %s",ref); return 0; }

  if (partcnt == 0) { error(0,"search called without partitions, ref %s",ref); return 0; }
  else if (partcnt == 1) {
    if (portparts[gdep] == 0) return info(0,"port %u not in partmap",gdep);
    if (portparts[garr] == 0) return info(0,"port %u not in partmap",garr);
    net = getnet(0);
    nethistop = min(net->histop,src->nethistop);
    for (stop = nstoplo; stop <= nstophi; stop++) {
      if (src->timestop == 0 && stop > 1 && src->locost < 60) timelimit(src,300);
      else if (src->timestop == 1 && stop > 2 && src->locost < 120) timelimit(src,300);
      info(Notty,"search %u stops",stop);
      conn = srcglocal(gnet,0,gdep,garr,stop,src);
      if (conn) {
        allconn = conn;
        info(Notty,"found trip at %u stops",stop);
        if (stop > nethistop + 1) timelimit(src,300);
      }
      vrb0(0,"timestop %u",src->timestop);
      if (gettime_usec() > src->querytlim) { info(0,"timeout at %u %lu",src->tlim,(src->querytlim - src->queryt0) / 1000); return allconn; }
    }
    return allconn;
  }

  for (part = 0; part < partcnt; part++) {
    if (portparts[gdep * partcnt + part] == 0 || portparts[garr * partcnt + part] == 0) continue;
    info(Notty,"dep %u and arr %u share part %u",gdep,garr,part);
    for (stop = nstoplo; stop < nstophi; stop++) {
      info(Notty,"search %u stops in part %u",stop,part);
      conn = srcglocal(gnet,part,gdep,garr,stop,src);
      if (conn) {
        allconn = conn;
        info(Notty,"found trip at %u stops in part %u ",stop,part);
        if (stop > 2) return conn;
      }
    }
    if (nstophi < 2) return allconn;
  }

  // no shared parts, or intrapart had no result: search iterpart

  // special cases
  if (portparts[gdep * partcnt + tpart]) {
    info0(0,"search interpart: dep in top");
    conn = srcxdpart(gnet,gdep,garr,src,ref);
  } else if (portparts[garr * partcnt + tpart]) {
    info0(0,"search interpart: arr in top");
    conn = srcxapart(gnet,gdep,garr,src,ref);
  }
  if (conn) return conn;

  // generic case
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

// merge legs if on same route
static int mergelegs(struct trip *tp)
{
  ub4 len = tp->len;
  ub4 dur12,t1,t2,l1,l = 0;
  ub4 rid,tid,hop1,hop2,h1,chop,gchop,dep,arr,gdep,garr;
  ub4 part1,part2;
  gnet *gnet = getgnet();
  lnet *net;
  struct hop *hp;
  struct route *rp;
  int merged = 0;

  if (len < 2) return 0;

  if (globs.nomergeroute) return 0;

  ub4 *ridhops,*ridhopbase = gnet->ridhopbase;

  while (l + 1 < len) {
    l1 = l + 1;
    tid = tp->tid[l];
    if (tid == hi32 || tid != tp->tid[l1]) { l++; continue; }
    part1 = tp->trip[l * 2];
    part2 = tp->trip[l1 * 2];
    hop1 = tp->trip[l * 2 + 1];
    hop2 = tp->trip[l1 * 2 + 1];
    if (part1 != part2) { l++; continue; }

    net = getnet(part1);
    if (hop1 >= net->chopcnt || hop2 >= net->chopcnt) { l++; continue; }
    dep = net->portsbyhop[hop1 * 2];
    arr = net->portsbyhop[hop2 * 2 + 1];
    if (dep >= net->portcnt || arr >= net->portcnt) { l++; continue; }
    gdep = net->p2gport[dep];
    garr = net->p2gport[arr];

    if (hop1 < net->hopcnt) {
      hp = net->hops + hop1;
    } else if (hop1 < net->chopcnt) {
      h1 = net->choporg[2 * hop1];
      hp = net->hops + h1;
    } else { l++; continue; }

    rid = hp->rid;
    rp = net->routes + rid;

    ridhops = ridhopbase + rp->hop2pos;

    // get hop from rid,dep,arr. orgs in case of compound
    ub4 hopndx = 0,h1ndx = 0,h2ndx = 0;
    ub4 h,ghop1 = hi32,ghop2 = hi32;
    while (hopndx < rp->hopcnt && (ghop1 == hi32 || ghop2 == hi32)) {
      h = rp->hops[hopndx];
      if (h >= gnet->chopcnt) break;
      if (gnet->portsbyhop[h * 2] == gdep) { ghop1 = h; h1ndx = hopndx; }
      if (gnet->portsbyhop[h * 2 + 1] == garr) { ghop2 = h;  h2ndx = hopndx; }
      hopndx++;
    }
    if (ghop1 == hi32 || ghop2 == hi32) { l++; continue; }
    else if (ghop1 >= gnet->hopcnt) { l++; continue; }
    else if (ghop2 >= gnet->hopcnt) { l++; continue; }
    if (ghop1 == ghop2) gchop = ghop1;
    else gchop = ridhops[h1ndx * rp->hopcnt + h2ndx];
    if (gchop >= gnet->chopcnt) { l++; continue; }

    chop = net->g2phop[gchop];
    info(0,"merge leg %u+%u on equal tid %u, rid %u hop %u-%u = %u %u",l,l1,tid,rid,ghop1,ghop2,gchop,chop);
    if (chop >= net->chopcnt) { l++; continue; }
    info(0,"merge leg %u+%u on equal tid %u, rid %u hop %u-%u = %u",l,l1,tid,rid,ghop1,ghop2,gchop);

    tp->trip[l * 2 + 1] = chop;
    t1 = tp->t[l]; t2 = tp->t[l1];
    dur12 = t2 - t1 + tp->dur[l1];
    tp->dur[l] = dur12;
    tp->info[l] = 1;
    if (len + 1 > l1) {
      memmove(tp->trip + 2 * l1,tp->trip + 2 * (l + 2),(len - l1 - 1) * 2 * sizeof(tp->trip));
      memmove(tp->t + l + 1,tp->t + l + 2,(len - l1 - 1) * sizeof(tp->t));
      memmove(tp->tid + l + 1,tp->tid + l + 2,(len - l1 - 1) * sizeof(tp->tid));
      memmove(tp->dur + l + 1,tp->dur + l + 2,(len - l1 - 1) * sizeof(tp->dur));
    }
    merged = 1;
    len--;
  }
  tp->len = len;
  return merged;
}

// handle criteria and reporting
int plantrip(search *src,char *ref,ub4 xdep,ub4 xarr,ub4 nstoplo,ub4 nstophi)
{
  ub4 dep,arr,sdep,sarr,srdep,srarr;
  ub4 nleg,leg,t,t2;
  ub4 conn;
  struct gnetwork *net = getgnet();
  ub4 portcnt = net->portcnt;
  ub4 sportcnt = net->sportcnt;
  struct port *parr,*pdep,*ports = net->ports;
  struct sport *psarr,*psdep,*sports = net->sports;
  ub4 deptmin,depttmin,deptmax,tmax;
  ub8 t0,dt,totcnt;
  ub4 utcofs;
  ub4 resmax = sizeof(src->resbuf);
  struct trip *stp,*stp2;
  int same;
  char dtstr[128];

  if (xdep == xarr) return error(0,"departure %u equal to arrival",xarr);

  if (xdep >= portcnt) {
    sdep = xdep - portcnt;
    if (sdep >= sportcnt) return error(0,"departure %u not in %u subportlist",sdep,sportcnt);
    psdep = sports + sdep;
    dep = psdep->parent;
    srdep = psdep->seq;
    info(0,"resolved dep %u into %u.%u",sdep,dep,srdep);
  } else {
    srdep = hi32;
    dep = xdep;
  }

  if (xarr >= portcnt) {
    sarr = xarr - portcnt;
    if (sarr >= sportcnt) return error(0,"arrival %u not in %u subportlist",sarr,sportcnt);
    psarr = sports + sarr;
    arr = psarr->parent;
    srarr = psarr->seq;
    info(0,"resolved arr %u into %u.%u",sarr,arr,srarr);
  } else {
    srarr = hi32;
    arr = xarr;
  }

  pdep = ports + dep;
  parr = ports + arr;

  info(0,"dep %u.%u arr %u.%u %s to %s",dep,srdep,arr,srarr,pdep->name,parr->name);

  inisrc(src,"src",0);
  src->dep = dep;
  src->arr = arr;

  if (dep == arr) {
    stp = src->trips;
    if (gtriptoports(net,dep,arr,srdep,srarr,stp,src->resbuf,resmax,&src->reslen,0)) return 1;
    info(0,"reslen %u",src->reslen);
    return 0;
  }

  if (nstophi >= Nxstop) { warning(0,"limiting max %u-stop to %u", nstophi,Nxstop); nstophi = Nxstop - 1; }
  if (nstoplo > nstophi) { warning(0,"setting min %u-stop to %u", nstoplo,nstophi); nstoplo = nstophi; }

  info(0,"search geo dep %u arr %u between %u and %u stops, ref %s",dep,arr,nstoplo,nstophi,ref);
  info(0,"max walk distance \ag%u, summed \ag%u",src->walklimit,src->sumwalklimit);

  src->lodist = hi32;
  src->timestop = hi32;

  src->geodist = fgeodist(pdep,parr);

  utcofs = utc12ofs(src->utcofs12);

  if (src->deptmin_cd == 0) deptmin = net->t0;
  else deptmin = yymmdd2min(src->deptmin_cd,utcofs);

  depttmin = hhmm2min(src->depttmin_cd);
  deptmin += depttmin;
  deptmin = max(deptmin,net->t0);
  deptmin = min(deptmin,net->t1);

  src->deptmid = deptmin;

  deptmin = max(deptmin - src->minday * 1440,net->t0);
  deptmin = min(deptmin,net->t1);

  if (src->plusday >= 99) deptmax = 0; // auto
  else {
    deptmax = src->deptmid + src->plusday * 1440;
    deptmax = min(deptmax,net->t1);
  }

  src->deptmin = deptmin;
  src->udeptmax = deptmax;

  src->histop = nstophi;

  info(CC,"search dep %u arr %u on \ad%u-\ad%u \au%u %s to %s geodist %u",dep,arr,deptmin,deptmax,utcofs,pdep->name,parr->name,src->geodist);

  t0 = src->queryt0 = gettime_usec();
  src->querytlim = hi64;
  src->tlim = hi32;
  timelimit(src,Timelimit);
  src->locost = hi32;

  conn = dosrc(net,nstoplo,nstophi,src,ref);

  dt = gettime_usec() - t0;

//  info(0,"searched %u local variants in %u local searches %u noloc",src->avarxcnt,src->locsrccnt,src->locnocnt);
  infocc(net->partcnt > 1,0,"%u of %u depvars %u of %u midvars %u of %u arrvars %u stored",src->dvarxcnt,src->dvarcnt,src->tvarxcnt,src->tvarcnt,src->avarxcnt,src->avarcnt,src->varcnt);

  if (dt > 2000000) fmtstring(dtstr,"%u.%u ",(ub4)(dt / 1000000),(ub4)(dt % 1000000) / 10000);
  if (dt > 2000) fmtstring(dtstr,"%u.%u milli",(ub4)(dt / 1000),(ub4)(dt % 1000) / 10);
  else fmtstring(dtstr,"%u micro",(ub4)dt);

  for (leg = 0; leg < Nxleg; leg++) {
    totcnt = src->totevcnt[leg];
    infocc(totcnt,0,"leg %u \ah%lu events",leg,totcnt);
  }

  ub4 duriv = (ub4)min(dt / 1000,Elemcnt(src->querydurs) - 1);
  src->querydurs[duriv]++;
  if (dt > src->querymaxdur) {
    src->querymaxdur = dt;
    src->querymaxdep = dep;
    src->querymaxarr = arr;
  }

  if (conn == 0) {
    src->notrips++;
    src->reslen += mysnprintf(src->resbuf,src->reslen,resmax,"no trip found in %u stops\n\nsearched \ah%lu combinations with \ah%lu departures in %sseconds\n",src->hisrcstop,src->combicnt,src->totevcnt[0],dtstr);
    info(0,"%s",src->resbuf);
    if (src->avarxcnt) return info(0,"no time found for %u stop\as",nstophi);
    else return info(0,"no route found for %u stop\as",nstophi);
  }

  // if trip outside departure window, redo search with expanded dep window
  stp = src->trips;
  nleg = stp->len;
  if (deptmax && stp->cnt && nleg > 1) {
    tmax = stp->t[nleg-1];
    if (tmax > deptmax) {
      info(0,"redo search on last dep \ad%u above departure limit \ad%u",tmax,deptmax);
      t0 = src->queryt0 = gettime_usec();
      src->querytlim = t0 + (1000UL * Timelimit) / 2;
      src->deptmax = tmax;
      conn = dosrc(net,nstoplo,nstophi,src,ref);
    }
  }

  nleg = src->lostop + 1;
//  info(0,"found %u-%u in %u/%u legs",dep,arr,src->trips[0].len,nleg);

  // discard shortest route-only if not shorter
  if (src->trips[0].cnt && src->trips[1].cnt && (src->trips[1].dist >= src->trips[0].dist || src->trips[0].dist - src->trips[1].dist < 30)) {
    src->trips[1].cnt = 0;
  }

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
    while (mergelegs(stp)) ;
    if (gtriptoports(net,dep,arr,srdep,srarr,stp,src->resbuf,resmax,&src->reslen,utcofs)) return 1;
  }
  src->reslen += mysnprintf(src->resbuf,src->reslen,resmax,"\nstat\tsearched \ah%lu combination\as with \ah%lu departure\as in %sseconds\n",src->combicnt,src->totevcnt[1],dtstr);
  info(0,"%s",src->resbuf);
  return 0;
}
