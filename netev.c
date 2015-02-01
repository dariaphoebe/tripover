// netev.c - time events for prepared net

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <stddef.h>
//#include <stdarg.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
//#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "time.h"
//#include "util.h"

//#include "netbase.h"
//#include "netio.h"
//#include "event.h"
#include "net.h"
#include "netev.h"

static const ub4 daymin = 60 * 24;   // convenience

static const ub4 sampleshh = 16;
static const ub4 samplescc = 16;

void ininetev(void)
{

  msgfile = setmsgfile(__FILE__);
  iniassert();
}

// get an average total duration between two plain hops
// take a handful of samples.
static ub4 avgdurhh(net *net,ub4 hop1,ub4 hop2)
{
  struct hop *hp1,*hp2,*hops = net->hops;
  struct timepat *tp1,*tp2;
  ub4 ttmax = 120,ttmin = 5; // todo
  ub4 gndx,gndx2;
  ub4 dt,dur1,dur2,dtsum,avgdt;
  ub8 *events = net->events;

  hp1 = hops + hop1;
  hp2 = hops + hop2;
  tp1 = &hp1->tp;
  tp2 = &hp2->tp;

  ub4 cnt1 = tp1->genevcnt;
  ub4 cnt2 = tp2->genevcnt;
  if (cnt1 == 0 || cnt2 == 0) return hi32;

  ub8 *ev1 = events + tp1->evofs;
  ub8 *ev2 = events + tp2->evofs;

  ub4 gt01 = tp1->gt0;
  ub4 gt02 = tp2->gt0;

  ub4 rt1,rt2,t1,t2;

  ub4 dtcnt = 0;

  gndx2 = dtsum = 0;
  ub4 lodt = hi32;

  rt2 = (ub4)ev2[0]; t2 = rt2 + gt02;
  for (gndx = 0; gndx < cnt1; gndx += max(1,cnt1 / sampleshh) ) {
    rt1 = (ub4)ev1[gndx * 2];
    dur1 = ev1[gndx * 2 + 1] >> 32;
    t1 = rt1 + gt01;
    while (t2 < t1 + dur1 + ttmin) {
      if (++gndx2 >= cnt2) break;
      rt2 = (ub4)ev2[gndx2 * 2];
      t2 = rt2 + gt02;
    }
    if (gndx2 >= cnt2) break;
    if (t2 - (t1 + dur1) > ttmax) continue;
    dtcnt++;
    dur2 = ev2[gndx2 * 2 + 1] >> 32;
    dt = t2 + dur2 - t1;
    lodt = min(lodt,dt);
    dtsum += dt;
  }
  if (dtsum == 0 || lodt == hi32) return 10000;
  avgdt = dtsum / max(dtcnt,1);
  return avgdt;
}

// get an average total duration between two compound hops
// take a handful of samples.
static ub4 avgdurcc(net *net,ub4 hop1,ub4 hop2)
{
  struct hop *hp1,*hp2,*hops = net->hops;
  struct timepat *tp1,*tp2;
  ub4 *choporg = net->choporg;
  ub4 *hopdur = net->hopdur;
  ub4 ttmax = 120,ttmin = 5; // todo
  ub4 gndx,gndx2;
  ub4 h11,h12,h21,h22;
  ub4 dt,dur1,dur2,dtsum,avgdt;
  ub8 *events = net->events;

  h11 = choporg[hop1 * 2];
  h12 = choporg[hop1 * 2 + 1];
  h21 = choporg[hop2 * 2];
  h22 = choporg[hop2 * 2 + 1];

  hp1 = hops + h11;
  hp2 = hops + h22;
  tp1 = &hp1->tp;
  tp2 = &hp2->tp;

  ub4 cnt1 = tp1->genevcnt;
  ub4 cnt2 = tp2->genevcnt;
  if (cnt1 == 0 || cnt2 == 0) return hi32;

  ub8 *ev1 = events + tp1->evofs;
  ub8 *ev2 = events + tp2->evofs;

  ub4 gt01 = tp1->gt0;
  ub4 gt02 = tp2->gt0;

  ub4 rt1,rt2,t1,t2;

  ub4 dtcnt = 0;

  gndx2 = dtsum = 0;
  ub4 lodt = hi32;

  rt2 = (ub4)ev2[0]; t2 = rt2 + gt02;
  dur1 = hopdur[hop1];
  dur2 = hopdur[hop2];
  if (dur1 == hi32 || dur2 == hi32) return 1000; // todo hopadur[]
  for (gndx = 0; gndx < cnt1; gndx += max(1,cnt1 / samplescc) ) {
    rt1 = (ub4)ev1[gndx * 2];
    t1 = rt1 + gt01;
    while (t2 < t1 + dur1 + ttmin) {
      if (++gndx2 >= cnt2) break;
      rt2 = (ub4)ev2[gndx2 * 2];
      t2 = rt2 + gt02;
    }
    if (gndx2 >= cnt2) break;
    if (t2 - (t1 + dur1) > ttmax) continue;
    dtcnt++;
    dt = t2 + dur2 - t1;
    lodt = min(lodt,dt);
    dtsum += dt;
  }
  if (dtsum == 0 || lodt == hi32) return 10000;
  avgdt = dtsum / max(dtcnt,1);
  return avgdt;
}

// get an average total duration between compound and plain hop
// take a handful of samples.
static ub4 avgdurch(net *net,ub4 hop1,ub4 hop2)
{
  struct hop *hp1,*hp2,*hops = net->hops;
  struct timepat *tp1,*tp2;
  ub4 *choporg = net->choporg;
  ub4 *hopdur = net->hopdur;
  ub4 ttmax = 120,ttmin = 5; // todo
  ub4 gndx,gndx2;
  ub4 h11,h12,h21,h22;
  ub4 dt,dur1,dur2,dtsum,avgdt;
  ub8 *events = net->events;

  h11 = choporg[hop1 * 2];
  h12 = choporg[hop1 * 2 + 1];

  hp1 = hops + h11;
  hp2 = hops + hop2;
  tp1 = &hp1->tp;
  tp2 = &hp2->tp;

  ub4 cnt1 = tp1->genevcnt;
  ub4 cnt2 = tp2->genevcnt;
  if (cnt1 == 0 || cnt2 == 0) return hi32;

  ub8 *ev1 = events + tp1->evofs;
  ub8 *ev2 = events + tp2->evofs;

  ub4 gt01 = tp1->gt0;
  ub4 gt02 = tp2->gt0;

  ub4 rt1,rt2,t1,t2;

  ub4 dtcnt = 0;

  gndx2 = dtsum = 0;
  ub4 lodt = hi32;

  rt2 = (ub4)ev2[0]; t2 = rt2 + gt02;
  dur1 = hopdur[hop1];
  if (dur1 == hi32) return 1000; // todo hopadur[]
  for (gndx = 0; gndx < cnt1; gndx += max(1,cnt1 / samplescc) ) {
    rt1 = (ub4)ev1[gndx * 2];
    t1 = rt1 + gt01;
    while (t2 < t1 + dur1 + ttmin) {
      if (++gndx2 >= cnt2) break;
      rt2 = (ub4)ev2[gndx2 * 2];
      t2 = rt2 + gt02;
    }
    if (gndx2 >= cnt2) break;
    if (t2 - (t1 + dur1) > ttmax) continue;
    dtcnt++;
    dur2 = ev2[gndx2 * 2 + 1] >> 32;
    dt = t2 + dur2 - t1;
    lodt = min(lodt,dt);
    dtsum += dt;
  }
  if (dtsum == 0 || lodt == hi32) return 10000;
  avgdt = dtsum / max(dtcnt,1);
  return avgdt;
}

// get an average total duration between plain and compound hops
// take a handful of samples.
static ub4 avgdurhc(net *net,ub4 hop1,ub4 hop2)
{
  struct hop *hp1,*hp2,*hops = net->hops;
  struct timepat *tp1,*tp2;
  ub4 *choporg = net->choporg;
  ub4 *hopdur = net->hopdur;
  ub4 ttmax = 120,ttmin = 5; // todo
  ub4 gndx,gndx2;
  ub4 h11,h12,h21,h22;
  ub4 dt,dur1,dur2,dtsum,avgdt;
  ub8 *events = net->events;

  h21 = choporg[hop2 * 2];
  h22 = choporg[hop2 * 2 + 1];

  hp1 = hops + hop1;
  hp2 = hops + h22;
  tp1 = &hp1->tp;
  tp2 = &hp2->tp;

  ub4 cnt1 = tp1->genevcnt;
  ub4 cnt2 = tp2->genevcnt;
  if (cnt1 == 0 || cnt2 == 0) return hi32;

  ub8 *ev1 = events + tp1->evofs;
  ub8 *ev2 = events + tp2->evofs;

  ub4 gt01 = tp1->gt0;
  ub4 gt02 = tp2->gt0;

  ub4 rt1,rt2,t1,t2;

  ub4 dtcnt = 0;

  gndx2 = dtsum = 0;
  ub4 lodt = hi32;

  rt2 = (ub4)ev2[0]; t2 = rt2 + gt02;
  dur2 = hopdur[hop2];
  if (dur2 == hi32) return 1000; // todo hopadur[]
  for (gndx = 0; gndx < cnt1; gndx += max(1,cnt1 / samplescc) ) {
    rt1 = (ub4)ev1[gndx * 2];
    dur1 = ev1[gndx * 2 + 1] >> 32;
    t1 = rt1 + gt01;
    while (t2 < t1 + dur1 + ttmin) {
      if (++gndx2 >= cnt2) break;
      rt2 = (ub4)ev2[gndx2 * 2];
      t2 = rt2 + gt02;
    }
    if (gndx2 >= cnt2) break;
    if (t2 - (t1 + dur1) > ttmax) continue;
    dtcnt++;
    dt = t2 + dur2 - t1;
    lodt = min(lodt,dt);
    dtsum += dt;
  }
  if (dtsum == 0 || lodt == hi32) return 10000;
  avgdt = dtsum / max(dtcnt,1);
  return avgdt;
}

// first step of estimated average total trip duration
// stores arrival times of last hop
ub4 prepestdur(net *net,ub4 *trip,ub4 len,ub4 *estdurs,ub4 estdurcnt)
{
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 whopcnt = net->whopcnt;
  ub4 *hopdur = net->hopdur;
  ub4 o,avgdur,midur;
  ub4 h;
  struct hop *hp,*hops = net->hops;

  if (len == 1) {  // basic net1 case
    h = *trip;
    if (h < hopcnt) {
      hp = hops + h;
      avgdur = hp->tp.avgdur;
      return avgdur;
    } else if (h < chopcnt) { // compounds to be refined
      midur = hopdur[h];
      return midur;
    } else { // walks
      midur = hopdur[h];
      return midur;
    }
  }
  // rest todo
  return 1000;
}

// estimate average total trip duration, using arrival times of preceding triplet
ub4 estdur(net *net,ub4 *estdurs,ub4 estdurcnt,ub4 *trip1,ub4 len1,ub4 *trip2,ub4 len2)
{
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 whopcnt = net->whopcnt;
  ub4 *hopdur = net->hopdur;
  ub4 *choporg = net->choporg;
  ub4 o,avgdur,midur;
  ub4 h1,h2,ch1;
  struct hop *hp1,*hp2,*hops = net->hops;
  struct timepat *tp;

  if (len1 == 1 && len2 == 1) { // basic net1 case
    h1 = *trip1; h2 = *trip2;
    if (h1 < hopcnt && h2 < hopcnt) {  // plain-plain
      midur = avgdurhh(net,h1,h2);
      return midur;
    } else if (h1 < hopcnt && h2 < chopcnt) { // plain-cmp
      midur = avgdurhc(net,h1,h2);
      return midur;
    } else if (h1 < chopcnt && h2 < hopcnt) { // cmp-plain
      midur = avgdurch(net,h1,h2);
      return midur;
    } else if (h1 < chopcnt && h2 < chopcnt) { // cmp-cmp
      midur = avgdurcc(net,h1,h2);
      return midur;
    } else if (h1 < chopcnt) { // x-walk
      if (h1 < hopcnt) tp = &hops[h1].tp;
      else { ch1 = choporg[h1 * 2]; tp = &hops[ch1].tp; }
      midur = tp->avgdur + hopdur[h2] + 5;
      return midur;
    } else if (h2 < chopcnt) { // walks
      if (h2 < hopcnt) tp = &hops[h2].tp;
      else { ch1 = choporg[h2 * 2]; tp = &hops[ch1].tp; }
      midur = tp->avgdur + hopdur[h1] + 5;
      return midur;
    } else { // walk-walk
      midur = hopdur[h1] + hopdur[h2] + 5;
      return midur;
    }
  }

// todo
  return 1000;
}
