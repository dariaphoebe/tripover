// event.c - time events like depart, arrive

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <stddef.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "time.h"
#include "util.h"
//#include "bitfields.h"
#include "netbase.h"
#include "netio.h"
#include "event.h"

static const ub4 maxdays = 365 * 2;  // longest schedule period supported

static const ub4 daymin = 60 * 24;   // convenience

static const ub4 hop2watch = 0; // debug provision

void inievent(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

/* merge services and time ranges for each route
 input is a list of gtfs-style entries like 'each wed dep 11.33 arr 11.36 valid 20140901-20141231'
 output is
 - N * periodmap : bitmap for each day in the overall timetable validity
 - N * repeatable day as list of <time,trip> tuples.
 - list of <time,trip> tuples over whole period for items not covered above.

 N is typically 4-6, to cover most common repeatable patterns like weekday/weekend and schedule switches

 actual entry is tripid ( flightno in air) and duration
 times are in minutes UTC since Epoch

 expand for search and merged per dep,arr in netn
 */
void showxtime(struct timepatbase *tp,ub4 *xp,ub4 xlim)
{
  ub4 t,x,min,lmin;
  ub4 t0 = tp->t0, t1 = tp->t1;

  t = t0;
  error_ge(t1,xlim);

  while (t < t1) {
    x = xp[t];
    if (x == 0) { t++; continue; }

    min = t + tp->ht0;
    lmin = min2lmin(min,tp->utcofs);
    vrb(0,"hop %u \ad%u \ad%u tid %x",tp->hop,min,lmin,x & 0xffffff);
    t++;
  }
}

// expand gtfs-style entries into a minute-time axis over validity range
// returns number of unique departures
ub4 fillxtime(struct timepatbase *tp,ub4 *xp,ub1 *xpacc,ub4 xlen,ub4 ht0,ub4 ht1,struct sidbase *sp,ub4 tdep,ub4 tid)
{
  ub4 t,n = 0;
  ub4 dow,t0,t1,tt,tlo,thi,t0wday,tdow,rdep;
  ub4 dayid = 0;
  ub4 hop = tp->hop;

  dow = sp->dow;
  t0 = sp->t0;
  t1 = sp->t1;  // inclusive
  t0wday = sp->t0wday;

  tlo = tp->t0; thi = tp->t1;

  error_lt(t0,ht0);
  error_lt(t1,ht0);
  error_ge(t0,ht1);
  error_ge(t1,ht1);

  error_nz(tid >> 24,tid);

  rdep = t0 + tdep - ht0;

  if (hop == hop2watch) info(0,"hop %u deptime %u at %x schedule period \ad%u-\ad%u startday %u",hop,tdep,dow,ht0,ht1,t0wday);

  if (tdep > daymin * 2) warning(0,"deptime > %u days",tdep / daymin);
  else if (tdep >= daymin) {
    tdep -= daymin; ht1 += daymin;
    if (t0wday == 6) t0wday = 0; else t0wday++;
  }
  if (tdep > ht1 - ht0) return warning(0,"hop %u deptime %u above schedule period \ad%u-\ad%u",hop,tdep,ht0,ht1);

  t = t0 - ht0;
  tdow = (1 << t0wday);
  dayid = (t - t0) / daymin;

  while (t + ht0 + tdep <= t1) {
    if (dow & tdow) {
      error_ge(t + ht0 + tdep,ht1);
      tt = t + tdep;
      if (tt >= xlen) {
        warning(0,"tdep %u xlen %u n %u",tdep,xlen,n);
        return n;
      }
      if (xp[tt] == hi32) {
        xp[tt] = tid | (dayid << 24);
        tlo = min(tlo,tt);
        thi = max(thi,tt);
        n++;
        xpacc[tt >> 4] = 1;
      } // else duplicate/overlap
    }
    t += daymin;
    if (tdow == (1 << 6)) tdow = 1;
    else tdow <<= 1;
  }
  if (n) { tp->t0 = tlo; tp->t1 = thi; } // keep track of first and last actual departure
  tp->evcnt = n;
  if (hop == hop2watch) info(0,"%u events tlo %u thi %u",n,tlo,thi);
  return n;
}

// similar to above, second pass after alloc
ub4 fillxtime2(struct timepatbase *tp,ub4 *xp,ub1 *xpacc,ub4 xlen,ub4 ht0,ub4 ht1,struct sidbase *sp,ub4 tdep,ub4 tid)
{
  ub4 t,n = 0;
  ub4 dow,t0,t1,tt,tlo,thi,t0wday,tdow,rdep;
  ub4 dayid = 0;
  ub4 hop = tp->hop;

  dow = sp->dow;
  t0 = sp->t0;
  t1 = sp->t1;
  t0wday = sp->t0wday;

  tlo = tp->t0; thi = tp->t1;

  rdep = t0 + tdep - ht0;

  if (tdep > daymin * 2) warning(0,"deptime > %u days",tdep / daymin);
  else if (tdep >= daymin) {
    tdep -= daymin; ht1 += daymin;
    if (t0wday == 6) t0wday = 0; else t0wday++;
  }
  if (tdep > ht1 - ht0) return warning(0,"hop %u deptime %u above schedule period \ad%u-\ad%u",hop,tdep,ht0,ht1);

  t = t0 - ht0;
  tdow = (1 << t0wday);
  dayid = (t - t0) / daymin;

  while (t + ht0 + tdep <= t1) {
    if (dow & tdow) {
//      error_ge(t + ht0 + tdep,ht1);
      tt = t + tdep;
      if (tt >= xlen) {
        warning(0,"tdep %u xlen %u n %u",tdep,xlen,n);
        return n;
      }
      if (xp[tt] == hi32) {
        xp[tt] = tid | (dayid << 24);
        n++;
        xpacc[tt >> 4] = 1;
      }
    }
    t += daymin;
    if (tdow == (1 << 6)) tdow = 1;
    else tdow <<= 1;
  }
  return n;
}

// find day-repeatable patterns. returns number of compressed departures
// result is 4 times a repeat pattern + leftover
// times are relative to hop origin
ub4 findtrep(struct timepatbase *tp,ub4 *xp,ub1 *xpacc,ub8 *xp2,ub4 xlim,ub4 evcnt)
{
  ub4 hop = tp->hop;
  ub4 t0,t1;
  ub4 t,x,tid,prvt,rep,hirep = 0,evcnt2 = 0,zevcnt = 0;
  ub4 rt,hit,dayid,tlo = hi32,thi = 0;
  ub8 sum,sum1,sum2;

  if (evcnt == 0) return 0;

  t0 = tp->t0; t1 = tp->t1;

  t = t0;
  prvt = t;
  error_ge(t1,xlim);

  // pass 1: mark candidates on repeat count and time span
  while (t < t1) {
    if (xpacc[t >> 4] == 0) { t += 16; continue; }
    x = xp[t];
    if (x == hi32) { t++; continue; }

    tid = x & 0xffffff;
    dayid = x >> 24;       // first day in localtime this dep is valid

    tlo = min(tlo,t);
    thi = max(thi,t);

    evcnt2++;
    error_gt(evcnt2,evcnt);

    rep = 0; sum1 = sum2 = 0;
    rt = t;
    while (rt >= daymin) rt -= daymin;

    while (rt < t1) { // count days with identical dep at exactly 24h time difference, including self
      if ( (xp[rt] & 0xffffff) == tid) { // same trip ID, same 24h time
        rep++;
        sum1 = (sum1 + ~dayid) % hi32;   // fletcher64 holds the entire date list signature
        sum2 = (sum2 + sum1) % hi32;
      }
      dayid++;
      rt += daymin;
    }
    sum = (sum2 << 32) | sum1;

    xp2[2 * t] = rep;
    xp2[2 * t + 1] = sum;
    if (rep > hirep) { vrb(CC,"hirep %u at %u sum %lx",rep,t,sum); hirep = rep; hit = t; }
//    else info(CC,"rep %u at %u sum %lx",rep,t,sum);
    prvt = t;

    t++;
  }

  if (hop == hop2watch) info(0,"hop %u hirep %u for %u events range %u tlo %u thi %u lim %u",hop,hirep,evcnt,t1 - t0,tlo,thi,t1 - t0);
  error_ne(evcnt2,evcnt);

  if (hirep == 0) {
    warning(0,"hop %u hirep 0 for %u event\as",hop,evcnt);
    return evcnt;
  } else if (hirep < 4) { // not worth to compress. todo: criteria
    dayid = (t1 - t0) / daymin;
    genmsg(dayid > 14 ? Info : Vrb,0,"hop %u hirep %u for %u events",hop,hirep,evcnt);
    tp->genevcnt = evcnt;
    return evcnt;
  }

  // pass 2: collect best repeats
  ub4 hix,span = 0,spant = 0,spanrep = 0;
  ub4 hi0t0 = 0,hi0t1 = 0,hi1t0 = 0,hi1t1 = 0,hi2t0 = 0,hi2t1 = 0,hi3t0 = 0,hi3t1 = 0;
  ub4 hi0span = 0,hi1span = 0,hi2span = 0,hi3span = 0;
  ub4 hi0 = 0,hi1 = 0,hi2 = 0,hi3 = 0;
  ub8 prvsum = 0;
  ub8 hi0sum = 0,hi1sum = 0,hi2sum = 0,hi3sum = 0;
  ub4 hi0rep = 0,hi1rep = 0,hi2rep = 0,hi3rep = 0;

  evcnt2 = 0;
  t = t0;
  while (t < t1 && evcnt2 < evcnt) {
    if (xpacc[t >> 4] == 0) { t += 16; continue; }
    if (xp[t] == hi32) { t++; continue; }

    rep = (ub4)xp2[t * 2];   // from above
    sum = xp2[2 * t + 1];

    // identical sum means same day list: look for a long span
    // use a top-4 of such repeats
    if (sum == prvsum && t - spant < daymin) {
      spanrep = min(spanrep,rep);
      span++;
      if (hop == hop2watch) vrb(0,"t %u span %u rep %u sum %lx",t,span,spanrep,sum);
    } else {
      hix = span * spanrep;
      if (hix > hi0 && hix > hi1 && hix > hi2 && hix > hi3) {
        hi0 = hix;
        hi0span = span;
        hi0rep = spanrep;
        hi0sum = prvsum;
        hi0t0 = spant;
        hi0t1 = prvt;
        if (hop == hop2watch) info(0,"hi0 t %u span %u rep %u sum %lx",t,span,spanrep,prvsum);
      } else if (hix > hi1 && hix > hi2 && hix > hi3 && prvsum != hi0sum) {
        hi1 = hix;
        hi1span = span;
        hi1rep = spanrep;
        hi1sum = prvsum;
        hi1t0 = spant;
        hi1t1 = prvt;
        if (hop == hop2watch) info(0,"hi1 t %u span %u rep %u sum %lx",t,span,spanrep,prvsum);
      } else if (hix > hi2 && hix > hi3 && prvsum != hi0sum && prvsum != hi1sum) {
        hi2 = hix;
        hi2span = span;
        hi2rep = spanrep;
        hi2sum = prvsum;
        hi2t0 = spant;
        hi2t1 = prvt;
        if (hop == hop2watch) info(0,"hi2 t %u span %u rep %u sum %lx",t,span,spanrep,prvsum);
      } else if (hix > hi3 && prvsum != hi0sum && prvsum != hi1sum && prvsum != hi2sum) {
        hi3 = hix;
        hi3span = span;
        hi3rep = spanrep;
        hi3sum = prvsum;
        hi3t0 = spant;
        hi3t1 = prvt;
        if (hop == hop2watch) info(0,"hi3 t %u span %u rep %u sum %lx",t,span,spanrep,prvsum);
      }
      spanrep = rep;
      span = 1;
      spant = t;
      evcnt2 = hi0 + hi1 + hi2 + hi3;
    }
    prvsum = sum;
    prvt = t;
    t++;
  }
  genmsg(hop == hop2watch ? Info : Vrb,CC,"hop %u his %u %u %u %u span %u rep %u left %u of %u",hop,hi0,hi1,hi2,hi3,hi0span,hirep,evcnt - hi0 - hi1 - hi2 - hi3,evcnt);
  error_gt(hi0 + hi1 + hi2 + hi3,evcnt);

  zevcnt = evcnt - hi0 - hi1 - hi2 - hi3;   // leftover non-repeating ones
  tp->genevcnt = zevcnt;

  zevcnt += hi0span + hi1span + hi2span + hi3span;  // repeat pattern itself
  genmsg(hop == hop2watch ? Info : Vrb,0,"hi %u,%u,%u,%u span %u rep %u left %u of %u",hi0,hi1,hi2,hi3,hi0span,hirep,evcnt - hi0 - hi1 - hi2 - hi3,evcnt);

  // store for next pass
  tp->hispans[0] = hi0span;
  tp->hispans[1] = hi1span;
  tp->hispans[2] = hi2span;
  tp->hispans[3] = hi3span;

  tp->hireps[0] = hi0rep;
  tp->hireps[1] = hi1rep;
  tp->hireps[2] = hi2rep;
  tp->hireps[3] = hi3rep;

  tp->hisums[0] = hi0sum;
  tp->hisums[1] = hi1sum;
  tp->hisums[2] = hi2sum;
  tp->hisums[3] = hi3sum;

  tp->hit0s[0] = hi0t0;
  tp->hit1s[0] = hi0t1;
  tp->hit0s[1] = hi1t0;
  tp->hit1s[1] = hi1t1;
  tp->hit0s[2] = hi2t0;
  tp->hit1s[2] = hi2t1;
  tp->hit0s[3] = hi3t0;
  tp->hit1s[3] = hi3t1;

  return zevcnt;
}

// comparable to above, fill pass usig info above
ub4 filltrep(block *evmem,block *evmapmem,struct timepatbase *tp,ub4 *xp,ub1 *xpacc,ub4 xlim)
{
  ub4 hop = tp->hop;
  ub4 t0,t1,t,tdays,tdays5,day;
  ub4 x,tid,prvt,rep,zevcnt = 0;
  ub4 rt,dayid;
  ub8 sum,sum1,sum2;

  ub4 hi0t0,hi0t1,hi1t0,hi1t1,hi2t0,hi2t1,hi3t0,hi3t1;
  ub4 hi0span,hi1span,hi2span,hi3span;
  ub8 hi0sum,hi1sum,hi2sum,hi3sum;
  ub4 hi0rep,hi1rep,hi2rep,hi3rep;
  ub4 hi0ndx = 0,hi1ndx = 0,hi2ndx = 0,hi3ndx = 0;
  ub4 *evs;
  ub2 *days;
  ub4 hi0pat,hi1pat,hi2pat,hi3pat,gen,hi0day,hi1day,hi2day,hi3day,genday;
  ub4 gndx = 0;

  evs = blkdata(evmem,tp->evofs,ub4);
  days = blkdata(evmapmem,tp->dayofs,ub2);

  t0 = tp->t0; t1 = tp->t1;
  tdays = tp->tdays;
  tdays5 = tdays * 5;

  error_ge(t1,xlim);

  t = t0;
  prvt = t;

  hi0rep = tp->hireps[0];
  hi1rep = tp->hireps[1];
  hi2rep = tp->hireps[2];
  hi3rep = tp->hireps[3];

  hi0sum = tp->hisums[0];
  hi1sum = tp->hisums[1];
  hi2sum = tp->hisums[2];
  hi3sum = tp->hisums[3];

  hi0span = tp->hispans[0];
  hi1span = tp->hispans[1];
  hi2span = tp->hispans[2];
  hi3span = tp->hispans[3];

  hi0t0 = tp->hit0s[0];
  hi0t1 = tp->hit1s[0];
  hi1t0 = tp->hit0s[1];
  hi1t1 = tp->hit1s[1];
  hi2t0 = tp->hit0s[2];
  hi2t1 = tp->hit1s[2];
  hi3t0 = tp->hit0s[3];
  hi3t1 = tp->hit1s[3];

  // store repeats as-is for the repeating span only
  hi0pat = 0;
  hi1pat = hi0span * 2;
  hi2pat = hi1pat + hi1span * 2;
  hi3pat = hi2pat + hi2span * 2;

  // and a day mask array with count for each day
  hi0day = 0;
  hi1day = tdays;
  hi2day = tdays * 2;
  hi3day = tdays * 3;
  genday = tdays * 4;

  if (hop == hop2watch) info(0,"hop %u span %u,%u,%u,%u",hop,hi0span,hi1span,hi2span,hi3span);

  gen = (hi0span + hi1span + hi2span + hi3span) * 2;
  bound(evmem,gen,ub4);

  if (gen == 0) { // no repetition
    while (t < t1) {
      if (xpacc[t >> 4] == 0) { t += 16; continue; }
      x = xp[t];
      if (x == hi32) { t++; continue; }

      error_ge(gndx,2 * tp->genevcnt);
      bound(evmem,gen + gndx + 2,ub4);
      evs[gen + gndx++] = t;
      evs[gen + gndx++] = x;
      day = (t - t0) / daymin;
      error_ge(genday + day,tdays5);
      days[genday + day]++;
      t++;
    }
    genmsg(gndx > 128 ? Info : Vrb,0,"hop %u fill %u nonrepeating events",hop,gndx / 2);
    return gndx / 2;
  }

  // re-mark candidates on repeat count and time span
  while (t < t1) {
    if (xpacc[t >> 4] == 0) { t += 16; continue; }
    x = xp[t];
    if (x == hi32) { t++; continue; }

    tid = x & 0xffffff;
    dayid = x >> 24;

    rep = 0; sum1 = sum2 = 0;

    rt = t;
    while (rt >= daymin) rt -= daymin;
    while (rt < t1) { // count days with identical dep, including self
      if (rt == t)
        ; // self
      if ( (xp[rt] & 0xffffff) == tid) {
        rep++;
        sum1 = (sum1 + ~dayid) % hi32;
        sum2 = (sum2 + sum1) % hi32;
      }
      dayid++;
      rt += daymin;
    }
    sum = (sum2 << 32) | sum1;

    // either store the repeat pattern, or mark the repeat day
    if (sum == hi0sum) {
      if (t < hi0t0) {
        day = (t - t0) / daymin;
        error_ge_cc(hi0day + day,tdays5,"hop %u day %u t %u hit0 %u",hop,day,t,hi0t0);
        days[hi0day + day]++;
      } else if (t == hi0t0) {
        hi0ndx = 2;
        evs[hi0pat] = t;
        evs[hi0pat] = x;
      } else if (hi0ndx < hi0span * 2) {
        evs[hi0pat + hi0ndx++] = t;
        evs[hi0pat + hi0ndx++] = x;
      } else if (t <= hi0t1) {
        day = (t - hi0t0) / daymin;
        error_ge_cc(hi0day + day,tdays5,"hop %u day %u t %u hit0 %u",hop,day,t,hi0t0);
        days[hi1day + day]++;
      } // else warning(0,"t %u > hi1t1 %u",t,hi1t1);

    } else if (sum == hi1sum) {
      if (t < hi1t0) {
        day = (t - t0) / daymin;
        error_ge_cc(hi1day + day,tdays5,"hop %u day %u t %u hit0 %u",hop,day,t,hi1t0);
        days[hi1day + day]++;
      } else if (t == hi1t0) {
        hi1ndx = 2;
        evs[hi1pat] = t;
        evs[hi1pat] = x;
      } else if (hi1ndx < hi1span * 2) {
        evs[hi1pat + hi1ndx++] = t;
        evs[hi1pat + hi1ndx++] = x;
      } else if (t <= hi1t1) {
        day = (t - hi1t0) / daymin;
        error_ge_cc(hi1day + day,tdays5,"hop %u day %u t %u hi1t0 %u",hop,day,t,hi1t0);
        days[hi1day + day]++;
      }
    } else if (sum == hi2sum) {
      if (t < hi2t0) {
        day = (t - t0) / daymin;
        error_ge_cc(hi2day + day,tdays5,"hop %u day %u t %u hit0 %u",hop,day,t,hi2t0);
        days[hi2day + day]++;
      } else if (t == hi2t0) {
        hi2ndx = 2;
        evs[hi2pat] = t;
        evs[hi2pat] = x;
      } else if (hi2ndx < hi2span * 2) {
        evs[hi2pat + hi2ndx++] = t;
        evs[hi2pat + hi2ndx++] = x;
      } else if (t <= hi2t1) {
        day = (t - hi2t0) / daymin;
        error_ge_cc(hi2day + day,tdays5,"hop %u day %u t %u hi1t0 %u",hop,day,t,hi2t0);
        days[hi2day + day]++;
      }
    } else if (sum == hi3sum) {
      if (t < hi3t0) {
        day = (t - t0) / daymin;
        error_ge_cc(hi3day + day,tdays5,"hop %u day %u t %u hit0 %u",hop,day,t,hi3t0);
        days[hi3day + day]++;
      } else if (t == hi3t0) {
        hi3ndx = 2;
        evs[hi3pat] = t;
        evs[hi3pat] = x;
      } else if (hi3ndx < hi3span * 2) {
        evs[hi3pat + hi3ndx++] = t;
        evs[hi3pat + hi3ndx++] = x;
      } else if (t <= hi3t1) {
        day = (t - hi3t0) / daymin;
        error_ge_cc(hi3day + day,tdays5,"hop %u day %u t %u hi1t0 %u",hop,day,t,hi3t0);
        days[hi3day + day]++;
      }
    } else { // nonrepeating leftovers
      if (hop == hop2watch) vrb(0,"hop %u t %u t0 %u gndx %u sum %lx",hop,t,t0,gndx,sum);
      if (gndx < 2 * tp->genevcnt) {
        error_ge(gndx,2 * tp->genevcnt);
        bound(evmem,gen + gndx + 2,ub4);
        evs[gen + gndx++] = t;
        evs[gen + gndx++] = x;
        day = (t - t0) / daymin;
        error_ge(genday + day,tdays5);
        days[genday + day]++;
      }
    }

    t++;
  }
  zevcnt = (gndx + hi1ndx) / 2;

  return zevcnt;
}
