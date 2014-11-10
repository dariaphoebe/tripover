// netbase.c - base network with primary data

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Functions to setup or create a base network */

#include <string.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "time.h"
#include "util.h"
#include "bitfields.h"
#include "netbase.h"
#include "netio.h"

// longest schedule period supported
static const ub4 maxdays = 365 * 2;

static const ub4 daymin = 60 * 24;

// holds everything primary. in contrast, net.h contains derived info
static netbase basenet;

netbase *getnetbase(void) { return &basenet; }

void ininetbase(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

static int addport(struct portbase *ports,ub4 newport,ub4 lat,ub4 lon,double rlat,double rlon,ub4 size,enum txkind kind)
{
  struct portbase *pp;
  ub4 port;

  for (port = 0; port < newport; port++) {
    pp = ports + port;
    if (pp->lat == lat && pp->lon == lon) {
      info(0,"port %u is colocated with %u at latlon %u %u",port,newport,lat,lon);
      return 1;
    }
  }
  if (lat == 0 && lon == 0) return error(0,"port %u latlon 0",newport);
  if (lat >= 180 * Latscale) warning(0,"port %u lat %u out of range",newport,lat);
  if (lon >= 360 * Lonscale) warning(0,"port %u lon %u out of range",newport,lon);

  pp = ports + newport;
  pp->lat = lat;
  pp->lon = lon;
  pp->rlat = rlat;
  pp->rlon = rlon;
  if (kind == Air) pp->air = 1;
  else if (kind == Rail) pp->rail = 1;
  pp->id = port;
  pp->size = size;
  memcpy(pp->name,"noname",6);
  pp->namelen = 6;
  return 0;
}

/* generate artificial network for testing purposes
 for realism, ports are randomly placed on a fractal landscape 
 ports get a random size, used to bias the #connections
 no clustering yet
*/
#define Zmap (1024+1)
#define Zhist (256+2)

static ub4 heightmap[Zmap * Zmap];

static ub4 getz(ub8 lat,ub8 lon)
{
  ub8 y = lat * Zmap / (180UL * Latscale);
  ub8 x = lon * Zmap / (360UL * Lonscale);

  error_ge(y,Zmap);
  error_ge(x,Zmap);
  return heightmap[y * Zmap + x];
}

int mkrandnet(ub4 portcnt,ub4 hopcnt)
{
  struct portbase *ports,*pdep,*parr;
  struct hopbase *hops,*hp;
  struct range zrange;
  ub1 *net0;
  ub4 hist[Zhist];
  ub4 iv,cnt,waterlvl,z,lat,lon,railcnt,iter,aircnt,aimed,landrange;
  ub4 lolon,lolat,hilon,hilat;
  double rlat,rlon;
  ub4 dep,arr,curhop,depcnt;
  ub4 depstats[16];

  if (portcnt == 0 || hopcnt == 0) return 1;

  info(0,"generate artificial %u port %u hop network",portcnt,hopcnt);

  ports = alloc(portcnt,struct portbase,0,"baseports",portcnt);
  hops = alloc(hopcnt,struct hopbase,0,"basehops",hopcnt);
  net0 = alloc(portcnt * portcnt, ub1,0,"net0",portcnt);

  // fractal land to create net on
  mkheightmap(heightmap,Zmap);
  mkhist(heightmap,Zmap,&zrange,Zhist,hist,"randnet",Vrb);

  lolat = lolon = hi32;
  hilat = hilon = 0;

  iv = 0; cnt = 0;
  while (iv < Zhist && cnt < Zmap * 70 / 100) {
    cnt += hist[iv++];
  }
  if (iv) iv--;
  waterlvl = zrange.lo + zrange.hilo * iv / (Zhist - 2);
  info(0,"70%% = %u before bin %u", waterlvl, iv);
  landrange = zrange.hi - waterlvl;

  aimed = portcnt / 4;

  iter = railcnt = 0;
  while (railcnt < aimed && iter++ < (1 << 20)) {
    rlat = (frnd(16000) - 9000) * 0.01 * M_PI / 180;
    rlon = (frnd(36000) - 18000) * 0.01 * M_PI / 180;
    lat = rad2lat(rlat);
    lon = rad2lon(rlon);
    lolat = min(lolat,lat);
    hilat = max(hilat,lat);
    lolon = min(lolon,lon);
    hilon = max(hilon,lon);
    z = getz(lat,lon);
    if (z < waterlvl) continue;
    if (z - waterlvl < rnd(landrange)) {
      vrb(0,"port %u: lat %u lon %u rlat %e rlon %e",railcnt,lat,lon,rlat,rlon);
      if (addport(ports,railcnt,lat,lon,rlat,rlon,rnd(100),Rail)) continue;
      railcnt++;
    }
  }
  error_ge(iter,1<<20);

  iter = aircnt = 0;
  while (railcnt + aircnt < portcnt && iter++ < (1 << 20)) {
    rlat = (frnd(16000) - 9000) * 0.01 * M_PI / 180;
    rlon = (frnd(36000) - 18000) * 0.01 * M_PI / 180;
    lat = rad2lat(rlat);
    lon = rad2lon(rlon);
    lolat = min(lolat,lat);
    hilat = max(hilat,lat);
    lolon = min(lolon,lon);
    hilon = max(hilon,lon);
    z = getz(lat,lon);
    if (z < waterlvl) continue;
    if (z - waterlvl < rnd(landrange)) {
      vrb(0,"port %u: lat %u lon %u rlat %e rlon %e",railcnt+aircnt,lat,lon,rlat,rlon);
      if (addport(ports,railcnt + aircnt,lat,lon,rlat,rlon,rnd(100),Air)) continue;
      aircnt++;
    }
  }
  error_ge(iter,1<<20);
  basenet.ports = ports;

  curhop = 0;
  iter = 0;
  while (curhop < hopcnt && iter++ < (1 << 20)) {
    dep = rnd(portcnt);
    arr = rnd(portcnt);
    if (dep == arr) continue;

    error_ge(dep,portcnt);
    error_ge(arr,portcnt);

    if (rnd(100) > ports[dep].size) continue;

    net0[dep * portcnt + arr] = 1;

    hp = hops + curhop;
    hp->dep = dep;
    hp->arr = arr;
    hp->id = curhop;
    memcopy(hp->name,"noname",6);
    hp->namelen = 6;
    curhop++;
    pdep = ports + dep;
    parr = ports + arr;
  }

  aclear(depstats);
  ub4 depivs = Elemcnt(depstats) - 1;
  for (dep = 0; dep < portcnt; dep++) {
    depcnt = 0;
    for (arr = 0; arr < portcnt; arr++) {
      depcnt += net0[dep * portcnt + arr];
    }
    depstats[min(depivs,depcnt)]++;
  }
  for (iv = 0; iv <= depivs; iv++) vrb(0,"%u ports with %u departures", depstats[iv], iv);

  basenet.hops = hops;
  basenet.portcnt = portcnt;
  basenet.hopcnt = hopcnt;

  basenet.latscale = Latscale;
  basenet.lonscale = Lonscale;

  basenet.latrange[0] = lolat;
  basenet.latrange[1] = hilat;
  basenet.lonrange[0] = lolon;
  basenet.lonrange[1] = hilon;

  prepbasenet();

  if (globs.writext) net2ext(&basenet);
  if (globs.writpdf) net2pdf(&basenet);

  info0(0,"done generating artificial net");
  return 0;
}

/* merge services and time ranges for each route
 input is a list of gtfs-style entries like 'each wed dep 11.33 arr 11.36 valid 20140901-20141231'
 output is
 - N * periodmap : bitmap for each day in the overall timetable validity
 - N * repeatable day as list of <timepoint,next> tuples.
 - list of <timepoint,next> tuples over whole period for items not covered above.

 N is typically 4-6, to cover most common repeatable patterns like weekday/weekend and schedule switches

 actual entry is tripid ( flightno in air) and duration
 times are in minutes UTC since Epoch

 expand for search and merged per dep,arr in netn
 */

// expand gtfs-style entries into a minute-time axis
// returns number of unique departures
static ub4 fillxtime(ub4 *xp,ub4 xlen,ub4 ht0,ub4 ht1,struct sidbase *sp,ub4 tdep,ub4 durtid)
{
  ub4 t,n = 0;
  ub4 dow,t0,t1,tlo = hi32,thi = 0,t0wday,tdow,rdep;

  dow = sp->dow;
  t0 = sp->t0;
  t1 = sp->t1;  // inclusive
  t0wday = sp->t0wday;

  error_lt(t0,ht0);
  error_lt(t1,ht0);
  error_ge(t0,ht1);
  error_ge(t1,ht1);

  error_z(durtid,0);

  rdep = t0 + tdep - ht0;

  if (tdep > daymin * 2) warning(0,"deptime > %u days",tdep / daymin);
  if (tdep > ht1 - ht0) warning(0,"deptime %u above schedule period %u .. %u",tdep,ht0,ht1);

  t = t0 - ht0;

  tdow = (1 << t0wday);
  while (t + ht0 + tdep <= t1) {
    if (dow & tdow) {
      error_ge(t + ht0 + tdep,ht1);
      if (t + tdep >= xlen) {
        warning(0,"tdep %u xlen %u n %u",tdep,xlen,n);
        return n;
      }
      if (xp[t + tdep]) vrb(0,"duplicate at t %u t0 %u t1 %u sid %x tdep %u tid %u",t,t0,t1,sp->sid,tdep,durtid);
      else {
        xp[t + tdep] = durtid;
        tlo = min(tlo,t + tdep);
        thi = max(thi,t + tdep);
        n++;
      }
    }
    t += daymin;
    if (tdow == (1 << 6)) tdow = 1;
    else tdow <<= 1;
  }

  return n;
}

static void showxtime(ub4 *xp,ub4 xlim,ub4 t0,ub4 t1)
{
  ub4 t,x;
  char buf[256];

  t = 0;
  error_ge(t1 - t0,xlim);

  while (t < t1 - t0) {
    x = xp[t];
    if (x == 0) { t++; continue; }

    mintoyymmdd(t + t0,buf,sizeof(buf));
    info(0,"%s tid %x",buf,x);
  }
}

struct timepat {
  ub4 t0,t1;
  ub4 evtcnt;
  ub4 repcnt;
  ub4 repspan;

};

// find day-repeatable patterns. returns number of compressed departure entries
// todo fill struct above for result storage
static ub4 findtrep(ub4 *xp,ub8 *xp2,ub4 xlim,ub4 t0,ub4 t1,ub4 evcnt)
{
  ub4 t,x,d,prvt,ofs,rep,hirep = 0,evcnt2 = 0,zevcnt = 0;
  ub4 rt,hit,tt,dt,tlo = hi32,thi = 0;
  ub8 sum,sum1,sum2,hisum;

  t = 0;
  prvt = t;
  d = 0;
  error_ge(t1 - t0,xlim);

  // pass 1: mark candidates on repeat count and time span
  while (t < t1 - t0) {
    x = xp[t];
    if (x == 0) { t++; continue; }

    tlo = min(tlo,t);
    thi = max(thi,t);

    evcnt2++;
    error_gt(evcnt2,evcnt);

    ofs = t - prvt;
    rep = 0; sum1 = sum2 = 0;
    tt = t / daymin;
    rt = t - (tt * daymin);
    while (rt + 1 < t1 - t0) {
      if (xp[rt] == x || (rt && xp[rt-1] == x) || xp[rt+1] == x) {
        rep++;
        dt = rt / daymin;
        sum1 = (sum1 + ~dt) & hi32;   // fletcher64
        sum2 = (sum2 + sum1) & hi32;
      }
      rt += daymin;
    }
    sum = (sum2 << 32) | sum1;

    xp2[2 * t] = rep;
    xp2[2 * t + 1] = sum;
    if (rep > hirep) { hirep = rep; hit = t; }

    prvt = t;

    t++;
  }

  vrb(0,"hirep %u for %u events range %u tlo %u thi %u lim %u",hirep,evcnt,t1 - t0,tlo,thi,t1 - t0);
  error_ne(evcnt2,evcnt);

  if (hirep < 4) { info(0,"hirep %u",hirep); return 0; }

  // pass 2: collect best repeats
  ub4 hispan = 0,span = 0,spant = 0,spanrep = 0,hit0 = 0,hit1 = 0;
  ub4 hi2span = 0,hi3span = 0,hi4span = 0;
  ub4 hi = 0,hi2 = 0,hi3 = 0,hi4 = 0;
  ub8 prvsum = 0;
  hisum = 0;
  ub8 hi2sum = 0,hi3sum = 0,hi4sum = 0;

  for (t = 0; t < t1 - t0; t++) {
    rep = (ub4)xp2[t * 2];
    if (rep == 0) continue;

    sum = xp2[2 * t + 1];
    if (sum == prvsum && t - spant < daymin) {
      spanrep = min(spanrep,rep);
      span++;
    } else {
      if (span * spanrep > hi) {
        hi = span * spanrep;
        hispan = span;
        hirep = spanrep;
        hisum = prvsum;
        hit0 = spant;
        hit1 = prvt;
      }
      if (span * spanrep > hi2 && prvsum != hisum) {
        hi2 = span * spanrep;
        hi2span = span;
        hi2sum = prvsum;
      }
      if (span * spanrep > hi3 && prvsum != hisum && prvsum != hi2sum) {
        hi3 = span * spanrep;
        hi3span = span;
        hi3sum = prvsum;
      }
      if (span * spanrep > hi4 && prvsum != hisum && prvsum != hi2sum && prvsum != hi3sum) {
        hi4 = span * spanrep;
        hi4span = span;
        hi4sum = prvsum;
      }

      spanrep = rep;
      span = 1;
      spant = t;
    }
    prvsum = sum;
    prvt = t;
  }
  error_gt(hi + hi2 + hi3 + hi4,evcnt);
  zevcnt = evcnt - hi - hi2 - hi3 - hi4 + hispan + hi2span + hi3span + hi4span;
  vrb(0,"hi %u hi2 %u hi3 %u span %u rep %u left %u of %u",hi,hi2,hi3,hispan,hirep,evcnt - hi - hi2 - hi3,evcnt);

  // todo store resuls

  return zevcnt;
}

// create <departure,next) list from expanded time
static ub4 mkdtime(ub4 *dt,ub4 *xp,struct sidbase *sp,ub4 dtlim)
{
  ub4 t,x,d,prvt;
  ub4 t0,t1;

  t0 = sp->t0;
  t1 = sp->t1;

  t = t0;
  prvt = t;
  d = 0;
  while (t < t1 && d + 2 < dtlim) {
    x = xp[t];
    if (x == 0) { t++; continue; }

    dt[d] = t - prvt;
    dt[d+1] = x;
    d += 2;
    prvt = t;
    t++;
  }

  return d;
}

int prepbasenet(void)
{
  struct portbase *ports;
  struct hopbase *hops,*hp;
  struct sidbase *sids,*sp;
  ub4 portcnt,hopcnt,sidcnt,dep,arr;
  ub4 hop;

  hops = basenet.hops;
  hopcnt = basenet.hopcnt;
  portcnt = basenet.portcnt;
  ports = basenet.ports;
  sids = basenet.sids;
  sidcnt = basenet.sidcnt;

  ub4 *tbp,*timesbase = basenet.timesbase;
  ub4 timescnt = basenet.timescnt;
  ub4 tndx,timecnt,timespos,evcnt,zevcnt;
  ub4 sid,tid,tdep,tarr;
  ub4 t0,t1,ht0,ht1;
  ub8 cumevcnt = 0,cumzevcnt = 0;

  // workspace to expand single time 
  ub4 xtimelen = maxdays * daymin;
  ub4 *xp = alloc(xtimelen,ub4,0,"xtime",maxdays);
  ub8 *xp2 = alloc(xtimelen * 2,ub8,0,"xtime2",maxdays);

  ub4 *ep,*events;

  info(0,"preparing %u base hops",hopcnt);

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    dep = hp->dep;
    arr = hp->arr;
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    hp->valid = 1;
    timespos = hp->timespos;
    timecnt = hp->timecnt;
    error_gt(timespos + timecnt,timescnt);
    bound(&basenet.timesmem,(timespos + timecnt) * 4,ub4);
    tbp = timesbase + timespos * 4;
    evcnt = 0;
    memset(xp,0,xtimelen * sizeof(ub4));
    memset(xp2,0,xtimelen * 2 * sizeof(ub8));
    ht0 = hi32; ht1 = 0;
    for (tndx = 0; tndx < timecnt; tndx++) {
      sid = tbp[0];
      tid = tbp[1];
      tdep = tbp[2];
      tarr = tbp[3];
      error_ge(sid,sidcnt);
      sp = sids + sid;

      t0 = sp->t0;
      t1 = sp->t1;
      ht0 = min(ht0,t0);
      ht1 = max(ht1,t1);

      if (hop == 244) vrb(0,"hop %u tndx %u rsid %x tid %x dow %x dep %u t0 %u t1 %u days %u",hop,tndx,sp->rsid,tid,sp->dow,tdep,t0,t1,(t1 - t0) / 1440);
      evcnt += fillxtime(xp,xtimelen,hp->t0,hp->t1,sp,tdep,tid);
      tbp += 4;
    }
    if (timecnt == 0) continue;
    error_ne(hp->t0,ht0);
    ht1 += daymin;  // make end date exclusive
    error_ne(hp->t1,ht1);

    zevcnt = findtrep(xp,xp2,xtimelen,ht0,ht1,evcnt);

    hp->evcnt = evcnt;
    vrb(0,"hop %u \ah%u time events %s",hop,evcnt,hp->name);
    cumevcnt += evcnt;
    cumzevcnt += zevcnt;
  }
  info(0,"\ah%lu org time events to \ah%lu",cumevcnt,cumzevcnt);

  events = ep = basenet.events = mkblock(&basenet.eventmem,cumzevcnt,ub4,Noinit,"time events");

  info(0,"done preparing %u base hops",hopcnt);

  // next step todo

  return 0;
}
