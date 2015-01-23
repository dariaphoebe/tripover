// netbase.c - base network with primary data

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Functions to setup or create a base network */

#include <string.h>
#include <stdarg.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "time.h"
#include "util.h"
#include "netbase.h"
#include "netio.h"
#include "event.h"

// static const ub4 maxdays = 365 * 2;  // longest schedule period supported

static const ub4 daymin = 60 * 24;   // convenience

// holds everything primary. in contrast, net.h contains derived info
static netbase basenet;

static const ub4 hop2watch = hi32; // tmp debug provision
static const ub4 tid2watch = hi32;
static const ub4 rrid2watch = hi32;

netbase *getnetbase(void) { return &basenet; }

#include "watch.h"

static int vrbena;

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
  mkhist(caller,heightmap,Zmap,&zrange,Zhist,hist,"randnet",Vrb);

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
 - N * repeatable day as list of <time,trip> tuples.
 - list of <time,trip> tuples over whole period for items not covered above.

 N is typically 4-6, to cover most common repeatable patterns like weekday/weekend and schedule switches

 actual entry is tripid ( flightno in air) and duration
 times are in minutes UTC since Epoch

 expand for search and merged per dep,arr in netn
 */

static const ub4 maxev4hop = 180 * 24 * 60;   // each minute for half year
static const ub4 maxzev = 500 * 1024 * 1024;  // todo arbitrary

int prepbasenet(void)
{
  struct portbase *ports,*pdep,*parr,*pp;
  struct hopbase *hops,*hp;
  struct sidbase *sids,*sp;
  ub4 portcnt,hopcnt,sidcnt,dep,arr;
  ub4 hop;
  char *dname,*aname;

  hops = basenet.hops;
  hopcnt = basenet.hopcnt;
  portcnt = basenet.portcnt;
  ports = basenet.ports;
  sids = basenet.sids;
  sidcnt = basenet.sidcnt;

  ub4 gt0 = basenet.t0;
  ub4 gt1 = basenet.t1;
  ub4 gdt = gt1 - gt0;

  ub4 *tbp,*timesbase = basenet.timesbase;
  ub4 tndx,timecnt,timespos,evcnt,zevcnt,cnt;
  ub4 sid,rsid,tid,rtid,rid,rrid;
  ub4 tdep,tarr,tripseq,tdepsec,tarrsec,prvtdep;
  ub4 chcnt,i;
  ub4 t0,t1,ht0,ht1,hdt,tdays,mapofs;
  ub8 cumevcnt = 0,cumzevcnt = 0,cumtdays = 0;
  ub4 evhops = 0;
  ub4 dur,lodur,hidur,midur,prvdur,duracc,eqdur = 0,accdur = 0;
  ub4 sumtimes = 0;

  info(0,"preparing %u base hops",hopcnt);

  rsid2logcnt = getwatchitems("rsid",rsids2log,Elemcnt(rsids2log));
  hop2logcnt = getwatchitems("hop",hops2log,Elemcnt(hops2log));
  rsidlog(hi32,"%c",0);

  // workspace to expand single time 
  ub4 xtimelen = gdt + 3 * daymin;
  ub8 *xp = alloc(xtimelen,ub8,0xff,"time",gdt);
  ub8 *xp2 = alloc(xtimelen * 2,ub8,0,"time",gdt);
  ub1 *xpacc = alloc((xtimelen >> 4) + 2,ub1,0,"time",gdt);
  block *eventmem,*evmapmem;

  ub8 *events;
  struct timepatbase *tp;
  struct routebase *rp,*routes = basenet.routes;
  ub1 *daymap,*sidmaps = basenet.sidmaps;

  struct eta eta;

  ub4 cumhoprefs = basenet.chainhopcnt;
  ub4 cumhoprefs2 = 0;
  struct chainbase *cp,*chains = basenet.chains;

  ub8 *chip,*chainidxs = alloc(cumhoprefs,ub8,0,"chain idx",cumhoprefs);
  struct chainhopbase *chp,*chp2,*chainhops = alloc(cumhoprefs,struct chainhopbase,0,"chain hops",cumhoprefs);

  ub4 chain;
  ub4 rawchaincnt = basenet.rawchaincnt;
  ub4 hirrid = basenet.hirrid;

  ub4 ridcnt = basenet.ridcnt;

  double fdist;
  ub4 dist;

  ub4 ofs,chainofs = 0;
  for (chain = 0; chain < rawchaincnt; chain++) {
    cp = chains + chain;
    cp->hopofs = chainofs;
    chainofs += cp->hoprefs;
  }
  error_ne(chainofs,cumhoprefs);

  // pass 1: expand time entries, determine memuse and assign chains
  for (hop = 0; hop < hopcnt; hop++) {

    if (progress(&eta,"hop %u of %u in pass 1, \ah%lu events",hop,hopcnt,cumevcnt)) return 1;

    hp = hops + hop;
    if (hp->valid == 0) continue;

    msgprefix(0,"hop %u",hop);

    dep = hp->dep;
    arr = hp->arr;
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    error_eq(dep,arr);
    pdep = ports + dep;
    parr = ports + arr;

    dname = pdep->name;
    aname = parr->name;

    if (pdep->lat == parr->lat && pdep->lon == parr->lon) {
      info(Iter,"ports %u-%u coloc %u,%u %s to %s",dep,arr,pdep->lat,pdep->lon,dname,aname);
      dist = 0;
    } else {
      fdist = geodist(pdep->rlat,pdep->rlon,parr->rlat,parr->rlon);
      if (fdist < 1e-10) warning(Iter,"port %u-%u distance ~0 for latlon %u,%u-%u,%u %s to %s",dep,arr,pdep->lat,pdep->lon,parr->lat,parr->lon,dname,aname);
      else if (fdist < 0.001) warning(Iter,"port %u-%u distance %e for latlon %u,%u-%u,%u %s to %s",dep,arr,fdist,pdep->lat,pdep->lon,parr->lat,parr->lon,dname,aname);
      else if (fdist > 1.0e+8) warning(Iter,"port %u-%u distance %e for latlon %u,%u-%u,%u %s to %s",dep,arr,fdist,pdep->lat,pdep->lon,parr->lat,parr->lon,dname,aname);
      dist = (ub4)fdist;
    }
    hp->dist = dist;

    rrid = hp->rrid;
    rid = hp->rid;

    if (rrid == rrid2watch || rrid == 15) info(0,"hop %u rrid %u rid %u %s to %s route %s",hop,rrid,rid,pdep->name,parr->name,hp->name);

    hoplog(hop,0,"rrid %x %u-%u %s to %s",rrid,dep,arr,pdep->name,parr->name);

    // times
    timespos = hp->timespos;
    timecnt = hp->timecnt;
//    info(0,"hop %u timepos %u cnt %u",hop,timespos,timecnt);
    if (timecnt) bound(&basenet.timesmem,(timespos + timecnt - 1) * 5,ub4);
    tbp = timesbase + timespos * 5;
    evcnt = 0;
    error_le(hp->t1,hp->t0);
    hdt = hp->t1 - gt0 + 1 * daymin;
    error_ge(hdt,xtimelen);
    memset(xp,0xff,hdt * sizeof(*xp));
    memset(xp2,0,hdt * 2 * sizeof(*xp2));
    memset(xpacc,0,(hdt >> 4) + 1);
    ht0 = hi32; ht1 = 0;
    tp = &hp->tp;
    tp->hop = hop;
    tp->t0 = tp->t1 = 0;
    tp->gt0 = gt0;

    if (timecnt == 0) continue;

    tp->t0 = hi32;
    lodur = hi32; hidur = 0;
    for (tndx = 0; tndx < timecnt; tndx++) {
      sid = tbp[0];
      tid = tbp[1];
      tdepsec = tbp[2];
      tarrsec = tbp[3];
      tripseq = tbp[4];

      tdep = tdepsec / 60;
      tarr = tarrsec / 60;

      error_lt(tarr,tdep);
      dur = tarr - tdep;
      lodur = min(lodur,dur);
      hidur = max(hidur,dur);
      error_ge(sid,sidcnt);
      sp = sids + sid;
      rsid = sp->rsid;

      t0 = sp->t0;
      t1 = sp->t1;

      error_le(t1,t0);

      ht0 = min(ht0,t0);
      ht1 = max(ht1,t1);
      tp->utcofs = sp->utcofs;
      mapofs = sp->mapofs;
      daymap = sidmaps + mapofs;

      hoplog(hop,0,"tndx %u rsid %x tid %x dep %u t0 %u t1 %u days %u",tndx,sp->rsid,tid,tdep,t0,t1,(t1 - t0) / 1440);

//      rsidlog(rsid,"hop %u tndx %u tid %x dep %u t0 %u t1 %u days %u",hop,tndx,tid,tdep,t0,t1,(t1 - t0) / 1440);

      cnt = fillxtime(tp,xp,xpacc,xtimelen,gt0,sp,daymap,tdep,tid);
      hoplog(hop,0,"tid %u rsid %x \ad%u \ad%u td \ad%u ta \ad%u %u events",tid,sp->rsid,t0,t1,tdep,tarr,cnt);
      if (cnt == 0) {
        vrbcc(vrbena,0,"tid %u rsid %x \ad%u \ad%u td \ad%u ta \ad%u no events",tid,sp->rsid,t0,t1,tdep,tarr);
//        return 1;
        tbp[0] = sidcnt; // disable for next pass
        tbp += 5;
        continue;
      } // else info(0,"hop %u tid %u td \ad%u ta \ad%u %u events",hop,tid,tdep,tarr,cnt);

      if (rawchaincnt == 0) { info(0,"hop %u no chains",hop); continue; }

      // create chains: list of hops per trip, sort on tripseq later
      if (tid != hi32) {
        error_ge(tid,rawchaincnt);
        cp = chains + tid;
        rtid = cp->rtid;
        chip = chainidxs + cp->hopofs;
        chcnt = cp->hopcnt;
        ofs = cp->hopofs;
        error_ge(ofs + chcnt,cumhoprefs);
        chp = chainhops + ofs + chcnt;
        pp = ports + cp->dep;
        error_z(tripseq,tid);
        infocc(tid == 57021,0,"xxx tid %u seq %u idx %u at %p",tid,tripseq,chcnt,chip);
        error_ge(chcnt,cp->hoprefs);
        if (chcnt == 0) {
          chip[0] = (ub8)tripseq << 32;
          error_z(chip[0],tid);
          chp->hop = hop;
          chp->tdep = tdep;
          chp->tarr = tarr;
          cp->rrid = rrid;
          cp->rid = rid;
          cp->dep = dep;
          cp->hopcnt = 1;
          cumhoprefs2++;
        } else {
          if (cp->rrid != rrid) warning(0,"hop %u tid %x on route %u vs %u",hop,tid,rrid,cp->rrid);
          chp2 = chainhops + ofs;
          for (i = 0; i < chcnt; i++) {
            if (chp2[i].hop == hop) {
              warn(Iter,"rrid %u r.tid %u.%u skip equal hop %u at %u %s to %s start %s %s",rrid,rtid,tid,hop,i,pdep->name,parr->name,pp->name,hp->name);
              break;
            } else if ( (chip[i] >> 32) == tripseq) {
              warn(Iter,"rrid %x tid %u skip equal seq %u at %u %s to %s start %s",rrid,tid,tripseq,i,pdep->name,parr->name,pp->name);
              break;
            }
          }
          if (i == chcnt) {
            if (tid == tid2watch || hop == hop2watch) info(0,"add hop %u tid %u at %u",hop,tid,i);
            chip[chcnt] = chcnt | ((ub8)tripseq << 32);
            chp->hop = hop;
            chp->tdep = tdep;
            chp->tarr = tarr;
            error_ge(chcnt,cp->hoprefs);
            cp->hopcnt = chcnt + 1;
            cumhoprefs2++;
            if (tid == tid2watch) info(0,"rrid %x rid %u tid %u hop %u at %u %s to %s",rrid,rid,tid,hop,i,pdep->name,parr->name);
          }
        }
      } else info(Iter,"hop %u no tid",hop);

      evcnt += cnt;
      if (evcnt > maxev4hop) {
        warning(0,"hop %u exceeds event max %u %s",hop,maxev4hop,hp->name);
        hp->timecnt = tndx;
        break;
      }
      tp->evcnt = evcnt;
      tbp += 5;
      sumtimes++;
    }

    if (evcnt == 0) {
      genmsg(timecnt > 600 ? Info : Vrb,0,"hop %u no events for %u time entries",hop,timecnt);
      tp->t0 = min(tp->t0,tp->t1);
      continue;
    }
    evhops++;
    hoplog(hop,0,"final date range %u-%u",tp->t0,tp->t1);

    lodur = min(lodur,hidur);
    tp->lodur = lodur;
    tp->hidur = hidur;

    switch (hp->kind) {
      case Air:  duracc = 15; break;
      case Rail: if (lodur > 12 * 60) duracc = 10; else duracc = 2; break;
      case Bus:  if (lodur > 60) duracc = 10; else duracc = 5; break;
      case Ferry: duracc = 15; break;
      case Walk: duracc = 15; break;
      case Unknown: duracc = 15; break;
      case Kindcnt: duracc = 15; break;
    }

//    infocc(lodur == 0,0,"hop %u lodur 0 hidur %u",hop,hidur);
    if (lodur == hidur) {
      tp->midur = hidur;
      eqdur++;
    } else if ( hidur - lodur <= duracc) {
      tp->midur = hidur;
      accdur++;
    } else {
      tp->midur = hi32;
    }
    tp->duracc = duracc;

//    else if (hidur > 2 * lodur) info(0,"hop %u lodur %u hidur %u",hop,lodur,hidur);
//    else vrb0(0,"hop %u lodur %u hidur %u",hop,lodur,hidur);

    error_ne(hp->t0,ht0);
    ht1 += daymin;  // make end date exclusive
    error_ne(hp->t1,ht1);

    hp->evcnt = evcnt;

    if (hop == hop2watch) showxtime(tp,xp,xtimelen);

    if (tp->t0 > tp->t1) {
      warning(0,"hop %u t0 %u t1 %u for %u events",hop,tp->t0,tp->t1,evcnt);
      tp->t0 = ht0;
      tp->t1 = ht1;
    } else tp->t1++;
    tdays = (tp->t1 - tp->t0) / daymin + 1;
    tp->tdays = tdays;
    cumtdays += tdays;

    zevcnt = findtrep(tp,xp,xpacc,xp2,xtimelen,evcnt);

    if (cumevcnt + zevcnt > maxzev) {
      warning(0,"hop %u: exceeding total event max %u %s",hop,maxzev,hp->name);
      break;
    }

    hp->zevcnt = zevcnt;
    vrb(0,"hop %u \ah%u to %u time events %s",hop,evcnt,zevcnt,hp->name);
    cumevcnt += evcnt;

    cumzevcnt += zevcnt;
  }
  msgprefix(0,NULL);

  info(0,"\ah%lu org time events to \ah%lu",cumevcnt,cumzevcnt);
  info(0,"\ah%u org chainhops to \ah%u",cumhoprefs,cumhoprefs2);
  info(0,"%u routes",ridcnt);
  infocc(evhops < hopcnt,0,"%u of %u hops without time events",hopcnt - evhops,hopcnt);
  info(0,"%u of %u hops with constant duration, %u within accuracy",eqdur,hopcnt,accdur);

  // prepare hoplist in chain
  ub4 ci,idx,iv,eqcnt = 0,hichlen = 0,lochlen = hi32,hichain = 0,lochain = 0;
  ub4 cumchaincnt = 0,ridchainofs = 0;
  ub8 sum1,sum2,sum;
  ub4 chstats[128];
  ub4 ivhi = Elemcnt(chstats) - 1;

  aclear(chstats);

  for (chain = 0; chain < rawchaincnt; chain++) {
    cp = chains + chain;
    cnt = cp->hopcnt;
    chstats[min(cnt,ivhi)]++;
    rid = cp->rid;
    if (cnt == 0) { info(Iter,"chain %u rtid %u rrid %u has no hops",chain,cp->rtid,cp->rrid); continue; }
    else if (cnt > 2 && rid != hi32) {
      if (cnt > hichlen) { hichlen = cnt; hichain = chain; }
      if (cnt < lochlen) { lochlen = cnt; lochain = chain; }
      infovrb(cnt > 120,0,"chain %u has %u hops",chain,cnt);
      rrid = cp->rrid;
      error_gt(rrid,hirrid,chain);
      rp = routes + rid;
      rp->chaincnt++;
      cumchaincnt++;
      ofs = cp->hopofs;
      chip = chainidxs + ofs;
      sort8(chip,cnt,FLN,"chainhops");

      sum1 = sum2 = 0;
      dist = 0; midur = prvdur = 0;
      for (ci = 0; ci < cnt; ci++) {
        tripseq = chip[ci] >> 32;
        error_z(tripseq,chain);
        idx = chip[ci] & hi32;
        error_ge(idx,cnt);
        chp = chainhops + ofs + idx;
        hop = chp->hop;
        error_ge(hop,hopcnt);
        hp = hops + hop;
        dist += hp->dist;
        if (hp->tp.midur == hi32) { prvdur = midur; midur = hi32; }
        else midur += hp->tp.midur;
        chp->dist = dist;
        chp->midur = midur;
        if (midur == hi32) midur = prvdur;
        sum1 = (sum1 + hop) % hi32;
        sum2 = (sum2 + sum1) % hi32;
      }
      sum = (sum2 << 32) | sum1;
      cp->code = sum;

    } else {
      vrb0(Iter,"skip chain %u with %u hop\as",chain,cnt); cp->hopcnt = 0;
    }
  }

  for (iv = 0; iv <= ivhi; iv++) infocc(chstats[iv],0,"%u chain\as of length %u",chstats[iv],iv);

  // list shortest and longest chain
  lochlen = min(lochlen,hichlen);
  cp = chains + hichain;
  rid = cp->rid; rrid = cp->rrid;
  info(0,"\ah%u chains len %u .. %u",cumchaincnt,lochlen,hichlen);
  info(0,"longest chain %u len %u rid %u rrid %u",hichain,hichlen,rid,rrid);
  cnt = cp->hopcnt;
  chp = chainhops + cp->hopofs;
  chip = chainidxs + cp->hopofs;
  for (ci = 0; ci < cnt; ci++) {
    idx = chip[ci] & hi32;
    tripseq = chip[ci] >> 32;
    hop = chp[idx].hop;
    tdep = chp[idx].tdep;
    hp = hops + hop;
    pdep = ports + hp->dep;
    parr = ports + hp->arr;
    info(0,"  hop %u %u-%u at \ad%u %s to %s",hop,hp->dep,hp->arr,tdep,pdep->name,parr->name);
  }

  cnt = 0;
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    if (hp->rrid != rrid) continue;
    pdep = ports + hp->dep;
    parr = ports + hp->arr;
    vrb(0,"  hop %u %u-%u %s to %s",hop,hp->dep,hp->arr,pdep->name,parr->name);
    cnt++;
  }
  info(0,"%u hops on rid %u rrid %u",cnt,rid,rrid);

  cp = chains + lochain;
  info(0,"shortest chain %u len %u rid %u rrid %u",lochain,lochlen,cp->rid,cp->rrid);
  cnt = cp->hopcnt;
  ofs = cp->hopofs;
  chp = chainhops + ofs;
  chip = chainidxs + ofs;
  for (ci = 0; ci < cnt; ci++) {
    idx = chip[ci] & hi32;
    tripseq = chip[ci] >> 32;
    hop = chp[idx].hop;
    hp = hops + hop;
    pdep = ports + hp->dep;
    parr = ports + hp->arr;
    info(0,"  hop %u %u-%u seq %u %s to %s",hop,hp->dep,hp->arr,tripseq,pdep->name,parr->name);
  }

  // list chains per route
  ub4 *rcp,*routechains = alloc(cumchaincnt,ub4,0,"chain routechains",cumchaincnt);

  for (rid = 0; rid < ridcnt; rid++) {
    rp = routes + rid;
    cnt = rp->chaincnt;
    info(0,"rid %u rrid %u has %u chains",rid,rp->rrid,cnt);
    rp->chainofs = ridchainofs;
    ridchainofs += cnt;
  }
  error_ne(ridchainofs,cumchaincnt);

  ub4 hi2chainlen = 0,hirid = 0;
  for (chain = 0; chain < rawchaincnt; chain++) {
    cp = chains + chain;
    cnt = cp->hopcnt;
    rid = cp->rid;
    if (cnt < 3 || rid == hi32) continue;
    rp = routes + cp->rid;
    rcp = routechains + rp->chainofs + rp->chainpos;
    error_ge(rp->chainofs + rp->chainpos,cumchaincnt);
    rp->hichainlen = max(rp->hichainlen,cnt);
    if (rp->hichainlen > hi2chainlen) { hi2chainlen = rp->hichainlen; hirid = cp->rid; }
    *rcp = chain;
    rp->chainpos++;
  }
  info(0,"longest chain %u for route %u",hi2chainlen,hirid);

  for (rid = 0; rid < ridcnt; rid++) {
    rp = routes + rid;
    cnt = rp->hichainlen;
    vrb(0,"rid %u cnt %u",rid,cnt);
  }

  basenet.hichainlen = hichlen;

  basenet.routechains = routechains;
  basenet.chainhops = chainhops;
  basenet.chainidxs = chainidxs;

  eventmem = &basenet.eventmem;
  evmapmem = &basenet.evmapmem;
  events = basenet.events = mkblock(eventmem,cumzevcnt * 2,ub8,Init0,"time events");

  basenet.evmaps = mkblock(evmapmem,cumtdays * 5,ub2,Init0,"time eventmaps");
  
  ub4 evofs = 0,dayofs = 0;
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    tp = &hp->tp;
    evcnt = hp->zevcnt;
    tp->evofs = evofs;
    evofs += 2 * evcnt;
    tdays = tp->tdays;
    tp->dayofs = dayofs;
    dayofs += tdays * 5;
  }
  error_ne(evofs,cumzevcnt * 2);

  // pass 2: fill from time entries
  info(0,"preparing \ah%lu events in %u base hops pass 2",cumevcnt,hopcnt);

  ub8 cumevcnt2 = 0,cumzevcnt2 = 0;
  for (hop = 0; hop < hopcnt; hop++) {

    if (progress(&eta,"hop %u of %u in pass 2, \ah%lu events",hop,hopcnt,cumevcnt2)) return 1;

    hp = hops + hop;
    if (hp->valid == 0) continue;

    msgprefix(0,"hop %u",hop);

    dep = hp->dep;
    arr = hp->arr;
    timespos = hp->timespos;
    timecnt = hp->timecnt;
    tbp = timesbase + timespos * 5;
    evcnt = 0;
    hdt = hp->t1 - gt0 + daymin;
    tp = &hp->tp;
    tp->evcnt = 0;
    memset(xp,0xff,hdt * sizeof(*xp));
    memset(xpacc,0,(hdt >> 4) + 1);
    ht0 = hi32; ht1 = 0;
    for (tndx = 0; tndx < timecnt; tndx++) {
      sid = tbp[0];
      if (sid >= sidcnt) { tbp += 5; continue; }  // skip non-contributing entries disabled above

      tid = tbp[1];
      tdepsec = tbp[2];
      tarrsec = tbp[3];
      tripseq = tbp[4];

      tdep = tdepsec / 60;
      tarr = tarrsec / 60;

      dur = tarr - tdep;
      sp = sids + sid;

      t0 = sp->t0;
      t1 = sp->t1;
      tp->utcofs = sp->utcofs;
      mapofs = sp->mapofs;
      daymap = sidmaps + mapofs;

      cnt = fillxtime2(tp,xp,xpacc,xtimelen,gt0,sp,daymap,tdep,tid,dur);
      hoplog(hop,0,"tid %u rsid %x \ad%u \ad%u td \ad%u ta \ad%u %u events",tid,sp->rsid,t0,t1,tdep,tarr,cnt);
      error_z(cnt,hop);
      evcnt += cnt;
      if (evcnt > maxev4hop) {
        warning(0,"hop %u exceeds event max %u %s",hop,maxev4hop,hp->name);
        hp->timecnt = tndx;
        break;
      }
      tp->evcnt = evcnt;
      tbp += 5;
    }
    if (timecnt == 0) continue;
    if (evcnt == 0) {
      genmsg(timecnt > 20 ? Info : Vrb,0,"hop %u no events for %u time entries",hop,timecnt);
      continue;
    }
    error_ne(evcnt,hp->evcnt);

    zevcnt = filltrep(eventmem,evmapmem,tp,xp,xpacc,xtimelen);
    hoplog(hop,0,"evtcnt %u zevcnt %u and %u",evcnt,zevcnt,hp->zevcnt);
    noexit error_ne_cc(zevcnt,hp->zevcnt,"hop %u",hop);
    if (zevcnt != hp->zevcnt) warning(Iter,"hop %u zevcnt %u != hp->zevcnt %u",hop,zevcnt,hp->zevcnt);

    if (cumevcnt2 + zevcnt > maxzev) {
      warning(0,"hop %u: exceeding total event max %u %s",hop,maxzev,hp->name);
      break;
    }

    vrb(0,"hop %u \ah%u time events %s",hop,evcnt,hp->name);
    cumevcnt2 += evcnt;
    cumzevcnt2 += zevcnt;
  }
  msgprefix(0,NULL);
  info(0,"\ah%lu org time events to \ah%lu",cumevcnt,cumzevcnt2);
  error_ne(cumevcnt,cumevcnt2);
  error_ne(cumzevcnt,cumzevcnt2);

  info(0,"done preparing %u base hops",hopcnt);

  return 0;
}

void ininetbase(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
  vrbena = (getmsglvl() >= Vrb);
}
