// search.c - core trip planning

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* perform the actual trip search.
   currently a rudimentary routing-only search.
   time work in progress
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

static void inisrc(search *src,const char *desc,ub4 arg)
{
  ub4 i,leg;
  ub4 *ev;

  fmtstring(src->desc,"%s %u",desc,arg);

  src->costlim = 0;
  src->lodist = hi32;

  src->tripcnt = src->triplen = 0;
  for (i = 0; i < Nxleg; i++) {
    src->trip[i] = hi32;
    src->tripparts[i] = hi16;
    src->tripports[i] = hi32;
  }
  ev = src->evpool = alloc(Nxleg * Maxevs * 3,ub4,0,"src events",Maxevs);
  for (leg = 0; leg < Nxleg; leg++) {
    src->depevs[leg] = ev;
    ev += Maxevs * 3;
  }
}

static ub4 mkdepevs(search *src,ub8 *events,ub2 *evmaps,struct timepat *tp,struct hop *hp)
{
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 gt0 = tp->gt0;
  ub4 ht0 = tp->ht0;
  ub4 ht1 = tp->ht1;
  ub4 t0 = tp->t0;
  ub4 t1 = tp->t1;
  ub4 dcnt = 0,gencnt = tp->genevcnt;
  ub4 leg,gndx,agndx,dmax = Maxevs;
  ub4 t,dur,*dev,*adev;
  ub8 x,*ev,*aev;
  ub2 *map;

//  return 0;

  if (t0 == t1) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,t0,t1,deptmin,deptmax);
//  else if (ht0 > deptmax) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,ht0,ht1,deptmin,deptmax);
//  else if (ht1 <= deptmin) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,ht0,ht1,deptmin,deptmax);
  if (t0 + gt0 > deptmax) return vrb0(0,"hop %u tt range \aD%u-\aD%u, dep window \aD%u-\aD%u",hp->gid,t0 + gt0,t1 + gt0,deptmin,deptmax);
  else if (t1 + gt0 <= deptmin) return info(0,"hop %u tt range \aD%u-\aD%u, dep window \aD%u-\aD%u",hp->gid,t0 + gt0,t1 + gt0,deptmin,deptmax);

  ev = events + tp->evofs;
  map = evmaps + tp->dayofs;
  dev = src->depevs[0];
  t = 0;
  for (gndx = 0; gndx < gencnt * 2; gndx++) {
    t = (ub4)ev[gndx * 2];
    if (t + gt0 < deptmin) { info(Iter,"t %u gt0 %u deptmin %u",t,gt0,deptmin); continue; }
    else if (t + gt0 >= deptmax) {
      info(Iter,"t %u gt0 %u deptmax %u",t,gt0,deptmax);
      break;
    }
    x = ev[gndx * 2 + 1];
    dur = (x >> 32);
    dev[dcnt * 3] = t + gt0;
    dev[dcnt * 3 + 1] = (ub4)x; // tid+dayid
    dev[dcnt * 3 + 2] = dur;

    dcnt++;
    if (dcnt >= dmax) {
      warning(0,"exceeding %u dep event",dcnt);
      break;
    }
  }
  src->tps[0] = tp;
  src->dcnts[0] = dcnt;
//  info(0,"%u dep events",dcnt);
  return dcnt;
}

static ub4 addevs(search *src,ub8 *events,ub2 *evmaps,ub4 leg,struct timepat *tp,struct hop *hp)
{
  struct timepat *atp;
  ub4 deptmin = src->deptmin;
  ub4 deptmax = src->deptmax;
  ub4 gt0 = tp->gt0;
  ub4 ht0 = tp->ht0;
  ub4 ht1 = tp->ht1;
  ub4 t0 = tp->t0;
  ub4 t1 = tp->t1;
  ub4 dcnt = 0,gencnt = tp->genevcnt;
  ub4 gndx,agndx,dmax = Maxevs;
  ub4 adndx,adcnt;
  ub4 t,tid,at,dur,dt,adt,*dev,*adev;
  ub4 ttmax = 120; // todo
  ub8 x,*ev,*aev;
  ub2 *map;

  return 0;
  info0(0,"addevs");

  error_z(leg,0);
  ub4 aleg = leg - 1;

  if (t0 == t1) return info(0,"hop %u tt range %u-%u, dep window %u-%u",hp->gid,t0,t1,deptmin,deptmax);

  if (t0 + gt0 > deptmax + src->dtlos[leg]) return info(0,"hop %u tt range \aD%u-\aD%u, dep window \aD%u-\aD%u",hp->gid,t0 + gt0,t1 + gt0,deptmin,deptmax);
  else if (t1 + gt0 <= deptmin + src->dtlos[leg]) return info(0,"hop %u tt range \aD%u-\aD%u, dep window \aD%u-\aD%u",hp->gid,t0 + gt0,t1 + gt0,deptmin,deptmax);

  src->tps[leg] = tp;

  ev = events + tp->evofs;
  map = evmaps + tp->dayofs;

  atp = src->tps[aleg];
  aev = events + atp->evofs;

  dev = src->depevs[leg];
  adev = src->depevs[aleg];
  adcnt = src->dcnts[aleg];

  t = at = 0;
  gndx = agndx = adndx = 0;
  while (gndx < gencnt * 2 && adndx < adcnt && dcnt < dmax) {
    at = adev[adndx * 3];
    adt = adev [adndx * 3 + 2];
    while (gndx < gencnt && t < at + ttmax) {
      t = (ub4)ev[gndx * 2];
      if (t + gt0 < at) { gndx++; continue; }
      else if (t + gt0 < deptmin) { info(Iter,"t %u gt0 %u deptmin %u",t,gt0,deptmin); continue; }
      else if (t + gt0 >= deptmax) {
        info(Iter,"t %u gt0 %u deptmax %u",t,gt0,deptmax);
        break;
      }
      x = ev[gndx * 2 + 1];
      gndx++;
      tid = x & hi24;
      dur = (x >> 32);
      dt = dur + t + gt0 + adt - at; // accumulate total trip time
      dev[dcnt * 3] = t + gt0;
      dev[dcnt * 3 + 1] = tid;
      dev[dcnt * 3 + 2] = dt;
      dcnt++;
      if (dcnt >= dmax) {
        warning(0,"exceeding %u dep event",dcnt);
        break;
      }
    }
    adndx++;
  }
  src->dcnts[leg] = dcnt;
  info(0,"%u dep events for leg %u",dcnt,leg);
  return dcnt;
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

  ub4 cost,dist = 0,leg,l,l1,l2;
  ub4 *vp;

  ub4 costlim = src->costlim;
  ub4 *hopdist;
  ub4 v0 = 0;

  ub4 portcnt = net->portcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 *choporg = net->choporg;

  ub4 ln = callee & 0xffff;

  if (stop > histop) { vrb0(0,"%s: net part %u only has %u-stop connections, skip %u",desc,part,histop,stop); return 0; }

  // todo mockup
  deptmin = yymmdd2min(20140912,0);
  deptmax = yymmdd2min(20140914,0);

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
      memcpy(src->trip,vp,nleg * sizeof(ub4));
      for (leg = 0; leg < nleg; leg++) src->tripparts[leg] = (ub2)part;
      if (dist && dist == lodists[da]) info(0,"%u-stop found lodist %u at var %u %s:%u",stop,dist,v0,desc,ln);
    }
    // time
    l = vp[0];
    if (l < hopcnt) {
      hp = hops + l;
      tp = &hp->tp;
      mkdepevs(src,events,evmaps,tp,hp);
    } else {
      l1 = choporg[l * 2];
      l2 = choporg[l * 2 + 1];
      hp = hops + l1;
      tp = &hp->tp;
      mkdepevs(src,events,evmaps,tp,hp);
    }
    for (leg = 1; leg < nleg; leg++) {
      l = vp[leg];
      if (l < hopcnt) {
        hp = hops + l;
        tp = &hp->tp;
        addevs(src,events,evmaps,leg,tp,hp);
      } else {
        l1 = choporg[l * 2];
        l2 = choporg[l * 2 + 1];
        hp = hops + l1;
        tp = &hp->tp;
        addevs(src,events,evmaps,leg,tp,hp);
      }
    }

    v0++;
    vp += nleg;
    src->locvarcnt++;
  } while (v0 < cnt && cost > costlim && globs.sigint == 0);
  src->locsrccnt++;

  src->triplen = nleg;
  src->tripcnt = 1;

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
  ub4 tportcnt,dportcnt,aportcnt;
  ub4 hopcnt,chopcnt,tchopcnt,achopcnt;
  struct network *dnet,*anet;
  ub4 dep,arr,dmid,amid,depmid,tdepmid,tdmid,tamid,adepmid,amidarr;
  ub2 *cnts,*tcnts,*acnts,cnt,tcnt,acnt,var,tvar,avar;
  ub4 *ofss,*tofss,*aofss,ofs,tofs,aofs;
  ub4 *vp,*tvp,*avp;
  ub4 dstop,tstop,astop,histop;
  ub4 nleg,ntleg,naleg,l,l1,l2,leg,tleg,aleg;
  block *lstblk,*tlstblk,*alstblk;
  ub4 *lst,*tlst,*alst;
  ub4 *lodists,*tlodists,*alodists,lodist,tlodist,alodist;
  ub4 *hopdist,*thopdist,*ahopdist;
  ub4 dist,distrange,distiv,iv,pct;
  ub4 varcnt = 0;
  ub4 distbins[Distbins];
  ub4 distsums[Distbins];
  ub4 distlims[Percbins];
  ub8 *events;
  ub2 *evmaps;
  struct hop *hp,*hops;
  struct timepat *tp;
  ub4 *choporg;

  ub4 dvarcnt,tvarcnt,avarcnt,dvarxcnt,tvarxcnt,avarxcnt;

  dvarcnt = tvarcnt = avarcnt = dvarxcnt = tvarxcnt = avarxcnt = 0;

  aclear(distbins);
  aclear(distsums);

  dnet = getnet(dpart);
  anet = getnet(apart);

  tportcnt = tnet->portcnt;
  dportcnt = dnet->portcnt;
  aportcnt = dnet->portcnt;

  hopcnt = dnet->hopcnt;
  chopcnt = dnet->chopcnt;
  tchopcnt = tnet->chopcnt;
  achopcnt = anet->chopcnt;

  choporg = dnet->choporg;

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

  events = dnet->events;
  evmaps = dnet->evmaps;
  hops = dnet->hops;

  distrange = max(src->geodist,1) * 100;

  tcnt = 0;
  for (tstop = 0; tstop <= min(tnet->histop,histop); tstop++) {
    tcnts = tnet->concnt[tstop];
    tcnt = tcnts[tdepmid];
    if (tcnt) {
      info(0,"histop %u net %u dist %u top con at %u stop",histop,dnet->histop,src->geodist,tstop);
      break;
    }
  }
  if (tcnt == 0) {
    info(0,"no conn for top %u-%u",tdmid,tamid);
    return 0;
  }

  for (pct = 0; pct < Percbins; pct++) distlims[pct] = pct * 10;

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

      dist = 0;
      for (leg = 0; leg < nleg; leg++) {
        l = vp[leg];
        error_ge(l,chopcnt);
        dist += hopdist[l];
      }
      dist = max(dist,lodist);
      distiv = (dist - lodist) * Distbins / distrange;
      if (distiv >= Distbins) continue;

      if (varcnt > 100) {
        pct = distsums[distiv] * Percbins / varcnt;
        if (pct >= Percbins || distsums[distiv] > distlims[pct]) continue;
      }
      dvarxcnt++;

      for (leg = 0; leg < nleg; leg++) {
        l = vp[leg];

        if (l < hopcnt) {
          hp = hops + l;
          tp = &hp->tp;
          if (leg == 0) mkdepevs(src,events,evmaps,tp,hp);
          else addevs(src,events,evmaps,leg,tp,hp);
        } else {
          l1 = choporg[l * 2];
          l2 = choporg[l * 2 + 1];
          hp = hops + l1;
          tp = &hp->tp;
          if (leg == 0) mkdepevs(src,events,evmaps,tp,hp);
          else addevs(src,events,evmaps,leg,tp,hp);
        }
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
          if (distiv >= Distbins) continue;

          if (varcnt > 10) {
            pct = distsums[distiv] * Percbins / varcnt;
            if (pct >= Percbins || distsums[distiv] > distlims[pct]) continue;
          }
          tvarxcnt++;

//          addevs();
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
                  error(0,"arr part %u port %u-%u stop %u var %u leg %u at %p %s",apart,amid,arr,astop,avar,aleg,avp,alstblk->desc);
                  break;
                }
                error_ge(l,achopcnt);
                dist += ahopdist[l];
              }
              dist = max(dist,lodist + tlodist + alodist);
//              noexit error_lt(dist,alodist + tlodist + lodist);  todo
              distiv = (dist - lodist - tlodist - alodist) * Distbins / distrange;
              if (distiv >= Distbins) continue;

              if (varcnt > 10) {
                pct = distsums[distiv] * Percbins / varcnt;
                if (pct >= Percbins || distsums[distiv] > distlims[pct]) continue;
              }

              varcnt++;
              distbins[distiv]++;
              for (iv = 0; iv <= distiv; iv++) distsums[iv]++;

              avarxcnt++;
//              addevs();

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

  info(0,"%u of %u depvars %u of %u midvars %u of %u arrvars",dvarxcnt,dvarcnt,tvarxcnt,tvarcnt,avarxcnt,avarcnt);

  return 0;
}

static ub4 srcxpart(struct gnetwork *gnet,ub4 gdep,ub4 garr,search *src,char *ref)
{
  struct network *net,*tnet,*anet;
  ub1 *portparts = gnet->portparts;
  ub4 partcnt = gnet->partcnt;
  ub4 dep,arr,deparr;
  ub4 dstop,astop,tstop,dlostop,dhistop;
  ub4 conn,xconn = 0;
  ub4 part,tpart,dpart,apart;
  ub4 ti,ati,tcnt,tacnt;
  ub4 dtmid,gdtmid,tdmid,tamid,atmid,gatmid,gamid,xamid,xmid,gdmid;
  ub4 portcnt,tportcnt,aportcnt;
  ub4 *tp2g;
  ub4 stats[8];
  ub4 iv,niv = Elemcnt(stats);
  int rv;

  ub2 *xmap,*xamap,*xmapdbase,*xmapabase,stopset,xm,xam;
  ub1 *tmap;
  block *xpartdmap = &gnet->xpartdmap;
  block *xpartamap = &gnet->xpartamap;

  if (partcnt == 1) { error(0,"interpart search called without partitions, ref %s",ref); return 0; }

  aclear(stats);

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
      info(0,"conn %x for top %u-%u at %p",tmap[deparr],tdmid,tamid,tmap + deparr);
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
          stats[7]++;

          conn = srcxpart2(tnet,dpart,apart,gdep,garr,gdmid,gamid,src);
        }
      }
    }
  }

  for (iv = 0; iv < niv; iv++) info(0,"stats %u %u",iv,stats[iv]);

  return xconn;
}

static ub4 dosrc(struct gnetwork *gnet,ub4 nstoplo,ub4 nstophi,search *src,char *ref)
{
  ub4 gportcnt = gnet->portcnt;
  ub1 *portparts = gnet->portparts;
  ub4 part,partcnt = gnet->partcnt;
  ub4 gdep = src->dep;
  ub4 garr = src->arr;
  ub4 stop;

  ub4 conn;

  if (partcnt == 0) { error(0,"search called without partitions, ref %s",ref); return 0; }
  else if (gportcnt == 0) { error(0,"search without ports, ref %s",ref); return 0; }

  for (part = 0; part < partcnt; part++) {
    if (partcnt > 1 && (portparts[gdep * partcnt + part] == 0 || portparts[garr * partcnt + part] == 0)) continue;
    info(0,"dep %u and arr %u share part %u",gdep,garr,part);
    for (stop = nstoplo; stop < nstophi; stop++) {
      conn = srcglocal(gnet,part,gdep,garr,stop,src);
      if (conn) return conn;
    }
    return 0;
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

  // todo mockup
  deptmin = yymmdd2min(20140912,0);
  deptmax = yymmdd2min(20140914,0);

  src->deptmin = deptmin;
  src->deptmax = deptmax;
  src->histop = nstophi;

  t0 = gettime_usec();

  info(CC,"search geo dep %u arr %u %s to %s",dep,arr,pdep->name,parr->name);
  conn = dosrc(net,nstoplo,nstophi,src,ref);
  info(0,"searched %u local variants in %u local searches %u noloc",src->avarxcnt,src->locsrccnt,src->locnocnt);
  info(0,"%u of %u depvars %u of %u midvars %u of %u arrvars",src->dvarxcnt,src->dvarcnt,src->tvarxcnt,src->tvarcnt,src->avarxcnt,src->avarcnt);

  dt = gettime_usec() - t0;
  info(0,"search took \a.%u usec",(ub4)dt);

  if (src->tripcnt == 0) return info(0,"no route found for %u stops",nstophi);
  nleg = src->lostop + 1;
  info(0,"found %u-%u in %u/%u legs",dep,arr,src->triplen,nleg);
  if (gtriptoports(net,src->trip,src->tripparts,src->triplen,src->tripports)) {
    src->tripcnt = 0;
    return 1;
  }

  for (portno = 0; portno <= nleg; portno++) {
    port = src->tripports[portno];
    pp = ports + port;
    info(0,"port %u %s",port, pp->name);
  }

  return 0;
}
