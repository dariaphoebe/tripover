// netprep.c - prepare net from base

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/*
  copy work lists from i/o based base lists
 */

#include <string.h>
#include <stdlib.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "util.h"
#include "netbase.h"
#include "netio.h"
#include "netprep.h"
#include "net.h"
#include "condense.h"

void ininetprep(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

// check for a->b->c and c->b->a  given b
static int sameroute2a(struct hop *hops,ub4 b,ub4 *rids,ub4 *deps,ub4 *arrs)
{
  ub4 hopa1,hopa2,hopc1,hopc2,porta1,porta2,portc1,portc2;
  struct hop *hpa1,*hpa2,*hpc1,*hpc2;
  ub4 rid1 = rids[0];
  ub4 rid2 = rids[1];

  hopc1 = deps[0];
  hpc1 = hops + hopc1;
  error_ne(hpc1->dep,b);
  error_ne(hpc1->rid,rid1);
  portc1 = hpc1->arr;

  hopc2 = arrs[1];
  hpc2 = hops + hopc2;
  error_ne(hpc2->arr,b);
  error_ne(hpc2->rid,rid2);
  portc2 = hpc2->arr;

  if (portc1 != portc2) return 0;

  hopa1 = deps[1];
  hpa1 = hops + hopa1;
  error_ne(hpa1->dep,b);
  error_ne(hpa1->rid,rid2);
  porta1 = hpa1->arr;

  hopa2 = arrs[0];
  hpa2 = hops + hopa2;
  error_ne(hpa2->arr,b);
  error_ne(hpa2->rid,rid1);
  porta2 = hpa2->arr;

  if (porta1 != porta2) return 0;
  else return 1;
}

static int sameroute2b(struct hop *hops,ub4 b,ub4 *rids,ub4 *deps,ub4 *arrs)
{
  ub4 hopa1,hopa2,hopc1,hopc2,porta1,porta2,portc1,portc2;
  struct hop *hpa1,*hpa2,*hpc1,*hpc2;
  ub4 rid1 = rids[0];
  ub4 rid2 = rids[1];

  hopc1 = deps[0];
  hpc1 = hops + hopc1;
  error_ne(hpc1->dep,b);
  error_ne(hpc1->rid,rid1);
  portc1 = hpc1->arr;

  hopc2 = arrs[0];
  hpc2 = hops + hopc2;
  error_ne(hpc2->arr,b);
  error_ne(hpc2->rid,rid2);
  portc2 = hpc2->arr;

  if (portc1 != portc2) return 0;

  hopa1 = deps[1];
  hpa1 = hops + hopa1;
  error_ne(hpa1->dep,b);
  error_ne(hpa1->rid,rid2);
  porta1 = hpa1->arr;

  hopa2 = arrs[1];
  hpa2 = hops + hopa2;
  error_ne(hpa2->arr,b);
  error_ne(hpa2->rid,rid1);
  porta2 = hpa2->arr;

  if (porta1 != porta2) return 0;
  else return 1;
}

int prepnet(netbase *basenet)
{
  struct gnetwork *gnet = getgnet();
  struct portbase *bports,*bpp;
  struct hopbase *bhops,*bhp;
  struct sidbase *bsids,*bsp;
  struct chainbase *bchains,*bcp;
  struct routebase *broutes,*brp;

  ub8 *bchip,*bchainidxs;
  struct chainhopbase *bchp,*bchainhops;

  struct port *ports,*pdep,*parr,*pp;
  struct hop *hops,*hp;
  struct sidtable *sids,*sp;
  struct chain *chains,*cp;
  struct route *routes,*rp;
  struct timepatbase *btp;
  struct timepat *tp;

  ub4 bportcnt,portcnt;
  ub4 bhopcnt,hopcnt;
  ub4 dep,arr;
  ub4 bsidcnt,sidcnt,bchaincnt,chaincnt,chainhopcnt;
  ub4 bridcnt,ridcnt;
  ub4 nlen,cnt,acnt,dcnt,sid,rid,rrid;

  enum txkind kind;
  ub4 hop,port,chain;

  ub4 *rrid2rid = basenet->rrid2rid;
  ub4 hirrid = basenet->hirrid;

  bhopcnt = basenet->hopcnt;
  bportcnt = basenet->portcnt;
  bsidcnt = basenet->sidcnt;
  bridcnt = basenet->ridcnt;
  bchaincnt = basenet->rawchaincnt;
  if (bportcnt == 0 || bhopcnt == 0) return error(0,"prepnet: %u ports, %u hops",bportcnt,bhopcnt);

  // filter but leave placeholder in gnet to make refs match
  ports = alloc(bportcnt,struct port,0,"ports",bportcnt);
  portcnt = 0;
  bports = basenet->ports;
  for (port = 0; port < bportcnt; port++) {
    bpp = bports + port;
    pp = ports + port;
    nlen = bpp->namelen;
    if (bpp->ndep == 0 && bpp->narr == 0) {
      info(0,"skip unconnected port %u %s",port,bpp->name);
      if (nlen) memcpy(pp->name,bpp->name,nlen);
      pp->namelen = nlen;
      continue;
    }

    // new ndep,narr filled lateron
    pp->valid = 1;
    pp->id = pp->allid = pp->gid = portcnt;
    pp->cid = bpp->cid;
    nlen = bpp->namelen;
    if (nlen) {
      memcpy(pp->name,bpp->name,nlen);
      pp->namelen = nlen;
    } else info(0,"port %u has no name", port);
    pp->lat = bpp->lat;
    error_z(pp->lat,port);
    pp->lon = bpp->lon;
    pp->rlat = bpp->rlat;
    pp->rlon = bpp->rlon;
    pp->utcofs = bpp->utcofs;
    portcnt++;
  }
  info(0,"%u from %u ports",portcnt,bportcnt);
  portcnt = bportcnt;

  sidcnt = bsidcnt;
  sids = alloc(sidcnt,struct sidtable,0,"sids",sidcnt);
  bsids = basenet->sids;
  for (sid = 0; sid < bsidcnt; sid++) {
    bsp = bsids + sid;
    sp = sids + sid;
    sp->sid = bsp->sid;
    sp->t0 = bsp->t0;
    sp->t1 = bsp->t1;
    nlen = bsp->namelen;
    if (nlen) {
      memcpy(sp->name,bsp->name,nlen);
      sp->namelen = nlen;
    }
  }
  info(0,"%u sids",sidcnt);

  ridcnt = bridcnt;
  routes = alloc(ridcnt,struct route,0,"routes",ridcnt);
  broutes = basenet->routes;
  for (rid = 0; rid < ridcnt; rid++) {
    brp = broutes + rid;
    rp = routes + rid;
    rp->rrid = brp->rrid;
    rp->kind = brp->kind;
    rp->reserve = brp->reserve;
    rp->chainofs = brp->chainofs;
    rp->chaincnt = brp->chaincnt;
    rp->hopcnt = brp->hopcnt;
    rp->hichainlen = brp->hichainlen;
    nlen = brp->namelen;
    if (nlen) {
      memcpy(rp->name,brp->name,nlen);
      rp->namelen = nlen;
    }
    nsethi(rp->hop2chop,Chainlen * Chainlen);
  }
  info(0,"%u routes",ridcnt);

  ub4 *gportsbyhop = alloc(bhopcnt * 2, ub4,0xff,"net portsbyhop",bhopcnt);
  ub4 dist,*hopdist = alloc(bhopcnt,ub4,0,"net hopdist",bhopcnt);
  ub4 midur,*hopdur = alloc(bhopcnt,ub4,0,"net hopdur",bhopcnt);

  ub4 *drids,*arids,*deps,*arrs;
  ub4 t0,t1,tdep,prvtdep,evcnt;

  hops = alloc(bhopcnt,struct hop,0,"hops",bhopcnt);
  hopcnt = 0;
  bhops = basenet->hops;
  for (hop = 0; hop < bhopcnt; hop++) {
    bhp = bhops + hop;
    dep = bhp->dep;
    arr = bhp->arr;
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    if (dep == arr) {
      bpp = bports + dep;
      warning(0,"nil hop %u %s at %u",dep,bpp->name,bhp->cid);
      continue;
    } else if (bhp->valid == 0) {
      bpp = bports + dep;
      warning(0,"invalid hop %u %s at %u",dep,bpp->name,bhp->cid);
      continue;
    }

    pdep = ports + dep;
    parr = ports + arr;
    error_z(pdep->valid,hop);
    error_z(parr->valid,hop);

    gportsbyhop[hop * 2] = dep;
    gportsbyhop[hop * 2 + 1] = arr;

    hp = hops + hop;
    hp->gid = hop;
    nlen = bhp->namelen;
    if (nlen) {
      memcpy(hp->name,bhp->name,nlen);
      hp->namelen = nlen;
    }
    rrid = bhp->rrid;
    error_gt(rrid,hirrid,hop);
    hp->rrid = rrid;
    rid = rrid2rid[rrid];
    hp->rid = rid;
    if (rid != hi32) {
      rp = routes + rid;
      hp->reserve = rp->reserve;
    } else {
      rp = NULL;
      hp->reserve = 0;
    }
    hp->rhop = bhp->rhop;

    tp = &hp->tp;
    btp = &bhp->tp;
    t0 = btp->t0;
    t1 = btp->t1;
    evcnt = btp->evcnt;
    tp->utcofs = btp->utcofs;
    tp->tdays = btp->tdays;
    tp->gt0 = btp->gt0;
    tp->t0 = t0;
    tp->t1 = t1;

    tp->evcnt = evcnt;
    tp->genevcnt = btp->genevcnt;
    error_ne(tp->genevcnt,evcnt);

    tp->evofs = btp->evofs;
    tp->dayofs = btp->dayofs;

    tp->lodur = btp->lodur;
    tp->hidur = btp->hidur;
    midur = tp->midur = btp->midur;
    hopdur[hop] = midur;
    tp->avgdur = btp->avgdur;
    tp->duracc = btp->duracc;

    // mark local links
    pdep = ports + dep;
    parr = ports + arr;
    dcnt = pdep->ndep;
    deps = pdep->deps;
    drids = pdep->drids;
    if (dcnt == 0) {
      deps[0] = hop;
      drids[0] = rid;
      pdep->ndep = 1;
    } else if (dcnt == 1) {
      if (deps[0] != hop || drids[0] != rid) {
        deps[1] = hop;
        drids[1] = rid;
        pdep->ndep = 2;
      }
    } else if ( (deps[0] == hop && drids[0] == rid) || (deps[1] == hop && drids[1] == rid) ) ;
    else pdep->ndep = dcnt + 1;

    acnt = parr->narr;
    arrs = parr->arrs;
    arids = parr->arids;
    if (acnt == 0) {
      arrs[0] = hop;
      arids[0] = rid;
      parr->narr = 1;
    } else if (acnt == 1) {
      if (arrs[0] != hop || arids[0] != rid) {
        arrs[1] = hop;
        arids[1] = rid;
        parr->narr = 2;
      }
    } else if ( (arrs[0] == hop && arids[0] == rid) || (arrs[1] == hop && arids[1] == rid) ) ;
    else parr->narr = acnt + 1;

    kind = bhp->kind;
    hp->kind = kind;

    dist = hp->dist = bhp->dist;
    hopdist[hop] = dist;
    error_z(dist,hop);

    hp->dep = dep;
    hp->arr = arr;
    hopcnt++;
  }
  if (hopcnt == 0) return error(0,"nil hops out of %u",bhopcnt);
  info(0,"%u from %u hops",hopcnt,bhopcnt);
  hopcnt = bhopcnt;

  // prepare <rid,dep,arr> to hop lookup
  ub4 rhop;

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    rid = hp->rid;
    if (rid == hi32) continue;
    error_ge(rid,ridcnt);
    rp = routes + rid;
    rhop = hp->rhop;
    if (rhop < Chainlen) rp->hops[rhop] = hop;
  }

  ub4 oneridcnt = 0;
  ub4 onerid;
  bool oneroute;
  ub4 ndep,narr;

  // mark single-route only ports  
  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    ndep = pp->ndep;
    narr = pp->narr;
    drids = pp->drids;
    arids = pp->arids;
    arrs = pp->arrs;
    deps = pp->deps;

    oneroute = 0; onerid = 0;
    if (ndep == 0 && narr == 0) continue;
    else if (ndep > 2 || narr > 2) continue;
    else if (ndep == 0 && narr == 1 && arids[0] != hi32) {
      oneroute = 1;
      pp->onerid = arids[0];
    } else if (ndep == 1 && narr == 0 && drids[0] != hi32) {
      oneroute = 1;
      pp->onerid = drids[0];
    } else if (ndep == 1 && narr == 1 && drids[0] == arids[0] && drids[0] != hi32) {
      oneroute = 1;
      pp->onerid = arids[0];
    } else if (ndep == 2 && narr == 2 && drids[0] != hi32 && drids[1] != hi32) {
      if (drids[0] == drids[1] && arids[0] == arids[1] && drids[0] == arids[0]) {
        oneroute = 1;
        pp->onerid = arids[0];
      } else if (drids[0] == arids[0] && drids[1] == arids[1]) {
        if (sameroute2a(hops,port,drids,deps,arrs)) {
          oneroute = 1;
          pp->onerid = arids[0];
        }
      } else if (drids[0] == arids[1] && drids[1] == arids[0]) {
        if (sameroute2b(hops,port,drids,deps,arrs)) {
          oneroute = 1;
          pp->onerid = arids[0];
        }
      }
    }
    if (oneroute) {
      pp->oneroute = 1;
      pp->onerid = onerid;
      oneridcnt++;
    }
  }

  info0(0,"global connectivity");
  showconn(ports,portcnt,0);

  ub4 i,idx,bofs,ofs,seq,prvseq,rtid;

  chains = alloc(bchaincnt,struct chain,0,"chains",bchaincnt);
  bchains = basenet->chains;
  bchainhops = basenet->chainhops;
  bchainidxs = basenet->chainidxs;
  chaincnt = chainhopcnt = ofs = 0;
  for (chain = 0; chain < bchaincnt; chain++) {
    bcp = bchains + chain;
    cp = chains + chain;
    cnt = bcp->hopcnt;
    if (cnt < 3) { vrb(0,"skip dummy chain %u with %u hop\as",chain,cnt); continue; }
    cp->hopcnt = cnt;
    cp->rhopcnt = bcp->rhopcnt;
    cp->rrid = bcp->rrid;
    cp->rid = bcp->rid;
    cp->tid = chain;
    cp->rtid = bcp->rtid;
    cp->hopofs = ofs;
    ofs += cnt;
    chaincnt++;
  }
  chainhopcnt = ofs;
  info(0,"%u from %u chains with %u hops",chaincnt,bchaincnt,chainhopcnt);
  chaincnt = bchaincnt;

  ub4 *tid2rtid = alloc(chaincnt,ub4,0,"chain tid2rtid",chaincnt);
  struct chainhop *chp,*chainhops = alloc(chainhopcnt,struct chainhop,0,"chain hops",chainhopcnt);

  // write in sorted order
  for (chain = 0; chain < chaincnt; chain++) {
    cp = chains + chain;
    bcp = bchains + chain;
    cnt = cp->hopcnt;
    rtid = cp->rtid;
    tid2rtid[chain] = rtid;
    if (cnt < 3) continue;

    ofs = cp->hopofs;
    bofs = bcp->hopofs;
    bchip = bchainidxs + bofs;
    cp->rhopcnt = bcp->rhopcnt;
    cp->rhopofs = bcp->rhopofs;
    seq = prvtdep = 0;
    for (i = 0; i < cnt; i++) {
      chp = chainhops + ofs + i;
      idx = bchip[i] & hi32;
      error_ge(idx,cnt);
      bchp = bchainhops + bofs + idx;
      prvseq = seq;
      seq = (ub4)(bchip[i] >> 32);
      hop = bchp->hop;
      tdep = bchp->tdep;
      error_le(seq,prvseq);
      chp->hop = hop;
      chp->tdep = tdep;
      error_lt(tdep,prvtdep);
      prvtdep = tdep;
      chp->tarr = bchp->tarr;
      error_lt(chp->tarr,chp->tdep);
      chp->midur = bchp->midur;
    }
  }

  gnet->portcnt = portcnt;
  gnet->hopcnt = hopcnt;
  gnet->sidcnt = sidcnt;
  gnet->chaincnt = chaincnt;
  gnet->ridcnt = ridcnt;

  gnet->ports = ports;
  gnet->hops = hops;
  gnet->sids = sids;
  gnet->chains = chains;
  gnet->routes = routes;

  gnet->portsbyhop = gportsbyhop;
  gnet->hopdist = hopdist;
  gnet->hopdur = hopdur;

  gnet->chainhops = chainhops;
  gnet->chainrhops = basenet->chainrhops;
  gnet->chainhopcnt = chainhopcnt;

  gnet->hirrid = hirrid;
  gnet->rrid2rid = rrid2rid;

  gnet->tid2rtid = tid2rtid;
  gnet->hichainlen = basenet->hichainlen;

  gnet->eventmem = &basenet->eventmem;
  gnet->evmapmem = &basenet->evmapmem;
  gnet->events = basenet->events;
  gnet->evmaps = basenet->evmaps;
  gnet->t0 = basenet->t0;
  gnet->t1 = basenet->t1;

  gnet->walklimit = m2geo(globs.walklimit);
  gnet->sumwalklimit = m2geo(globs.sumwalklimit);
  gnet->walkspeed = m2geo(globs.walkspeed);

  info(0,"precomputed walk limit set to %u m, summed %u",globs.walklimit,globs.sumwalklimit);

  // write reference for name lookup
  if (wrportrefs(basenet)) return 1;

//  if (condense(gnet)) return 1; not (yet?)

  return 0;
}
