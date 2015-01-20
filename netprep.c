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
#include "bitfields.h"
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

int prepnet(netbase *basenet)
{
  struct network *net;
  struct gnetwork *gnet = getgnet();

  struct portbase *bports,*bpp;
  struct hopbase *bhops,*bhp;
  struct sidbase *bsids,*bsp;
  struct chainbase *bchains,*bcp;
  struct routebase *broutes,*brp;

  ub8 *bchip,*bchainidxs;
  struct chainhopbase *bchp,*bchainhops;
// todo events

  struct port *ports,*pports,*pdep,*parr,*pp,*gp;
  struct hop *hops,*phops,*hp,*ghp;
  struct sidtable *sids,*sp;
  struct chain *chains,*cp;
  struct route *routes,*rp;
  struct timepatbase *btp;
  struct timepat *tp;

  ub4 *portsbyhop;
  char *dname,*aname;

  ub4 bportcnt,portcnt,tportcnt;
  ub4 bhopcnt,hopcnt,nilhopcnt,pridcnt;
  ub4 dep,arr,depp,arrp;
  ub4 bsidcnt,sidcnt,bchaincnt,chaincnt,chainlen,chainhopcnt;
  ub4 bridcnt,ridcnt;
  ub4 nlen,cnt,acnt,dcnt,tcnt,sid,rid,rrid,part,tpart,xmaplen;
  ub4 pportcnt,zpportcnt,phopcnt,partcnt,part2,newpartcnt;
  enum txkind kind;
  ub4 hop,port,zport,zpport,chain,phop,pport;
  ub4 hirrid;
  ub4 *hopcnts,*portcnts;
  ub4 *g2p,*p2g,*g2phop;
  ub4 *p2zp,*zp2p,*port2zport;
  struct partition *parts,*partp;
  ub4 *rrid2rid = basenet->rrid2rid;

  bhopcnt = basenet->hopcnt;
  bportcnt = basenet->portcnt;
  bsidcnt = basenet->sidcnt;
  bridcnt = basenet->ridcnt;
  bchaincnt = basenet->rawchaincnt;
  chainhopcnt = basenet->chainhopcnt;
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
    rp->chainofs = brp->chainofs;
    rp->chaincnt = brp->chaincnt;
    rp->hichainlen = brp->hichainlen;
    nlen = brp->namelen;
    if (nlen) {
      memcpy(rp->name,brp->name,nlen);
      rp->namelen = nlen;
    }
  }
  info(0,"%u routes",ridcnt);

  ub4 *gportsbyhop = alloc(bhopcnt * 2, ub4,0xff,"net portsbyhop",bhopcnt);
  ub4 *drids,*arids,*deps,*arrs;
  ub4 t0,t1,evcnt;

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

//    info(0,"hop %u %s at %u %u-%u",hop,bhp->name,bhp->cid,dep,arr);

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
    hp->rrid = rrid;
    rid = rrid2rid[rrid];
    hp->rid = rid;

    tp = &hp->tp;
    btp = &bhp->tp;
    t0 = btp->t0;
    t1 = btp->t1;
    evcnt = btp->evcnt;
//    warninfo(t1 == t0,0,"hop %u tt range %u-%u",hop,t0,t1);
    tp->utcofs = btp->utcofs;
    tp->tdays = btp->tdays;
    tp->gt0 = btp->gt0;
    tp->t0 = t0;
    tp->t1 = t1;
    tp->evcnt = evcnt;
    tp->genevcnt = btp->genevcnt;
    tp->evofs = btp->evofs;
    tp->dayofs = btp->dayofs;

    tp->lodur = btp->lodur;
    tp->hidur = btp->hidur;
    tp->midur = btp->midur;
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

    hp->dist = bhp->dist;

    hp->dep = dep;
    hp->arr = arr;
    hopcnt++;
  }
  if (hopcnt == 0) return error(0,"nil hops out of %u",bhopcnt);
  info(0,"%u from %u hops",hopcnt,bhopcnt);
  nilhopcnt = bhopcnt - hopcnt;
  hopcnt = bhopcnt;

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
    cp->rrid = bcp->rrid;
    cp->rid = bcp->rid;
    cp->tid = chain;
    cp->rtid = bcp->rtid;
    bofs = bcp->hopofs;
    cp->hopofs = ofs;
    ofs += cnt;
    chaincnt++;
  }
  chainhopcnt = ofs;
  info(0,"%u from %u chains with %u hops",chaincnt,bchaincnt,chainhopcnt);
  chaincnt = bchaincnt;

  ub4 *tid2rtid = alloc(chaincnt,ub4,0,"chain tid2rtid",chaincnt);
  struct chainhop *chp,*chainhops = alloc(chainhopcnt,struct chainhop,0,"chain hops",chainhopcnt);

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
    prvseq = seq = 0;
    for (i = 0; i < cnt; i++) {
      chp = chainhops + ofs + i;
      idx = bchip[i] & hi32;
      error_ge(idx,cnt);
      bchp = bchainhops + bofs + idx;
      prvseq = seq;
      seq = bchip[i] >> 32;
      infocc(seq == 0,0,"tid %u rtid %u idx %u/%u at %p",chain,rtid,i,cnt,bchip + i);
      error_le(seq,prvseq);
      chp->hop = bchp->hop;
      chp->tdep = bchp->tdep;
      chp->tarr = bchp->tarr;
      chp->midur = bchp->midur;
      chp->dist = bchp->dist;
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

  gnet->chainhops = chainhops;
  gnet->chainhopcnt = chainhopcnt;

  gnet->tid2rtid = tid2rtid;
  gnet->hichainlen = basenet->hichainlen;

  gnet->eventmem = &basenet->eventmem;
  gnet->evmapmem = &basenet->evmapmem;
  gnet->events = basenet->events;
  gnet->evmaps = basenet->evmaps;

  // write reference for name lookup
  if (wrportrefs(basenet)) return 1;

  if (condense(gnet)) return 1;

  return 0;
}
