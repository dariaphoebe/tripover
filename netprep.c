// netprep.c - prepare net from base

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/*
  copy work lists from i/o based base lists

  partition ports into partitions by assigning hops a partition
  ports can be members of multiple partitions
  each partition thus includes member ports plus a placeholder for an aggregrated entire partition
  
  1 M ports in 1 K parts -> 1k parts of (1.xk + 1k) each
  x depends on partition criteria
  placeholders can have limited n-stop connectivity
  condense reduces 1M
 */

#include <string.h>

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

static int marklocal(struct network *net)
{
  // mark local links
  ub4 ndep,narr,nudep,nuarr,nvdep,nvarr;
  struct hop *hp,*hops = net->hops;
  struct port *pdep,*parr,*ports = net->ports;
  ub4 hop,dep,arr;
  ub4 rrid;
  ub4 hopcnt = net->hopcnt;
  ub4 portcnt = net->portcnt;

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;

    dep = hp->dep;
    arr = hp->arr;
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    pdep = ports + dep;
    parr = ports + arr;

    ndep = pdep->ndep;
    narr = parr->narr;

    pdep->ndep = ndep+1;
    parr->narr = narr+1;

    if (hp->kind == Walk) continue;

    rrid = hp->rrid;

    nudep = pdep->nudep;
    nvdep = pdep->nvdep;
    nuarr = parr->nuarr;
    nvarr = parr->nvarr;

    if (ndep < Nlocal) {
      pdep->deps[ndep] = arr; pdep->drids[ndep] = rrid;
    }
    if (narr < Nlocal) {
      parr->arrs[narr] = dep; parr->arids[narr] = rrid;
    }

#if 0
    if (nudep == 0) {
      pdep->deps[0] = arr; pdep->nudep = pdep->nvdep = 1; pdep->drids[0] = rrid;
    } else if (nudep < Nlocal) {
      undx = nveq = 0;
      while (undx < nudep && pdep->deps[undx] != arr) {
        if (rrid != hi32 && (rrid & varmask) == (pdep->drids[undx] & varmask) ) nveq = 1;
        undx++;
      }
      if (undx == nudep) {
        pdep->deps[undx] = arr; pdep->nudep = undx + 1;
        pdep->drids[undx] = rrid;
        if (nveq == 0 || rrid == hi32) pdep->nvdep = nvdep + 1;
      }
    }
    if (nuarr == 0) {
      parr->arrs[0] = dep; parr->nuarr = parr->nvarr = 1; parr->arids[0] = rrid;
    } else if (nuarr < Nlocal) {
      undx = nveq = 0;
      while (undx < nuarr && parr->arrs[undx] != dep) {
        if (rrid != hi32 && (rrid & varmask) == (parr->arids[undx] & varmask) ) nveq = 1;
        undx++;
      }
      if (undx == nuarr) {
        parr->arrs[undx] = dep; parr->nuarr = undx + 1;
        parr->arids[undx] = rrid;
        if (nveq == 0 || rrid == hi32) parr->nvarr = nvarr + 1;
      }
    }
#endif

  }
  return 0;
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

  ub8 *bchainhops;
// todo events

  struct port *ports,*pports,*pdep,*parr,*pp,*gp;
  struct hop *hops,*phops,*hp,*ghp;
  struct sidtable *sids,*sp;
  struct chain *chains,*cp;
  struct route *routes,*rp;

  ub4 *portsbyhop;

  ub4 bportcnt,portcnt,xportcnt,bhopcnt,hopcnt,pridcnt;
  ub4 dep,arr,aport,depp,arrp;
  ub4 bsidcnt,sidcnt,bchaincnt,chaincnt,chainhopcnt;
  ub4 bridcnt,ridcnt;
  ub4 nlen,cnt,acnt,dcnt,sid,rid,rrid,part,hpart,xmaplen;
  ub4 pportcnt,zpportcnt,phopcnt,partcnt,npart1;
  enum txkind kind;
  ub4 hop,port,zport,zpport,chain,phop,pport;
  ub4 variants,varmask,hirrid;
  ub4 *hopcnts,*portcnts,*ridcnts;
  ub1 *portparts;
  ub4 *g2p,*p2g,*g2phop;
  ub4 *p2zp,*zp2p,*port2zport;
  struct partition *parts,*partp,*apartp;
  ub4 latscale = basenet->latscale;
  ub4 lonscale = basenet->lonscale;
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
    if (bpp->ndep == 0 && bpp->narr == 0) { info(0,"skip unconnected port %u %s",port,bpp->name); continue; }
    // new ndep,narr filled lateron

    pp = ports + port;
    pp->valid = 1;
    pp->id = pp->allid = pp->gid = bportcnt;
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
    } else if (bhp->valid == 0) continue;

//    info(0,"hop %u %s at %u %u-%u",hop,bhp->name,bhp->cid,dep,arr);

    pdep = ports + dep;
    parr = ports + arr;
    error_z(pdep->valid,hop);
    error_z(parr->valid,hop);

    gportsbyhop[hop * 2] = dep;
    gportsbyhop[hop * 2 + 1] = arr;

    hp = hops + hop;
    nlen = bhp->namelen;
    if (nlen) {
      memcpy(hp->name,bhp->name,nlen);
      hp->namelen = nlen;
    }
    rrid = bhp->rrid;
    hp->rrid = rrid;
    rid = rrid2rid[rrid];
    hp->rid = rid;

    // mark local links, filtering duplicates e.g on loops
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

    hp->dep = dep;
    hp->arr = arr;
    hopcnt++;
  }
  if (hopcnt == 0) return error(0,"nil hops out of %u",bhopcnt);
  info(0,"%u from %u hops",hopcnt,bhopcnt);
  hopcnt = bhopcnt;

  info0(0,"global connectivity");
  showconn(ports,portcnt,0);

  chains = alloc(bchaincnt,struct chain,0,"chains",bchaincnt);
  bchains = basenet->chains;
  bchainhops = basenet->chainhops;
  chaincnt = 0;
  for (chain = 0; chain < bchaincnt; chain++) {
    bcp = bchains + chain;
    cp = chains + chain;
    cnt = bcp->hopcnt;
    if (cnt < 3) { vrb(0,"skip dummy chain %u with %u hop\as",chain,cnt); continue; }
    cp->hopcnt = cnt;
    cp->rrid = bcp->rrid;
    cp->hopofs = bcp->hopofs;
    chaincnt++;
  }
  info(0,"%u from %u chains",chaincnt,bchaincnt);
  chaincnt = bchaincnt;

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

  gnet->routechains = basenet->routechains;
  gnet->chainhops = bchainhops;
  gnet->chainhopcnt = chainhopcnt;

  gnet->maxvariants = variants = basenet->maxvariants;
  gnet->routevarmask = varmask = basenet->routevarmask;

  gnet->eventmem = &basenet->eventmem;
  gnet->evmapmem = &basenet->evmapmem;
  gnet->events = basenet->events;
  gnet->evmaps = basenet->evmaps;

  if (condense(gnet)) return 1;

  // prepare partitioning
  hopcnts = gnet->hopcnts;
  portcnts = gnet->portcnts;
  ridcnts = gnet->ridcnts;
  parts = gnet->parts;

  port2zport = gnet->port2zport;
  error_zp(port2zport,0);

  hirrid = basenet->hirrid;

  ub4 *routeparts = alloc(hirrid + 1,ub4,0,"routes",hirrid);

  // todo: ad-hoc partitioning
  partcnt = max(portcnt / 3000,1);
  info(0,"%u partition\as",partcnt);
  portparts = alloc(partcnt * portcnt,ub1,0,"portparts",portcnt);
  npart1 = (partcnt - 1);

  if (partcnt > 1) {
    for (rrid = 1; rrid <= hirrid; rrid++) {
      rid = rrid2rid[rrid];
      if (rid >= ridcnt) continue;
      part = min(rrid * partcnt / hirrid,partcnt - 1);
      routeparts[rrid] = part;
      rp = routes + rid;
      vrb(0,"part %u rid %u",part,rid);
      rp->part = part;
    }
  }

  xmaplen = 0;

  for (rid = 0; rid < ridcnt; rid++) {
    rp = routes + rid;
    part = rp->part;
    ridcnts[part]++;
  }

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    dep = hp->dep;
    arr = hp->arr;
    if (dep == arr) { info(0,"hop %u %u-%u",hop,dep,arr); continue; }

    rrid = hp->rrid;

    // todo: ad-hoc partitioning
    part = routeparts[rrid];
    hp->part = part;
    hopcnts[part]++;

    // todo inferred walk links to all parts ?

    pdep = ports + dep;
    parr = ports + arr;

    pdep->ndep++;
    parr->narr++;

    partp = parts + part;
    error_z_cc(pdep->lat,"dport %u %s",dep,pdep->name);
    error_z_cc(pdep->lon,"dport %u %s",dep,pdep->name);
    error_z_cc(parr->lat,"aport %u %s",arr,parr->name);
    error_z_cc(parr->lon,"aport %u %s",arr,parr->name);

    dcnt = portparts[dep * partcnt + part];
    if (dcnt == 0) {
      updbbox(pdep->lat,pdep->lon,partp->bbox);
      pdep->part = part;
      portcnts[part]++;
    }
    if (dcnt < 255) portparts[dep * partcnt + part] = (ub1)dcnt + 1;

    acnt = portparts[arr * partcnt + part];
    if (acnt == 0 && dep != arr) {
      updbbox(parr->lat,parr->lon,partp->bbox);
      parr->part = part;
      portcnts[part]++;
    }
    if (acnt < 255) portparts[arr * partcnt + part] = (ub1)acnt + 1;

    kind = hp->kind;
    switch(kind) {
    case Walk: pdep->nwalkdep++; parr->nwalkarr++; break;
    case Air: break;
    case Rail: break;
    case Bus: break;
    case Ferry: break;
    case Unknown: info(0,"hop %s has unknown transport mode", hp->name); break;
    case Kindcnt: break;
    }

  }

  ub4 partstats[8];
  ub4 iv,xmap,partivs = Elemcnt(partstats) - 1;
  aclear(partstats);
  for (port = 0; port < portcnt; port++) {
    cnt = xmap = 0;
    gp = ports + port;
    if (gp->ndep == 0 && gp->narr == 0) continue;

    for (part = 0; part < partcnt; part++) {
      net = getnet(part);
      if (portparts[port * partcnt + part]) {
        if (cnt < Nxpart) {
          gp->pmapofs[cnt] = xmaplen;
          gp->partnos[cnt] = (ub2)part;
          xmaplen += portcnts[part];
        }
        cnt++;
      }
    }
    gp->partcnt = cnt;

    partstats[min(partivs,cnt)]++;
  }
  cnt = partstats[0];
  if (cnt) warning(0,"%u port\as in no partition", cnt);
  cnt = partstats[1];
  genmsg(cnt ? Info : Warn,0,"%u port\as in 1 partition", cnt);
  for (iv = 2; iv <= partivs; iv++) {
    cnt = partstats[iv];
    if (cnt) info(0,"%u port\as in %u partition\as each", cnt, iv);
  }
  info(0,"\ah%u xmaps",xmaplen);

  if (xmaplen == 0) return 1;

  gnet->partcnt = partcnt;
  gnet->xpartbase = mkblock(&gnet->xpartmap,xmaplen,ub2,Init0,"xmap for %u parts", partcnt);
  gnet->portparts = portparts;
  gnet->hirrid = hirrid;

  // separate into partitions
  for (part = 0; part < partcnt; part++) {
    net = getnet(part);
    partp = parts + part;

    pportcnt = portcnts[part];
    phopcnt = hopcnts[part];
    pridcnt = ridcnts[part];
    zpportcnt = 0;

    if (pportcnt == 0) {
      warning(0,"partition %u has no ports",part);
      continue;
    }
    if (phopcnt == 0) {
      warning(0,"partition %u has no hops",part);
      continue;
    }

    msgprefix(0,"s%u ",part);

    // count
    ub4 hpcnt,hxcnt;
    hpcnt = hxcnt = 0;
    for (hop = 0; hop < hopcnt; hop++) {
      error_ge(hop,hopcnt);
      ghp = hops + hop;
      hpart = ghp->part;
      dep = ghp->dep;
      arr = ghp->arr;
      if (dep == arr) continue;

      error_ge(dep,portcnt);
      error_ge(arr,portcnt);
      pdep = ports + dep;
      parr = ports + arr;
      if (hpart == part) hpcnt++;
      else if (pdep->partcnt == 1 && parr->partcnt == 1) { }
      else if (pdep->partcnt > 1 || parr->partcnt > 1) {
        if (pdep->partcnt > 1) vrb(0,"dport %u parts %u %s",dep,pdep->partcnt,pdep->name);
        if (parr->partcnt > 1) vrb(0,"aport %u parts %u %s",arr,parr->partcnt,parr->name);
        if (portparts[dep * partcnt + part] || portparts[arr * partcnt + part]) hxcnt++;
      }
    }
    error_ne(hpcnt,phopcnt);

    xportcnt = pportcnt + npart1;
    info(0,"partition %u: %u+%u ports %u+%u hops",part,pportcnt,npart1,phopcnt,hxcnt);
    phopcnt += hxcnt;

    pports = alloc(xportcnt,struct port,0,"ports",xportcnt);
    phops = alloc(phopcnt,struct hop,0,"phops",phopcnt);

    g2p = alloc(portcnt,ub4,0xff,"g2p-ports",portcnt);
    p2g = alloc(xportcnt,ub4,0xff,"p2g-ports",xportcnt);
    g2phop = alloc(hopcnt,ub4,0xff,"g2p-hops",hopcnt);

    p2zp = alloc(max(portcnt,xportcnt),ub4,0xff,"p2zp-ports",portcnt);
    zp2p = alloc(portcnt,ub4,0xff,"p2zp-ports",portcnt);

    portsbyhop = alloc(phopcnt * 2, ub4,0xff,"net portsbyhop",portcnt);

    // assign ports : members of this part, plus placeholder for each other part
    pp = pports;
    pport = 0;
    for (port = 0; port < portcnt; port++) {
      gp = ports + port;
      if (portparts[port * partcnt + part] == 0) continue;
      memcpy(pp,gp,sizeof(*pp));
      pp->id = pport;
      pp->gid = port;
      pp->ngdep = pp->ndep;
      pp->ngarr = pp->narr;
      pp->ndep = pp->narr = 0;
      g2p[port] = pport;
      p2g[pport] = port;

      zport = port2zport[port];
      error_ge(zport,portcnt);
      zpport = zp2p[zport];
      if (zpport == hi32) {
        zpport = zp2p[zport] = zpportcnt++;
      }
      p2zp[pport] = zpport;
      zp2p[zpport] = pport;
      pp++;
      pport++;
    }

    // placeholders for other parts
    for (aport = 0; aport < partcnt; aport++) {
      if (aport == part) continue;
      apartp = parts + aport;
      pp->id = pport;
      pp->gid = portcnt + part * partcnt + aport;
      pp->part = aport;
      fmtstring(pp->name,"part %u placeholder for part %u",aport,part);
      error_z(apartp->bbox[Boxcnt],aport);
      pp->lat = apartp->bbox[Midlat];
      pp->lon = apartp->bbox[Midlon];
      if (pp->lat == 0) {
        error(0,"port %u lat 0 %s",pport,pp->name);
      }
      pp->rlat = lat2rad(pp->lat,latscale);
      pp->rlon = lon2rad(pp->lon,lonscale);
      pp->isagg = 1;
      p2zp[pport] = pport;
      zp2p[pport] = pport;

      pp++;
      pport++;    
    }

    // separate and resequence hops
    ub4 dpcnt,apcnt,hpcnt2,hxcnt2;

    hp = phops;
    phop = hpcnt2 = hxcnt2 = 0;
    for (hop = 0; hop < hopcnt; hop++) {
      ghp = hops + hop;
      hpart = ghp->part;
      dep = ghp->dep;
      arr = ghp->arr;
      error_ge(dep,portcnt);
      error_ge(arr,portcnt);
      pdep = ports + dep;
      parr = ports + arr;
      dpcnt = pdep->partcnt;
      apcnt = parr->partcnt;
      error_z(dpcnt,hop);
      error_z(apcnt,hop);
      if (hpart == part) { // this part to this part
        if (phop >= phopcnt) warning(0,"hop %u phop %u phopcnt %u %u %u",hop,phop,phopcnt,hpcnt2,hxcnt2);
        error_ge(phop,phopcnt);
        hpcnt2++;
        error_gt(hpcnt2,hpcnt);
        g2phop[hop] = phop;
        memcpy(hp,ghp,sizeof(*hp));
        hp->gid = hop;
        depp = g2p[dep];
        error_ge(depp,pportcnt);
        hp->dep = depp;
        arrp = g2p[arr];
        error_ge(arrp,pportcnt);
        hp->arr = arrp;
        portsbyhop[phop * 2] = depp;
        portsbyhop[phop * 2 + 1] = arrp;
        hp++;
        phop++;
      } else if (dpcnt == 1 && apcnt == 1) { // other part to same other part: skip
      } else if (dpcnt > 1 || apcnt > 1) {   // possible interpart
        if (portparts[dep * partcnt + part] == 0 && portparts[arr * partcnt + part] == 0) continue;
        if (phop >= phopcnt) info(0,"hop %u phop %u phopcnt %u %u %u",hop,phop,phopcnt,hpcnt2,hxcnt2);
        error_ge(phop,phopcnt);
        hxcnt2++;
        error_gt(hxcnt2,hxcnt);
        if (portparts[dep * partcnt + part]) {
          depp = g2p[dep];
          error_ge(depp,pportcnt);
          hp->dep = depp;
        } else {
          hp->dep = pportcnt + pdep->part;
          if (pdep->part > part) hp->dep--;
        }
        if (portparts[arr * partcnt + part]) {
          arrp = g2p[arr];
          error_ge(arrp,pportcnt);
          hp->arr = arrp;
        } else {
          hp->arr = pportcnt + parr->part;
          if (parr->part > part) hp->arr--;
        }
        if (hp->dep == hp->arr) {
          warning(0,"dep %u equals arr",hp->dep);
          continue;
        }
        hp->rrid = hi32;
        portsbyhop[phop * 2] = hp->dep;
        portsbyhop[phop * 2 + 1] = hp->arr;
        hp++;
        phop++;
      }
    }
    error_ne(phop,phopcnt);

    net->part = part;
    net->partcnt = partcnt;

    net->pportcnt = pportcnt;
    net->zportcnt = zpportcnt;
    net->portcnt = xportcnt;
    net->hopcnt = phopcnt;
    net->ports = pports;
    net->hops = phops;
    net->g2pport = g2p;
    net->p2gport = p2g;
    net->g2phop = g2phop;

    net->port2zport = p2zp;
    net->zport2port = zp2p;

    net->portsbyhop = portsbyhop;

    net->routes = routes;  // not partitioned
    net->ridcnt = ridcnt;
    net->pridcnt = ridcnts[part];

    // global
    net->chains = chains;
    net->chaincnt = chaincnt;
    net->routechains = gnet->routechains;
    net->chainhops = bchainhops;
    net->chainhopcnt = chainhopcnt;

    net->eventmem = gnet->eventmem;
    net->evmapmem = gnet->evmapmem;
    net->events = gnet->events;
    net->evmaps = gnet->evmaps;

    net->maxvariants = variants;
    net->routevarmask = varmask;

    marklocal(net);

    info(0,"partition %u connectivity",part);
    showconn(pports,pportcnt,1);
  }
  msgprefix(0,NULL);

  return 0;
}
