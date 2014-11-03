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
  condense reduces either 1M or 1.x k
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

void ininetprep(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

static int marklocal(struct network *net)
{
  // mark local links
  ub4 undx,ndep,narr,nudep,nuarr,nvdep,nvarr,nveq;
  struct hop *hp,*hops = net->allhops;
  struct port *pdep,*parr,*ports = net->allports;
  ub4 hop,dep,arr;
  ub4 routeid;
  ub4 hopcnt = net->allhopcnt;
  ub4 portcnt = net->allportcnt;
  ub4 varmask = net->routevarmask;

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

    routeid = hp->routeid;

    nudep = pdep->nudep;
    nvdep = pdep->nvdep;
    nuarr = parr->nuarr;
    nvarr = parr->nvarr;

    if (nudep == 0) {
      pdep->deps[0] = arr; pdep->nudep = pdep->nvdep = 1; pdep->drids[0] = routeid;
    } else if (nudep < Nlocal) {
      undx = nveq = 0;
      while (undx < nudep && pdep->deps[undx] != arr) {
        if (routeid != hi32 && (routeid & varmask) == (pdep->drids[undx] & varmask) ) nveq = 1;
        undx++;
      }
      if (undx == nudep) {
        pdep->deps[undx] = arr; pdep->nudep = undx + 1;
        pdep->drids[undx] = routeid;
        if (nveq == 0 || routeid == hi32) pdep->nvdep = nvdep + 1;
      }
    }
    if (nuarr == 0) {
      parr->arrs[0] = dep; parr->nuarr = parr->nvarr = 1; parr->arids[0] = routeid;
    } else if (nuarr < Nlocal) {
      undx = nveq = 0;
      while (undx < nuarr && parr->arrs[undx] != dep) {
        if (routeid != hi32 && (routeid & varmask) == (parr->arids[undx] & varmask) ) nveq = 1;
        undx++;
      }
      if (undx == nuarr) {
        parr->arrs[undx] = dep; parr->nuarr = undx + 1;
        parr->arids[undx] = routeid;
        if (nveq == 0 || routeid == hi32) parr->nvarr = nvarr + 1;
      }
    }
  }
  return 0;
}

int prepnet(netbase *basenet)
{
  struct network *net;
  struct gnetwork *gnet = getgnet();
  struct portbase *bports,*bpp,*bpdep,*bparr;
  struct hopbase *bhops,*bhp;
  struct port *ports,*pports,*pdep,*parr,*pp,*gp;
  struct hop *hops,*phops,*hp,*ghp;
  ub4 bportcnt,portcnt,xportcnt,bhopcnt,hopcnt,dep,arr,aport,depp,arrp;
  ub4 nlen,cnt,acnt,dcnt,routeid,part,hpart,xmaplen;
  ub4 pportcnt,phopcnt,partcnt,npart1;
  enum txkind kind;
  ub4 hop,port,phop,pport;
  ub4 variants,varmask,maxrid;
  ub4 *hopcnts,*portcnts;
  ub1 *portparts;
  ub4 *g2p,*p2g;
  struct partition *parts,*partp,*apartp;
  ub4 latscale = basenet->latscale;
  ub4 lonscale = basenet->lonscale;

  bhopcnt = basenet->hopcnt;
  bportcnt = basenet->portcnt;
  if (bportcnt == 0 || bhopcnt == 0) return 1;

  // filter unconnected ports
  portcnt = 0;
  bports = basenet->ports;
  for (port = 0; port < bportcnt; port++) {
    bpp = bports + port;
    bpp->id = portcnt;
    if (bpp->ndep == 0 && bpp->narr == 0) {
      info(0,"omitting unconnected port %u %s",bpp->id,bpp->name);
    } else portcnt++;
    if (bpp->namelen == 0) warning(0,"port %u has no name",bpp->id);
  }
  info(0,"%u from %u ports",portcnt,bportcnt);
  ports = alloc(portcnt,struct port,0,"ports",portcnt);

  // filter nil hops
  hopcnt = 0;
  bhops = basenet->hops;
  for (hop = 0; hop < bhopcnt; hop++) {
    bhp = bhops + hop;
    dep = bhp->dep;
    arr = bhp->arr;
    error_ge(dep,bportcnt);
    error_ge(arr,bportcnt);
    if (dep == arr) {
      bpp = bports + dep;
      warning(0,"nil hop %u %s at %u",dep,bpp->name,bhp->cid);
    } else hopcnt++;
  }
  hops = alloc(hopcnt,struct hop,0,"hops",hopcnt);

  portcnt = 0;
  for (port = 0; port < bportcnt; port++) {
    bpp = bports + port;
    if (bpp->ndep == 0 && bpp->narr == 0) continue;

    pp = ports + portcnt;
    pp->id = pp->allid = pp->gid = portcnt;
    pp->cid = bpp->cid;
    nlen = bpp->namelen;
    if (nlen) {
      memcpy(pp->name,bpp->name,nlen);
      pp->namelen = nlen;
    }
    pp->lat = bpp->lat;
    pp->lon = bpp->lon;
    pp->rlat = bpp->rlat;
    pp->rlon = bpp->rlon;
    pp->utcofs = bpp->utcofs;
    portcnt++;
  }

  hopcnt = 0;
  for (hop = 0; hop < bhopcnt; hop++) {
    bhp = bhops + hop;
    dep = bhp->dep;
    arr = bhp->arr;
    if (dep == arr) continue;

    hp = hops + hopcnt;
    hp->id = hopcnt;
    nlen = bhp->namelen;
    if (nlen) {
      memcpy(hp->name,bhp->name,nlen);
      hp->namelen = nlen;
    }
    routeid = bhp->routeid;
    hp->routeid = routeid;
    kind = bhp->kind;
    hp->kind = kind;

    bpdep = bports + dep;
    bparr = bports + arr;
    dep = bpdep->id;
    arr = bparr->id;
    hp->dep = dep;
    hp->arr = arr;

    hopcnt++;
  }

  hopcnts = gnet->hopcnts;
  portcnts = gnet->portcnts;
  parts = gnet->parts;

  maxrid = basenet->maxrouteid;

  // todo: ad-hoc partitioning
  partcnt = max(portcnt / 1000,1);
  portparts = alloc(partcnt * portcnt,ub1,0,"portparts",portcnt);
  npart1 = (partcnt - 1);

  xmaplen = 0;

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    dep = hp->dep;
    arr = hp->arr;
    routeid = hp->routeid;

    // todo: ad-hoc partitioning
    if (partcnt > 1) {
      part = (maxrid / routeid) % partcnt;
    } else part = 0;
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
    case Unknown: info(0,"hop %s has unknown transport mode", hp->name); break;
    case Kindcnt: break;
    }

  }

  gnet->portcnt = portcnt;
  gnet->hopcnt = hopcnt;
  gnet->ports = ports;
  gnet->hops = hops;

  gnet->partcnt = partcnt;
  gnet->portparts = portparts;

  gnet->maxrouteid = maxrid = basenet->maxrouteid;
  gnet->maxvariants = variants = basenet->maxvariants;
  gnet->routevarmask = varmask = basenet->routevarmask;

  ub4 partstats[8];
  ub4 iv,xmap,partivs = Elemcnt(partstats) - 1;
  aclear(partstats);
  for (port = 0; port < portcnt; port++) {
    cnt = xmap = 0;
    gp = ports + port;
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

  gnet->xpartbase = mkblock(&gnet->xpartmap,xmaplen,ub2,Init0,"xmap for %u parts", partcnt);

  // separate into partitions
  for (part = 0; part < partcnt; part++) {
    net = getnet(part);
    partp = parts + part;

    pportcnt = portcnts[part];
    phopcnt = hopcnts[part];

    if (pportcnt == 0) {
      warning(0,"partition %u has no ports",part);
      continue;
    }
    if (phopcnt == 0) {
      warning(0,"partition %u has no hops",part);
      continue;
    }

    // count
    ub4 hpcnt,hxcnt;
    hpcnt = hxcnt = 0;
    for (hop = 0; hop < hopcnt; hop++) {
      error_ge(hop,hopcnt);
      ghp = hops + hop;
      hpart = ghp->part;
      dep = ghp->dep;
      arr = ghp->arr;
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

    // assign ports : members of this part, plus placeholder for each other part
    pp = pports;
    pport = 0;
    for (port = 0; port < portcnt; port++) {
      gp = ports + port;
      if (portparts[port * partcnt + part] == 0) continue;
      memcpy(pp,gp,sizeof(*pp));
      pp->id = pport;
      pp->ndep = pp->narr = 0;
      g2p[port] = pport;
      p2g[pport] = port;
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
      pp++;
      pport++;    
    }

    // separate and resequence hops
    ub4 dpcnt,apcnt,hpcnt2,hxcnt2;

    hp = phops;
    phop = hpcnt2 = hxcnt2 = 0;
    for (hop = 0; hop < hopcnt; hop++) {
      if (phop >= phopcnt) info(0,"hop %u phop %u phopcnt %u %u %u",hop,phop,phopcnt,hpcnt2,hxcnt2);
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
        error_ge(phop,phopcnt);
        hpcnt2++;
        error_gt(hpcnt2,hpcnt);
        memcpy(hp,ghp,sizeof(*hp));
        hp->id = phop;
        hp->gid = hop;
        depp = g2p[dep];
        error_ge(depp,pportcnt);
        hp->dep = depp;
        arrp = g2p[arr];
        error_ge(arrp,pportcnt);
        hp->arr = arrp;
        hp++;
        phop++;
      } else if (dpcnt == 1 && apcnt == 1) { // other part to same other part: skip
      } else if (dpcnt > 1 || apcnt > 1) {   // possible interpart
        if (portparts[dep * partcnt + part] == 0 && portparts[arr * partcnt + part] == 0) continue;
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
        hp->routeid = hi32;
        hp++;
        phop++;
      }
    }
    error_ne(phop,phopcnt);

    net->part = part;
    net->partcnt = partcnt;

    net->pportcnt = pportcnt;
    net->allportcnt = xportcnt;
    net->allhopcnt = phopcnt;
    net->allports = pports;
    net->allhops = phops;
    net->g2pport = g2p;
    net->p2gport = p2g;

    net->maxrouteid = maxrid;
    net->maxvariants = variants;
    net->routevarmask = varmask;

    marklocal(net);

    info(0,"partition %u connectivity",part);
    showconn(pports,pportcnt);
  }

  info0(0,"global connectivity");
  showconn(ports,portcnt);

  return 0;
}
