// condense.c - make network more dense

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* a single tripover server loads a single net.
   net can be global, regional or local, filtered out of a single base net
   use lossless and lossy compression for search net only to accomodate memory
   all nets have the complete basenet
   client issues query to any, tpically global.
   answer contains region ids and hints for subsequent refined query.

   smalldep to small arr ->  smalldep.1 via near bigdep.1  via bighub.2 via bigarr.3 to smallarr.3
    next query smalldep to bigdep @region1 with time results from previous
    next query bigdep.1 to bighub.2 @region1 -> if bigedge.1 or bigedge.2 exist then part of global result
    next query bigarr.2 to smalarr.3 @region3

  separate ports into :
 - full aka regular
 - mini  e.g. on one route with 2 deps,  2 arrs to a peer mini
 - macro group of related minis merged

- main net setup with only full+macro. interpolate minis at search

mini: partially reconstructible. regio and global nets
     1m -> 100k ports ?

? spoke: partially reconstructible regio and global nets
  a <-> b   with a's only peer to b. merge with b

    -> 50k ?

knot: lossy, global only ?
  within small coords and time, connects > 2
  complete metro nets to single macro ?
  merge set of nearby dens-connected nodes to single knotport
  reduce links : any to knot member -> any to knot

  a->b->
  |  |
  d<-c->
     |


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
#include "net.h"
#include "condense.h"

#undef hdrstop

void inicondense(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

static int nolink(struct port *pdep,struct port *parr,ub4 arr)
{
  if (pdep->ndep == 0 || parr->narr == 0) { info(0,"ndep %u narr %u",pdep->ndep,parr->narr); return 1; }

  if (pdep->nudep == 1) {
    if (pdep->deps[0] != arr) vrb(0,"arr %u != %u ",pdep->deps[0],arr);
    return (pdep->deps[0] != arr);
  }
  if (pdep->deps[0] != arr && pdep->deps[1] != arr) vrb(0,"arr %u %u not %u",pdep->deps[0],pdep->deps[1],arr);
  return (pdep->deps[0] != arr && pdep->deps[1] != arr);
}

static ub4 newhop(struct hop *dst,struct hop *src,ub4 hop,ub4 dep,ub4 arr)
{
  *dst = *src;
  dst->dep = dep; dst->arr = arr;
  return hop + 1;
}

static ub4 newport(struct port *dst,struct port *src,ub4 port)
{
  *dst = *src;
  dst->id = port;
  dst->ndep = dst->narr = 0;
  return port + 1;
}

// create condensed net out of base net
int condense(struct network *net)
{
  ub4 port,portcnt,allportcnt = net->allportcnt;
  ub4 fullportcnt,allport,macport;
  ub4 hop,hopcnt,allhop,allhopcnt = net->allhopcnt;
  struct port *pp,*npp,*ports,*pparr,*ppdep,*parr,*pdep,*allports = net->allports;
  struct hop *hp,*ahp,*hops,*allhops = net->allhops;
  ub4 *macports,*macseqs,*all2full,*mac2port,*all2mac;
  ub4 *minilst;
  ub4 nmac,macsize,macseq,mergeiter,merged,miniofs,miniport;
  ub4 dep,arr,ndep,narr,ndeparr,narrdep,ndepdep,narrarr,depndx,arrndx;
  ub4 allarr,alldep,cnt;
  ub4 minicnt,iv,latlo,lathi,lonlo,lonhi,lat2hi,lat2lo,lon2lo,lon2hi,macid,nloc,dlat,dlon;
  int mini,change;

  if (allportcnt == 0 || allhopcnt == 0) return 1;

  if (allportcnt < 2000) { // todo heuristic

    net->portcnt = allportcnt;
    net->hopcnt = allhopcnt;
    net->ports = allports;
    net->hops = allhops;
    return 0;
  }

  info(0,"condensing %u ports",allportcnt);

  for (port = 0; port < allportcnt; port++) {
    pp = allports + port;
    pp->macid = port;
    pp->macbox[0] = pp->macbox[1] = pp->lat;
    pp->macbox[2] = pp->macbox[3] = pp->lon;
  }

// condense into mini, full and macro ports
  minicnt = 0;

  nloc = Elemcnt(pp->deps);

  ub4 nominis[8];
  aclear(nominis);

// first part : a-b-c-d where b and c only connect to each other and a or d
  for (port = 0; port < allportcnt; port++) {
    if (allportcnt - minicnt < 1000) break; // todo heuristic

    pp = allports + port;

    ndep = pp->nudep;
    narr = pp->nuarr;
    if (ndep > 2 || narr > 2) { nominis[0]++; continue; }

    mini = 1;
    for (depndx = 0; depndx < min(ndep,nloc); depndx++) {
      dep = pp->deps[depndx];
      error_ge(dep,allportcnt);

      // mini only if connecting to mini ?
      parr = allports + dep;
      narrdep = parr->nudep;
      if (narrdep > 2) { mini = 0; nominis[1]++; break; }
      narrarr = parr->nuarr;
      if (narrarr > 2) { mini = 0; nominis[2]++; break; }
      error_z(narrarr,dep);

      // sanity check
      if (parr->arrs[0] != port && (narrarr == 1 || parr->arrs[1] != port)) {
        warning(0,"port %u-%u not in local net",port,dep);
        mini = 0;
        break;
      }

      // a to b and b to a
      if (ndep > 1 && nolink(parr,pp,port)) {
        vrb(Iter,"hop %u-%u does not link back on dep %s for %u-%u links",port,dep,pp->name,pp->nudep,pp->nuarr);
        mini = 0;
        nominis[3]++;
        break;
      }
    }
    if (mini == 0) continue;

    for (arrndx = 0; arrndx < min(narr,nloc); arrndx++) {
      arr = pp->arrs[arrndx];
      error_ge(arr,allportcnt);

      // mini only if connecting to mini ?
      pdep = allports + arr;
      ndeparr = pdep->nuarr;
      if (ndeparr > 2) { mini = 0; nominis[4]++; break; }
      ndepdep = pdep->nudep;
      if (ndepdep > 2) { mini = 0; nominis[5]++; break; }
      error_z(ndepdep,arr);

      // sanity check
      if (pdep->deps[0] != port && (ndepdep == 1 || pdep->deps[1] != port)) {
        warning(0,"port %u-%u not in local net",port,arr);
        mini = 0;
        break;
      }

      // a to b and b to a
      if (narr > 1 && nolink(pp,pdep,arr)) {
        vrb(Iter,"hop %u-%u does not link back on arr",arr,port);
        nominis[6]++;
        mini = 0;
        break;
      }
    }
    if (mini == 0) continue;

    pp->mini = 1;
    pp->macid = hi32;
    minicnt++;
  }
  info(0,"%u of %u miniports",minicnt,allportcnt);
  for (iv = 0; iv < Elemcnt(nominis); iv++) info(0,"nomini on %u: %u",iv,nominis[iv]);

  ub4 diffminis[4];
  aclear(diffminis);

  macseq = 0;
  mergeiter = 0;

  // merge minis into macros
  do {
    change = 0;
    mergeiter++;
    merged = 0;

    for (port = 0; port < allportcnt; port++) {
      pp = allports + port;

      if (pp->mini == 0) continue;
      if (pp->macid != hi32) continue;
      macid = hi32;

      ndep = pp->nudep;
      narr = pp->nuarr;

      latlo = pp->macbox[0]; lathi = pp->macbox[1];
      lonlo = pp->macbox[2]; lonhi = pp->macbox[3];

      // rudimentary: assign id based on geo range
      // todo: use schedule time

      depndx = 0;
      while (depndx < min(ndep,nloc)) {
        dep = pp->deps[depndx++];
        error_eq(dep,port);
        pparr = allports + dep;

        if (pparr->mini == 0) { diffminis[0]++; continue; }

        lat2lo = pparr->macbox[0]; lat2hi = pparr->macbox[1];
        lon2lo = pparr->macbox[2]; lon2hi = pparr->macbox[3];

        dlat = max(lathi,lat2hi) - min(latlo,lat2lo);
        dlon = max(lonhi,lon2hi) - min(lonlo,lon2lo);
        if (dlat > 400 || dlon > 400) { diffminis[1]++; continue; }

        macid = pparr->macid;

        if (macid == hi32) {
          macid = macseq++;
          pparr->macid = macid;
        }
        pp->macid = macid;
        pp->macbox[0] = pparr->macbox[0] = min(latlo,lat2lo);
        pp->macbox[1] = pparr->macbox[1] = max(lathi,lat2hi);
        pp->macbox[2] = pparr->macbox[2] = min(lonlo,lon2lo);
        pp->macbox[3] = pparr->macbox[3] = max(lonhi,lon2hi);
        change = 1;
      }
      pparr = NULL;

      arrndx = 0;
      while (arrndx < min(narr,nloc)) {
        arr = pp->arrs[arrndx++];
        error_eq(arr,port);

        ppdep = allports + arr;
        if (ppdep->mini == 0) { diffminis[2]++; continue; }

        lat2lo = ppdep->macbox[0]; lat2hi = ppdep->macbox[1];
        lon2lo = ppdep->macbox[2]; lon2hi = ppdep->macbox[3];

        dlat = max(lathi,lat2hi) - min(latlo,lat2lo);
        dlon = max(lonhi,lon2hi) - min(lonlo,lon2lo);
        if (dlat > 400 || dlon > 400) { diffminis[3]++; continue; }

        if (macid == hi32 && ppdep->macid == hi32) {
          macid = macseq++;
          ppdep->macid = pp->macid = macid;
        } else if (macid == hi32 && ppdep->macid != hi32) {
          pp->macid = ppdep->macid;
        } else if (macid != hi32 && ppdep->macid == hi32) {
          ppdep->macid = macid;
        } else {
          vrb(0,"port %u arr %u merge on mac %u and %u",port,arr,macid,ppdep->macid);
          ppdep->macid = macid;
        }
        pp->macbox[0] = ppdep->macbox[0] = min(latlo,lat2lo);
        pp->macbox[1] = ppdep->macbox[1] = max(lathi,lat2hi);
        pp->macbox[2] = ppdep->macbox[2] = min(lonlo,lon2lo);
        pp->macbox[3] = ppdep->macbox[3] = max(lonhi,lon2hi);
        change = 1;
      }
      ppdep = NULL;

      if (change) {
        vrb(0,"change at macid %u port %u",macid,port);
        merged++;
      }

    } // each allport

    info(0,"miniport merge iteration %u merged %u",mergeiter,merged);

  } while (change && mergeiter < 1000);

  // cancel single minis
  minicnt = 0;
  for (port = 0; port < allportcnt; port++) {
    pp = allports + port;

    if (pp->mini) {
      if (pp->macid == hi32) {
        pp->mini = 0;
      } else minicnt++;
    }
  }
  info(0,"%u of %u miniports",minicnt,allportcnt);

  info(0,"miniport merge took %u iterations",mergeiter);

  // count and allocate macros

  macports = alloc(allportcnt,ub4,0,"macports",allportcnt);

  all2mac = alloc(allportcnt,ub4,0xff,"all2mac",allportcnt);  // todo not used ?

  mac2port = alloc(allportcnt,ub4,0,"mac2port",allportcnt);

  for (port = 0; port < allportcnt; port++) {
    pp = allports + port;
    if (pp->mini == 0) continue;

    macid = pp->macid;
    error_ge(macid,allportcnt);
    macports[macid]++;
  }

  nmac = 0;
  struct range macrange;
  ub4 macivs[16];
  for (macid = 0; macid < allportcnt; macid++) {
    macsize = macports[macid];
    if (macsize) nmac++;
  }
  info(0,"%u macro ports", nmac);
  mkhist(macports,allportcnt,&macrange,Elemcnt(macivs),macivs,"macro ports",Info);

  for (port = 0; port < allportcnt; port++) {
    pp = allports + port;
    if (pp->mini == 0) continue;

    macid = pp->macid;
    macsize = macports[macid];
    error_z(macsize,port);
    pp->macsize = macsize;
  }

  macseqs = alloc(allportcnt,ub4,0xff,"macseqs",allportcnt);
  macseq = 0;
  for (port = 0; port < allportcnt; port++) {
    pp = allports + port;
    if (pp->mini == 0) continue;

    macid = pp->macid;
    if (macseqs[macid] == hi32) {
      pp->macid = macseqs[macid] = macseq++;
      pp->macone = 1;
    } else pp->macid = macseqs[macid];
  }

  info(0,"sequenced %u macro ports", macseq);
  error_ne(nmac,macseq);

  fullportcnt = allportcnt - minicnt;
  portcnt = fullportcnt + nmac;

  info(0,"%u ports, %u full %u mini %u macro %u new",allportcnt,fullportcnt,minicnt,nmac,portcnt);

  ports = alloc(portcnt,struct port,0,"ports",portcnt);
  npp = ports;

  all2full = alloc(allportcnt,ub4,0xff,"all2full",allportcnt);

  minilst = alloc(minicnt,ub4,0,"minilst",minicnt);

  info(0,"assigning %u+%u=%u ports",fullportcnt,nmac,portcnt);

  // full ports
  macport = port = 0;
  for (allport = 0; allport < allportcnt; allport++) {  // full ports
    pp = allports + allport;
    if (pp->mini) continue;

    error_ge(port,portcnt);
    npp = ports + port;
    vrb(0,"assign full port %u",port);
    all2full[allport] = port;
    port = newport(npp,pp,port);
  }
  info(0,"assigned %u full ports",port);

  // macros: first member of each mini group
  miniofs = 0;
  for (allport = 0; allport < allportcnt; allport++) {
    pp = allports + allport;
    if (pp->macone == 0) {
      if (pp->mini == 1) {
        macid = pp->macid;
        all2mac[allport] = macid;
      }
      continue;
    }
    error_ge(port,portcnt);
    npp = ports + port;

    macid = pp->macid;
    all2mac[allport] = macid;
    mac2port[macid] = port;

    port = newport(npp,pp,port);

    npp->macid = macid;
    npp->miniofs = miniofs;

    miniofs += pp->macsize;
  }

  error_ne(port,portcnt);
  error_ne(miniofs,minicnt);

  info(0,"assign %u miniports",minicnt);

  for (allport = 0; allport < allportcnt; allport++) {
    pp = allports + allport;
    if (pp->macone == 0) continue;

    macid = pp->macid;
    miniofs = pp->miniofs;
    cnt = 0;
    for (miniport = 0; miniport < allportcnt; miniport++) {
      if (miniport != allport && all2mac[miniport] == macid) {
        error_ge(cnt,pp->macsize);
        minilst[miniofs + cnt++] = miniport;
      }
    }
  }

  info(0,"arranging %u hops",allhopcnt);

  // rearrange hops
  hopcnt = 0;
  for (hop = 0; hop < allhopcnt; hop++) {
    hp = allhops + hop;

    dep = hp->dep;
    arr = hp->arr;
    pdep = allports + dep;
    parr = allports + arr;
    if (all2full[dep] != hi32 && all2full[arr] != hi32) hopcnt++; // full to full
    else if (all2full[dep] != hi32) { // full to mac
      if (parr->mini) hopcnt++;
    } else if (all2full[arr] != hi32) { // mac to full
      if (pdep->mini) hopcnt++;
    } else {   // mac to mac
      if (pdep->macid != parr->macid) hopcnt++;
    }
  }

  info(0,"%u from %u hops",hopcnt,allhopcnt);

  hops = alloc(hopcnt,struct hop,0,"hops",hopcnt);
  hp = hops;

  hop = 0;

  for (allhop = 0; allhop < allhopcnt; allhop++) {
    ahp = allhops + allhop;
    alldep = ahp->dep;
    allarr = ahp->arr;
    dep = all2full[alldep];
    arr = all2full[allarr];

    error_gt(hop,hopcnt);

    hp = hops + hop;

    if (dep != hi32 && arr != hi32) { // full to full
      error_eq(dep,arr);
      hop = newhop(hp,ahp,hop,dep,arr);

    } else if (dep != hi32) {  // full to mac
      macid = all2mac[allarr];
      if (macid != hi32) {
        arr = mac2port[macid];
        error_ge(arr,portcnt);
        hop = newhop(hp,ahp,hop,dep,arr);
      } else info(0,"todo hop %u from %u dep %u arr %u",hop,allhop,alldep,allarr);

    } else if (arr != hi32) {  // mac to full

      macid = all2mac[alldep];
      if (macid != hi32) {
        dep = mac2port[macid];
        error_ge(dep,portcnt);
        hop = newhop(hp,ahp,hop,dep,arr);
      } else info(0,"todo hop %u from %u dep %u arr %u",hop,allhop,alldep,allarr);

    } else { // mac to mac
      pdep = allports + alldep;
      parr = allports + allarr;
      error_z(pdep->mini,alldep);
      error_z(parr->mini,allarr);
      if (pdep->macid == parr->macid) continue;
      error_eq(alldep,allarr);
      dep = mac2port[pdep->macid];
      arr = mac2port[parr->macid];
      if (dep == hi32) error_z(0,alldep);
      error_eq(dep,arr);
      hop = newhop(hp,ahp,hop,dep,arr);
    }
  }
  error_ne(hop,hopcnt);

  info0(0,"compress done");

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    dep = hp->dep;
    arr = hp->arr;
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    error_eq(dep,arr);
    pdep = ports + dep;
    pdep->ndep++;
    parr = ports + arr;
    parr->narr++;
  }

  net->portcnt = portcnt;
  net->hopcnt = hopcnt;
  net->ports = ports;
  net->hops = hops;

  net->mac2port = mac2port;

  return 0;
}
