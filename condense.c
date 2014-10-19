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

// todo: from config and conditional/relative
static int offrange(ub8 dlat,ub8 dlon)
{
  ub8 dsquare = dlat * dlat + dlon * dlon;
  return dsquare > 1000000;
}

// create condensed net out of full net
int condense(struct network *net)
{
  ub4 port,portcnt,allportcnt = net->allportcnt;
  ub4 fullportcnt,allport;
  ub4 hop,hopcnt,allhop,allhopcnt = net->allhopcnt;
  struct port *pp,*npp,*ports,*pparr,*ppdep,*parr,*pdep,*allports = net->allports;
  struct hop *hp,*ahp,*hops,*allhops = net->allhops;
  ub4 *macports,*macseqs,*all2full,*mac2port,*all2mac;
  ub4 *minilst;
  ub4 nmac,macsize,macseq,mergeiter,merged,miniofs,miniport;
  ub4 dep,arr,ndep,narr,nvdep,nvarr,depndx,arrndx;
  ub4 allarr,alldep,cnt;
  ub4 routeid;
  ub4 varmask = net->routevarmask,maxvariant = net->maxvariants;
  ub4 minicnt,iv,latlo,lathi,lonlo,lonhi,lat2hi,lat2lo,lon2lo,lon2hi,macid,dlat,dlon;
  int change,docondense;

  if (allportcnt == 0) return info0(0,"skip condense on 0 ports");
  if (allhopcnt == 0) return info0(0,"skip condense on 0 hops");

  docondense = dorun(Runcondense);

  if (allportcnt < 200) docondense = 0;  // todo heuristic

  if (docondense == 0) {
    info(0,"no condense for %u ports",allportcnt);
    net->portcnt = allportcnt;
    net->hopcnt = allhopcnt;
    net->ports = allports;
    net->hops = allhops;
    return 0;
  }

  info(0,"condensing %u ports",allportcnt);

  info(0,"%u possible variants, mask %x", maxvariant,varmask);

  for (port = 0; port < allportcnt; port++) {
    pp = allports + port;
    pp->macid = port;
    pp->macbox[0] = pp->macbox[1] = pp->lat;
    pp->macbox[2] = pp->macbox[3] = pp->lon;
  }

// condense into mini, full and macro ports
  minicnt = 0;

// part 1: cluster on inferred walk links
// gtfs feeds group platform stops into parent stations 
  for (port = 0; port < allportcnt; port++) {

    pp = allports + port;

    if (pp->nwalkdep != 0 && pp->nwalkarr != 0) {
      vrb(0,"port %s mini on walk deps %u arrs %u", pp->name,pp->nwalkdep,pp->nwalkarr);
      pp->mini = 1;
      pp->macid = hi32;
      minicnt++;
    }
  }
  info(0,"pass 1: %u of %u miniports",minicnt,allportcnt);

  ub4 nominis[8];
  ub4 okminis[8];
  aclear(nominis);
  aclear(okminis);

// part 2 : a-b-c-d where b and c only connect to each other and a or d
  for (port = 0; port < allportcnt; port++) {
    if (allportcnt - minicnt < 50) break; // todo heuristic

    pp = allports + port;
    if (pp->mini) continue;

    ndep = pp->nudep;
    narr = pp->nuarr;
    nvdep = pp->nvdep;
    nvarr = pp->nvarr;

    if (ndep == 0 && narr == 0) {
      warning(0,"port %u %s is not connected",port,pp->name);
      continue;
    } else if (nvdep == 0 && nvarr == 0) {
      nominis[0]++;
      continue;
    }

    if (nvdep == 0 && nvarr == 1) { // e.g. term or unidir
      okminis[0]++;
      pp->mini = 1;
      continue;
    } else if (nvdep == 1 && nvarr == 0) { // e.g term or unidir
      okminis[1]++;
      pp->mini = 1;
      continue;
    } else if (nvdep == 1 && nvarr == 1) { // bidir route
      if ( (pp->drids[0] & varmask) == (pp->arids[0] & varmask) ) {
        okminis[2]++;
        pp->mini = 1;
        continue;
      } else nominis[0]++;
    } else if (nvdep == 2 && nvarr == 2) {
      nominis[1]++;
      continue;
    } else if (nvdep == 2 && nvarr == 1) {
      nominis[2]++;
      continue;
    } else if (nvdep == 1 && nvarr == 2) {
      nominis[3]++;
      continue;
    } else {
      nominis[4]++;
      continue;
    }

  }

  minicnt = 0;
  for (port = 0; port < allportcnt; port++) {
    pp = allports + port;
    if (pp->mini) {
      pp->macid = hi32;
      minicnt++;
    }
  }
  info(0,"pass 2: %u of %u miniports",minicnt,allportcnt);
  for (iv = 0; iv < Elemcnt(okminis); iv++) info(0,"okmini on %u: %u",iv,okminis[iv]);
  for (iv = 0; iv < Elemcnt(nominis); iv++) info(0,"nomini on %u: %u",iv,nominis[iv]);

  ub4 diffminis[8];
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
      if (pp->macid != hi32) continue;   // already assigned

      macid = hi32;

      if (pp->nwalkdep) {
        for (hop = 0; hop < allhopcnt; hop++) {
          hp = allhops + hop;
          if (hp->kind == Walk && hp->dep == port) {
            arr = hp->arr;
            pparr = allports + arr;
            if (pparr->mini) {
              macid = pparr->macid;
              if (macid == hi32) {
                macid = macseq++;
                pparr->macid = macid;
              }
              pp->macid = macid;
              change = 1;
              break;
            }
          }
        }
      }

      if (macid == hi32 && pp->nwalkarr) {
        for (hop = 0; hop < allhopcnt; hop++) {
          hp = allhops + hop;
          if (hp->kind == Walk && hp->arr == port) {
            dep = hp->dep;
            ppdep = allports + dep;
            if (ppdep->mini) {
              macid = ppdep->macid;
              if (macid == hi32) {
                macid = macseq++;
                ppdep->macid = macid;
              }
              pp->macid = macid;
              change = 1;
              break;
            }
          }
        }
      }

      if (change) continue;

      ndep = pp->nudep;
      narr = pp->nuarr;

      latlo = pp->macbox[0]; lathi = pp->macbox[1];
      lonlo = pp->macbox[2]; lonhi = pp->macbox[3];

      // rudimentary: assign id based on geo range
      // todo: use schedule time

      depndx = 0;
      while (depndx < ndep && change == 0) {
        dep = pp->deps[depndx];
        routeid = pp->drids[depndx];
        depndx++;
        if (routeid == hi32) { diffminis[0]++; continue; }

        error_eq(dep,port);
        pparr = allports + dep;

        if (pparr->mini == 0) { diffminis[1]++; continue; }

        lat2lo = pparr->macbox[0]; lat2hi = pparr->macbox[1];
        lon2lo = pparr->macbox[2]; lon2hi = pparr->macbox[3];

        dlat = max(lathi,lat2hi) - min(latlo,lat2lo);
        dlon = max(lonhi,lon2hi) - min(lonlo,lon2lo);
        if (offrange(dlat,dlon)) { diffminis[2]++; continue; }

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
        depndx++;
        change = 1;
      }
      if (change) continue;
      else diffminis[3]++;

      pparr = NULL;

      arrndx = 0;
      while (arrndx < narr) {
        arr = pp->arrs[arrndx];
        error_eq(arr,port);

        routeid = pp->arids[arrndx];
        arrndx++;
        if (routeid == hi32) { diffminis[3]++; continue; }

        ppdep = allports + arr;
        if (ppdep->mini == 0) { diffminis[4]++; continue; }

        lat2lo = ppdep->macbox[0]; lat2hi = ppdep->macbox[1];
        lon2lo = ppdep->macbox[2]; lon2hi = ppdep->macbox[3];

        dlat = max(lathi,lat2hi) - min(latlo,lat2lo);
        dlon = max(lonhi,lon2hi) - min(lonlo,lon2lo);
        if (offrange(dlat,dlon)) { diffminis[5]++; continue; }

        macid = ppdep->macid;
        if (macid == hi32) {
          macid = macseq++;
          ppdep->macid = macid;
        }
        pp->macid = macid;

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
      } else { diffminis[6]++; continue; }

    } // each allport

    vrb(0,"miniport merge iteration %u merged %u",mergeiter,merged);

  } while (change && mergeiter < 10000);
  info(0,"tentative %u macros",macseq);
  for (iv = 0; iv < Elemcnt(diffminis); iv++) info(0,"diffmini on %u: %u",iv,diffminis[iv]);

  ub4 onemini = 0;

  // cancel single minis
  minicnt = 0;
  for (port = 0; port < allportcnt; port++) {
    pp = allports + port;

    if (pp->mini) {
      if (pp->macid == hi32) {
        pp->mini = 0;
        onemini++;
      } else minicnt++;
    }
  }
  info(0,"%u of %u miniports, %u single",minicnt,allportcnt,onemini);

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

  if (nmac) mkhist(macports,allportcnt,&macrange,Elemcnt(macivs),macivs,"macro ports",Info);

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

  all2full = alloc(allportcnt,ub4,0xff,"all2full",allportcnt);

  if (minicnt) minilst = alloc(minicnt,ub4,0,"minilst",minicnt);
  else minilst = NULL;

  info(0,"assigning %u+%u=%u ports",fullportcnt,nmac,portcnt);

  // full ports
  port = 0;
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
