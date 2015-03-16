// net.c - main network setup

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Initialize the network once at startup :

   create a pre-computed connectivity network used in search

   - Build connectivity matrix between any 2 full ports
     base matrix for direct (non-stop) hops
     derived matrix for each of n intermediate hops

     each matrix contains a list of possible trips from port A to port B
     the list is trimmed on heuristics, such as distance, cost, timing

   - Prepare various metrics used for heuristics
 */

#include <string.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "util.h"
#include "time.h"
#include "net.h"
#include "netn.h"
#include "netev.h"

#undef hdrstop

static const ub2 cnt0lim_part = 256; // todo configurable

static struct network gs_nets[Npart];

static struct gnetwork gs_gnet;

void ininet(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

struct network *getnet(ub4 part)
{
  error_ge(part,Npart);
  return gs_nets + part;
}

struct gnetwork *getgnet(void)
{
  return &gs_gnet;
}

// infer walk links
static int mkwalks(struct network *net)
{
  ub4 portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  struct port *ports,*pdep,*parr;
  char *dname,*aname;
  ub4 dist,lodist = hi32,hidist = 0;
  ub4 dep,arr,deparr,port2;
  ub4 walklimit = net->walklimit;
  ub4 walkspeed = net->walkspeed;  // geo's per hour
  struct eta eta;

  if (portcnt == 0) return error(0,"no ports for %u hops net",hopcnt);
  if (hopcnt == 0) return error(0,"no hops for %u port net",portcnt);

  if (walklimit < 2) {
    net->whopcnt = chopcnt;
    return info(0,"no walk links for %u m limit",walklimit * 10);
  }

  port2 = portcnt * portcnt;

  ports = net->ports;

  // geographical direct-line distance
  ub4 *dist0 = alloc(port2, ub4,0xff,"net0 geodist",portcnt);

  for (dep = 0; dep < portcnt; dep++) {
    pdep = ports + dep;
    if (pdep->valid == 0) continue;
//    error_zz(pdep->lat,pdep->lon);
  }
  for (dep = 0; dep < portcnt; dep++) {
    if (progress(&eta,"port %u of %u for \ah%u distance pairs",dep,portcnt,port2)) return 1;
    pdep = ports + dep;
    if (pdep->valid == 0) continue;
    dname = pdep->name;
    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;
      parr = ports + arr;
      if (parr->valid == 0) continue;
      aname = parr->name;
      error_eq_cc(pdep->gid,parr->gid,"%s %s",dname,aname);
      deparr = dep * portcnt + arr;
      if (pdep->lat && pdep->lat == parr->lat && pdep->lon && pdep->lon == parr->lon) {
        info(Iter,"ports %u-%u coloc %u,%u-%u,%u %s to %s",dep,arr,pdep->lat,pdep->lon,parr->lat,parr->lon,dname,aname);
        dist = 0;
      } else {
        dist = fgeodist(pdep,parr);
//        infocc(dist == 0 || dep == 0,0,"ports %u-%u dist %u %s to %s",dep,arr,dist,dname,aname);
      }
//      error_z(dist,arr);
      dist0[deparr] = dist;
      lodist = min(dist,lodist);
      hidist = max(dist,hidist);
    }
  }

  ub4 geohist[128];
  ub4 geohist2[64];
  ub4 cnt,iv,ivcnt = Elemcnt(geohist);
  ub4 iv2cnt = Elemcnt(geohist2);
  ub4 hidist2 = min(hidist,walklimit * 2);
  ub4 range =  max(1,hidist - lodist);
  ub4 range2 =  max(1,hidist2 - lodist);
  aclear(geohist);
  aclear(geohist2);
  for (dep = 0; dep < portcnt; dep++) {
    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;
      deparr = dep * portcnt + arr;
      dist = dist0[deparr];
      pdep = ports + dep;
      parr = ports + arr;
      if (dist == 1) info(0,"port dist %u %u-%u %s to %s",dist,dep,arr,pdep->name,parr->name);
      else if (dist == hi32) {
        geohist[ivcnt-1]++;
        geohist2[iv2cnt-1]++;
      } else {
        iv = ((dist - lodist) * ivcnt) / range;
        geohist[min(iv,ivcnt-1)]++;
        iv = ((dist - lodist) * iv2cnt) / range2;
        geohist2[min(iv,iv2cnt-1)]++;
      }
    }
  }
  info(0,"geodist range %u - %u",lodist,hidist);
  ub8 sumcnt = 0;
  for (iv = 0; iv < ivcnt; iv++) {
    cnt = geohist[iv];
    if (cnt == 0) continue;
    sumcnt += cnt;
    info(0,"%u \ag%u = \ah%u: %u %%",iv,lodist + (iv * range) / ivcnt,cnt,(ub4)((sumcnt * 100) / port2));
  }
  info(0,"walklimit \ag%u",walklimit);
  sumcnt = 0;
  for (iv = 0; iv < iv2cnt; iv++) {
    cnt = geohist2[iv];
    if (cnt == 0) continue;
    sumcnt += cnt;
    info(0,"%u \ag%u = \ah%u: %u pm",iv,lodist + (iv * range2) / iv2cnt,cnt,(ub4)((sumcnt * 1000) / port2));
  }

  ub4 *orgportsbyhop = net->portsbyhop;
  ub4 *orghopdist = net->hopdist;
  ub4 *orghopdur = net->hopdur;

#if 0
  ub1 *net0 = alloc(port2,ub1,0,"net net0",portcnt);
  for (hop = 0; hop < hopcnt; hop++) {
    dep = orgportsbyhop[hop * 2];
    arr = orgportsbyhop[hop * 2 + 1];
    if (dep == hi32 || arr == hi32) continue;
    net0[dep * portcnt + arr] = 1;
  }
#endif

  ub4 whop,whopcnt = 0;
  for (deparr = 0; deparr < port2; deparr++) {
    if (dist0[deparr] > walklimit) continue;
    whopcnt++;
  }
  info(0,"\ah%u inferred walk links below dist %u",whopcnt,walklimit);

  ub4 newhopcnt = chopcnt + whopcnt;

  ub4 *portsbyhop = alloc(newhopcnt * 2,ub4,0,"net portsbyhop",newhopcnt);
  memcpy(portsbyhop,orgportsbyhop,chopcnt * 2 * sizeof(ub4));

  ub4 *hopdist = alloc(newhopcnt,ub4,0,"net hopdist",newhopcnt);
  memcpy(hopdist,orghopdist,chopcnt * sizeof(ub4));

  ub4 *hopdur = alloc(newhopcnt,ub4,0,"net hopdur",newhopcnt);
  memcpy(hopdur,orghopdur,chopcnt * sizeof(ub4));

  ub4 hiwdist = 0,hiwhop = hi32;
  whop = chopcnt;
  for (deparr = 0; deparr < port2; deparr++) {
    dist = dist0[deparr];
    if (dist > walklimit) continue;

    dep = deparr / portcnt;
    arr = deparr % portcnt;
    portsbyhop[whop * 2] = dep;
    portsbyhop[whop * 2 + 1] = arr;

    if (walkspeed) hopdur[whop] = (max(dist,1) * 60) / walkspeed;
    else hopdur[whop] = 60 * 24 * 7;

    hopdist[whop] = dist;
    if (dist > hiwdist) { hiwdist = dist; hiwhop = whop; }
    whop++;
  }
  if (hiwhop < whop) {
    dep = portsbyhop[hiwhop * 2];
    arr = portsbyhop[hiwhop * 2 + 1];
    pdep = ports + dep;
    parr = ports + arr;
    info(0,"longest walk link dist %u hop %u %u-%u %s to %s",hiwdist,hiwhop,dep,arr,pdep->name,parr->name);
  }

  afree(dist0 ,"net0 geodist");

  net->whopcnt = whop;
  net->portsbyhop = portsbyhop;
  net->hopdist = hopdist;
  net->hopdur = hopdur;
  return 0;
}

// assess connectivity
static int conchk(struct network *net)
{
  ub4 portcnt = net->portcnt;
  ub4 vportcnt = net->vportcnt;

  ub4 port2 = portcnt * portcnt;

  struct port *pdep,*ports = net->ports;

  ub4 dep,arr,deparr;
  ub2 *cnts = net->con0cnt;

  ub1 *conns = alloc(portcnt,ub1,0,"net dotlinks",portcnt);

  ub4 prvconcnt,concnt = 0;
  ub4 iter = 0;

  // start with first hop
  for (deparr = 0; deparr < port2; deparr++) {
    if (cnts[deparr]) break;
  }
  dep = deparr / portcnt; arr = deparr % portcnt;
  conns[dep] = conns[arr] = 1;

  do {
    prvconcnt = concnt;

    info(0,"iter %u conns %u of %u",iter,concnt,vportcnt);

    for (dep = 0; dep < portcnt; dep++) {
      if (conns[dep] == 0) continue;

      deparr = dep * portcnt;
      for (arr = 0; arr < portcnt; arr++) {
        if (cnts[deparr + arr] == 0) continue;
        if (conns[arr] == 0) { conns[arr] = 1; concnt++; }
      }
    }

    for (arr = 0; arr < portcnt; arr++) {
      if (conns[arr] == 0) continue;

      for (dep = 0; dep < portcnt; dep++) {
        if (cnts[dep * portcnt + arr] == 0) continue;
        if (conns[dep] == 0) { conns[dep] = 1; concnt++; }
      }
    }
    info(0,"iter %u conns %u",iter,concnt);

  } while (concnt > prvconcnt);

  infocc(concnt < vportcnt,0,"%u ports not in group",vportcnt - concnt);

  for (dep = 0; dep < portcnt; dep++) {
    if (conns[dep]) continue;
    pdep = ports + dep;
    if (pdep->ndep == 0 && pdep->narr == 0) continue;
    info(0,"%u dep %u arr %u %s",dep,pdep->ndep,pdep->narr,pdep->name);
  }

  return 0;
}

// create 0-stop connectivity matrix and derived info.
// n-stop builds on this
static int mknet0(struct network *net)
{
  ub4 portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 whopcnt = net->whopcnt;
  ub4 partcnt = net->partcnt;

  struct port *ports,*pdep,*parr;
  struct hop *hops,*hp;

  char *dname,*aname;
  ub4 *portsbyhop;
  ub2 concnt,cntlim,gen,*con0cnt;
  ub4 ofs,*con0ofs;
  ub4 hop,l1,l2,*con0lst;
  ub4 dist,*lodists,*hopdist;
  ub4 dep,arr,port2,da,depcnt,arrcnt;
  ub4 rid;
  ub4 needconn,haveconn;
  ub2 iv;

  if (portcnt == 0) return error(0,"no ports for %u hops net",hopcnt);
  if (hopcnt == 0) return error(0,"no hops for %u port net",portcnt);
  error_lt(chopcnt,hopcnt);

  cntlim = cnt0lim_part;

  info(0,"init 0-stop connections for %u port %u hop network",portcnt,hopcnt);

  port2 = portcnt * portcnt;

  ports = net->ports;
  hops = net->hops;
  ub4 *choporg = net->choporg;

  portsbyhop = net->portsbyhop;

  con0cnt = alloc(port2, ub2,0,"net0 concnt",portcnt);
  con0ofs = alloc(port2, ub4,0xff,"net0 conofs",portcnt);

  con0lst = mkblock(net->conlst,whopcnt,ub4,Init1,"net0 0-stop conlst");

  ub1 *allcnt = alloc(port2, ub1,0,"net allcnt",portcnt);

  if (partcnt > 1) lodists = alloc(port2, ub4,0xff,"net0 lodist",portcnt);
  else lodists = NULL;

  ub4 *hoprids = alloc(whopcnt,ub4,0xff,"net hoprids",chopcnt);

  hopdist = net->hopdist;

  // create 0-stop connectivity
  // support multiple hops per port pair
  ub4 ovfcnt = 0,hicon = 0,hida = 0, nhopcnt = 0; 
  for (hop = 0; hop < whopcnt; hop++) {
    dep = portsbyhop[hop * 2];
    arr = portsbyhop[hop * 2 + 1];
    if (dep == hi32 || arr == hi32 || dep == arr) continue;

    if (hop < hopcnt) {
      hp = hops + hop;
      error_ne(dep,hp->dep);
      error_ne(arr,hp->arr);
      rid = hp->rid;
    } else if (hop < chopcnt) {
      hp = hops + choporg[hop * 2];
      rid = hp->rid;
    } else rid = hi32;

    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    pdep = ports + dep;
    parr = ports + arr;
    if (pdep->valid == 0 || parr->valid == 0) continue;

    dname = pdep->name;
    aname = parr->name;
    if (dep == arr) {
      warning(0,"hop %u dep == arr %u %s",hop,dep,dname);
      continue;
    }

    hoprids[hop] = rid;

    da = dep * portcnt + arr;

    dist = hopdist[hop];
    if (lodists) lodists[da] = min(lodists[da],dist);

    concnt = con0cnt[da];

    if (concnt >= cntlim) {
      ovfcnt++;
      continue;
    } else if (concnt == cntlim-1) {
      info(0,"connect overflow hop %u port %u-%u %s to %s",hop,dep,arr,dname,aname);
      concnt++;
      ovfcnt++;
    } else concnt++;
    nhopcnt++;
    if (concnt > hicon) { hicon = concnt; hida = da; }
    con0cnt[da] = concnt;

//    if (hop < hopcnt && dist != dist0[da]) warning(0,"hop %u dist %u vs %u",hop,dist,dist0[da]);
  }
  if (ovfcnt) warning(0,"limiting 0-stop net by \ah%u",ovfcnt);
  infocc(nhopcnt != whopcnt,0,"marked %u out of %u hops, skipped %u",nhopcnt,whopcnt,whopcnt - nhopcnt);

  dep = hida / portcnt; arr = hida % portcnt;
  pdep = ports + dep; parr = ports + arr;
  info(0,"highest conn %u between ports %u-%u %s to %s",hicon,dep,arr,pdep->name,parr->name);

  for (hop = 0; hop < whopcnt; hop++) {
    dep = portsbyhop[hop * 2];
    arr = portsbyhop[hop * 2 + 1];
    if (dep == hi32 || arr == hi32 || dep == arr) continue;
    da = dep * portcnt + arr;
    if (da != hida) continue;

    if (hop < hopcnt) {
      hp = hops + hop;
      info(0,"  hop %u rid %u rrid %u route %s",hop,hp->rid,hp->rrid,hp->name);
    } else if (hop < chopcnt) {
      l1 = choporg[hop * 2];
      l2 = choporg[hop * 2 + 1];
      hp = hops + l1;
      info(0,"  chop %u = %u-%u rid %u rrid %u route %s",hop,l1,l2,hp->rid,hp->rrid,hp->name);
    } else info(0," whop %u dist %u",hop,hopdist[hop]);
  }

  ofs = 0;
  needconn = haveconn = 0;

  for (dep = 0; dep < portcnt; dep++) {

    pdep = ports + dep;
    if (pdep->valid == 0) continue;

    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;
      parr = ports + arr;
      if (parr->valid == 0) continue;

      needconn++;

      da = dep * portcnt + arr;
      concnt = con0cnt[da];
      if (concnt == 0) continue;

      concnt = con0cnt[da];
      con0ofs[da] = ofs;
      ofs += concnt;
    }
  }

  net->lstlen[0] = ofs;

  // pass 2: fill
  info(0,"pass 2 0-stop %u hop net",hopcnt);
  nclear(con0cnt,port2); // accumulate back below
  for (hop = 0; hop < whopcnt; hop++) {
    dep = portsbyhop[hop * 2];
    arr = portsbyhop[hop * 2 + 1];
    if (dep == hi32 || arr == hi32 || dep == arr) continue;
    pdep = ports + dep;
    parr = ports + arr;
    if (pdep->valid == 0 || parr->valid == 0) continue;

    da = dep * portcnt + arr;
    gen = con0cnt[da];
    ofs = con0ofs[da];
    if (gen >= cntlim) continue;
    con0lst[ofs+gen] = hop;
    con0cnt[da] = (ub2)(gen + 1);
    allcnt[da] = 1;
  }

  for (dep = 0; dep < portcnt; dep++) {
    for (arr = 0; arr < portcnt; arr++) {
      da = dep * portcnt + arr;
      concnt = con0cnt[da];
      if (concnt) haveconn++;
    }
  }
  info(0,"  0-stop connectivity \ah%3u of \ah%3u  = %02u%%",haveconn,needconn,haveconn * 100 / max(needconn,1));

  // get connectivity stats
  ub4 hicnt = 0,hiport = 0;
  ub4 depstats[32];
  ub4 arrstats[32];
  ub4 depivs = Elemcnt(depstats) - 1;
  ub4 arrivs = Elemcnt(arrstats) - 1;

  aclear(arrstats);
  aclear(depstats);

  for (dep = 0; dep < portcnt; dep++) {
    depcnt = 0;
    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;
      depcnt += con0cnt[dep * portcnt + arr];
      if (depcnt > hicnt) { hicnt = depcnt; hiport = dep; }
    }
//    error_ne(depcnt,ports[dep].ndep);
    depstats[min(depivs,depcnt)]++;
  }
  for (iv = 0; iv <= depivs; iv++) {
    if (depstats[iv]) info(0,"%u port\as reaches %u port\as", depstats[iv], iv);
  }
  pdep = ports + hiport;
  info(0,"port %u is reached by %u ports %s",hiport,hicnt,pdep->name);

  hicnt = hiport = 0;
  for (arr = 0; arr < portcnt; arr++) {
    arrcnt = 0;
    for (dep = 0; dep < portcnt; dep++) {
      if (dep == arr) continue;
      arrcnt += con0cnt[dep * portcnt + arr];
      if (arrcnt > hicnt) { hicnt = arrcnt; hiport = arr; }
    }
//    error_ne(arrcnt,ports[arr].narr);
    arrstats[min(arrivs,arrcnt)]++;
  }
  for (iv = 0; iv <= arrivs; iv++) {
    if (arrstats[iv]) info(0,"%u port\as reached by %u port\as", arrstats[iv], iv);
  }
  parr = ports + hiport;
  info(0,"port %u reached by %u ports %s",hiport,hicnt,parr->name);

  net->con0cnt = con0cnt;
  net->con0ofs = con0ofs;

  net->allcnt = allcnt;

  net->concnt[0] = con0cnt;
  net->conofs[0] = con0ofs;

  net->lodist[0] = lodists; // only for partitioned

  net->hoprids = hoprids;

  net->needconn = needconn;

  return 0;
}

// create n-stop connectivity matrix and derived info
static int mk_netn(struct network *net,ub4 nstop)
{
  ub4 portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 whopcnt = net->whopcnt;
  struct port *ports,*pdep,*parr;
  block *lstblk;
  ub2 *concnt;
  ub4 *lst;
  ub4 ofs,*conofs;
  ub4 dep,arr,deparr;
  ub4 nstop1,cnt,nleg;
  int rv;

  // only fill n-stop if no (nstop-1) exists
  bool nilonly;

  // max #variants per [dep,arr]
  ub4 varlimit,var12limit;

  error_z(nstop,0);
  error_ge(nstop,Nstop);
  error_zz(portcnt,hopcnt);

  vrb0(0,"init %u-stop connections for %u port %u hop network",nstop,portcnt,whopcnt);

  // todo configurable
  switch (nstop) {
  case 1: nilonly = 0; varlimit = 64; var12limit = 256; break;
  case 2: nilonly = 0; varlimit = 16; var12limit = 64; break;
  case 3: nilonly = 1; varlimit = 8; var12limit = 64; break;
  default: nilonly = 1; varlimit = 2; var12limit = 32; break;
  }

  if (nilonly && net->needconn <= net->haveconn[nstop-1]) {
    return info(0,"skip %u-stop init on %u-stop coverage complete",nstop,nstop-1);
  }

  switch(nstop) {
  case 1: rv = mknet1(net,varlimit,var12limit,nilonly); break;
  case 2: rv = mknet2(net,varlimit,var12limit,nilonly); break;
  default: rv = mknetn(net,nstop,varlimit,var12limit,nilonly); break;
  }

  if (rv) return rv;

  if (net->lstlen[nstop] == 0) return 0;

  ports = net->ports;

  // get connectivity stats

  unsigned long doneconn,doneperc,leftcnt,needconn = net->needconn;
  ub4 n,da,nda = 0,port,hascon,hicon,arrcon,loarrcon,lodep = 0;
  ub4 tports[Nstop];
  ub4 gtports[Nstop];
  ub4 deparrs[16];
  ub4 nstops[16];
  ub4 lodeparrs[16];
  ub4 lonstops[16];
  ub4 ndacnt = Elemcnt(deparrs);
  struct port *pp;

  for (nda = 0; nda < ndacnt; nda++) {
    deparrs[nda] = lodeparrs[nda] = hi32;
    nstops[nda] = lonstops[nda] = 0;
  }

  doneconn = 0;
  loarrcon = hi32;

  for (dep = 0; dep < portcnt; dep++) {
    arrcon = 0;
    nda = 1;
    pdep = ports + dep;
    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;
      deparr = dep * portcnt + arr;
      hascon = 0;
      nstop1 = 0;
      while (nstop1 <= nstop) {
        concnt = net->concnt[nstop1];
        error_zp(concnt,nstop1);
        if (concnt[deparr]) { hascon = 1; break; }
        nstop1++;
      }
      if (hascon) {
        arrcon++;
//        allcnt[deparr] = 1;
        if (nstop1 >= nstops[0]) { nstops[0] = nstop1; deparrs[0] = deparr; }
      } else {
        if (nda < ndacnt) deparrs[nda++] = deparr;

#if 0
        // verify no dep-mid-arr exist
        for (mid = 0; mid < portcnt; mid++) {
          if (mid == dep || mid == arr) continue;
          depmid = dep * portcnt + mid; midarr = mid * portcnt + arr;
          if (con0cnt[depmid] && con0cnt[midarr]) warninfo(limited == 0,Iter,"no conn port %u-%u with mid %u exists",dep,arr,mid);
        }
#endif

      }
    }
    doneconn += arrcon;
    if (arrcon < loarrcon) {
      loarrcon = arrcon;
      lodep = dep;
      memcpy(lodeparrs,deparrs,ndacnt * sizeof(ub4));
      memcpy(lonstops,nstops,ndacnt * sizeof(ub4));
    }
    leftcnt = portcnt - arrcon - 1;
    if (leftcnt) {
      infovrb(nstop > 2,Notty,"port %u lacks %u connection\as %s",dep,(ub4)leftcnt,pdep->name);
      for (da = 1; da < nda; da++) {
        deparr = deparrs[da];
        if (deparr == hi32) break;
        arr = deparr % portcnt;
        parr = ports + arr;
        infovrb(nstop > 3,Notty,"port %u %s no %u-stop connection to %u %s",dep,pdep->name,nstop,arr,parr->name);
      }
    }
  }
  net->haveconn[nstop] = doneconn;
  error_gt(doneconn,needconn,0);
  doneperc = doneconn * 100 / needconn;
  if (doneperc > 98) info(0,"0-%u-stop connectivity \ah%3lu of \ah%3lu = %02u%%, left %lu", nstop,doneconn,needconn,(ub4)doneperc,needconn - doneconn);
  else info(0,"0-%u-stop connectivity \ah%3lu of \ah%3lu = %02u%%", nstop,doneconn,needconn,(ub4)doneperc);

  pdep = ports + lodep;
  leftcnt = portcnt - loarrcon - 1;
  if (leftcnt) info(0,"port %u lacks %u connection\as %s",lodep,(ub4)leftcnt,pdep->name);

  for (nda = 0; nda < ndacnt; nda++) {
    deparr = lodeparrs[nda];
    if (deparr == hi32) break;
    hicon = lonstops[nda];
    dep = deparr / portcnt;
    arr = deparr % portcnt;
    pdep = ports + dep;
    parr = ports + arr;
    concnt = net->concnt[hicon];
    if (concnt == NULL) return error(0,"%u stops cnt nil",hicon);
    cnt = concnt[deparr];
    info(0,"%u-%u %u vars at %u stops %s to %s",dep,arr,cnt,hicon,pdep->name,parr->name);
    if (cnt == 0) continue;
    nleg = hicon + 1;
    conofs = net->conofs[hicon];
    ofs = conofs[deparr];
    lstblk = net->conlst + hicon;
    lst = blkdata(lstblk,ofs * nleg,ub4);
    if (triptoports(net,lst,nleg,tports,gtports)) break;
    for (n = 0; n <= nleg; n++) {
      port = tports[n];
      if (port >= portcnt) warning(0,"port #%u %x",n,port);
      else {
        pp = ports + port;
        info(0,"port #%u %u %s",n,port,pp->name);
      }
    }
  }
  return 0;
}

// count connections within part
static ub2 hasconn(struct network *net,ub4 deparr)
{
  ub4 nstop = 0;
  ub2 **concnts = net->concnt,*cnts,cnt = 0;

  while (nstop <= net->histop && cnt == 0) {
    cnts = concnts[nstop++];
    if (cnts) cnt = cnts[deparr];
  }

  return cnt;
}

// get connections within part as mask per stop
static ub2 getconn(ub4 callee,struct network *net,ub4 deparr)
{
  ub4 nstop = 0;
  ub2 **concnts = net->concnt,*cnts,cnt;
  ub2 res = 0,mask = 0x80;

  enter(callee);
  error_ge(deparr,net->portcnt * net->portcnt);
  while (nstop <= net->histop) {
    cnts = concnts[nstop++];
    if (cnts) {
      cnt = cnts[deparr];
      if (cnt) res |= mask;
    }
    mask >>= 1;
  }
  leave(callee);
  return res;
}

// create and fill interpart connection xmap
static int mkxmap(ub4 callee,struct gnetwork *gnet)
{
  struct network *net,*tnet;
  ub4 partcnt = gnet->partcnt;
  ub4 portcnt,tportcnt,gportcnt = gnet->portcnt;
  ub4 part,tpart;
  ub4 gdep,garr,dep,arr,tdep,tarr,deparr,xdep,xarr;
  struct port *gpdep,*gparr,*gports = gnet->ports;
  ub1 *portparts = gnet->portparts;
  ub2 *xmappos,*xmapbase,*xmapabase,stopset,x;
  block *xpartdmap = &gnet->xpartdmap;
  block *xpartamap = &gnet->xpartamap;
  ub1 *conmask;
  ub4 gcnt,gi,partno;
  ub4 hascon,npxcon = 0,nxcon = 0;
  ub4 *gp2t;

  struct eta eta;

  if (partcnt < 2) return info(0,"no inter-partition maps for %u partition\as",partcnt);

  enter(callee);

/* have separate partitions with hi-conn or high-partmember ports
   each port ( with low conn or partmember? ) has list of conn to above
 */
  info0(0,"fill inter-partition reach maps pass 1");

  tpart = gnet->tpart;
  tnet = getnet(tpart);
  tportcnt = tnet->portcnt;
  conmask = tnet->conmask = alloc(tportcnt * tportcnt,ub1,0,"part topnet conn",tportcnt);

  gp2t = tnet->g2pport;

  for (tdep = 0; tdep < tportcnt; tdep++) {
    for (tarr = 0; tarr < tportcnt; tarr++) {
      deparr = tdep * tportcnt + tarr;
      if (tdep == tarr) { conmask[deparr] = 0x80; continue; }
      stopset = getconn(caller,tnet,deparr);
      if (stopset == 0) continue;
      conmask[deparr] = (ub1)stopset;
    }
  }

  ub4 xmaplen = tportcnt * gportcnt;

  info(0,"alloc xmap %u ports * %u tports",gportcnt,tportcnt);
  xmapbase = gnet->xpartdbase = mkblock(xpartdmap,xmaplen,ub2,Init0,"part xmap for %u parts", partcnt);
  xmapabase = gnet->xpartabase = mkblock(xpartamap,xmaplen,ub2,Init0,"part xmap for %u parts", partcnt);

  for (gdep = 0; gdep < gportcnt; gdep++) {
    progress(&eta,"port %u of %u in %u parts",gdep,gportcnt,partcnt);

    xmappos = xmapbase + gdep * tportcnt;

    hascon = 0;

    gpdep = gports + gdep;
    if (gpdep->tpart) {
      tdep = gp2t[gdep];
      for (tarr = 0; tarr < tportcnt; tarr++) {
        deparr = tdep * tportcnt + tarr;
        stopset = getconn(caller,tnet,deparr);
        if (stopset) {
          hascon = 1;
          x = xmappos[tarr];
          if (stopset > (x & 0xff)) xmappos[tarr] = stopset | (ub2)(tpart << 8);
        }
      }
      if (hascon) nxcon++;
      continue;
    }

    partno = 0;
    for (part = 0; part < tpart; part++) {
      if (portparts[gdep * partcnt + part] == 0) continue;

      net = getnet(part);
      portcnt = net->portcnt;
      dep = net->g2pport[gdep];
      error_ge(dep,portcnt);
      gcnt = net->tportcnt;
      for (gi = 0; gi < gcnt; gi++) {
        arr = net->tports[gi];
        error_ge(arr,portcnt);
        deparr = dep * portcnt + arr;
        stopset = getconn(caller,net,deparr);
        if (stopset) {
          hascon = 1;
          npxcon++;
          garr = net->p2gport[arr];
          xarr = tnet->g2pport[garr];
          error_ge(xarr,tportcnt);
          bound(xpartdmap,gdep * tportcnt + xarr,ub2);
          x = xmappos[xarr];
          if (stopset > (x & 0xff)) xmappos[xarr] = stopset | (ub2)(partno << 8);
        }
      } // each top in gdep.part
      partno++;
    } // each gdep.part
    if (hascon) {
      infocc(nxcon < 3,0,"dport %u has %u top conns %s",gdep,npxcon,gpdep->name);
      nxcon++;
    }

  } // each gdep

  info(0,"%u of %u deps with any top part connection, %u total conns",nxcon,gportcnt,npxcon);

  nxcon = npxcon = 0;
  for (garr = 0; garr < gportcnt; garr++) {
    progress(&eta,"port %u of %u in %u parts",garr,gportcnt,partcnt);

    xmappos = xmapabase + garr * tportcnt;
    hascon = 0;

    gparr = gports + garr;
    if (gparr->tpart) {

      tarr = gp2t[garr];
      for (tdep = 0; tdep < tportcnt; tdep++) {
        deparr = tdep * tportcnt + tarr;
        stopset = getconn(caller,tnet,deparr);
        if (stopset) {
          hascon = 1;
          x = xmappos[tdep];
          if (stopset > (x & 0xff)) xmappos[tdep] = stopset | (ub2)(tpart << 8);
        }
      }
      if (hascon) nxcon++;
      continue;
    }

    partno = 0;
    for (part = 0; part < tpart; part++) {
      if (portparts[garr * partcnt + part] == 0) continue;

      net = getnet(part);
      portcnt = net->portcnt;
      arr = net->g2pport[garr];
      error_ge(arr,portcnt);
      gcnt = net->tportcnt;
      for (gi = 0; gi < gcnt; gi++) {
        dep = net->tports[gi];
        error_ge(dep,portcnt);
        deparr = dep * portcnt + arr;
        stopset = getconn(caller,net,deparr);
        if (stopset) {
          hascon = 1;
          npxcon++;
          gdep = net->p2gport[dep];
          xdep = tnet->g2pport[gdep];
          error_ge(xdep,tportcnt);
          bound(xpartamap,garr * tportcnt + xdep,ub2);
          x = xmappos[xdep];
          if (stopset > (x & 0xff)) xmappos[xdep] = stopset | (ub2)(partno << 8);
        }
      } // each top in garr.part
      partno++;
    } // each garr.part
    if (hascon) {
      infocc(nxcon < 6,0,"aport %u has %u top conns %s",garr,npxcon,gparr->name);
      nxcon++;
    }
  } // each garr

  info(0,"%u of %u arrs with any top part connection, %u total conns",nxcon,gportcnt,npxcon);

  leave(callee);

  return 0;
}

// show cumulative global connectivity
static int showgconn(ub4 callee,struct gnetwork *gnet)
{
  struct network *tnet,*danet;
  struct port *gpdep,*gparr,*gports = gnet->ports;
  struct hop *hp,*hops = gnet->hops;
  char *dname,*aname;
  ub1 *portparts = gnet->portparts;
  ub4 partcnt = gnet->partcnt;
  ub4 daportcnt,gportcnt = gnet->portcnt;
  ub4 hopcnt = gnet->hopcnt;
  ub4 ridcnt = gnet->ridcnt;
  ub4 gdep,garr,dep,arr,tdep,tarr,tdmid,tamid,gdmid,gamid;
  ub4 part;
  ub4 rid,hop;

  ub4 xconn;
  ub4 tpart;

  ub4 deparr;
  ub4 *tp2g,*gp2t;
  ub4 tportcnt;
  ub8 gconn = 0,gxconn = 0,gxconn1 = 0,gxconn2 = 0;
  ub4 lconn;
  ub4 sample;

  struct eta eta;

  ub2 *xmap,*xamap,*xmapdbase,*xmapabase,xm,xam;
  ub1 *tmap;
  block *xpartdmap = &gnet->xpartdmap;
  block *xpartamap = &gnet->xpartamap;

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    rid = hp->rid;
    if (rid == hi32) continue;
    error_ge(rid,ridcnt);
    infocc(rid == 0,0,"hop %u rid 0",hop);
  }

  if (gportcnt < 2) return info(0,"skip global conn for %u ports net",gportcnt);

  enter(callee);

  if (partcnt > 1) {
    xmapdbase = blkdata(xpartdmap,0,ub2);
    xmapabase = blkdata(xpartamap,0,ub2);
  } else {
    xmapdbase = xmapabase = NULL;
  }

  tpart = gnet->tpart;
  tnet = getnet(tpart);
  tportcnt = tnet->portcnt;
  tmap = tnet->conmask;
  tp2g = tnet->p2gport;
  gp2t = tnet->g2pport;

  if (gportcnt > 500) {
    sample = gportcnt / 200;
    info(0,"sampling connectivity in %u steps",sample);
  } else sample = 1;

  gdmid = gamid = 0;

  info(0,"global connectivity, sample %u",sample);

  for (gdep = 0; gdep < gportcnt; gdep += sample) {

    if (gdep == 0) progress(&eta,"port %u of %u in %u-part global connect",gdep,gportcnt,partcnt);
    else if (progress(&eta,"port %u of %u in global connect : \ah%lu + \ah%lu",gdep,gportcnt,gconn,gxconn)) return 1;

    gpdep = gports + gdep;
    dname = gpdep->name;

    xmap = xmapdbase + gdep * tportcnt;

    for (garr = 0; garr < gportcnt; garr += sample) {

      if (garr == gdep) continue;

      gparr = gports + garr;
      aname = gparr->name;

      lconn = xconn = 0;
      part = 0;
      while (part < partcnt && lconn == 0) {
        if (portparts[gdep * partcnt + part] && portparts[garr * partcnt + part]) {
          danet = getnet(part);
          dep = danet->g2pport[gdep];
          arr = danet->g2pport[garr];
          daportcnt = danet->portcnt;
          error_ge(dep,daportcnt);
          error_ge(arr,daportcnt);
          lconn = hasconn(danet,dep * daportcnt + arr);
        }
        part++;
      }
      if (lconn) {
        gconn++;
        infocc(gconn < 6,0,"local conn %u-%u %s to %s",gdep,garr,dname,aname);
        continue;

      } else if (partcnt == 1) {
        continue;

      } else if (gpdep->tpart) { // dep in top, arr not

        error_zp(xmapabase,partcnt);
        xamap = xmapabase + garr * tportcnt;
        tdep = gp2t[gdep];

        if (xamap[tdep]) {
          gxconn++; gxconn1++;
          infocc(gxconn < 6,0,"interpart-t1 conn %u-%u via %u %s to %s",gdep,garr,gamid,dname,aname);
          continue;
        }

      } else if (gparr->tpart) { // arr in top, dep not

        error_zp(xmapdbase,partcnt);
        tarr = gp2t[garr];

        if (xmap[tarr]) {
          gxconn++; gxconn2++;
          infocc(gxconn < 3,0,"interpart-t2 conn %u-%u via %u-%u %s to %s",gdep,garr,gdmid,gamid,dname,aname);
          continue;
        }
      }

      // no shared part, no conn above: go through topnet
      xamap = xmapabase + garr * tportcnt;

      for (tdmid = 0; tdmid < tportcnt; tdmid++) {
        xm = xmap[tdmid];
        if (xm == 0) continue;
        gdmid = tp2g[tdmid];
        for (tamid = 0; tamid < tportcnt; tamid++) {
          if (tdmid == tamid) continue;

          xam = xamap[tamid];
          if (xam == 0) continue;

          deparr = tdmid * tportcnt + tamid;
          if (tmap[deparr] == 0) continue;
          gamid = tp2g[tamid];

          xconn = 1;
          break;
        }
        if (xconn) break;
      }
      if (xconn) {
        gxconn++;
        infocc(gxconn < 3,0,"interpart conn %u-%u via %u-%u %s to %s",gdep,garr,gdmid,gamid,dname,aname);
      }

    } // each arr

  } // each dep

  ub8 gneedconn = gportcnt * (gportcnt - 1);
  ub8 sample2 = sample * sample;
  if (sample == 1) info(0,"global connectivity \ah%lu+\ah%lu of \ah%lu = %lu%%",gconn,gxconn,gneedconn,(gconn + gxconn) * 100 / gneedconn);
  else info(0,"global connectivity ~\ah%lu+\ah%lu of \ah%lu ~ %lu%%",gconn * sample2,gxconn * sample2,gneedconn,(gconn + gxconn) * sample2 * 100 / gneedconn);
  info(0,"\ah%lu interpart 1 \ah%lu interpart 2",gxconn1 * sample2,gxconn2 * sample2);
  leave(callee);

  return 0;
}

// initialize basic network, and connectivity for each number of stops
// the number of stops need to be determined such that all port pairs are reachable
int mknet(ub4 maxstop)
{
  ub4 portcnt,hopcnt;
  ub4 nstop,histop,allhistop = hi32,part,partcnt;
  int rv,netok = 0;
  struct gnetwork *gnet = getgnet();
  struct network *net;
  int doconchk = globs.engvars[Eng_conchk];

  if (dorun(FLN,Runmknet,0) == 0) return 0;

  partcnt = gnet->partcnt;
  if (partcnt == 0) return warn(0,"no partitions for %u ports net",gnet->portcnt);

  for (part = 0; part < partcnt; part++) {

    info(0,"mknet partition %u of %u",part,partcnt);
    net = getnet(part);

    portcnt = net->portcnt;
    hopcnt = net->hopcnt;

    if (portcnt == 0) { info0(0,"skip mknet on 0 ports"); continue; }
    if (hopcnt == 0) { info0(0,"skip mknet on 0 hops"); continue; }

    if (partcnt > 1) msgprefix(0,"p%u/%u ",part,partcnt);

    rv = mkwalks(net);
    if (rv) return msgprefix(1,NULL);

    if (dorun(FLN,Runnet0,0)) {
      if (mknet0(net)) return msgprefix(1,NULL);
      netok = 1;

      if (doconchk) rv = conchk(net);
      if (rv) return msgprefix(1,NULL);
    } else continue;

    histop = maxstop;
//    if (net->istpart) histop++;
    limit_gt(histop,Nstop,0);

    if (histop && dorun(FLN,Runnetn,0)) {
      if (mksubevs(net)) return msgprefix(1,NULL);

      for (nstop = 1; nstop <= histop; nstop++) {
        if (mk_netn(net,nstop)) return msgprefix(1,NULL);
        info(0,"nstop %u lstlen %lu",nstop,net->lstlen[nstop]);
        if (net->lstlen[nstop] == 0) break;
        net->histop = nstop;
      }
      info(0,"partition %u static network init done",part);
      allhistop = min(allhistop,net->histop);
      rmsubevs(net);

    } else {
      info(0,"partition %u no n-stop static network init",part);
      allhistop = 0;
    }
    msgprefix(0,NULL);

  } // each part

  if (netok == 0) return 0;

  gnet->histop = allhistop;

  if (mkxmap(caller,gnet)) return 1;

  if (showgconn(caller,gnet)) return 1;

  globs.netok = 1;
  return 0;
}

int showconn(struct port *ports,ub4 portcnt,int local)
{
  ub4 nodep,noarr,nodeparr,oneroute,oneroutes;
  ub4 port,ndep,narr,ngdep,ngarr,n;
  ub4 *drids,*arids;
  struct port *pp;

  ub4 constats[256];
  ub4 depstats[256];
  ub4 arrstats[256];
  ub4 statmax = Elemcnt(depstats) - 1;

  aclear(constats);
  aclear(depstats);
  aclear(arrstats);
  nodep = noarr = nodeparr = oneroutes = 0;

  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    ndep = pp->ndep; narr = pp->narr;
    ngdep = pp->ngdep; ngarr = pp->ngarr;
    drids = pp->drids; arids = pp->arids;
    oneroute = 1;
    if (ndep == 0 && narr == 0) {
      if (local && (ngdep | ngarr)) warning(0,"port %u is unconnected, global has %u dep %u arr %s",port,ngdep,ngarr,pp->name);
      else if (!local) info(0,"port %u is unconnected %s",port,pp->name);
      nodeparr++;
      oneroute = 0;
    } else if (ndep == 0) {
      if (local && ngdep) warning(0,"port %u has 0 deps %u arr\as, global has %u %s",port,narr,ngdep,pp->name);
      else if (!local) info(0,"port %u has no deps - %s",port,pp->name);
      nodep++;
      if (narr > 1 && arids[0] != arids[1]) oneroute = 0;
    } else if (narr == 0) {
      if (local && ngarr) warning(0,"port %u has %u dep\as 0 arrs, global has %u %s",port,ndep,ngarr,pp->name);
      else if (!local) info(0,"port %u has no arrs - %s",port,pp->name);
      noarr++;
      if (ndep > 1 && drids[0] != drids[1]) oneroute = 0;
    } else {
      if (ndep > 1 && drids[0] != drids[1]) oneroute = 0;
      if (narr > 1 && arids[0] != arids[1]) oneroute = 0;
    }
    if (ndep < 16 && narr < 16) constats[(ndep << 4) | narr]++;
    if (narr) depstats[min(ndep,statmax)]++;
    if (ndep) arrstats[min(narr,statmax)]++;
    oneroutes += oneroute;
  }
  if (nodeparr) genmsg(local ? Info : Warn,0,"%u of %u ports without connection",nodeparr,portcnt);
  if (nodep) info(0,"%u of %u ports without departures",nodep,portcnt);
  if (noarr) info(0,"%u of %u ports without arrivals",noarr,portcnt);
  for (ndep = 0; ndep < (local ? 3 : 4); ndep++) {
    for (narr = 0; narr < (local ? 3 : 4); narr++) {
      n = constats[(ndep << 4) | narr];
      if (n) info(0,"%u port\as with %u dep\as and %u arr\as", n,ndep,narr);
    }
  }
  info0(0,"");
  for (ndep = 0; ndep < (local ? 16 : 32); ndep++) {
    n = depstats[ndep];
    if (n) info(0,"%u port\as with %u dep\as and 1+ arrs", n,ndep);
  }
  n = depstats[statmax];
  if (n) info(0,"%u port\as with %u+ deps and 1+ arrs", n,statmax);
  for (narr = 0; narr < (local ? 16 : 32); narr++) {
    n = arrstats[narr];
    if (n) info(0,"%u ports with %u arr\as and 1+ deps", n,narr);
  }
  n = arrstats[statmax];
  if (n) info(0,"%u port\as with %u+ arrs and 1+ deps", n,statmax);
  info(0,"%u of %u ports on a single route",oneroutes,portcnt);
  return 0;
}

// check whether a triplet passes thru the given ports
void checktrip_fln(struct network *net,ub4 *legs, ub4 nleg,ub4 dep,ub4 arr,ub4 dist,ub4 fln)
{
  ub4 legno,legno2,leg,leg2,arr0,arr1,arr2,dep1,dep2,cdist;
  ub4 whopcnt = net->whopcnt;
  ub4 *portsbyhop = net->portsbyhop;
  ub4 *hopdist = net->hopdist;

  error_eq_fln(arr,dep,"arr","dep",fln);

  for (legno = 0; legno < nleg; legno++) {
    leg = legs[legno];
    error_ge_fln(leg,whopcnt,"hop","hopcnt",fln);
  }
  if (nleg > 2) {
    for (legno = 0; legno < nleg; legno++) {
      leg = legs[legno];
      dep1 = portsbyhop[leg * 2];
      arr1 = portsbyhop[leg * 2 + 1];
      for (legno2 = legno+1; legno2 < nleg; legno2++) {
        leg2 = legs[legno2];
        dep2 = portsbyhop[leg2 * 2];
        arr2 = portsbyhop[leg2 * 2 + 1];
        error_eq_fln(leg,leg2,"leg1","leg2",fln);
        error_eq_fln(dep1,dep2,"dep1","dep2",fln);
        error_eq_fln(dep1,arr2,"dep1","arr2",fln);
        if (legno2 != legno + 1) error_eq_fln(arr1,dep2,"arr1","dep2",fln);
        error_eq_fln(arr1,arr2,"arr1","arr2",fln);
      }
    }
  }

  leg = legs[0];
  dep1 = portsbyhop[leg * 2];
  arr0 = portsbyhop[leg * 2 + 1];
  error_ne_fln(dep1,dep,"hop.dep","dep",fln);

  cdist = hopdist[leg];

  leg = legs[nleg-1];
  arr1 = portsbyhop[leg * 2 + 1];

  error_ne_fln(arr1,arr,"hop.arr","arr",fln);

  for (legno = 1; legno < nleg; legno++) {
    leg = legs[legno];
    dep1 = portsbyhop[leg * 2];
    error_ne_fln(arr0,dep1,"prv.arr","dep",fln);
    arr0 = portsbyhop[leg * 2 + 1];
    cdist += hopdist[leg];
  }
  if (dist != hi32) error_ne_fln(dist,cdist,"dist","cdist",fln);
}

// check whether a triplet passes thru the given ports
void checktrip3_fln(struct network *net,ub4 *legs, ub4 nleg,ub4 dep,ub4 arr,ub4 via,ub4 dist,ub4 fln)
{
  ub4 legno,leg,hdep,harr;
  ub4 *portsbyhop = net->portsbyhop;
  int hasvia = 0;

  error_lt_fln(nleg,2,"nleg","2",fln);
  checktrip_fln(net,legs,nleg,dep,arr,dist,fln);
  if (via == dep) errorfln(fln,Exit,FLN,"via %u == dep",via);
  if (via == arr) errorfln(fln,Exit,FLN,"via %u == arr",via);

  for (legno = 1; legno < nleg; legno++) {
    leg = legs[legno];
    hdep = portsbyhop[leg * 2];
    harr = portsbyhop[leg * 2 + 1];
    if (hdep == via) hasvia = 1;
  }
  if (hasvia == 0) errorfln(fln,Exit,FLN,"via %u not visited for %u-%u",via,dep,arr);
}

int triptoports_fln(ub4 fln,struct network *net,ub4 *trip,ub4 triplen,ub4 *ports,ub4 *gports)
{
  ub4 leg,l,dep,arr = hi32;
  ub4 whopcnt = net->whopcnt;
  ub4 portcnt = net->portcnt;

  error_ge(triplen,Nleg);

  for (leg = 0; leg < triplen; leg++) {
    l = trip[leg];
    if (l >= whopcnt) return errorfln(fln,0,FLN,"leg %u hop %u above %u",leg,l,whopcnt);
    dep = net->portsbyhop[2 * l];
    error_ge(dep,portcnt);
    if (leg) {
      if (dep != arr) return errorfln(fln,0,FLN,"leg %u hop %u dep %u not connects to preceding arr %u",leg,l,dep,arr);
    }
    error_ge(dep,net->portcnt);
    arr = net->portsbyhop[2 * l + 1];
    error_ge(arr,portcnt);
    ports[leg] = dep;
    gports[leg] = net->p2gport[dep];
  }
  ports[triplen] = arr;
  gports[triplen] = net->p2gport[arr];
//  if (triplen < Nstop) ports[triplen+1] = gports[triplen+1] = hi32;
  return 0;
}

int gtriptoports(struct gnetwork *gnet,ub4 udep,ub4 uarr,ub4 usrdep,ub4 usrarr,struct trip *ptrip,char *buf,ub4 buflen,ub4 *ppos,ub4 utcofs)
{
  struct network *net;
  struct port *pdep,*parr,*gports = gnet->ports;
  struct sport *psdep,*psarr,*gsports = gnet->sports;
  struct hop *hops,*hp,*hp2;
  struct route *rp,*routes = gnet->routes;
  ub4 *trip = ptrip->trip;
  const char *name,*rname,*dname,*aname,*mode = "";
  const char *suffix;
  ub4 rid = hi32,rrid,tid;
  ub4 part,leg,prvleg,ghop,l,l1 = 0,l2 = 0,dep,arr = hi32,deparr,gdep,garr = hi32;
  ub4 sdep,sarr,srdep,srarr;
  ub4 tdep,tarr,thop,txtime,prvtarr = 0;
  ub4 dist,dist0,dt;
  ub4 gportcnt = gnet->portcnt;
  ub4 gsportcnt = gnet->sportcnt;
  ub4 partcnt = gnet->partcnt;

  ub4 hopcnt,whopcnt,chopcnt;
  ub4 portcnt,chaincnt;
  ub4 *portsbyhop;
  ub4 *hopdist;
  ub4 *choporg;
  ub4 *p2g;
  ub4 pos = *ppos;
  ub4 triplen = ptrip->len;
  ub4 *ports = ptrip->port;
  double dlat,dlon,alat,alon,fdist;
  ub4 walkspeed = gnet->walkspeed;  // geo's per hour
  double deplat,deplon,arrlat,arrlon,prvarrlat,prvarrlon,srdist;
  ub4 sdist;

  if (triplen == 0) { // trivial case: within same parent group
    if (udep == uarr && usrdep == usrarr) return 1;
    else if (udep != uarr) return warning(0,"nil trip for %u port net",gportcnt);
    pdep = gports + udep;
    if (usrdep != hi32) {
      sdep = pdep->subofs + usrdep;
      psdep = gsports + sdep;
      dname = psdep->name;
      dlat = psdep->rlat; dlon = psdep->rlon;
    } else {
      dname = pdep->name;
      dlat = pdep->rlat; dlon = pdep->rlon;
    }
    if (usrarr != hi32) {
      sarr = pdep->subofs + usrarr;
      psarr = gsports + sarr;
      aname = psarr->name;
      alat = psarr->rlat; alon = psarr->rlon;
    } else {
      aname = pdep->name;
      alat = pdep->rlat; alon = pdep->rlon;
    }

    fdist = geodist(dlat,dlon,alat,alon);
    dist = (ub4)fdist;
    dt = (dist * 60) / walkspeed;

    pos += mysnprintf(buf,pos,buflen,"sum\t\at%u\t\ag%u\t%u\t%u\t%s-%u\n",dt,dist,0,0,"t",0);

    pos += mysnprintf(buf,pos,buflen,"trip\t\t%s\t\t# %u\n",dname,1);
    pos += mysnprintf(buf,pos,buflen,"trip\t\twalk\t\ag%u\n",dist);

    pos += mysnprintf(buf,pos,buflen,"trip\t\t%s\n",aname);

    *ppos = pos;

    info(0,"%s",buf);
    return 0;
  }

  error_ge(triplen,Nxleg);

  pos += mysnprintf(buf,pos,buflen,"%s  tz = utc\au%u\n",ptrip->desc,utcofs);

  parr = NULL;
  arrlat = arrlon = 0;

  for (leg = 0; leg < triplen; leg++) {
    part = trip[leg * 2];
    error_ge_cc(part,partcnt,"leg %u of %u",leg,triplen);
    net = getnet(part);
    hops = net->hops;
    hopcnt = net->hopcnt;
    hopdist = net->hopdist;
    whopcnt = net->whopcnt;
    chopcnt = net->chopcnt;
    portcnt = net->portcnt;
    chaincnt = net->chaincnt;
    portsbyhop = net->portsbyhop;
    choporg = net->choporg;
    p2g = net->p2gport;

    l = trip[leg * 2 + 1];
    if (l >= whopcnt) return error(0,"part %u leg %u hop %u >= %u",part,leg,l,whopcnt);
    dep = portsbyhop[2 * l];
    error_ge(dep,portcnt);
    gdep = p2g[dep];
    error_ge(gdep,gportcnt);
    pdep = gports + gdep;
    dist = hopdist[l];
    sdep = sarr = hi32;
    if (l < hopcnt) {
      hp = hops + l;
      name = hp->name; rid = hp->rid; rrid = hp->rrid; ghop = hp->gid;
    } else if (l < chopcnt) {
      l1 = choporg[2 * l];
      l2 = choporg[2 * l + 1];
      if (l1 >= hopcnt) return error(0,"part %u compound leg %u = %u-%u >= %u",part,l,l1,l2,hopcnt);
      hp = hops + l1;
      name = hp->name; rid = hp->rid; rrid = hp->rrid; ghop = hp->gid;
      hp2 = hops + l2;
    } else {
      hp = NULL;
      name = "walk"; rid = rrid = ghop = hi32;
    }

    if (leg) {
      if (gdep != garr) {
        prvleg = leg - 1;
        if (l < whopcnt) return error(0,"leg %u hop %u.%u dep %u not connects to preceding arr %u.%u %s vs %s route %s",leg,part,l,gdep,trip[prvleg * 2],garr,pdep->name,parr->name,name);
        else return error(0,"leg %u chop %u.%u = %u-%u dep %u not connects to preceding arr %u.%u %s vs %s route %s",leg,part,l,l1,l2,gdep,trip[prvleg * 2],garr,pdep->name,parr->name,name);
      }
    } else if (gdep != udep) return error(0,"trip starting with %u, not inital dep %u",gdep,udep);

    arr = portsbyhop[2 * l + 1];
    error_ge(arr,portcnt);
    ports[leg] = gdep;
    garr = p2g[arr];
    error_ge(garr,gportcnt);
    parr = gports + garr;

    prvarrlat = arrlat; prvarrlon = arrlon;

    if (leg == 0) srdep = usrdep; else srdep = ptrip->srdep[leg];
    if (leg == triplen - 1) srarr = usrarr; else srarr = ptrip->srarr[leg];
    if (srdep != hi32 && srdep >= pdep->subcnt) {
      warn(Notty,"leg %u srdep %u subcnt %u",leg,srdep,pdep->subcnt);
      srdep = hi32;
    }
    if (srdep != hi32) {
      sdep = pdep->subofs + srdep;
      if (sdep >= gsportcnt) {
        warn(Notty,"sdep %u sportcnt %u",sdep,gsportcnt);
        srdep = hi32;
      }
    }
    if (srdep != hi32) {
      psdep = gsports + sdep;
      dname = psdep->name;
      deplat = psdep->rlat;
      deplon = psdep->rlon;
      vrb0(0,"dname %s for srdep %u",dname,srdep);
    } else {
      dname = pdep->name;
      deplat = pdep->rlat;
      deplon = pdep->rlon;
    }

    if (srarr != hi32 && srarr >= parr->subcnt) {
      warn(Notty,"srarr %u subcnt %u",srarr,parr->subcnt); // todo
      srarr = hi32;
    }
    if (srarr != hi32) {
      sarr = parr->subofs + srarr;
      if (sarr >= gsportcnt) {
        warn(Notty,"sarr %u sportcnt %u",sarr,gsportcnt);
        srarr = hi32;
      }
    }
    if (srarr != hi32) {
      psarr = gsports + sarr;
      aname = psarr->name;
      arrlat = psarr->rlat;
      arrlon = psarr->rlon;
      vrb0(0,"aname %s for srarr %u",aname,srarr);
    } else {
      aname = parr->name;
      arrlat = parr->rlat;
      arrlon = parr->rlon;
    }

    deparr = dep * portcnt + arr;
    dist0 = fgeodist(pdep,parr);
    tdep = ptrip->t[leg];
    tarr = tdep ? tdep + ptrip->dur[leg] : 0;
    tid = ptrip->tid[leg];
    if (rid != hi32) {
      rp = routes + rid;
      rname = rp->name;
      switch(rp->kind) {
      case Airdom: mode = "plane-dom"; break;
      case Airint: mode = "plane-int"; break;
      case Rail: mode = "train"; break;
      case Bus: mode = "bus"; break;
      case Ferry: mode = "ferry"; break;
      case Walk: mode = "walk";
      case Unknown: case Kindcnt:  mode = "unknown";
      }
    } else { rname = "(unnamed)"; mode = ""; }
    if (ptrip->info[leg] & 1) suffix = " *";
    else suffix = "";
    if (l < hopcnt) info(0,"leg %u hop %u dep %u.%u at \ad%u arr %u at \ad%u %s to %s route %s r.rid %u.%u tid %u %s%s",leg,ghop,part,gdep,tdep,garr,tarr,pdep->name,parr->name,rname,rrid,rid,tid,mode,suffix);
    else if (l < chopcnt) {
      hp2 = hops + l2;
      noexit error_ne(rid,hp2->rid);
      if (tdep && tid >= chaincnt) error(0,"tid %u above %u",tid,chaincnt);
      noexit error_zp(hp,l);
      info(0,"leg %u chop %u-%u dep %u.%u at \ad%u arr %u at \ad%u %s to %s route %s r.rid %u.%u tid %u %s%s",leg,hp->gid,hp2->gid,part,gdep,tdep,garr,tarr,pdep->name,parr->name,rname,rrid,rid,tid,mode,suffix);
    } else info(0,"leg %u whop %u dep %u.%u at \ad%u arr %u at \ad%u %s to %s %s",leg,l,part,gdep,tdep,garr,tarr,pdep->name,parr->name,mode);

    // dep
    txtime = 0;
    if (leg) { // add transfer time
      srdist = geodist(deplat,deplon,prvarrlat,prvarrlon);
      sdist = (ub4)srdist;
      if (tdep && prvtarr) {
        if (tdep >= prvtarr) txtime = tdep - prvtarr;
        else warn(0,"leg %u depart \ad%u before previous arrival \ad%u",leg,tdep,prvtarr);
      }
      if (tdep) pos += mysnprintf(buf,pos,buflen,"trip\t\ad%u\t%s%s\t\ag%u\t\at%u\t# %u\n",min2lmin(tdep,utcofs),dname,suffix,sdist,txtime,leg+1);
      else pos += mysnprintf(buf,pos,buflen,"trip\t\t%s%s\t\ag%u\t\t# %u\n",dname,suffix,sdist,leg+1);
    } else {
      if (tdep) pos += mysnprintf(buf,pos,buflen,"trip\t\ad%u\t%s%s\t\t\t# 1\n",min2lmin(tdep,utcofs),dname,suffix);
      else pos += mysnprintf(buf,pos,buflen,"trip\t\t%s%s\t\t\t# 1\n",dname,suffix);
    }

    // route
    if (tdep && tarr && tarr >= tdep) thop = tarr - tdep;
    else thop = 0;
    if (rid == hi32) pos += mysnprintf(buf,pos,buflen,"trip\t\t%s",name);
    else pos += mysnprintf(buf,pos,buflen,"trip\t%s\t%s",mode,rname);
    pos += mysnprintf(buf,pos,buflen,"\t\at%u",thop);
    if (dist != dist0) pos += mysnprintf(buf,pos,buflen,"\t\ag%u\t# (direct \ag%u)\n",dist,dist0);
    else pos += mysnprintf(buf,pos,buflen,"\t\ag%u\n",dist);

    // arr
    if (tarr) pos += mysnprintf(buf,pos,buflen,"trip\t\ad%u\t%s\n",min2lmin(tarr,utcofs),aname);
    else pos += mysnprintf(buf,pos,buflen,"trip\t\t%s\n",aname);
    prvtarr = tarr;

  } // each leg

  *ppos = pos;
  noexit error_ge(garr,gportcnt);
  ports[triplen] = garr;
  return 0;
}

ub4 fgeodist(struct port *pdep,struct port *parr)
{
  double dlat = pdep->rlat,dlon = pdep->rlon,alat = parr->rlat,alon = parr->rlon;

  if (pdep->lat == 0 || pdep->lon == 0 || parr->lat == 0 || parr->lon == 0) return 50000;

  double fdist = geodist(dlat,dlon,alat,alon);
  if (fdist > 2) return (ub4)fdist;

  ub4 dep = pdep->id;
  ub4 arr = parr->id;
  char *dname = pdep->name;
  char *aname = parr->name;
  double x = 180 / M_PI;
  info(0,"port %u-%u distance %e for %f,%f - %f,%f %s to %s",dep,arr,fdist,dlat * x,dlon * x,alat * x,alon * x,dname,aname);
  return (ub4)fdist;
}
