// net.c - main network setup

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Initialize the network once at startup :

   create a pre-computed connectivity network used in search

   - Separate ports in full and minor.
     Full ports have enough connectivity to include in a (full x full) matrix.
     Minor ports connect only to one or two single transfer stops.
     Actual plannig is done on these transfer ports.

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
#include "bitfields.h"
#include "net.h"
#include "compound.h"

#undef hdrstop

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

#define Geohist (20+2)

#define Watches 1
static ub4 watches = 0;
static ub4 watch_dep[Watches] = { 20 };
static ub4 watch_arr[Watches] = { 3 };

// todo configurable
static const ub4 walklimit = 7;
static const ub4 walkspeed = 5;  // minutes per dist unit

static ub2 cnt0lim_part = 256; // todo configurable

// infer walk links
static int mkwalks(struct network *net)
{
  ub4 portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;

  struct port *ports,*pdep,*parr;
  struct hop *hops,*hp;
  struct range distrange;

  char *dname,*aname;
  ub4 hop;
  ub4 dist,lodist = hi32,hidist = 0;
  double fdist;
  ub4 dep,arr,deparr,port2,depcnt,arrcnt;
  ub4 needconn,haveconn,watch;
  struct eta eta;

  if (portcnt == 0) return error(0,"no ports for %u hops net",hopcnt);
  if (hopcnt == 0) return error(0,"no hops for %u port net",portcnt);

  port2 = portcnt * portcnt;

  ports = net->ports;
  hops = net->hops;

  // geographical direct-line distance
  ub4 *dist0 = alloc(port2, ub4,0xff,"net0 geodist",portcnt);

  info(0,"calculating \ah%u distance pairs", port2);
  for (dep = 0; dep < portcnt; dep++) {
    pdep = ports + dep;
    error_zz(pdep->lat,pdep->lon);
  }
  for (dep = 0; dep < portcnt; dep++) {
    error_ge(dep,portcnt);
    pdep = ports + dep;
    dname = pdep->name;
    for (arr = 0; arr < portcnt; arr++) {
      error_ge(arr,portcnt);
      if (dep == arr) continue;
      parr = ports + arr;
      aname = parr->name;
      error_eq_cc(pdep->gid,parr->gid,"%s %s",dname,aname);
      deparr = dep * portcnt + arr;
      if (pdep->lat == parr->lat &&pdep->lon == parr->lon) {
        info(Iter,"ports %u-%u coloc %u,%u-%u,%u %s to %s",dep,arr,pdep->lat,pdep->lon,parr->lat,parr->lon,dname,aname);
        dist = 0;
      } else {
        dist = fgeodist(pdep,parr);
      }
//      error_z(dist,arr);
      dist0[deparr] = dist;
      lodist = min(dist,lodist);
      hidist = max(dist,hidist);
    }
  }
  info(0,"done calculating \ah%u distance pairs", port2);

  ub4 geohist[64];
  ub4 iv,ivcnt = Elemcnt(geohist);
  for (dep = 0; dep < portcnt; dep++) {
    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;
      deparr = dep * portcnt + arr;
      dist = dist0[deparr];
      iv = (dist - lodist) * ivcnt / (hidist - lodist);
      geohist[min(iv,ivcnt-1)]++;
    }
  }
  for (iv = 0; iv < ivcnt; iv++) if (geohist[iv]) info(0,"geodist bin %u: %u",iv,geohist[iv]);

  ub1 *net0 = alloc(port2,ub1,0,"net net0",portcnt);
  ub4 *walknet = alloc(port2,ub4,0xff,"net walknet",portcnt);

  ub4 *orgportsbyhop = net->portsbyhop;
  ub4 *orghopdist = net->hopdist;
  ub4 *orghopdur = net->hopdur;

  for (hop = 0; hop < hopcnt; hop++) {
    dep = orgportsbyhop[hop * 2];
    arr = orgportsbyhop[hop * 2 + 1];
    net0[dep * portcnt + arr] = 1;
  }

  ub4 whop,whopcnt = 0;
  for (deparr = 0; deparr < port2; deparr++) {
    if (net0[deparr] || dist0[deparr] > walklimit) continue;
    whopcnt++;
  }
  info(0,"\ah%u inferred walk links below dist %u",whopcnt,walklimit);

  ub4 newhopcnt = hopcnt + whopcnt;

  ub4 *portsbyhop = alloc(newhopcnt * 2,ub4,0,"net portsbyhop",newhopcnt);
  memcpy(portsbyhop,orgportsbyhop,hopcnt * 2 * sizeof(ub4));

  ub4 *hopdist = alloc(newhopcnt,ub4,0,"cmp hopdist",newhopcnt);
  memcpy(hopdist,orghopdist,hopcnt * sizeof(ub4));

  ub4 *hopdur = alloc(newhopcnt,ub4,0,"cmp hopdur",newhopcnt);
  memcpy(hopdur,orghopdur,hopcnt * sizeof(ub4));

  whop = hopcnt;
  for (deparr = 0; deparr < port2; deparr++) {
    dist = dist0[deparr];
    if (net0[deparr] || dist > walklimit) continue;
    walknet[deparr] = whop;
    dep = deparr / portcnt;
    arr = deparr % portcnt;
    portsbyhop[whop * 2] = dep;
    portsbyhop[whop * 2 + 1] = arr;
    hopdur[whop] = dist * walkspeed;
    hopdist[whop] = dist;
    whop++;
  }

  net->whopcnt = whop;
  net->dist0 = dist0;
  net->walknet = walknet;
  net->portsbyhop = portsbyhop;
  net->hopdist = hopdist;
  net->hopdur = hopdur;
  return 0;
}

// create 0-stop connectivity matrix and derived info.
// n-stop builds on this
static int mknet0(struct network *net)
{
  ub4 portcnt = net->portcnt;
  ub4 zportcnt = net->zportcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 whopcnt = net->whopcnt;

  struct port *ports,*pdep,*parr;
  struct hop *hops,*hp;
  struct range distrange;

  char *dname,*aname;
  ub4 *p2z,*z2p;
  ub4 *portsbyhop;
  ub2 concnt,cntlim,gen,*con0cnt;
  ub4 ofs,*con0ofs;
  ub4 hop,l1,l2,*con0lst;
  ub4 dist,*dist0,*lodists,*hidists,*hopdist;
  ub4 geohist[Geohist];
  double fdist;
  ub4 dep,arr,port2,zport2,da,depcnt,arrcnt;
  ub4 needconn,haveconn,watch;
  ub4 *walknet = net->walknet;
  ub2 iv;
  struct eta eta;

  if (portcnt == 0) return error(0,"no ports for %u hops net",hopcnt);
  if (hopcnt == 0) return error(0,"no hops for %u port net",portcnt);
  error_z(zportcnt,portcnt);
  error_lt(chopcnt,hopcnt);

  info(0,"init 0-stop connections for %u port %u hop network in %u zports",portcnt,hopcnt,zportcnt);

  port2 = portcnt * portcnt;
  zport2 = zportcnt * zportcnt;

  ports = net->ports;
  hops = net->hops;
  ub4 *choporg = net->choporg;

  p2z = net->port2zport;
  z2p = net->zport2port;

  portsbyhop = net->portsbyhop;

  con0cnt = alloc(port2, ub2,0,"net0 concnt",portcnt);
  con0ofs = alloc(port2, ub4,0xff,"net0 conofs",portcnt);

  con0lst = mkblock(net->conlst,chopcnt,ub4,Init1,"net0 0-stop conlst");

  lodists = alloc(port2, ub4,0xff,"net0 lodist",portcnt);
  hidists = alloc(port2, ub4,0,"net0 hidist",portcnt);

  // geographical direct-line distance
  dist0 = net->dist0;
  hopdist = net->hopdist;

  // create 0-stop connectivity
  // support multiple hops per port pair
  ub4 ovfcnt = 0,hicon = 0,hida = 0; 
  for (hop = 0; hop < chopcnt; hop++) {
    dep = portsbyhop[hop * 2];
    arr = portsbyhop[hop * 2 + 1];
    if (hop < hopcnt) {
      hp = hops + hop;
      error_ne(dep,hp->dep);
      error_ne(arr,hp->arr);
    }
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    pdep = ports + dep;
    parr = ports + arr;
    dname = pdep->name;
    aname = parr->name;
    if (dep == arr) {
      warning(0,"hop %u dep == arr %u %s",hop,dep,dname);
      continue;
    }
    for (watch = 0; watch < watches; watch++) {
      if (dep == watch_dep[watch]) {
        info(0,"hop %u %u-%u %s to %s",hop,dep,arr,dname,aname);
      }
    }

    da = dep * portcnt + arr;
    concnt = con0cnt[da];
    cntlim = cnt0lim_part;

    if (concnt >= cntlim) ovfcnt++;
    else if (concnt == cntlim-1) {
      info(0,"connect overflow hop %u port %u-%u %s to %s",hop,dep,arr,dname,aname);
      concnt++;
      ovfcnt++;
    } else concnt++;
    if (concnt > hicon) { hicon = concnt; hida = da; }
    con0cnt[da] = concnt;

    dist = hopdist[hop];
    lodists[da] = min(lodists[da],dist);
    hidists[da] = max(hidists[da],dist);
//    error_z(dist,hop);
    if (hop < hopcnt && dist != dist0[da]) warning(0,"hop %u dist %u vs %u",hop,dist,dist0[da]);
//    else if (dist < dist0[da]) warning(Iter,"chop %u dist %u vs %u",hop,dist,dist0[da]); // todo
  }
  if (ovfcnt) warning(0,"limiting 0-stop net by \ah%u",ovfcnt);

  dep = hida / portcnt; arr = hida % portcnt;
  pdep = ports + dep; parr = ports + arr;
  info(0,"highest conn %u between ports %u-%u %s to %s",hicon,dep,arr,pdep->name,parr->name);

  for (hop = 0; hop < chopcnt; hop++) {
    dep = portsbyhop[hop * 2];
    arr = portsbyhop[hop * 2 + 1];
    da = dep * portcnt + arr;
    if (da != hida) continue;

    if (hop < hopcnt) {
      hp = hops + hop;
      info(0,"  hop %u rid %u rrid %u route %s",hop,hp->rid,hp->rrid,hp->name);
    } else {
      l1 = choporg[hop * 2];
      l2 = choporg[hop * 2 + 1];
      hp = hops + l1;
      info(0,"  chop %u = %u-%u rid %u rrid %u route %s",hop,l1,l2,hp->rid,hp->rrid,hp->name);
    }
  }

  ofs = 0;
  needconn = haveconn = 0;
  for (dep = 0; dep < portcnt; dep++) {

    if (progress(&eta,"port %u of %u in pass 2 0-stop %u hop net",dep,portcnt,hopcnt)) return 1;

    pdep = ports + dep;
//    if (dport->ndep == 0) continue;
    dname = pdep->name;

    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;
      parr = ports + arr;
//      if (aport->narr == 0) continue;

      aname = parr->name;

      needconn++;

      da = dep * portcnt + arr;
      concnt = con0cnt[da];
      if (concnt == 0) continue;

      haveconn++;

      con0ofs[da] = ofs;
      gen = 0;
      hop = walknet[da];
      if (hop < chopcnt) {
        con0lst[ofs+gen] = hop;
        gen++;
      }

      for (hop = 0; hop < hopcnt && gen < concnt; hop++) {
        if (portsbyhop[2 * hop] != dep || portsbyhop[2 * hop + 1] != arr) continue;

//        hp = hops + hop;
//        if (hp->dep != dep || hp->arr != arr) {
//          warning(0,"hop %u %s %u-%u not %u-%u",hop,hp->name,hp->dep,hp->arr,dep,arr);
//          continue;
//        }
        error_ge(ofs,chopcnt);
        con0lst[ofs+gen] = hop;
        for (watch = 0; watch < watches; watch++) {
          if (dep == watch_dep[watch] /* && arr == watch_arr[watch] */) {
            info(0,"dep %u arr %u hop %u %u of %u at %p %s to %s",dep,arr,hop,gen,concnt,con0lst + ofs + gen,dname,aname);
          }
        }
        gen++;
        if (gen >= concnt) break;
      }

      for (hop = whopcnt; hop < chopcnt && gen < concnt; hop++) {
        if (portsbyhop[2 * hop] != dep || portsbyhop[2 * hop + 1] != arr) continue;
        error_ge(ofs,chopcnt);
        con0lst[ofs+gen++] = hop;
        if (gen >= concnt) break;
      }
      ofs += gen;
    }
  }
  net->lstlen[0] = ofs;
  info(0,"  0-stop connectivity \ah%3u of \ah%3u  left \ah%3u",haveconn,needconn,needconn - haveconn);

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
    if (depstats[iv]) info(0,"%u port\as reaches %u ports\as", depstats[iv], iv);
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

  net->concnt[0] = con0cnt;
  net->conofs[0] = con0ofs;

  net->lodist[0] = lodists;
  net->hidist[0] = hidists;

  net->needconn = needconn;

  return 0;
}

#define Distbins 256
static ub4 distbins[Distbins];

// create n-stop connectivity matrix and derived info
static int mknetn(struct network *net,ub4 nstop)
{
  ub4 part = net->part;
  ub4 portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  bool topnet = net->istpart;

  struct port *ports,*pmid,*pdep,*parr;
  char *dname,*aname;
  block *lstblk,*lstblk1,*lstblk2;
  ub4 *portsbyhop;
  ub2 *concnt,*cnts1,*cnts2;
  ub4 *portdst;
  ub4 ofs,ofs1,ofs2,endofs,*conofs,*conofs1,*conofs2;
  ub4 *lst,*newlst,*conlst1,*conlst2,*lst1,*lst11,*lst2,*lst22,*lstv1,*lstv2;
  ub4 *dist0,*hopdist,*distlims;
  ub4 dep,mid,arr,firstmid,port2,depcnt,depmid,midarr,deparr,iport1,iport2;
  ub4 iv,ivlim,watch;
  ub4 depstats[4];
  ub4 cnt,nstop1,n1,n2,n12,nleg1,nleg2,v1,v2,leg,leg1,leg2,nleg;
  size_t lstlen,newlstlen;
  ub4 midstop1,midstop2;
  ub4 lodist,lodist1,lodist2,lodist12,*lodists,*lodist1s,*lodist2s;
  ub4 hidist,hidist1,hidist2,hidist12,*hidists,*hidist1s,*hidist2s;
  ub4 dist1,dist2,dist12,distlim,distrange;
  ub4 cntlim,sumcnt,gen,outcnt;
  ub4 constats[16];

  // only fill n-stop if no (nstop-1) exists
  bool nilonly;

  // max #variants per [dep,arr]
  ub4 varlimit,var12limit;

  // todo
  ub4 portlimit = 6000;
  ub4 lstlimit = 1024 * 1024 * 128;

  ub4 dupcode,legport1,legport2;
  ub4 trip1ports[Nleg * 2];
  ub4 trip2ports[Nleg * 2];

  struct eta eta;

  error_z(nstop,0);
  error_zz(portcnt,hopcnt);

  info(0,"init %u-stop connections for %u port %u hop network",nstop,portcnt,hopcnt);

  // todo configurable
  switch (nstop) {
  case 1: nilonly = 0; varlimit = 8; var12limit = 64; break;
  case 2: nilonly = 0; varlimit = 4; var12limit = 32; break;
  case 3: nilonly = 1; varlimit = 4; var12limit = 32; break;
  default: nilonly = 1; varlimit = 2; var12limit = 32; break;
  }

  port2 = portcnt * portcnt;

  ports = net->ports;

  portsbyhop = net->portsbyhop;

  concnt = alloc(port2, ub2,0,"net concnt",portcnt);
  lodists = alloc(port2, ub4,0xff,"net lodist",portcnt);
  hidists = alloc(port2, ub4,0,"net hidist",portcnt);

  distlims = alloc(port2, ub4,0,"net distlims",portcnt);

  portdst = alloc(portcnt, ub4,0,"net portdst",portcnt);

  dist0 = net->dist0;
  hopdist = net->hopdist;

  error_zp(dist0,0);
  error_zp(hopdist,0);

  nleg = nstop + 1;
  nstop1 = nstop - 1;

  memset(trip1ports,0xff,sizeof(trip1ports));
  memset(trip2ports,0xff,sizeof(trip1ports));

/* Essentially we do for each (departure,arrival) pair:
   Search for a 'via' port such that trip (departure,via) and (via,arrival) exist
   This is done for a given number of total stops.
   Hence, search for such via with various stops for X and Y in dep-X-via-Y-arr
   Trim list of alternatives based on e.g. distance
   Store result by value. This is memory-intensive but keeps code simple

   In short: foreach dep  foreach arr foreach stopline  foreach a with n1[dep,a] with n2 [a,arr]
*/

/*
  pass 1 : foreach (dep,arr) pair at this #stops:
  bound overall best and worst
  currently, cost is distance only
  create histogram and derive threshold to use as filter in next pass
  estimate size of trip list matrix
  obtain basic stats
*/

  ub8 dupstats[16];
  ub4 cntstats[16];

  lstlen = 0;

  aclear(distbins);

  aclear(dupstats);
  aclear(cntstats);

  ub4 dmid,dmidcnt,*dmids = alloc(portcnt * nstop,ub4,0,"net vias",portcnt);
  ub4 dudep,duarr,mudep,muarr;
  ub4 *drdeps,*drarrs,*mrdeps,*mrarrs;
  char *mname;
  ub4 dmidcnts[Nstop];
  int fd;
  int dbg,limited = 0;
  char cachefile[256];

  fmtstring(cachefile,"cache/net_%u_part_%u_distlim.in",nstop,part);
  fd = fileopen(cachefile,0);
  if (fd != -1) {
    fileread(fd,distlims,port2 * sizeof(*distlims),cachefile);
    fileclose(fd,cachefile);
    info(0,"using dist limit cache file %s",cachefile);
  }

  ub2 *lwrcnts = net->concnt[nstop-1];

  // for each departure port
  for (dep = 0; dep < portcnt; dep++) {

    if (progress(&eta,"port %u of %u in pass 1 %u-stop net",dep,portcnt,nstop)) return 1;

    if (dep > portlimit) {
      warning(0,"limiting net by %u ports",portlimit);
      limited = 1;
      break;
    }

    if (lstlen > lstlimit / nleg) {
      warning(0,"limiting net by %u triplets",lstlimit / nleg);
      limited = 1;
      break;
    }

    pdep = ports + dep;
    dname = pdep->name;
    dudep = pdep->nudep;
    duarr = pdep->nuarr;
    drdeps = pdep->drids;
    drarrs = pdep->arids;

    // prepare eligible via's

    for (midstop1 = 0; midstop1 < nstop; midstop1++) {
      cnts1 = net->concnt[midstop1];

      dmid = 0;
      for (mid = 0; mid < portcnt; mid++) {
        if (mid == dep) continue;
        pmid = ports + mid;
        depmid = dep * portcnt + mid;

        n1 = cnts1[depmid];
        if (n1 == 0) continue;

        // todo: skip vias only on same route
        mname = pmid->name;
        mudep = pmid->nudep;
        muarr = pmid->nuarr;
        mrdeps = pmid->drids;
        mrarrs = pmid->arids;

        if (dudep == 1 && duarr == 1 && mudep == 1 && muarr == 1 && drdeps[0] == mrarrs[0] && drdeps[0] != hi32) {
          info(0,"skip %u-%u-x on same oneway route %x %s to %s",dep,mid,drdeps[0],dname,mname);
          continue;
        }
        if (dudep == 2 && duarr == 2 && mudep == 2 && muarr == 2) {
          if(drdeps[0] == mrarrs[0] && drdeps[0] != hi32 && drarrs[1] == mrdeps[1] && mrdeps[1] != hi32) {
            info(0,"skip %u-%u-x on same twoway route %x.%x %s to %s",dep,mid,drdeps[0],mrdeps[1],dname,mname);
            continue;
          }
        }

        dmids[midstop1 * portcnt + dmid++] = mid;
      }
      dmidcnts[midstop1] = dmid;
    }

    outcnt = 0;

    // for each arrival port
    for (arr = 0; arr < portcnt; arr++) {
      if (arr == dep) continue;

      deparr = dep * portcnt + arr;

      if (nilonly && lwrcnts[deparr]) { cntstats[9]++; continue; }

      parr = ports + arr;
      aname = parr->name;

      lodist = hi32; hidist = 0;
      cnt = cntlim = 0;

      // for each #stops between dep-via-arr.
      // e.g. trip dep-a-b-via-c-arr has 2 stops before and 1 after via
      for (midstop1 = 0; midstop1 < nstop; midstop1++) {
        midstop2 = nstop1 - midstop1;

        cnts1 = net->concnt[midstop1];
        cnts2 = net->concnt[midstop2];

        lodist1s = net->lodist[midstop1];
        lodist2s = net->lodist[midstop2];
        hidist1s = net->hidist[midstop1];
        hidist2s = net->hidist[midstop2];

        // for each via
        // first obtain distance range
        dmidcnt = dmidcnts[midstop1];
        for (dmid = 0; dmid < dmidcnt; dmid++) {
          mid = dmids[midstop1 * portcnt + dmid];
          if (mid == arr) continue;

          pmid = ports + mid;

          depmid = dep * portcnt + mid;

          n1 = cnts1[depmid];
          error_z(n1,mid);

          midarr = mid * portcnt + arr;
          n2 = cnts2[midarr];
          if (n2 == 0) { cntstats[6]++; continue; }

          error_ovf(n1,ub2);
          error_ovf(n2,ub2);

          n12 = n1 * n2;
          if (n12 > var12limit) { cntstats[7]++; n12 = var12limit; }
          cnt += n12;
          cntlim = min(cnt,varlimit);

          lodist1 = lodist1s[depmid];
          lodist2 = lodist2s[midarr];
          hidist1 = hidist1s[depmid];
          hidist2 = hidist2s[midarr];
          lodist12 = lodist1 + lodist2;
          hidist12 = hidist1 + hidist2;

          if (lodist12 < lodist) {
            lodist = lodist12;
          }
          if (hidist12 > hidist) {
            hidist = hidist12;
          }

        } // each mid stopover port
      } // each midpoint in stop list dep-a-b-arr

      if (cnt) {  // store range info
        lstlen += cntlim;
        concnt[deparr] = (ub2)cntlim;
        outcnt++;
        lodists[deparr] = lodist;
        hidists[deparr] = hidist;
//        error_zz(lodist,hidist);
        error_gt(lodist,hidist,arr);
        distrange = max(hidist - lodist,1);
      } else {
        distrange = 1;
      }

      // limits precomputed from file
      if (distlims[deparr]) continue;

      // if too many options, sort on distance.
      if (cnt > varlimit) {
        cntstats[8]++;

        // subpass 2: create distance histogram and derive threshold
        for (midstop1 = 0; midstop1 < nstop; midstop1++) {
          midstop2 = nstop1 - midstop1;
          nleg1 = midstop1 + 1;
          nleg2 = midstop2 + 1;

          cnts1 = net->concnt[midstop1];
          cnts2 = net->concnt[midstop2];

          lstblk1 = net->conlst + midstop1;
          lstblk2 = net->conlst + midstop2;

          conlst1 = blkdata(lstblk1,0,ub4);
          conlst2 = blkdata(lstblk2,0,ub4);

          conofs1 = net->conofs[midstop1];
          conofs2 = net->conofs[midstop2];

          distlim = hi32;

          for (mid = 0; mid < portcnt; mid++) {
            if (mid == dep || mid == arr) continue;
            depmid = dep * portcnt + mid;

            n1 = cnts1[depmid];
            if (n1 == 0) continue;

            midarr = mid * portcnt + arr;
            n2 = cnts2[midarr];
            if (n2 == 0) continue;

            for (watch = 0; watch < watches; watch++) {
              if (dep == watch_dep[watch]) vrb(0,"dep %u arr %u mid %u n1 %u n2 %u",dep,arr,mid,n1,n2);
            }

            ofs1 = conofs1[depmid];
            ofs2 = conofs2[midarr];
            error_eq(ofs1,hi32);
            error_eq(ofs2,hi32);

            lst1 = conlst1 + ofs1 * nleg1;
            lst2 = conlst2 + ofs2 * nleg2;

            bound(lstblk1,ofs1 * nleg1,ub4);
            bound(lstblk2,ofs2 * nleg2,ub4);

            // each dep-via alternative, except dep-*-arr-*-via
            for (v1 = 0; v1 < n1; v1++) {
              dist1 = 0;
              lst11 = lst1 + v1 * nleg1;

              for (leg1 = 0; leg1 < nleg1; leg1++) {
                leg = lst11[leg1];
                error_ge(leg,chopcnt);
                dist1 += hopdist[leg];
                trip1ports[leg1 * 2] = portsbyhop[leg * 2];
                trip1ports[leg1 * 2 + 1] = portsbyhop[leg * 2 + 1];
              }

              if (dist1 > distlim) { cntstats[1]++; continue; }

              dupcode = 0;
              for (legport1 = 1; legport1 < midstop1 * 2; legport1++) {
                iport1 = trip1ports[legport1];
                if (iport1 == dep) { dupcode = 1; break; }
                else if (iport1 == arr) { dupcode = 2; break; }
                else if (iport1 == mid) { dupcode = 3; break; }
              }

              if (dupcode) {
                dupstats[dupcode]++;
                continue;
              }

              for (watch = 0; watch < watches; watch++) {
                if (dep == watch_dep[watch]) vrb(0,"dep %u arr %u mid %u \av%u%p n1 %u n2 %u",dep,arr,mid,nleg1,lst11,n1,n2);
              }
              checktrip(net,lst11,nleg1,dep,mid,dist1);
              for (v2 = 0; v2 < n2; v2++) {
                dist12 = dist1;
                lst22 = lst2 + v2 * nleg2;

                for (leg2 = 0; leg2 < nleg2; leg2++) {
                  leg = lst22[leg2];
                  error_ge(leg,chopcnt);
                  dist12 += hopdist[leg];
                  trip2ports[leg2 * 2] = portsbyhop[leg * 2];
                  trip2ports[leg2 * 2 + 1] = portsbyhop[leg * 2 + 1];
                }
                if (dist12 > distlim) { cntstats[2]++; break; }

                dupcode = 0;
                for (legport2 = 1; legport2 < nleg2 * 2 - 1; legport2++) {
                  iport2 = trip2ports[legport2];
                  if (iport2 == dep) { dupcode = 4; break; }
                  else if (iport2 == arr) { dupcode = 5; break; }
                  else if (iport2 == mid) { dupcode = 6; break; }
                }

                if (dupcode) {
                  dupstats[dupcode]++;
                  continue;
                }

//                checktrip(lst22,nleg2,mid,arr,dist2);

                // filter out repeated 'B' visits in dep-*-B-*-via-*-B-*-arr
                if (nstop > 3) {
                  for (legport1 = 0; legport1 < nleg1 * 2; legport1++) {
                    iport1 = trip1ports[legport1];
                    for (legport2 = 1; legport2 < nleg2 * 2; legport2++) {
                      iport2 = trip2ports[legport2];
                      if (iport1 == iport2) { dupcode = 7; break; }
                    }
                    if (dupcode) break;
                  }
                  if (dupcode) {
                    dupstats[dupcode]++;
                    continue;
                  }
                }

                // mark in histogram, discard actual trip here
//                error_lt(dist12,lodist); // todo fails
//                error_gt(dist12,hidist);  // todo: fails
                iv = (dist12 - min(lodist,dist12)) * Distbins / distrange;
                if (iv < Distbins) distbins[iv]++;
              } // each v2
            } // each v1

            sumcnt = 0; ivlim = 0;
            while (sumcnt < cntlim && ivlim < Distbins) {
              sumcnt += distbins[ivlim++];
            }
            if (ivlim) ivlim--;
            distlim = lodist + ivlim * distrange / Distbins;

          } // each mid

        } // each midpoint

        // derive threshold to aim at desired number of alternatives
        sumcnt = 0; iv = 0;
        while (sumcnt < cntlim && iv < Distbins) {
          sumcnt += distbins[iv++];
        }
        if (iv) iv--;
        distlim = lodist + iv * distrange / Distbins;
        aclear(distbins);
      } else distlim = hidist;  // not cnt limited
      distlims[deparr] = distlim;

    } // each arrival port
    portdst[dep] = outcnt;

  } // each departure port

  for (iv = 0; iv < Elemcnt(dupstats); iv++) if (dupstats[iv]) info(0,"dup %u: \ah%lu",iv,dupstats[iv]);
  for (iv = 0; iv < Elemcnt(cntstats); iv++) if (cntstats[iv]) info(0,"cnt %u: \ah%u",iv,cntstats[iv]);

  info(0,"%u-stop pass 1 done, tentative \ah%lu triplets",nstop,lstlen);

  if (lstlen == 0) {
    warning(0,"no connections at %u-stop",nstop);
    return 0;
  }

  ub4 cnt1,newcnt;
  struct range portdr;
  ub4 ivportdst[32];
  mkhist(caller,portdst,portcnt,&portdr,Elemcnt(ivportdst),ivportdst,"outbounds by port",Vrb);

  fmtstring(cachefile,"cache/net_%u_part_%u_distlim",nstop,part);
  fd = filecreate(cachefile,0);
  if (fd != -1) {
    filewrite(fd,distlims,port2 * sizeof(*distlims),cachefile);
    fileclose(fd,cachefile);
  }

  // prepare list matrix and its offsets
  conofs = alloc(port2, ub4,0xff,"net conofs",portcnt);  // = org

  lstblk = net->conlst + nstop;

  lst = alloc((ub4)lstlen * nleg,ub4,0xff,"netv n-stop conlst",nstop);
  net->lstlen[nstop] = lstlen;

  cnts1 = net->concnt[nstop1];
  ofs = newcnt = 0;
  for (deparr = 0; deparr < port2; deparr++) {
    cnt = concnt[deparr];
    cnt1 = cnts1[deparr];
    if (cnt) {
      conofs[deparr] = ofs;
      ofs += cnt;
      if (cnt1 == 0) newcnt++;
    }
  }
  error_ne(ofs,lstlen);
  info(0,"\ah%u new connections",newcnt);

  aclear(dupstats);

  // pass 2: fill based on range and thresholds determined above
  // most code comments from pass 1 apply
  ofs = 0;
  for (dep = 0; dep < portcnt; dep++) {

    if (progress(&eta,"port %u of %u in pass 2 %u-stop net",dep,portcnt,nstop)) return 1;

    if (dep > portlimit) continue;

    for (arr = 0; arr < portcnt; arr++) {
      if (arr == dep) continue;
      deparr = dep * portcnt + arr;

      cnt = concnt[deparr];
      if (cnt == 0) continue;
      gen = concnt[deparr] = 0;

      dbg = (part == 0 && dep == 1068 && arr == 0);

      distlim = distlims[deparr];

      conofs[deparr] = ofs;
      lstv1 = lst + ofs * nleg;
      error_ge(ofs,lstlen);

      endofs = ofs + cnt - 1;

      error_ge(endofs,lstlen);

      infocc(dbg,0,"port %u-%u concnt %u at %p-%p",dep,arr,gen,lstv1,lst + endofs * nleg);

      midstop1 = 0;
      while (midstop1 < nstop && gen < cnt) {
        midstop2 = nstop1 - midstop1;
        nleg1 = midstop1 + 1;
        nleg2 = midstop2 + 1;

        cnts1 = net->concnt[midstop1];
        cnts2 = net->concnt[midstop2];

        lstblk1 = net->conlst + midstop1;
        lstblk2 = net->conlst + midstop2;

        conlst1 = blkdata(lstblk1,0,ub4);
        conlst2 = blkdata(lstblk2,0,ub4);

        mid = 0; firstmid = hi32;
        while (mid < portcnt && gen < cnt) {
          if (mid == dep || mid == arr) { mid++; continue; }
          depmid = dep * portcnt + mid;
          n1 = cnts1[depmid];
          if (n1 == 0) { mid++; continue; }

          midarr = mid * portcnt + arr;
          n2 = cnts2[midarr];
          if (n2 == 0) { mid++; continue; }

          firstmid = min(firstmid,mid);
          n12 = n1 * n2;

          conofs1 = net->conofs[midstop1];
          conofs2 = net->conofs[midstop2];

          ofs1 = conofs1[depmid];
          ofs2 = conofs2[midarr];
          lst1 = conlst1 + ofs1 * nleg1;
          lst2 = conlst2 + ofs2 * nleg2;

          bound(lstblk1,ofs1 * nleg1,ub4);
          bound(lstblk2,ofs2 * nleg2,ub4);

          v1 = 0;
          while (v1 < n1 && gen < cnt) {
            dist1 = 0;
            lst11 = lst1 + v1 * nleg1;

            for (leg1 = 0; leg1 < nleg1; leg1++) {
              leg = lst11[leg1];
              error_ge(leg,chopcnt);
              dist1 += hopdist[leg];
              trip1ports[leg1 * 2] = portsbyhop[leg * 2];
              trip1ports[leg1 * 2 + 1] = portsbyhop[leg * 2 + 1];
            }
            if (dist1 > distlim) { v1++; continue; }

            error_ne(trip1ports[0],dep);
            error_eq(trip1ports[0],mid);
            error_eq(trip1ports[0],arr);
            error_eq(trip1ports[midstop1 * 2 + 1],dep);
            error_ne(trip1ports[nleg1 * 2 - 1],mid);
            error_eq(trip1ports[nleg1 * 2 - 1],arr);
            dupcode = 0;
            for (legport1 = 1; legport1 < nleg1 * 2 - 1; legport1++) {
              iport1 = trip1ports[legport1];
              if (iport1 == dep) { dupcode = 1; break; }
              else if (iport1 == arr) { dupcode = 2; break; }
              else if (iport1 == mid) { dupcode = 3; break; }
            }

            if (dupcode) { v1++; continue; }

            checktrip(net,lst11,nleg1,dep,mid,dist1);
            v2 = 0;
            while (v2 < n2 && gen < cnt) {
              dist2 = 0;
              lst22 = lst2 + v2 * nleg2;

              for (leg2 = 0; leg2 < nleg2; leg2++) {
                leg = lst22[leg2];
                error_ge(leg,chopcnt);
                dist2 += hopdist[leg];
                trip2ports[leg2 * 2] = portsbyhop[leg * 2];
                trip2ports[leg2 * 2 + 1] = portsbyhop[leg * 2 + 1];
              }
              error_ne(trip2ports[0],mid);
              error_eq(trip2ports[0],dep);
              error_eq(trip2ports[0],arr);
              error_ne(trip2ports[nleg2 * 2 - 1],arr);
              error_eq(trip2ports[nleg2 * 2 - 1],dep);
              error_eq(trip2ports[nleg2 * 2 - 1],mid);
              dupcode = 0;
              for (legport2 = 1; legport2 < nleg2 * 2 - 1; legport2++) {
                iport2 = trip2ports[legport2];
                if (iport2 == dep) { dupcode = 4; break; }
                else if (iport2 == arr) { dupcode = 5; break; }
                else if (iport2 == mid) { dupcode = 6; break; }
              }

              if (dupcode) { v2++; continue; }

              // filter out repeated visits a-B-c-d-B-f
              if (nstop > 3) {
                for (legport1 = 0; legport1 < nleg1 * 2; legport1++) {
                  iport1 = trip1ports[legport1];
                  for (legport2 = 1; legport2 < nleg2 * 2; legport2++) {
                    iport2 = trip2ports[legport2];
                    if (iport1 == iport2) { dupcode = 7; break; }
                  }
                  if (dupcode) break;
                }
                if (dupcode) {
                  v2++;
                  continue;
                }
              }

//              checktrip(lst22,nleg2,mid,arr,dist2);
              dist12 = dist1 + dist2;
              if (dist12 <= distlim || gen == 0) {
                gen++;
                for (leg1 = 0; leg1 < nleg1; leg1++) {
                  leg = lst11[leg1];
                  error_ge(leg,chopcnt);
                  lstv1[leg1] = leg;
                }
//                memcpy(lstv1,lst11,nleg1 * sizeof(ub4));
//                vrb(CC,"dep %u arr %u mid %u \av%u%p base %p n1 %u n2 %u",dep,arr,mid,nleg,lstv1,lst,n1,n2);
//                checktrip(lstv1,nleg1,dep,mid,dist1);
                lstv2 = lstv1 + nleg1;
                for (leg2 = 0; leg2 < nleg2; leg2++) {
                  leg = lst22[leg2];
                  error_ge(leg,chopcnt);
                  lstv2[leg2] = leg;
                }
//                memcpy(lstv2,lst22,nleg2 * sizeof(ub4));
//                vrb(CC,"dep %u arr %u mid %u \av%u%p base %p n1 %u n2 %u",dep,arr,mid,nleg,lstv1,lst,n1,n2);
//                checktrip3(lstv1,nleg,dep,arr,mid,dist12);
                lstv1 = lstv2 + nleg2;
              }
              v2++;
            }
            v1++;
          }
          mid++;
        } // each mid

        // if none found for any dep-Mid-arr, but mid exists, use first one
        if (cnt && gen == 0 && firstmid != hi32) {
          warn(Iter,"dep %u arr %u no conn for mid %u cnt %u distlim %u",dep,arr,firstmid,cnt,distlim);
          depmid = dep * portcnt + firstmid;
          midarr = mid * portcnt + arr;

          error_z(cnts1[depmid],firstmid);
          error_z(cnts2[midarr],firstmid);

          conofs1 = net->conofs[midstop1];
          conofs2 = net->conofs[midstop2];

          ofs1 = conofs1[depmid];
          ofs2 = conofs2[midarr];
          lst1 = conlst1 + ofs1 * nleg1;
          lst2 = conlst2 + ofs2 * nleg2;

          for (leg1 = 0; leg1 < nleg1; leg1++) {
            leg = lst1[leg1];
            lstv1[leg1] = leg;
          }
          lstv2 = lstv1 + nleg1;
          for (leg2 = 0; leg2 < nleg2; leg2++) {
            leg = lst2[leg2];
            lstv2[leg2] = leg;
          }
          lstv1 = lstv2 + nleg2;
          gen = 1;
        }

        midstop1++;
      } // each mid stopover port

      error_gt(gen,cnt,arr);
      warncc(cnt && gen == 0,Iter,"dep %u arr %u no conn distlim %u",dep,arr,distlim);

      ofs += gen;
      concnt[deparr] = (ub2)gen;
      infocc(dbg,0,"port %u-%u concnt %u",dep,arr,gen);
    } // each arrival port
  } // each departure port
  newlstlen = ofs;
  error_gt(newlstlen,lstlen,nstop);
  info(0,"pass 2 done, \ah%lu from \ah%lu triplets",newlstlen,lstlen);

  newlst = mkblock(lstblk,newlstlen * nleg,ub4,Init1,"netv part %u %u-stop conlst",part,nstop);
  net->lstlen[nstop] = newlstlen;

  memcpy(newlst,lst,newlstlen * nleg * sizeof(ub4));
  afree(lst,"netv conlst");
  for (ofs = 0; ofs < newlstlen * nleg; ofs++) {
    leg = newlst[ofs];
    error_ge(leg,chopcnt);
  }

  for (iv = 0; iv < Elemcnt(dupstats); iv++) if (dupstats[iv]) info(0,"dup %u: \ah%lu",iv,dupstats[iv]);

  // get connectivity stats
  aclear(depstats);
  ub4 depivs = Elemcnt(depstats) - 1;
  for (dep = 0; dep < portcnt; dep++) {
    depcnt = 0;
    for (arr = 0; arr < portcnt; arr++) {
      depcnt++;
    }
//    ports[dep].depcnts[nstop] = (ub2)depcnt;
    depstats[min(depivs,depcnt)]++;
  }
  for (iv = 0; iv <= depivs; iv++) info(0,"%u ports with %u departures", depstats[iv], iv);

  struct range conrange;

  aclear(constats);
  mkhist2(concnt,port2,&conrange,Elemcnt(constats),constats,"connection",Info);

  net->concnt[nstop] = concnt;
  net->conofs[nstop] = conofs;

  net->lodist[nstop] = lodists;
  net->hidist[nstop] = hidists;

  net->lstlen[nstop] = lstlen;

  unsigned long doneconn,leftperc,leftcnt,needconn = net->needconn;
  ub4 n,da,nda = 0,port,hascon,hicon,arrcon,loarrcon,lodep = 0;
  ub4 tports[Nstop];
  ub4 gtports[Nstop];
  ub4 deparrs[16];
  ub4 nstops[16];
  ub4 lodeparrs[16];
  ub4 lonstops[16];
  ub4 ndacnt = Elemcnt(deparrs);
  struct port *pp;
  ub2 *con0cnt = net->con0cnt;

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
      vrb(0,"port %u lacks %u connection\as %s",dep,(ub4)leftcnt,pdep->name);
      for (da = 1; da < nda; da++) {
        deparr = deparrs[da];
        if (deparr == hi32) break;
        arr = deparr % portcnt;
        parr = ports + arr;
        vrb(0,"port %u no connection to %u %s",dep,arr,parr->name);
      }
    }
  }
  error_gt(doneconn,needconn,0);
  leftcnt = needconn - doneconn;
  leftperc = leftcnt * 100 / needconn;
  if (leftperc > 1) info(0,"0-%u-stop connectivity \ah%3lu of \ah%3lu  left \ah%3lu = %u%%", nstop,doneconn,needconn,leftcnt,(ub4)leftperc);
  else info(0,"0-%u-stop connectivity \ah%3lu of \ah%3lu  left \ah3%lu", nstop,doneconn,needconn,leftcnt);

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
  ub4 *tp2g,*gp2t;

  struct eta eta;

  if (partcnt < 2) return info(0,"no inter-partition maps for %u partition\as",partcnt);

  enter(callee);

/* have separate partitions with hi-conn or high-partmember ports
   each port ( with low conn or partmember? ) has list of conn to above
 */
  info0(0,"fill inter-partition reach maps pass 1");

  tpart = partcnt - 1;
  tnet = getnet(tpart);
  tportcnt = tnet->portcnt;
  conmask = tnet->conmask = alloc(tportcnt * tportcnt,ub1,0,"part topnet conn",tportcnt);

  tp2g = tnet->p2gport;
  gp2t = tnet->g2pport;

  for (tdep = 0; tdep < tportcnt; tdep++) {
    for (tarr = 0; tarr < tportcnt; tarr++) {
      if (tdep == tarr) continue;
      deparr = tdep * tportcnt + tarr;
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
static int dogconn(ub4 callee,struct gnetwork *gnet)
{
  struct network *net,*tnet,*anet,*danet;
  struct port *gpdep,*gparr,*gports = gnet->ports;
  char *dname,*aname;
  ub1 *portparts = gnet->portparts;
  ub4 partcnt = gnet->partcnt;
  ub4 daportcnt,gportcnt = gnet->portcnt;
  ub4 gdep,garr,dep,arr,tdep,tarr,tdmid,tamid,gdmid,gamid;
  ub4 part,apart;
  ub2 cnt;
  ub4 xconn;
  ub4 tpart;
  ub4 ti,ati,tcnt,tacnt;
  ub4 dtmid,gdtmid,atmid,depmid,midarr,deparr,gatmid,xamid,xmid,xda;
  ub4 *tp2g,*gp2t;
  ub4 portcnt,tportcnt,aportcnt;
  ub8 gconn = 0,gxconn = 0,gxconn1 = 0,gxconn2 = 0;
  ub4 lconn;
  ub4 sample,shareparts;
  ub4 stats[4];
  struct eta eta;
  int dbg = 0,tdepfirst,tarrfirst;

  ub2 *xmap,*xamap,*xmapdbase,*xmapabase,stopset,xm,xam;
  ub1 *tmap;
  block *xpartdmap = &gnet->xpartdmap;
  block *xpartamap = &gnet->xpartamap;

  if (gportcnt < 2) return info(0,"skip global conn for %u ports net",gportcnt);

  enter(callee);

  if (partcnt > 1) {
    xmapdbase = blkdata(xpartdmap,0,ub2);
    xmapabase = blkdata(xpartamap,0,ub2);
  } else {
    xmapdbase = xmapabase = NULL;
  }

  tpart = partcnt - 1;
  tnet = getnet(tpart);
  tportcnt = tnet->portcnt;
  tmap = tnet->conmask;
  tp2g = tnet->p2gport;
  gp2t = tnet->g2pport;

  if (gportcnt > 500) {
    sample = gportcnt / 500;
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
        infocc(gconn < 3,0,"local conn %u-%u %s to %s",gdep,garr,dname,aname);
        continue;

      } else if (partcnt == 1) {
        continue;

      } else if (gpdep->tpart) { // dep in top, arr not

        xamap = xmapabase + garr * tportcnt;
        tdep = gp2t[gdep];

        if (xamap[tdep]) {
          gxconn++; gxconn1++;
          infocc(gxconn < 3,0,"interpart-t1 conn %u-%u via %u %s to %s",gdep,garr,gamid,dname,aname);
          continue;
        }

      } else if (gparr->tpart) { // arr in top, dep not

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
  ub4 nstop,histop,part,partcnt;
  int rv;
  struct gnetwork *gnet = getgnet();
  struct network *net;

  if (dorun(FLN,Runmknet) == 0) return 0;

  partcnt = gnet->partcnt;
  if (partcnt == 0) return info0(0,"skip init zero-partition net");

  for (part = 0; part < partcnt; part++) {

    info(0,"mknet partition %u of %u",part,partcnt);
    net = getnet(part);

    portcnt = net->portcnt;
    hopcnt = net->hopcnt;

    if (portcnt == 0) { info0(0,"skip mknet on 0 ports"); continue; }
    if (hopcnt == 0) { info0(0,"skip mknet on 0 hops"); continue; }

    msgprefix(0,"s%u ",part);

    rv = mkwalks(net);
    if (rv) return msgprefix(1,NULL);

    rv = compound(net);
    if (rv) return msgprefix(1,NULL);

    if (dorun(FLN,Runnet0)) {
      if (mknet0(net)) return msgprefix(1,NULL);
    } else continue;

    histop = maxstop;
//    if (net->istpart) histop++;
    limit_gt(histop,Nstop,0);

    if (dorun(FLN,Runnetn)) {
      for (nstop = 1; nstop <= histop; nstop++) {
        if (mknetn(net,nstop)) return msgprefix(1,NULL);
        net->histop = nstop;
        if (net->lstlen[nstop] == 0) {
          break;
        }
      }
      info(0,"partition %u static network init done",part);
    } else info(0,"partition %u skipped static network init",part);

  } // each part

  for (part = 0; part < partcnt; part++) {
    net = getnet(part);
    gnet->histop = min(gnet->histop,net->histop);
  }

  msgprefix(0,NULL);

  if (mkxmap(caller,gnet)) return 1;

  if (dogconn(caller,gnet)) return 1;

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
  ub4 chopcnt = net->chopcnt;
  ub4 *portsbyhop = net->portsbyhop;
  ub4 *hopdist = net->hopdist;

  error_eq_fln(arr,dep,"arr","dep",fln);

  for (legno = 0; legno < nleg; legno++) {
    leg = legs[legno];
    error_ge_fln(leg,chopcnt,"hop","hopcnt",fln);
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
  error_ne_fln(dist,cdist,"dist","cdist",fln);
}

// check whether a triplet passes thru the given ports
void checktrip3_fln(struct network *net,ub4 *legs, ub4 nleg,ub4 dep,ub4 arr,ub4 via,ub4 dist,ub4 fln)
{
  ub4 legno,leg;
  struct hop *hp,*hops = net->hops;
  int hasvia = 0;

  error_lt_fln(nleg,2,"nleg","2",fln);
  checktrip_fln(net,legs,nleg,dep,arr,dist,fln);
  error_eq_fln(via,dep,"via","dep",fln);
  error_eq_fln(via,arr,"via","arr",fln);

  for (legno = 1; legno < nleg; legno++) {
    leg = legs[legno];
    hp = hops + leg;
    if (hp->dep == via) hasvia = 1;
  }
  error_z_fln(hasvia,0,"via","0",fln);
}

int triptoports_fln(ub4 fln,struct network *net,ub4 *trip,ub4 triplen,ub4 *ports,ub4 *gports)
{
  ub4 leg,l,dep,arr = hi32;
  ub4 chopcnt = net->chopcnt;
  ub4 portcnt = net->portcnt;

  error_ge(triplen,Nleg);

  for (leg = 0; leg < triplen; leg++) {
    l = trip[leg];
    if (l >= chopcnt) return errorfln(fln,0,FLN,"leg %u hop %u above %u",leg,l,chopcnt);
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
  if (triplen < Nstop) ports[triplen+1] = gports[triplen+1] = hi32;
  return 0;
}

int gtriptoports(struct gnetwork *gnet,struct trip *ptrip,char *buf,ub4 *pbuflen)
{
  struct network *net;
  struct port *gports = gnet->ports,*pdep,*parr = gports;
  struct hop *hops,*hp,*hp2;
  ub4 *trip = ptrip->trip;
  const char *name;
  ub4 rid,rrid;
  ub4 part,leg,prvleg,ghop,l,l1 = 0,l2 = 0,dep,arr = hi32,tdep,tarr,gdep,garr = hi32;
  ub4 gportcnt = gnet->portcnt;
  ub4 partcnt = gnet->partcnt;
  ub4 hopcnt,whopcnt,chopcnt;
  ub4 portcnt;
  ub4 *portsbyhop;
  ub4 *choporg;
  ub4 *p2g;
  ub4 pos = 0;
  ub4 buflen = *pbuflen;
  ub4 triplen = ptrip->len;
  ub4 *ports = ptrip->port;

  if (triplen == 0) return warning(0,"nil trip for %u port net",gportcnt);
  error_ge(triplen,Nxleg);

  for (leg = 0; leg < triplen; leg++) {
    part = trip[leg * 2];
    error_ge_cc(part,partcnt,"leg %u of %u",leg,triplen);
    net = getnet(part);
    hops = net->hops;
    hopcnt = net->hopcnt;
    whopcnt = net->whopcnt;
    chopcnt = net->chopcnt;
    portcnt = net->portcnt;
    portsbyhop = net->portsbyhop;
    choporg = net->choporg;
    p2g = net->p2gport;
    l = trip[leg * 2 + 1];
    if (l >= chopcnt) return error(0,"part %u leg %u hop %u >= %u",part,leg,l,chopcnt);
    dep = portsbyhop[2 * l];
    error_ge(dep,portcnt);
    gdep = p2g[dep];
    error_ge(gdep,gportcnt);
    pdep = gports + gdep;
    if (l < hopcnt) {
      hp = hops + l;
      name = hp->name; rid = hp->rid; rrid = hp->rrid; ghop = hp->gid;
    } else if (l < whopcnt) {
      hp = NULL;
      name = "walk"; rid = rrid = ghop = 0;
    } else {
      l1 = choporg[2 * l];
      l2 = choporg[2 * l + 1];
      if (l1 >= hopcnt) return error(0,"part %u compound leg %u = %u-%u >= %u",part,l,l1,l2,hopcnt);
      hp = hops + l1;
      name = hp->name; rid = hp->rid; rrid = hp->rrid; ghop = hp->gid;
    }
    if (leg) {
      if (gdep != garr) {
        prvleg = leg - 1;
        if (l < whopcnt) return error(0,"leg %u hop %u.%u dep %u not connects to preceding arr %u.%u %s vs %s route %s",leg,part,l,gdep,trip[prvleg * 2],garr,pdep->name,parr->name,name);
        else return error(0,"leg %u chop %u.%u = %u-%u dep %u not connects to preceding arr %u.%u %s vs %s route %s",leg,part,l,l1,l2,gdep,trip[prvleg * 2],garr,pdep->name,parr->name,name);
      }
    }
    arr = portsbyhop[2 * l + 1];
    error_ge(arr,portcnt);
    ports[leg] = gdep;
    garr = p2g[arr];
    error_ge(garr,gportcnt);
    parr = gports + garr;
    tdep = ptrip->t[leg];
    tarr = tdep + ptrip->dur[leg];
    if (l < whopcnt) info(0,"leg %u hop %u dep %u.%u at \ad%u arr %u at \ad%u %s to %s route %s rid %u",leg,ghop,part,gdep,tdep,garr,tarr,pdep->name,parr->name,name,rid);
    else {
      hp2 = hops + l2;
      error_ne(rid,hp2->rid);
      info(0,"leg %u chop %u-%u dep %u.%u at \ad%u arr %u at \ad%u %s to %s route %s rid %u",leg,hp->gid,hp2->gid,part,gdep,tdep,garr,tarr,pdep->name,parr->name,name,rid);
    }
    pos += mysnprintf(buf,pos,buflen,"leg %2u dep \ad%u %s\n",leg+1,tdep,pdep->name);
    pos += mysnprintf(buf,pos,buflen,"       arr \ad%u %s\n",tarr,parr->name);
    pos += mysnprintf(buf,pos,buflen,"       route %s id %u\n\n",name,rrid);
  }
  *pbuflen = pos;
  error_ge(garr,gportcnt);
  ports[triplen] = garr;
  return 0;
}

ub4 fgeodist(struct port *pdep,struct port *parr)
{
  ub4 dist;
  ub4 dep = pdep->id;
  ub4 arr = parr->id;
  double fdist = geodist(pdep->rlat,pdep->rlon,parr->rlat,parr->rlon);
  char *dname = pdep->name;
  char *aname = parr->name;

  if (fdist < 1e-10) warning(Iter,"port %u-%u distance ~0 for latlon %u,%u-%u,%u %s to %s",dep,arr,pdep->lat,pdep->lon,parr->lat,parr->lon,dname,aname);
  else if (fdist < 0.001) warning(Iter,"port %u-%u distance %e for latlon %u,%u-%u,%u %s to %s",dep,arr,fdist,pdep->lat,pdep->lon,parr->lat,parr->lon,dname,aname);
  else if (fdist > 1.0e+8) warning(Iter,"port %u-%u distance %e for latlon %u,%u-%u,%u %s to %s",dep,arr,fdist,pdep->lat,pdep->lon,parr->lat,parr->lon,dname,aname);
  dist = (ub4)fdist;
  return dist;
}
