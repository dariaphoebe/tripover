// net.c - main network setup

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

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
     the list is trimmed on heuristics, such as distance, ocst, timing

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
#include "condense.h"

#undef hdrstop

static struct network net;

void ininet(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

struct network *getnet(void) { return &net; }

#define Geohist (20+2)

#define Watches 4
static ub4 watches = 0;
static ub4 watch_dep[Watches];
static ub4 watch_arr[Watches];

// create 0-stop connectivity matrix and derived info.
// n-stop builds on this
static int mknet0(void)
{
  ub4 portcnt = net.portcnt;
  ub4 hopcnt = net.hopcnt;

  struct port *ports,*dport,*aport;
  struct hop *hops,*hp;
  struct range distrange;
  ub4 *portsbyhop;
  ub2 concnt,gen,*con0cnt;
  ub4 ofs,*con0ofs;
  ub4 hop,*con0lst;
  ub4 dist,*dist0,*hopdist;
  ub4 geohist[Geohist];
  double fdist;
  ub4 dep,arr,port2,da,depcnt,arrcnt,needconn,watch;
  ub2 iv;
  ub4 depstats[16];
  ub4 arrstats[16];
  struct eta eta;

  if (portcnt == 0) return error(0,"no ports for %u hops net",hopcnt);
  if (hopcnt == 0) return error(0,"no hops for %u port net",portcnt);

  info(0,"0-stop connections for %u port %u hop network",portcnt,hopcnt);

  port2 = portcnt * portcnt;

  ports = net.ports;
  hops = net.hops;

  con0cnt = alloc(port2, ub2,0,"con0cnt",portcnt);
  con0ofs = alloc(port2, ub4,0xff,"con0ofs",portcnt);

  con0lst = mkblock(net.conlst,hopcnt,ub4,Init1,"0-stop conlst");

  portsbyhop = alloc(hopcnt * 2, ub4,0xff,"con0ofs",portcnt);

  // geographical direct-line distance
  dist0 = alloc(port2, ub4,0,"geodist0",portcnt);
  hopdist = alloc(hopcnt,ub4,0,"hopdist",hopcnt);

  info(0,"calculating \ah%u distance pairs", port2);
  for (dep = 0; dep < portcnt; dep++) {
    dport = ports + dep;
    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;
      da = dep * portcnt + arr;
      aport = ports + arr;
      if (dport->lat == aport->lat && dport->lon == aport->lon) {
        warning(0,"port %u-%u distance 0 for latlon %u %u",dep,arr,dport->lat,dport->lon);
        dist = 0;
      } else {
        fdist = geodist(dport->rlat,dport->rlon,aport->rlat,aport->rlon);
        if (fdist < 0.001) warning(0,"port %u-%u distance %e for latlon %u %u-%u %u",dep,arr,fdist,dport->lat,dport->lon,aport->lat,aport->lon);
        else if (fdist > 1.0e+8) warning(0,"port %u-%u distance %e for latlon %u %u-%u %u",dep,arr,fdist,dport->lat,dport->lon,aport->lat,aport->lon);
        dist = (ub4)fdist;
         error_ovf(dist,ub4);
      }
//      error_z(dist,arr);
      dist0[da] = dist;
    }
  }
  info(0,"done calculating \ah%u distance pairs", port2);
  mkhist(dist0,port2,&distrange,Geohist,geohist,"geodist",Info);

  // create 0-stop connectivity
  // support multiple hops per port pair
  ub4 ovfcnt = 0;
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    dep = hp->dep;
    arr = hp->arr;
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    portsbyhop[hop * 2] = dep;
    portsbyhop[hop * 2 + 1] = arr;
    da = dep * portcnt + arr;
    concnt = con0cnt[da];
    if (concnt == hi16-1) ovfcnt++;
    else con0cnt[da] = (ub2)(concnt+1);
    dist = dist0[da];
//    error_z(dist,hop);
    hp->dist = dist;
    hopdist[hop] = dist;
  }
  if (ovfcnt) warning(0,"limiting 0-stop net by \ah%u",ovfcnt);

  ofs = 0;
  needconn = 0;
  for (dep = 0; dep < portcnt; dep++) {

    progress(&eta,"port %u of %u in pass 2 0-stop %u hop net",dep,portcnt,hopcnt);

    dport = ports + dep;
    if (dport->ndep == 0) continue;

    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;
      aport = ports + arr;
      if (aport->narr == 0) continue;

      needconn++;

      da = dep * portcnt + arr;
      concnt = con0cnt[da];
      if (concnt == 0) continue;

      con0ofs[da] = ofs;
      gen = 0;
      for (hop = 0; hop < hopcnt; hop++) {
        hp = hops + hop;
        if (hp->dep != dep || hp->arr != arr) continue;
        error_ge(ofs,hopcnt);
        error_ge(gen,concnt);
        con0lst[ofs+gen] = hop;
        for (watch = 0; watch < watches; watch++) {
          if (dep == watch_dep[watch] && arr == watch_arr[watch]) {
            info(0,"dep %u arr %u hop %u %u of %u at %p",dep,arr,hop,gen,concnt,con0lst + ofs + gen);
          }
        }
        gen++;
      }
      ofs += gen;
    }
  }
  net.lstlen[0] = ofs;

  // get connectivity stats
  aclear(depstats);
  for (dep = 0; dep < portcnt; dep++) {
    depcnt = 0;
    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;
      depcnt += con0cnt[dep * portcnt + arr];
      error_ovf(depcnt,ub2);
    }
    error_ne(depcnt,ports[dep].ndep);
    depstats[min(Elemcnt(depstats),depcnt)]++;
  }
  for (iv = 0; iv < Elemcnt(depstats); iv++) info(0,"%u ports with %u departures", depstats[iv], iv);

  aclear(arrstats);
  for (arr = 0; arr < portcnt; arr++) {
    arrcnt = 0;
    for (dep = 0; dep < portcnt; dep++) {
      if (dep == arr) continue;
      arrcnt += con0cnt[dep * portcnt + arr];
      error_ovf(arrcnt,ub2);
    }
    error_ne(arrcnt,ports[arr].narr);
    arrstats[min(Elemcnt(arrstats),arrcnt)]++;
  }
  for (iv = 0; iv < Elemcnt(arrstats); iv++) info(0,"%u ports with %u arrivals", arrstats[iv], iv);

  net.dist0 = dist0;
  net.hopdist = hopdist;
  net.portsbyhop = portsbyhop;

  net.con0cnt = con0cnt;
  net.con0ofs = con0ofs;

  net.concnt[0] = con0cnt;
  net.conofs[0] = con0ofs;

  net.lodist[0] = dist0;
  net.hidist[0] = dist0;

  net.needconn = needconn;

  return 0;
}

#define Distbins 256
static ub4 distbins[Distbins];

// create n-stop connectivity matrix and derived info
static int mknetn(ub4 nstop)
{
  ub4 portcnt = net.portcnt;
  ub4 hopcnt = net.hopcnt;

  struct port *ports,*pmid,*pdep,*parr;
  block *lstblk,*lstblk1,*lstblk2;
  ub4 *portsbyhop;
  ub2 *concnt,*cnts1,*cnts2;
  ub4 *portdst;
  ub4 ofs,ofs1,ofs2,endofs,*conofs,*conofs1,*conofs2;
  ub4 *lst,*conlst1,*conlst2,*lst1,*lst11,*lst2,*lst22,*lstv1,*lstv2;
  ub4 *dist0,*hopdist,*distlims;
  ub4 dep,mid,arr,port2,depcnt,depmid,midarr,deparr,iport1,iport2;
  ub4 iv,watch;
  ub4 depstats[4];
  ub4 cnt,nstop1,n1,n2,n12,n12lim,nleg1,nleg2,v1,v2,leg,leg1,leg2,nleg;
  size_t lstlen;
  ub4 midstop1,midstop2;
  ub4 lodist,lodist1,lodist2,lodist12,*lodists,*lodist1s,*lodist2s;
  ub4 hidist,hidist1,hidist2,hidist12,*hidists,*hidist1s,*hidist2s;
  ub4 dist1,dist2,dist12,distlim,distrange;
  ub4 cntlim,sumcnt,gen,outcnt;
  ub4 constats[16];

  ub4 varlimit = 32; // todo configurable and dependent on other aspects

  ub4 var12limit = 1024;
  ub4 dupcode,legport1,legport2;
  ub4 trip1ports[Nleg * 2];
  ub4 trip2ports[Nleg * 2];

  struct eta eta;

  error_z(nstop,0);
  error_zz(portcnt,hopcnt);

  info(0,"init %u-stop connections for %u port %u hop network",nstop,portcnt,hopcnt);

  port2 = portcnt * portcnt;

  ports = net.ports;

  portsbyhop = net.portsbyhop;

  concnt = alloc(port2, ub2,0,"concnt",portcnt);
  lodists = alloc(port2, ub4,0xff,"lodist",portcnt);
  hidists = alloc(port2, ub4,0,"hidist",portcnt);

  distlims = alloc(port2, ub4,0,"distlims",portcnt);

  portdst = alloc(portcnt, ub4,0,"portdst",portcnt);

  dist0 = net.dist0;
  hopdist = net.hopdist;

  error_zp(dist0,0);
  error_zp(hopdist,0);

  nleg = nstop + 1;
  nstop1 = nstop - 1;

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

  ub4 dupstats[8];

  lstlen = 0;

  aclear(distbins);

  aclear(dupstats);

  // for each departure port
  for (dep = 0; dep < portcnt; dep++) {

    progress(&eta,"port %u of %u in pass 1 %u-stop net",dep,portcnt,nstop);

    pdep = ports + dep;
    if (pdep->deps == 0) continue;

    outcnt = 0;

    // for each arrival port
    for (arr = 0; arr < portcnt; arr++) {
      if (arr == dep) continue;

      parr = ports + arr;
      if (parr->arrs == 0) continue;

      deparr = dep * portcnt + arr;

      lodist = hi32; hidist = 0;
      cnt = cntlim = 0;

      // for each #stops between dep-via-arr.
      // e.g. trip dep-a-b-via-c-arr has 2 stops before and 1 after via
      for (midstop1 = 0; midstop1 < nstop; midstop1++) {
        midstop2 = nstop1 - midstop1;

        cnts1 = net.concnt[midstop1];
        cnts2 = net.concnt[midstop2];

        lodist1s = net.lodist[midstop1];
        lodist2s = net.lodist[midstop2];
        hidist1s = net.hidist[midstop1];
        hidist2s = net.hidist[midstop2];

        // for each via
        // first obtain distance range
        for (mid = 0; mid < portcnt; mid++) {
          if (mid == dep || mid == arr) continue;
          pmid = ports + mid;
          if (pmid->ndep == 0 || pmid->narr == 0) continue;

          depmid = dep * portcnt + mid;

          n1 = cnts1[depmid];
          if (n1 == 0) continue;

          midarr = mid * portcnt + arr;
          n2 = cnts2[midarr];
          if (n2 == 0) continue;

          error_ovf(n1,ub2);
          error_ovf(n2,ub2);

          n12 = min(n1 * n2,var12limit);
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
        error_gt(lodist,hidist);
        distrange = max(hidist - lodist,1);
      } else {
        distrange = 1;
      }

      // if too many options, sort on distance.
      if (cnt > varlimit) {

        // subpass 2: create distance histogram and derive threshold
        for (midstop1 = 0; midstop1 < nstop; midstop1++) {
          midstop2 = nstop1 - midstop1;
          nleg1 = midstop1 + 1;
          nleg2 = midstop2 + 1;

          cnts1 = net.concnt[midstop1];
          cnts2 = net.concnt[midstop2];

          lstblk1 = net.conlst + midstop1;
          lstblk2 = net.conlst + midstop2;

          conlst1 = blkdata(lstblk1,0,ub4);
          conlst2 = blkdata(lstblk2,0,ub4);

          conofs1 = net.conofs[midstop1];
          conofs2 = net.conofs[midstop2];

          for (mid = 0; mid < portcnt; mid++) {
            if (mid == dep || mid == arr) continue;
            depmid = dep * portcnt + mid;

            n1 = cnts1[depmid];
            if (n1 == 0) continue;

            midarr = mid * portcnt + arr;
            n2 = cnts2[midarr];
            if (n2 == 0) continue;

            for (watch = 0; watch < watches; watch++) {
              if (dep == watch_dep[watch]) info(0,"dep %u arr %u mid %u n1 %u n2 %u",dep,arr,mid,n1,n2);
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
                error_ge(leg,hopcnt);
                dist1 += hopdist[leg];
                trip1ports[leg1 * 2] = portsbyhop[leg * 2];
                trip1ports[leg1 * 2 + 1] = portsbyhop[leg * 2 + 1];
              }
              dupcode = 0;
              for (legport1 = 1; legport1 < nleg1 * 2 - 1; legport1++) {
                iport1 = trip1ports[legport1];
                if (iport1 == dep) { dupcode = 1; break; }
                else if (iport1 == arr) { dupcode = 2; break; }
                else if (iport1 == mid) { dupcode = 3; break; }
              }

              dupstats[min(dupcode,Elemcnt(dupstats))]++;
              if (dupcode) continue;

              for (watch = 0; watch < watches; watch++) {
                if (dep == watch_dep[watch]) info(0,"dep %u arr %u mid %u \av%u%p n1 %u n2 %u",dep,arr,mid,nleg1,lst11,n1,n2);
              }
              checktrip(lst11,nleg1,dep,mid,dist1);
              for (v2 = 0; v2 < n2; v2++) {
                dist2 = 0;
                lst22 = lst2 + v2 * nleg2;

                for (leg2 = 0; leg2 < nleg2; leg2++) {
                  leg = lst22[leg2];
                  error_ge(leg,hopcnt);
                  dist2 += hopdist[leg];
                  trip2ports[leg2 * 2] = portsbyhop[leg * 2];
                  trip2ports[leg2 * 2 + 1] = portsbyhop[leg * 2 + 1];
                }
                dupcode = 0;
                for (legport2 = 1; legport2 < nleg2 * 2 - 1; legport2++) {
                  iport2 = trip2ports[legport2];
                  if (iport2 == dep) { dupcode = 4; break; }
                  else if (iport2 == arr) { dupcode = 5; break; }
                  else if (iport2 == mid) { dupcode = 6; break; }
                }

                dupstats[min(dupcode,Elemcnt(dupstats))]++;
                if (dupcode) continue;

//                checktrip(lst22,nleg2,mid,arr,dist2);

                // filter out repeated 'B' visits in dep-*-B-*-via-*-B-*-arr
                for (legport1 = 0; legport1 < nleg1 * 2; legport1++) {
                  iport1 = trip1ports[legport1];
                  for (legport2 = 1; legport2 < nleg2 * 2; legport2++) {
                    iport2 = trip2ports[legport2];
                    if (iport1 == iport2) { dupcode = 7; break; }
                  }
                  if (dupcode) break;
                }
                if (dupcode) {
                  dupstats[min(dupcode,Elemcnt(dupstats))]++;
                  continue;
                }

                // mark in histogram, discard actual trip here
                dist12 = dist1 + dist2;
                error_lt(dist12,lodist);
                error_gt(dist12,hidist);
                iv = (dist12 - lodist) * Distbins / distrange;
                distbins[iv]++;
              } // each v2
            } // each v1

          } // each mid
        } // each midpoint

        // derive threshold to aim at desired number of alternatives
        sumcnt = 0; iv = 0;
        while (sumcnt < cntlim && iv < Distbins) {
          sumcnt += distbins[iv++];
        }
        if (iv == Distbins) distlim = hidist;
        else {
          iv--;
          distlim = lodist + iv * distrange / Distbins;
        }
        aclear(distbins);
      } else distlim = hidist;  // cnt limited
      distlims[deparr] = distlim;

    } // each arrival port
    portdst[dep] = outcnt;

  } // each departure port

  for (iv = 0; iv < Elemcnt(dupstats); iv++) info(0,"dup %u: %u",iv,dupstats[iv]);

  info(0,"%u-stop pass 1 done, tentative \ah%lu triplets",nstop,lstlen);

  if (lstlen == 0) {
    warning(0,"no connections at %u-stop",nstop);
    return 0;
  }

  struct range portdr;
  ub4 ivportdst[32];
  mkhist(portdst,portcnt,&portdr,Elemcnt(ivportdst),ivportdst,"outbounds by port",Vrb);

  // prepare list matrix and its offsets
  conofs = alloc(port2, ub4,0xff,"conofs",portcnt);  // = org

  lstblk = net.conlst + nstop;

  lst = mkblock(lstblk,lstlen * nleg,ub4,Init1,"%u-stop conlst",nstop);
  net.lstlen[nstop] = lstlen;

  ofs = 0;
  for (deparr = 0; deparr < port2; deparr++) {
    cnt = concnt[deparr];
    if (cnt) {
      conofs[deparr] = ofs;
      ofs += cnt;
    }
  }
  error_ne(ofs,lstlen);

  aclear(dupstats);

  // pass 2: fill based on range and thresholds determined above
  // most code comments from pass 1 apply
  for (dep = 0; dep < portcnt; dep++) {

    progress(&eta,"port %u of %u in pass 2 %u-stop net",dep,portcnt,nstop);

    for (arr = 0; arr < portcnt; arr++) {
      if (arr == dep) continue;
      deparr = dep * portcnt + arr;

      cnt = concnt[deparr];
      if (cnt == 0) continue;
      gen = 0;

      distlim = distlims[deparr];

      ofs = conofs[deparr];
      lstv1 = lst + ofs * nleg;
      error_ge(ofs,lstlen);

      endofs = ofs + cnt - 1;

      bound(lstblk,endofs * nleg,ub4);

      midstop1 = 0;
      while (midstop1 < nstop && gen < cnt) {
        midstop2 = nstop1 - midstop1;
        nleg1 = midstop1 + 1;
        nleg2 = midstop2 + 1;

        cnts1 = net.concnt[midstop1];
        cnts2 = net.concnt[midstop2];
 
        lstblk1 = net.conlst + midstop1;
        lstblk2 = net.conlst + midstop2;

        conlst1 = blkdata(lstblk1,0,ub4);
        conlst2 = blkdata(lstblk2,0,ub4);

        mid = 0;
        while (mid < portcnt && gen < cnt) {
          if (mid == dep || mid == arr) { mid++; continue; }
          depmid = dep * portcnt + mid;
          n1 = cnts1[depmid];
          if (n1 == 0) { mid++; continue; }

          midarr = mid * portcnt + arr;
          n2 = cnts2[midarr];
          if (n2 == 0) { mid++; continue; }

          n12 = n1 * n2;
          n12lim = min(n12,var12limit);

          conofs1 = net.conofs[midstop1];
          conofs2 = net.conofs[midstop2];

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
              error_ge(leg,hopcnt);
              dist1 += hopdist[leg];
              trip1ports[leg1 * 2] = portsbyhop[leg * 2];
              trip1ports[leg1 * 2 + 1] = portsbyhop[leg * 2 + 1];
            }
            error_ne(trip1ports[0],dep);
            error_eq(trip1ports[0],mid);
            error_eq(trip1ports[0],arr);
            error_eq(trip1ports[nleg1 * 2 - 1],dep);
            error_ne(trip1ports[nleg1 * 2 - 1],mid);
            error_eq(trip1ports[nleg1 * 2 - 1],arr);
            dupcode = 0;
            for (legport1 = 1; legport1 < nleg1 * 2 - 1; legport1++) {
              iport1 = trip1ports[legport1];
              if (iport1 == dep) { dupcode = 1; break; }
              else if (iport1 == arr) { dupcode = 2; break; }
              else if (iport1 == mid) { dupcode = 3; break; }
            }

            dupstats[min(dupcode,Elemcnt(dupstats))]++;
            if (dupcode) { v1++; continue; }

            checktrip(lst11,nleg1,dep,mid,dist1);
            v2 = 0;
            while (v2 < n2 && gen < cnt) {
              dist2 = 0;
              lst22 = lst2 + v2 * nleg2;

              for (leg2 = 0; leg2 < nleg2; leg2++) {
                leg = lst22[leg2];
                error_ge(leg,hopcnt);
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

              dupstats[min(dupcode,Elemcnt(dupstats))]++;
              if (dupcode) { v2++; continue; }

              // filter out repeated visits a-B-c-d-B-f
              for (legport1 = 0; legport1 < nleg1 * 2; legport1++) {
                iport1 = trip1ports[legport1];
                for (legport2 = 1; legport2 < nleg2 * 2; legport2++) {
                  iport2 = trip2ports[legport2];
                  if (iport1 == iport2) { dupcode = 7; break; }
                }
                if (dupcode) break;
              }
              if (dupcode) {
                dupstats[min(dupcode,Elemcnt(dupstats))]++;
                v2++;
                continue;
              }

//              checktrip(lst22,nleg2,mid,arr,dist2);
              dist12 = dist1 + dist2;
              if (dist12 <= distlim) {
                gen++;
                memcpy(lstv1,lst11,nleg1 * sizeof(ub4));
//                vrb(CC,"dep %u arr %u mid %u \av%u%p base %p n1 %u n2 %u",dep,arr,mid,nleg,lstv1,lst,n1,n2);
//                checktrip(lstv1,nleg1,dep,mid,dist1);
                lstv2 = lstv1 + nleg1;
                memcpy(lstv2,lst22,nleg2 * sizeof(ub4));
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
        midstop1++;
      } // each mid stopover port
      error_gt(gen,cnt);
      concnt[deparr] = (ub2)gen;
    } // each arrival port
  } // each departure port

  info(0,"pass 2 done, \ah%lu triplets", lstlen);

  for (iv = 0; iv < Elemcnt(dupstats); iv++) info(0,"dup %u: %u",iv,dupstats[iv]);

  // get connectivity stats
  aclear(depstats);
  for (dep = 0; dep < portcnt; dep++) {
    depcnt = 0;
    for (arr = 0; arr < portcnt; arr++) {
      depcnt++;
    }
    ports[dep].depcnts[nstop] = (ub2)depcnt;
    depstats[min(Elemcnt(depstats),depcnt)]++;
  }
  for (iv = 0; iv < Elemcnt(depstats); iv++) info(0,"%u ports with %u departures", depstats[iv], iv);

  struct range conrange;

  aclear(constats);
  mkhist2(concnt,port2,&conrange,Elemcnt(constats),constats,"connection",Info);

  net.concnt[nstop] = concnt;
  net.conofs[nstop] = conofs;

  net.lodist[nstop] = lodists;
  net.hidist[nstop] = hidists;

  net.lstlen[nstop] = lstlen;

  unsigned long doneconn,leftperc,needconn = net.needconn;

  doneconn = 0;
  for (deparr = 0; deparr < port2; deparr++) {
    nstop1 = 0;
    while (nstop1 <= nstop) {
      concnt = net.concnt[nstop1];
      error_zp(concnt,nstop1);
      if (concnt[deparr]) { doneconn++; break; }
      nstop1++;
    }
  }
  error_gt(doneconn,needconn);
  leftperc = (needconn - doneconn) * 100 / needconn;
  info(0,"0-%u-stop connectivity \ah%lu of \ah%lu  left \ah%lu = %u%%", nstop,doneconn,needconn,needconn - doneconn,(ub4)leftperc);

  return 0;
}

// initialize basic network, and connectivity for each number of stops
// the number of stops need to be determined such that all port pairs are reachable
int mknet(ub4 maxstop)
{
  ub4 allportcnt = net.allportcnt;
  ub4 allhopcnt = net.allhopcnt;
  struct port *allports;
  struct hop *allhops;
  ub4 nstop;
  int rv;

  allportcnt = net.allportcnt;
  allhopcnt = net.allhopcnt;
  if (allportcnt == 0 || allhopcnt == 0) return 1;

  allports = net.allports;
  allhops = net.allhops;

  rv = condense(&net);
  if (rv) return 1;

  if (globs.nosteps || globs.doinit) {
    if (mknet0()) return 1;
  }

  limit_gt(maxstop,Nstop);

  net.maxstop = maxstop;

  if (globs.nosteps || globs.doinit) {
    for (nstop = 1; nstop <= maxstop; nstop++) {
      if (mknetn(nstop)) return 1;
      if (net.lstlen[nstop] == 0) {
        net.maxstop = nstop;
        break;
      }
    }
    info0(0,"static network init done");
  } else info0(0,"skipped static network init");

  return 0;
}

// check whether a triplet passes thru the given ports
void checktrip_fln(ub4 *legs, ub4 nleg,ub4 dep,ub4 arr,ub4 dist,ub4 fln)
{
  ub4 legno,legno2,leg,leg2,arr0,cdist,hopcnt = net.hopcnt;
  struct hop *hp,*hp2,*hops = net.hops;

  error_eq_fln(arr,dep,"arr","dep",fln);

  for (legno = 0; legno < nleg; legno++) {
    leg = legs[legno];
    error_ge_fln(leg,hopcnt,"hop","hopcnt",fln);
  }
  if (nleg > 2) {
    for (legno = 0; legno < nleg; legno++) {
      leg = legs[legno];
      hp = hops + leg;
      for (legno2 = legno+1; legno2 < nleg; legno2++) {
        leg2 = legs[legno2];
        hp2 = hops + leg2;
        error_eq_fln(leg,leg2,"leg1","leg2",fln);
        error_eq_fln(hp->dep,hp2->dep,"dep1","dep2",fln);
        error_eq_fln(hp->dep,hp2->arr,"dep1","arr2",fln);
        if (legno2 != legno + 1) error_eq_fln(hp->arr,hp2->dep,"arr1","dep2",fln);
        error_eq_fln(hp->arr,hp2->arr,"arr1","arr2",fln);
      }
    }
  }

  leg = legs[0];
  hp = hops + leg;
  error_ne_fln(hp->dep,dep,"hop.dep","dep",fln);
  arr0 = hp->arr;
  cdist = hp->dist;

  leg = legs[nleg-1];
  hp = hops + leg;
  error_ne_fln(hp->arr,arr,"hop.arr","arr",fln);

  for (legno = 1; legno < nleg; legno++) {
    leg = legs[legno];
    hp = hops + leg;
    error_ne_fln(arr0,hp->dep,"prv.arr","dep",fln);
    arr0 = hp->arr;
    cdist += hp->dist;
  }
  error_ne_fln(dist,cdist,"dist","cdist",fln);
}

// check whether a triplet passes thru the given ports
void checktrip3_fln(ub4 *legs, ub4 nleg,ub4 dep,ub4 arr,ub4 via,ub4 dist,ub4 fln)
{
  ub4 legno,leg;
  struct hop *hp,*hops = net.hops;
  int hasvia = 0;

  error_lt_fln(nleg,2,"nleg","2",fln);
  checktrip_fln(legs,nleg,dep,arr,dist,fln);
  error_eq_fln(via,dep,"via","dep",fln);
  error_eq_fln(via,arr,"via","arr",fln);

  for (legno = 1; legno < nleg; legno++) {
    leg = legs[legno];
    hp = hops + leg;
    if (hp->dep == via) hasvia = 1;
  }
  error_z_fln(hasvia,0,"via","0",fln);
}

int trip2ports(ub4 *trip,ub4 triplen,ub4 *ports)
{
  ub4 leg,l,dep,arr = hi32;

  for (leg = 0; leg < triplen; leg++) {
    l = trip[leg];
    dep = net.portsbyhop[2 * l];
    if (leg) {
      if (dep != arr) warning(0,"leg %u hop %u dep %u not connects to preceding arr %u",leg,l,dep,arr);
    }
    arr = net.portsbyhop[2 * l + 1];
    ports[leg] = dep;
  }
  ports[triplen] = arr;
  return 0;
}
