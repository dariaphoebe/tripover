// net.h - secondary, derived network defines

/* structures and definitions for the secondary network components
 * search-related defines are in src.h
 */

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define Nterm 4
#define Nstop 8

struct port {
  ub4 magic;
  ub4 id;      // index in net.ports
  struct portbase *base;

  ub4 gid;   // global port, index in net.gports, in full connectivity matrix
  ub4 lid;   // local port, index in net.lports, local connections only

  ub2 depcnt;

  ub2 prox0cnt;  // #ports in 0-stop proximity. aka direct neighbours

  ub4 terms[Nterm];  // terminal or transfer port 
  ub2 depcnts[Nstop];
};

struct hop {
  ub4 magic;
  ub4 id;
  struct hopbase *base;
  // todo
};

struct route {
  ub4 magic;
  ub4 id;
  struct routebase *base;
  // todo
};

struct carrier {
  ub4 magic;
  ub4 id;
  struct carrierbase *base;
  // todo
};

// a carrier has one or more services on a route
// a service has a set of maps
// each map has entries for each go
// multiple maps are made when needed for duration difference or not fitting in map
struct timetable {
  ub4 magic;
  ub4 id;
  struct timetablebase *base;

//  todo unrolled aka expanded timetables

};

#define Nleg (Nstop+1)

// holds all
struct network {
  ub4 portcnt;
  ub4 hopcnt;
  ub4 routecnt;
  ub4 carriercnt;
  ub4 timetablecnt;

  struct port *ports;
  struct hop *hops;
  struct route *routes;
  struct carrier *carriers;  
  struct timetable *timetables;  // [routecnt]

  struct networkbase *base;

// access
  ub4 tthops[Hopcnt];   // index in timetables above

  ub4 fports2ports[Portcnt];

  ub4 *dist0;      // [portcnt2] distance
  ub4 *hopdist;    // [hopcnt] idem
  ub4 *portsbyhop; // [hopcnt * 2] <dep,arr>

// connection matrices. cached separately ?
  ub2 *con0cnt;    // [port2]  0-stop connections
  ub4 *con0ofs;    // [port2]  offsets in lst

//  block con0lst;    // [hopcnt]

  ub4 maxstop;     // 

  size_t needconn;    // final required any-stop connectivity

// idem for each of n-stop...
  ub2 *concnt[Nstop];  // [port2]
  ub4 *conofs[Nstop];  // [port2]

  block conlst[Nstop];  // [x]

  ub4 *lodist[Nstop];  // [port2] lowest over-route distance
  ub4 *hidist[Nstop];  // [port2] lowest over-route distance

  ub4 *portdst[Nstop];  // [portcnt] #destinations per port

// pools
// ?  datetime *gopool;

};

extern int mknet(netbase *basenet,ub4 maxstop);

extern void ininet(void);
