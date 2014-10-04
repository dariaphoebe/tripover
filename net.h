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
  ub4 allid;

  char name[128];  // todo: use below structure instead
  ub4 namelen;
//  struct gname name;

  ub4 gid;   // global port, index in net.gports, in full connectivity matrix
  ub4 lid;   // local port, index in net.lports, local connections only

  ub4 lat,lon;
  double rlat,rlon;

  ub4 utcofs;

  bool full;
  bool mini;

  ub4 macid;
  ub4 macport;

  ub2 minicnt;
  ub2 macone;
  ub4 miniofs;
  ub4 macsize;

  ub4 macbox[4]; // latlon of mimi members

  ub4 ndep,narr,nudep,nuarr;   // generic connectivity info

  ub4 deps[2];     // store small net locally
  ub4 arrs[2];

  ub2 prox0cnt;  // #ports in 0-stop proximity. aka direct neighbours

  ub4 terms[Nterm];  // terminal or transfer port 
  ub2 depcnts[Nstop];
};

struct hop {
  ub4 magic;
  ub4 id;

  char name[128];  // todo: use below structure instead
  ub4 namelen;
//  struct gname name;

  ub4 dep,arr;

  ub4 dist;
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
  ub4 portcnt,allportcnt;
  ub4 hopcnt,allhopcnt;

  ub4 routecnt;
  ub4 carriercnt;
  ub4 timetablecnt;

  struct port *ports, *allports;
  struct hop *hops,*allhops;

  struct route *routes;
  struct carrier *carriers;  
  struct timetable *timetables;  // [routecnt]

// access
  ub4 tthops[Hopcnt];   // index in timetables above

  ub4 fports2ports[Portcnt];

  ub4 *dist0;      // [portcnt2] distance
  ub4 *hopdist;    // [hopcnt] idem
  ub4 *portsbyhop; // [hopcnt * 2] <dep,arr>

  ub4 *mac2port;  // [nmac < portcnt]

// connection matrices. cached separately ?
  ub2 *con0cnt;    // [port2]  0-stop connections
  ub4 *con0ofs;    // [port2]  offsets in lst

  ub4 maxstop;     // 

  size_t needconn;    // final required any-stop connectivity

// idem for each of n-stop...
  ub2 *concnt[Nstop];  // [port2]
  ub4 *conofs[Nstop];  // [port2]

  block conlst[Nstop];  // [x]
  size_t lstlen[Nstop];

  ub4 *lodist[Nstop];  // [port2] lowest over-route distance
  ub4 *hidist[Nstop];  // [port2] lowest over-route distance

  ub4 *portdst[Nstop];  // [portcnt] #destinations per port

// pools
// ?  datetime *gopool;

// minis
  ub4 minicnt;
  ub4 *minis;

};

extern int mknet(ub4 maxstop);
extern struct network *getnet(void);
extern int trip2ports(ub4 *trip,ub4 triplen,ub4 *ports);

#define checktrip(legs,nleg,dep,arr,dist) checktrip_fln((legs),(nleg),(dep),(arr),(dist),FLN)
#define checktrip3(legs,nleg,dep,arr,via,dist) checktrip3_fln((legs),(nleg),(dep),(arr),(via),(dist),FLN)
extern void checktrip_fln(ub4 *legs,ub4 nleg,ub4 dep,ub4 arr,ub4 dist,ub4 fln);
extern void checktrip3_fln(ub4 *legs,ub4 nleg,ub4 dep,ub4 arr,ub4 via,ub4 dist,ub4 fln);

extern void ininet(void);
