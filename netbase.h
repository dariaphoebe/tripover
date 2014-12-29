// netbase.h - primary network defines

/* structures and definitions for the primary network components :

 - port ( aka stop, station or hub ).
 - hop ( direct connection between 2 ports )
 - route ( sequence of hops, e.g. for rail and bus )
 - timetable ( aka schedule )
 - service. e.g. route for rail+bus, single hop for air

   examples:

   air
     port = Zaventem
     hop = AMS to BRU

  rail
    port = Leuven
    hop = Bruxelles Nord to Mechelen
    route = Amsterdam centraal to Paris Nord

  bus
    port = Bardon
    hop = Elimbah to Beerburrum
    route = 444N

   Derived, secondary data is mostly in net.h

   Search-related defines are in src.h
 */

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define Netbase_inc

enum txkind { Unknown,Air,Rail,Bus,Ferry,Walk,Kindcnt };

struct portbase {
  ub4 magic;
  ub4 id;
  ub4 cid;     // constant at net changes

  char name[128];  // todo: use below structure instead
  ub4 namelen;
//  struct gname name;

  ub4 ndep,narr;

  ub4 subcnt,subofs;

  bool parentsta;

  bool air;
  bool rail;

  ub4 lat,lon;
  double rlat,rlon;

  ub4 utcofs;

  ub4 size;
};

struct subportbase {
  ub4 id;      // index in net.ports
  ub4 subid;
  ub4 cid;     // constant at net changes

  char name[128];  // todo: use below structure instead
  ub4 namelen;

  ub4 seq;
};

struct chainbase {
  ub4 hoprefs;
  ub4 rtid;
  ub4 rrid;
  ub4 rid;
  ub4 dep;
  ub4 hopcnt;
  ub4 hopofs;
  ub8 code;
};

struct timepatbase {
  ub4 hop;
  ub4 utcofs;

  ub4 tdays;
  ub4 gt0;
  ub4 t0,t1;   // actual event range in min relative to gt0
  ub4 lodur,hidur;
  ub4 evcnt;
  ub4 genevcnt;
  ub4 evofs;
  ub4 dayofs;

  ub4 hispans[4];
  ub4 hireps[4];
  ub8 hisums[4];
  ub4 hit0s[4];
  ub4 hit1s[4];
};

struct hopbase {
  ub4 magic;
  ub4 id;       // index in net.hops
  ub4 cid;
  char name[128];  // todo: use below structure instead
  ub4 namelen;

//  struct gname name;

  ub1 valid;

  enum txkind kind;

  ub4 dep,arr;
  ub4 rrid,rid;

  struct timepatbase tp;

  ub4 timespos;
  ub4 timecnt;
  ub4 evcnt;
  ub4 zevcnt;
  ub4 t0,t1;   // overall date range of timetable : t1 exclusive

  ub4 dist;

  ub4 carriercnt;
  ub4 servicecnt;
};
 
struct routebase {
  ub4 magic;
  ub4 id;       // index in net.routes
  ub4 cid;
  ub4 rrid;

  char name[128];
  ub4 namelen;
//  struct gname name;

  ub4 end0,end1;

  ub4 dist;

  ub4 carriercnt;
  ub4 servicecnt;
  ub4 chaincnt;
  ub4 chainofs;
  ub4 chainpos;
  ub4 hichainlen;
};

struct carrierbase {
  ub4 magic;
  ub4 id;       // index in net.carriers
  ub4 cid;
  struct gname name;

  ub4 utcofs;
  ub4 hopcnt;
};

// a carrier has one or more services on a route
// a service has a set of maps
// each map has entries for each go
// go contains one or more trips in a repeating pattern
// multiple maps are made when needed for duration difference or not fitting in map
struct sidbase {
  ub4 rsid;
  ub4 sid;
  ub4 cid;

  char name[64];
  ub4 namelen;

  ub4 carrier;
  ub4 route;
  ub4 service;

  ub4 t0,t1;      // tt range in minutes std
  ub4 t0map;      // start of day map in minutes std

  ub4 lt0day,lt1day; // tt range in localtime days

//  ub4 dt;         // granularity of table in minutes
  ub4 utcofs;     // minutes east from utc + 12h

  ub4 mapofs;     // day map
  ub4 maplen;     // in days

  ub4 refcnt;

//  ub2 duration;   // minutes
//  ub2 gocnt;      // # entries below

//  ub4 gobase;     // datetime gos[gocnt] at net.dtmaps+base :  daysofweek, timesofday map
};

// optional: ~ unrolled timetable
struct fare {
  ub4 carrier;
  ub4 route;
  ub4 service;
  ub4 t0,t1;      // tt validity in minutes utc since epoch

  ub4 depcnt;
  ub4 farecnt;

  ub4 depbase;    // fareinc deps[depcnt * farecnt] at net.faremaps+base
};

// main structure holding everything
struct networkbase {
  ub4 portcnt;
  ub4 subportcnt;
  ub4 hopcnt;
  ub4 sidcnt;
  ub4 ridcnt;
  ub4 carriercnt;
  ub4 timetablecnt;

  ub4 rawchaincnt;
  ub4 chainhopcnt;
  ub4 timescnt;

  struct portbase *ports;
  struct subportbase *subports;
  struct hopbase *hops;
  struct sidbase *sids;
  struct routebase *routes;
  struct carrierbase *carriers;  
  struct chainbase *chains;
  struct timetablebase *timetables;  // [routecnt]

  ub8 *chainhops;    // tdep,hop
  ub4 *routechains;
  ub4 *timesbase;
  ub8 *events;
  ub2 *evmaps;
  ub1 *sidmaps;

  struct memblk portmem;
  struct memblk subportmem;
  struct memblk hopmem;
  struct memblk sidmem;
  struct memblk timesmem;
  struct memblk eventmem;
  struct memblk evmapmem;

  ub4 latscale,lonscale;
  ub4 latrange[2];
  ub4 lonrange[2];

  ub4 t0,t1;

// workspace
  ub4 *portwrk;   // [portcnt]

// access
  ub4 tthops[Hopcnt];   // index in timetables above
  ub4 farehops[Hopcnt];   // index in fares above

  ub4 *id2ports;         // [maxid]
  ub4 *subid2ports;      // [maxid]
  ub4 *id2hops;          // [maxid]
  ub4 *rsid2sids;
  ub4 *rrid2rid;
  ub4 maxportid;
  ub4 maxsubportid;
  ub4 maxsid;

  ub4 hitripid,hirrid;

  ub4 maxvariants,routevarmask;

  sb2 *faremaps;
};
typedef struct networkbase netbase;

extern netbase *getnetbase(void);
extern int mkrandnet(ub4 portcnt,ub4 hopcnt);
extern int prepbasenet(void);

extern void ininetbase(void);
