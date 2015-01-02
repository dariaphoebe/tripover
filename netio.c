// netio.c - networks to and from file

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* functions to read and write tripover internal networks, utility functions to write diagnostics
 */

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "math.h"
#include "util.h"
#include "time.h"

static ub4 msgfile;
#include "msg.h"

#include "bitfields.h"
#include "netbase.h"

#include "netio.h"

#include <valgrind/memcheck.h>

static ub4 pdfscale_lat = 1200;
static ub4 pdfscale_lon = 1200;

static ub4 lat2pdf(ub4 lat,ub4 lolat,ub4 dlat) { return (lat - lolat) * pdfscale_lat / dlat; }
static ub4 lon2pdf(ub4 lon,ub4 lolon,ub4 dlon) { return (lon - lolon) * pdfscale_lon / dlon; }

static const ub4 pdfmaxports = 1000;
static const ub4 pdfmaxhops = 1000;

static const ub4 timecntlimit = hi32;

static const ub4 timespanlimit = 30; // in days. todo: in config

static const ub4 hop2watch = 0;

static ub4 sid2add = hi32;

#include "watch.h"

/* tripover external format: easy manual editing, typical from single gtfs

files contain tab-separated columns

file ports.txt:

# starts comment
letters introduce commands:
  s <latlon scale,ofs>  scale and offset for lat and lon

numbers start a regular 'port' line:
 id name lat lon

numbers default to hexadecimal. prefix capital D for decimal. lon follows lat radix

file hops.txt
  patterned after ports.txt

commands:
# starts comment
tab introduces commands:
 t from to   # overal time validity of following block

any char except tab and newline introduce a regular hop line:
 name id dport aport route.seq dow.hhmm.rep.t0.t1.dur dow.hhmm.rep ...  # mm implies rep=60min
   dport,aport refers to port.id above
   route refers to route.id below
   seq  = hops from route start
   dow = days of week
   hhmm = hour:min
   rep = repetition interval
   t0,t1 is start and end of repetition
   dur = duration


routes.txt  todo
 id name hopcnt
 ...


  tripover internal format todo: meant for larger nets. typ combine sets of external

 gtfs:

  stop  = port
  route = service e.g. 385
  trip  = triplet
  trip-times = timetable
  shapes = net2pdf
  calendar = part of timetable
  calendar-dates = part of timetable

  airline timetables can use gtfs ?

  perl script to process gtfs into tripover external format
  c tool to combine multiple external nets ( e.g sydney + brisbane ) into internal

 */

enum stopopts { Stopopt_child = 1, Stopopt_parent = 2 };

static const char *kindnames[Kindcnt] = { "unknown","air","rail","bus","ferry","walk" };

enum extresult { Next, Newitem, Newcmd, Eof, Parserr };

static int memeq(const char *s,const char *q,ub4 n) { return !memcmp(s,q,n); }

// count non-empty and non-comment lines
static ub4 linecnt(const char *name,const char *buf, ub4 len)
{
  ub4 pos = 0,cnt = 0;
  const char nl = '\n';

  while (pos < len) {
    if (buf[pos] == '#') { while (pos < len && buf[pos] != nl) pos++; }
    else if (buf[pos] == nl) pos++;
    else {
      cnt++;
      while (pos < len && buf[pos] != nl) pos++;
    }
  }
  if (len && buf[len-1] != nl) warning(0,"%s has unterminated last line",name);
  info(0,"%s: %u data lines", name,cnt);
  return cnt;
}

static ub1 hexmap[256];

static void mkhexmap(void)
{
  char c;

  memset(hexmap,0xff,256);
  for (c = '0'; c <= '9'; c++) hexmap[(ub4)c] = c - '0';
  for (c = 'a'; c <= 'f'; c++) hexmap[(ub4)c] = c + 10 - 'a';
  for (c = 'A'; c <= 'F'; c++) hexmap[(ub4)c] = c + 10 - 'A';
  hexmap[9] = 0x20;
  hexmap[0x0a] = 0xfe;
  hexmap[0x20] = 0x20;
}

static enum extresult __attribute__ ((format (printf,5,6))) parserr(ub4 fln,const char *fname,ub4 linno,ub4 colno,const char *fmt, ...)
{
  va_list ap;
  char buf[1024];
  ub4 pos,len = sizeof(buf);

  pos = fmtstring(buf,"%s.%u.%u: parse error: ",fname,linno,colno);
  if (fmt) {
    va_start(ap,fmt);
    myvsnprintf(buf,pos,len,fmt,ap);
    va_end(ap);
  }
  errorfln(fln,0,FLN,"%s",buf);
  return Parserr;
}

static int __attribute__ ((format (printf,5,6))) inerr(ub4 fln,const char *fname,ub4 linno,ub4 colno,const char *fmt, ...)
{
  va_list ap;
  char buf[1024];
  ub4 pos,len = sizeof(buf);

  pos = fmtstring(buf,"%s.%u.%u: ",fname,linno,colno);
  if (fmt) {
    va_start(ap,fmt);
    myvsnprintf(buf,pos,len,fmt,ap);
    va_end(ap);
  }
  return errorfln(fln,0,FLN,"%s",buf);
}

static int __attribute__ ((format (printf,5,6))) parsewarn(ub4 fln,const char *fname,ub4 linno,ub4 colno,const char *fmt, ...)
{
  va_list ap;
  char buf[1024];
  ub4 pos,len = sizeof(buf);

  pos = fmtstring(buf,"%s.%u.%u: ",fname,linno,colno);
  if (fmt) {
    va_start(ap,fmt);
    myvsnprintf(buf,pos,len,fmt,ap);
    va_end(ap);
  }
  return warnfln(fln,0,"%s",buf);
}

/* basic tab-separated ints parser.
   first item is name, rest are integers
   string starting with . is a command, double dot to escape a dot
   # is comment
   D prefix for decimal, no prefix for hex
 */

#define Maxval 16 * 1024
#define Maxname 256

enum extstates { Init,Out,Val0,Hex1,Dec0,Dec1,Val1,Name,Cmd0,Fls};

struct extfmt {
  struct myfile mf;
  ub4 pos;
  enum extstates state;
  ub4 linno,colno;
  int iscmd;
  char name[Maxname];
  ub4 namelen;
  ub4 radix;
  ub4 val,valndx,vals[Maxval];
};

static enum extresult nextchar(struct extfmt *ef)
{
  char *fname,*name,c;
  ub4 pos,len,linno,colno,x,valndx,namelen;
  ub4 val,*vals;
  ub4 namemax = Maxname - 2;
  int newitem,iscmd;
  ub4 radix;
  enum extstates state;

  len = (ub4)ef->mf.len;
  pos = ef->pos;
//  info(0,"pos %u len %u",pos,len);
  if (pos >= len) return Eof;

  // state
  state = ef->state;
  valndx = ef->valndx;
  val = ef->val;
  linno = ef->linno;
  colno = ef->colno;
  namelen = ef->namelen;
  iscmd = ef->iscmd;
  radix = ef->radix;

  // convenience
  name = ef->name;
  fname = ef->mf.name;
  vals = ef->vals;

  c = ef->mf.buf[pos];
  ef->pos = pos + 1;

  newitem = 0;

//    info(0,"state %u c %c",state,c);

    switch(state) {

    case Init: linno = 1; // cascade
    case Out:
      valndx = 0;
      iscmd = 0;
      radix = 16;
      switch (c) {
        case '#': state = Fls; break;
        case '\t': valndx = namelen = 0; vals[0] = 0; state = Val0; break;
        case '\n': break;
        case ' ': break;
        case '.': iscmd = 1; state = Cmd0; break;
        default: name[0] = c; namelen = 1; state = Name;
      }
      break;

    case Cmd0:
      switch (c) {
        case '#': state = Fls; break;
        case '\t': valndx = namelen = 0; vals[0] = 0; state = Val0; break;
        case '\n': return parserr(FLN,fname,linno,colno,"unexpected newline");
        case '.': iscmd = 0; name[0] = '.'; namelen = 1; state = Name; break;
        default: name[0] = c; namelen = 1; state = Name;
      }
      break;

    case Name:
      if (c == '\t') {
        if (namelen + 2 < namemax) name[namelen] = 0;
        valndx = 0; state = Val0;
      }
      else if (c == '\n') return parserr(FLN,fname,linno,colno,"missing dep arr");
      else if (namelen + 2 < namemax) name[namelen++] = c;
      else if (name[namemax] != '!') {
        parsewarn(FLN,fname,linno,colno,"name exceeds %u",namemax);
        name[namemax] = '!';
      }
      break;

    case Val0:
      switch(c) {
        case '#': newitem = 1; state = Fls; break;
        case '\n': newitem = 1; state = Out; break;
        case '\t': case ' ': break;
        case 'D': case '.': state = Dec0; break;
        case 'x': radix = 16; break;
        case '\'': state = Val1; break;
        case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
          if (radix == 16) {
            val = hexmap[(ub4)c];
            state = Hex1;
          } else {
            val = c - '0';
            state = Dec1;
          }
          break;
        case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
          val = hexmap[(ub4)c];
          state = Hex1;
          break;
        default: return parserr(FLN,fname,linno,colno,"expected digit, found '%c'",c);
      }
      break;

    case Val1:    // generic string todo store
      switch(c) {
        case '#': newitem = 1; state = Fls; break;
        case '\n': newitem = 1; state = Out; break;
        case '\t': case ' ': state = Val0; break;
      }
      break;
    
    case Hex1:  // hex number
      if (valndx >= Maxval) return parserr(FLN,fname,linno,colno,"exceeding %u values",valndx);
      x = hexmap[(ub4)c];
      if (x < 16) val = (val << 4) | x;
      else if (x == 0x20) {
        vals[valndx++] = val;
        vals[valndx] = 0;
        state = Val0;
      } else if (x == 0xfe) { // newline
        vals[valndx++] = val;
        vals[valndx] = 0;
        newitem = 1;
        state = Out;
      } else return parserr(FLN,fname,linno,colno,"expected whitespace after number, found %c",c);
      break;

    case Dec0:  // dec number
      if (c == '.' || c == 'D') radix = 10;
      else if (c < '0' || c > '9') return parserr(FLN,fname,linno,colno,"expected decimal digit, found '%c'",c);
      else { val = c - '0'; state = Dec1; }
      break;

    case Dec1:  // dec number
      if (c == '\t' || c == ' ') {
        if (valndx >= Maxval) return parserr(FLN,fname,linno,colno,"exceeding %u values",valndx);
        vals[valndx++] = val;
        vals[valndx] = 0;
        state = Val0;
      } else if (c >= '0' && c <= '9') val = (val * 10) + (c - '0');
      else if (c == '\n') {
        vals[valndx++] = val;
        vals[valndx] = 0;
        newitem = 1;
        state = Out;
      } else return parserr(FLN,fname,linno,colno,"expected decimal digit, found '%c'",c);
      break;

    case Fls:
      if (c == '\n') { state = Out; } break;

    }

  if (c == '\n') { linno++; colno = 1; }
  else colno++;

  ef->state = state;

  ef->valndx = valndx;
  ef->val = val;
  ef->linno = linno;
  ef->colno = colno;
  ef->namelen = namelen;
  ef->iscmd = iscmd;
  ef->radix = radix;

  if (newitem) return iscmd ? Newcmd : Newitem;
  else return Next;
}

struct cmdvars {
  const char *name;
  ub4 namelen;
  ub4 nval;
  ub4 *pval;
  ub4 linno;
};

static ub4 variants;
static ub4 sumtimes;
static ub4 rtype_walk;
static ub4 rawtripcnt;
static ub4 hitripid;

static struct cmdvars hopvars[] = {
  {"variants",8,1,&variants,0},
  {"walk_id",7,1,&rtype_walk,0},
  {"sumtimes",8,1,&sumtimes,0},
  {"trips",5,1,&rawtripcnt,0},
  {"hitrip",6,1,&hitripid,0},
  {"",0,0,NULL,0}
};

static ub4 timebox[2];  // coded decimal local yyyymmdd inclusive
static ub4 utcofs12 = 1000 + 1200; // coded decimal + 12h
static ub4 utcofs;  // minutes east from utc + 12h
static ub4 dummy;

static struct cmdvars timevars[] = {
  {"timebox",7,2,timebox,0},
  {"dowstart",8,0,&dummy,0},
  {"utcofs",6,1,&utcofs12,0},
  {"",0,0,NULL,0}
};

static int docmd(struct cmdvars *cvs,ub4 namelen,const char *name,ub4 linno,const char *fname,ub4 *vals,ub4 valcnt)
{
  struct cmdvars *cv = cvs;
  ub4 nval,n;

  while (cv->namelen) {
    if (namelen == cv->namelen && memeq(name,cv->name,namelen)) {
      if (cv->linno) return parsewarn(FLN,fname,linno,0,"ignore %s previously defined at %u",name,cv->linno);
      nval = cv->nval;
      if (valcnt < nval) return parsewarn(FLN,fname,linno,0,"%s needs %u args, has %u",name,nval,valcnt);
      cv->linno = linno;
      for (n = 0; n < nval; n++) cv->pval[n] = vals[n];
      if (nval) info(0,"var %s : %u",name,vals[0]);
      return 1;
    }
    cv++;
  }
  return info(0,"ignore unknown cmd %s",name);
}

static int showconstats(struct portbase *ports,ub4 portcnt)
{
  struct portbase *pp;
  ub4 nodep,noarr,nodeparr,udeparr1;
  ub4 hidep,hiarr,hidport,hiaport;
  ub4 port,ndep,narr,n;

  ub4 constats[256];
  ub4 depstats[256];
  ub4 arrstats[256];

  ub4 depivs = Elemcnt(depstats) - 1;
  ub4 arrivs = Elemcnt(arrstats) - 1;

  aclear(constats);
  aclear(depstats);
  aclear(arrstats);
  nodep = noarr = nodeparr = udeparr1 = 0;
  hidep = hiarr = hidport = hiaport = 0;

  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    ndep = pp->ndep; narr = pp->narr;
    if (ndep == 0 && narr == 0) { warning(0,"port %u %u has no connections - %s",port,pp->id,pp->name); nodeparr++; }
    else if (ndep == 0) { vrb(0,"port %u has no deps - %s",port,pp->name); nodep++; }
    else if (narr == 0) { vrb(0,"port %u has no arrs - %s",port,pp->name); noarr++; }
    if (ndep < 16 && narr < 16) constats[(ndep << 4) | narr]++;
    depstats[min(ndep,depivs)]++;
    arrstats[min(narr,arrivs)]++;
    if (ndep > hidep) { hidep = ndep; hidport = port; }
    if (narr > hiarr) { hiarr = narr; hiaport = port; }
  }
  genmsg(nodeparr ? Info : Vrb,0,"%u of %u ports without connection",nodeparr,portcnt);
  if (nodep) info(0,"%u of %u ports without departures",nodep,portcnt);
  if (noarr) info(0,"%u of %u ports without arrivals",noarr,portcnt);
  for (ndep = 0; ndep < 3; ndep++) {
    for (narr = 0; narr < 3; narr++) {
      n = constats[(ndep << 4) | narr];
      if (n) info(0,"%u port\as with %u dep + %u arr", n,ndep,narr);
    }
  }

  for (ndep = 0; ndep <= min(depivs,16); ndep++) {
    n = depstats[ndep];
    if (n) info(0,"%u port\as with %u%s dep", n,ndep,ndep == depivs ? "+" : "");
  }
  for (narr = 0; narr <= min(arrivs,16); narr++) {
    n = arrstats[narr];
    if (n) info(0,"%u port\as with %u%s arr", n,narr,narr == arrivs ? "+" : "");
  }

  pp = ports + hidport;
  info(0,"port %u deps %u arrs %u %s",hidport,hidep,pp->narr,pp->name);
  pp = ports + hiaport;
  info(0,"port %u deps %u arrs %u %s",hiaport,pp->ndep,hiarr,pp->name);
  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    ndep = pp->ndep; narr = pp->narr;
    if (ndep > hidep / 2 || narr > hiarr / 2) info(0,"port %u deps %u arrs %u %s", port,ndep,narr,pp->name);
  }

  return 0;
}

// name id subid lat lon  # comment
static int rdextports(netbase *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawportcnt,extportcnt,portcnt,port;
  struct portbase *ports,*pp;
  int rv;
  char *buf;
  ub4 len,linno,colno,namelen,idlo,idhi,subidhi,mapidlen,maxid;
  ub4 lat,lon,id,subid,opts;
  ub4 lolon,lolat,hilon,hilat;
  char *name;
  ub4 valndx,*vals;
  ub4 latscale,latscaleline;
  ub4 lonscale,lonscaleline;

  latscale = Latscale;
  lonscale = Lonscale;

  struct extport {
    char name[128];
    ub4 namelen;
    ub4 id, subid;
    ub4 opts;
    bool parent,child;
    ub4 subcnt,subofs,seq;
    ub4 lat,lon;
  };
  struct extport *extports,*ep,*pep;
  ub4 namemax = sizeof(extports->name) - 1;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/ports.txt",dir);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawportcnt = linecnt(fname,buf, len);

  if (rawportcnt == 0) return warning(0,"%s is empty",fname);

  extportcnt = 0;
  extports = ep = alloc(rawportcnt,struct extport,0xff,"ext ports",rawportcnt);
  vg_set_undef(ep,rawportcnt * sizeof(struct extport));

//  vg_chk_def(ep,sizeof(struct extport));

  id = idhi = subidhi = maxid = 0;
  idlo = hi32;
  latscaleline = lonscaleline = 0;

  lolat = lolon = hi32;
  hilat = hilon = 0;

  vals = eft.vals;
  name = eft.name;

  do {
    res = nextchar(&eft);

    switch(res) {
    case Newcmd:
      namelen = eft.namelen;
      valndx = eft.valndx;
      linno = eft.linno;
      colno = eft.colno;
      if (namelen == 8 && memeq(name,"latscale",namelen)) {
        if (valndx < 1) return parserr(FLN,fname,linno,colno,"missing args, only %u",valndx);
        if (latscaleline) parsewarn(FLN,fname,linno,colno,"ignore %s previously defined at %u",name,latscaleline);
        else {
          latscaleline = linno;
          latscale = vals[0];
          info(0,"%s : %u",name,latscale);
        }
      } else if (namelen == 8 && memeq(name,"lonscale",namelen)) {
        if (valndx < 1) return parserr(FLN,fname,linno,colno,"missing args, only %u",valndx);
        if (lonscaleline) parsewarn(FLN,fname,linno,colno,"ignore %s previously defined at %u",name,lonscaleline);
        else {
          lonscaleline = linno;
          lonscale = vals[0];
          info(0,"%s : %u",name,lonscale);
        }
      } else info(0,"ignore unknown cmd %s %u",name,vals[0]);
      break;

      case Newitem:
      namelen = eft.namelen;
      valndx = eft.valndx;
      linno = eft.linno;
      colno = eft.colno;
      error_ge(extportcnt,rawportcnt);
      if (valndx < 4) return parserr(FLN,fname,linno,colno,"missing id,subid,lat,lon args, only %u",valndx);

      id = vals[0];
      subid = vals[1];
      lat = vals[2];
      lon = vals[3];
      if (valndx > 3) opts = vals[4];
      else opts = 0;
      vrb(0,"port %u id %u sub %u opts %x",extportcnt,id,subid,opts);
      if (id > idhi) idhi = id;
      if (id < idlo) idlo = id;
      if (subid > subidhi) subidhi = subid;
      clear(ep);
      ep->id = id;
      ep->subid = subid;
      if (id > 80000) info(0,"port %u id %u %x hi %u",extportcnt,id,id,idhi);
      ep->opts = opts;
      ep->parent = (opts & Stopopt_parent);
      ep->child  = (opts & Stopopt_child);
      if (id != subid && ep->parent) parsewarn(FLN,fname,linno,colno,"parent port %u has parent %u",subid,id);
      if (id == subid && ep->child) parsewarn(FLN,fname,linno,colno,"child port %u has no parent",id);

      if (lat >= 180 * latscale) { parsewarn(FLN,fname,linno,colno,"port %u lat %u out of range",id,lat); lat = 0; }
      if (lon >= 360 * lonscale) { parsewarn(FLN,fname,linno,colno,"port %u lon %u out of range",id,lon); lon = 0; }
      ep->lat = lat;
      ep->lon = lon;
      lolat = min(lolat,lat);
      hilat = max(hilat,lat);
      lolon = min(lolon,lon);
      hilon = max(hilon,lon);

      if (namelen > namemax) {
        parsewarn(FLN,fname,linno,colno,"name length %u exceeds max %u",namelen,namemax);
        namelen = namemax;
      }
      ep->namelen = namelen;
      memset(ep->name,0,sizeof(ep->name));
      if (namelen) memcpy(ep->name,name,namelen);
      else { parsewarn(FLN,fname,linno,colno,"port %u has no name",id); ep->name[0] = 0; }
      extportcnt++;
      ep++;
      break;  // newitem

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"read %u ports, id in %u..%u",extportcnt,idlo,idhi);
  if (extportcnt == 0) return 1;

  ub4 pid,subofs,seq;
  ub4 cnt,extport,subport,subportcnt;
  ub8 vgadr;
  struct subportbase *subports,*sp;

  // create mappings
  if (idhi > 100 * 1000 * 1000) warning(0,"max port id %u",idhi);
  if (subidhi > 100 * 1000 * 1000) warning(0,"max subport id %u",subidhi);

  mapidlen = max(idhi,subidhi) + 1;
  ub4 *id2ports = alloc(mapidlen,ub4,0xff,"ext id2port",idhi);
  ub4 *subid2ports = alloc(mapidlen,ub4,0xff,"ext subid2port",subidhi);

  port = 0;
  for (extport = 0; extport < extportcnt; extport++) {
    ep = extports + extport;
    id = ep->id;
    subid = ep->subid;
    error_gt_cc(id,idhi,"port %u",extport);
    error_gt(subid,subidhi,extport);
    ep->subcnt = 0;

    if (id == subid) { // parent or plain port
      if (id == 12792 || id == 7440 || id == 7285) info(0,"port %u id %u subid %u name %s",extport,id,subid,ep->name);
      if (id2ports[id] != hi32) warning(0,"port ID %u doubly defined as %x", id,id2ports[id]);
      else id2ports[id] = extport;
      if (subid2ports[id] != hi32) warning(0,"port subID %u doubly defined as %x", subid,subid2ports[subid]);
      else { subid2ports[id] = extport; if (extport == 0) info(0,"id %u port 0 %s",id,ep->namelen ? ep->name : "(no name)"); port++; }
    } else {
      if (id == 12792 || id == 7440) info(0,"port %u id %u subid %u name %s",extport,id,subid,ep->name);
      if (subid2ports[subid] != hi32) warning(0,"port subID %u doubly defined as %x", subid,subid2ports[subid]);
      else { subid2ports[subid] = extport; if (extport == 0) info(0,"subid %u",subid); }
    }
  }
  portcnt = port;

  for (extport = 0; extport < extportcnt; extport++) {
    ep = extports + extport;
    id = ep->id;
    if (subid2ports[id] == 0) info(0,"port %u id %u map 0 %s",extport,id,ep->name);
   }

  // check station assignment
  for (extport = 0; extport < extportcnt; extport++) {
    ep = extports + extport;
    id = ep->id;
    subid = ep->subid;
    if (id == subid) continue;

    if (subid2ports[id] == hi32) {
      warning(0,"port ID \ax%u has nonexisting parent \ax%u", subid,id);
      ep->id = ep->subid;
      ep->parent = ep->child = 0;
    }
  }

  // count members

  subportcnt = 0;
  for (extport = 0; extport < extportcnt; extport++) {
    ep = extports + extport;
    id = ep->id;
    subid = ep->subid;
    if (id == subid) continue;

    pid = id2ports[id];
    error_ge_cc(pid,extportcnt,"id %u",id);
    error_eq(pid,extport);
    pep = extports + pid;
    cnt = pep->subcnt;
    pep->subcnt = cnt + 1;
    ep->seq = cnt;
    vrb(0,"port %u has parent %u %s %s",subid,id,ep->name,pep->name);
    subportcnt++;
  }

  info(0,"%u ports, %u subports", portcnt, subportcnt);
  error_gt(portcnt,extportcnt,0);
  error_ge(subportcnt,extportcnt);

  if (portcnt == 0) return 0;

  ports = mkblock(&net->portmem,portcnt,struct portbase,Init0,"ext ports");
  net->ports = ports;

  if (subportcnt) {
    subports = mkblock(&net->subportmem,subportcnt,struct subportbase,Init0,"ext subports");
    net->subports = subports;
  } else subports = NULL;

  // assign subport ofs
  subofs = 0;
  for (extport = 0; extport < extportcnt; extport++) {
    ep = extports + extport;
    id = ep->id;
    subid = ep->subid;
    if (id != subid) continue;
    cnt = ep->subcnt;
    ep->subofs = subofs;
    if (cnt == 0) continue;
    vrb(0,"port %u subcnt %u %s",extport,cnt,ep->name);
    subofs += cnt;
  }
  error_ne(subofs,subportcnt);

  // fill
  port = subport = 0;
  for (extport = 0; extport < extportcnt; extport++) {
    ep = extports + extport;
    vg_chk_def(ep,sizeof(struct extport));
    id = ep->id;
    subid = ep->subid;
    opts = ep->opts;
    namelen = ep->namelen;
    lat = ep->lat;
    lon = ep->lon;
    if (id == subid) {
      pp = ports + port;
      pp->id = pp->cid = id;
      pp->parentsta = ep->parent;
      pp->lat = lat;
      pp->lon = lon;
      pp->namelen = namelen;
      if (namelen) memcpy(pp->name,ep->name,namelen);
      cnt = ep->subcnt;
      if (cnt) { // station with members
        subofs = ep->subofs;
        error_ge(subofs,subportcnt);
        pp->subcnt = cnt;
        pp->subofs = subofs;
      } else if (ep->parent) vrb(0,"parent station %u has no member stops %s",id,ep->name);
      port++;
    } else { // sub
      pid = id2ports[id];
      error_ge(pid,extportcnt);
      pep = extports + pid;
      vg_chk_def(pep,sizeof(struct extport));
      cnt = pep->subcnt;
      subofs = pep->subofs;
      error_ge(subofs,subportcnt);
      seq = ep->seq;
      vrb(0,"seq %u cnt %u",seq,cnt);
      error_z(cnt,pid);
      subport = subofs + seq;
      error_ge(subport,subportcnt);
      sp = subports + subport;
      error_nz(sp->id,subport);
      sp->id = id;
      sp->subid = subid;
      sp->seq = seq;
      sp->namelen = namelen;
      vgadr = vg_chk_def(ep,sizeof(struct extport)-1);
      infocc(vgadr != 0,0,"port %u ep undefined at ofs %ld",extport,(char *)vgadr - (char *)ep);
      vgadr = vg_chk_def(sp,sizeof(struct subportbase)-1);
      infocc(vgadr != 0,0,"port %u sp undefined at ofs %ld",subport,(char *)vgadr - (char *)sp);
      if (namelen) memcpy(sp->name,ep->name,namelen);
    }
  }

  // resequence and arrange
  for (id = 0; id <= idhi; id++) id2ports[id] = hi32;

  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    id = pp->id;
    id2ports[id] = port;
    pp->rlat = lat2rad(pp->lat,Latscale);
    pp->rlon = lon2rad(pp->lon,Lonscale);
//    info(0,"port %u geo %u,%u %e,%e %s",port,pp->lat,pp->lon,pp->rlat,pp->rlon,pp->name);
  }
  for (subid = 0; subid <= subidhi; subid++) subid2ports[subid] = hi32;
  for (port = 0; port < subportcnt; port++) {
    sp = subports + port;
    subid = sp->subid;
    subid2ports[subid] = port;
  }

  info(0,"read %u stops into %u ports from %s", extportcnt,portcnt, fname);
  info(0,"bbox lat %u - %u = %u scale %u",lolat,hilat,hilat-lolat,latscale);
  info(0,"bbox lon %u - %u = %u scale %u",lolon,hilon,hilon-lolon,lonscale);
  net->portcnt = portcnt;
  net->subportcnt = subportcnt;
  net->id2ports = id2ports;
  net->subid2ports = subid2ports;
  net->maxportid = idhi;
  net->maxsubportid = subidhi;
  net->latscale = latscale;
  net->lonscale = lonscale;
  net->latrange[0] = lolat;
  net->latrange[1] = hilat;
  net->lonrange[0] = lolon;
  net->lonrange[1] = hilon;
  return 0;
}

// expand regular sid part into localtime day map. times in days
static void expandsid(ub1 *map,ub4 maplen,ub4 t00,ub4 t0,ub4 t1,ub4 dow,ub4 t0day)
{
  ub4 t = t0 - t00;
  ub4 daymask = (1 << t0day);

  while (t < t1 - t00 && t < maplen) {
    if (daymask & dow) map[t] = 2;
    if (daymask == (1 << 6)) daymask = 1;
    else daymask <<= 1;
    t++;
  }
}

// service_id tid dow start end 
static int rdexttimes(netbase *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawsidcnt,sidcnt;
  struct sidbase *sids,*sp;
  ub4 rsid,sid,*rsid2sids;
  int rv;
  char *buf;
  ub4 len,linno,colno,namelen,valndx,valcnt,id,idhi,maxsid = 0;
  ub4 dow,t0,t1,t0_cd,t1_cd,t_cd,tbox0,tbox1,dtbox,t0wday;
  ub4 daybox0,daybox1,t0days,t1days,tday,cnt,daycnt;
  ub4 hh,mm;
  ub4 t0lo = hi32,t1hi = 0;
  char *name;
  ub4 *vals;
  ub1 *sidmaps,*map;
  ub4 mapofs,addcnt,subcnt,addndx,subndx;
  ub4 namemax = sizeof(sids->name) - 1;
  int initvars = 0;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/times.txt",dir);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawsidcnt = linecnt(fname,buf, len);

  if (rawsidcnt == 0) return warning(0,"%s is empty",fname);
  sidcnt = 0;

  rawsidcnt++;
  sids = sp = mkblock(&net->sidmem,rawsidcnt,struct sidbase,Init0,"");

  id = idhi = maxsid = 0;
  tbox0 = tbox1 = dtbox = daybox0 = daybox1 = 0;
  sidmaps = NULL;
  mapofs = 0;

  vals = eft.vals;
  name = eft.name;

  do {
    res = nextchar(&eft);

    switch(res) {

    case Newcmd:
      namelen = eft.namelen;
      valndx = eft.valndx;
      linno = eft.linno;
      colno = eft.colno;
      docmd(timevars,namelen,name,linno,fname,vals,valndx);
      break;

    case Newitem:
      if (initvars == 0) {
        if (timebox[1] < timebox[0]) {
          warning(0,"negative timebox from %u to %u",timebox[0],timebox[1]);
          timebox[1] = timebox[0] + 1;
        }
        tbox0 = timebox[0]; tbox1 = timebox[1];
        daybox0 = cd2day(tbox0);
        daybox1 = cd2day(tbox1) + 1;
        dtbox = daybox1 - daybox0;
        if (dtbox > timespanlimit) {
          warning(0,"timebox %u-%u limited to %u days",tbox0,tbox1,timespanlimit);
          daybox1 = daybox0 + timespanlimit;
          tbox1 = timebox[1] = day2cd(daybox1);
          dtbox = daybox1 - daybox0 + 1;
        }
        if (utcofs12 < 1200) { warning(0,"UTCoffset %d below lowest -1100", utcofs12 - 1200); utcofs12 = 1200; }
        else if (utcofs12 > 1400 + 1200) { warning(0,"UTCoffset %u above highest +1400", utcofs12 - 1200); utcofs12 = 1200; }
        hh = utcofs12 / 100;
        mm = utcofs12 % 100;
        utcofs = hh * 60 + mm;
        info(0,"UTC offset %u from %u:%u - 12:00",utcofs - 12 * 60,hh,mm);
        sidmaps = alloc(rawsidcnt * (dtbox+1),ub1,0,"time sidmap",dtbox);
        initvars = 1;
      }
      namelen = eft.namelen;
      valcnt = eft.valndx;
      linno = eft.linno;
      colno = eft.colno;
      error_gt(sidcnt+1,rawsidcnt,linno);
      if (valcnt < 6) return parserr(FLN,fname,linno,colno,"expect svcid,sid,dow,start,end,n+,n- args, found only %u args",valcnt);
      rsid = vals[0];
      dow = vals[1];
      t0_cd = vals[2];
      t1_cd = vals[3];
      addcnt = vals[4];
      subcnt = vals[5];
      valndx = 6;

      // overall date range in localtime days is daybox0 .. daybox1
      // sids are based no this
      vrb(0,"rsid %x dow %x %u..%u +%u -%u",rsid,dow,t0_cd,t1_cd,addcnt,subcnt);

      if (addcnt > dtbox) {
        infovrb(timespanlimit == hi32,0,"line %u sid %u has %u calendar entries, time range %u",linno,rsid,addcnt,dtbox);
        addcnt = dtbox;
      }
      if (subcnt > dtbox) {
        warning(0,"line %u sid %u has %u calendar entries, time range %u",linno,rsid,subcnt,dtbox);
        subcnt = dtbox;
      }
      if (valndx + addcnt + subcnt != valcnt) {
//        parsewarn(FLN,fname,linno,colno,"expected 6 + %u + %u args, found %u",addcnt,subcnt,valcnt); todo
      }

      if (dow > 0x7f) return inerr(FLN,fname,linno,colno,"invalid dayofweek mask %x",dow);

      if (t0_cd < tbox0) {
        warning(0,"line %u: sid %u has start date %u before %u",linno,rsid,t0_cd,tbox0);
        t0_cd = tbox0;
      }
      if (t1_cd > tbox1) {
        infovrb(timespanlimit == hi32,0,"line %u: sid %u has end date %u after %u",linno,rsid,t1_cd,tbox1);
        t1_cd = tbox1;
      }

      t0wday = cdday2wday(t0_cd);
      map = sidmaps + mapofs;

      error_z(dtbox,0);

      // expand the regular calendar
      if (dow) {
        t0days = cd2day(t0_cd);
        t1days = cd2day(t1_cd);
        expandsid(map,dtbox,daybox0,t0days,t1days,dow,t0wday);
      }

      // add and remove individual items
      addndx = 0;
      while (addndx++ < addcnt && valndx < valcnt) {
        t_cd = vals[valndx++];
        if (t_cd < tbox0) {
          warning(0,"line %u: sid %u has date %u before %u",linno,rsid,t0_cd,tbox0);
          continue;
        }
        else if (t_cd > tbox1) {
          infovrb(timespanlimit == hi32,0,"line %u: sid %u has end date %u after %u",linno,rsid,t1_cd,tbox1);
          continue;
        }
        tday = cd2day(t_cd);
        if (rsid == 0x1dc87) info(0,"rsid %x add %u - %u = %u at %u %p",rsid,t_cd,tbox0,tday - daybox0,mapofs + tday - daybox0,map + tday - daybox0);
        map[tday - daybox0] = 1;
      }

      subndx = 0;
      while (subndx++ < subcnt && valndx < valcnt) {
        t_cd = vals[valndx++];
        if (t_cd < tbox0) {
          warning(0,"line %u: sid %u has date %u before %u",linno,rsid,t0_cd,tbox0);
          continue;
        }
        if (t_cd > tbox1) {
          infovrb(timespanlimit == hi32,0,"line %u: sid %u has end date %u before %u",linno,rsid,t1_cd,tbox1);
          continue;
        }
        tday = cd2day(t_cd);
        if (map[tday - daybox0]) {
          vrb(0,"delete %u %u",t_cd,tday);
          map[tday - daybox0] = 0;
        }
      }

      // determine range
      daycnt = 0;
      for (tday = 0; tday < dtbox; tday++) {
        cnt = map[tday];
//        if (rsid == 0x1dc87 && cnt) info(0,"rsid %x day %u",rsid,tday);
        if (cnt) daycnt++;
      }
      tday = 0;
      while (tday < dtbox && map[tday] == 0) tday++;
      if (tday == dtbox) {
        warning(0,"line %u: rsid \ax%u has no service days",linno,rsid);
        t0_cd = t1_cd;
      } else {
        t0_cd = day2cd(tday + daybox0);
        tday = dtbox - 1;
        while (tday && map[tday] == 0) tday--;
        t1_cd = day2cd(tday + daybox0 + 1);
      }

      t0 = yymmdd2min(t0_cd,utcofs);
      t1 = yymmdd2min(t1_cd,utcofs) + 1440;

      t0lo = min(t0lo,t0);
      t1hi = max(t1hi,t1);

      rsidlog(rsid,"%u days in dow %x %u..%u \ad%u-\ad%u",daycnt,dow,t0_cd,t1_cd,t0,t1);

      error_le(t1,t0);

      sp->sid = sidcnt;
      sp->rsid = rsid;
      sp->t0 = t0;
      sp->t1 = t1;
      sp->utcofs = utcofs;
      sp->t0map = yymmdd2min(tbox0,utcofs);
      sp->lt0day = daybox0;
      sp->lt1day = daybox1;
      sp->maplen = dtbox + 1;
      error_gt(t0,t1,linno);
      error_zz(t0,t1);

      if (namelen > namemax) {
        parsewarn(FLN,fname,linno,colno,"name length %u exceeds max %u",namelen,namemax);
        namelen = namemax;
      }
      sp->namelen = namelen;
      memcopy(sp->name,name,namelen);
      maxsid = max(maxsid,rsid);
      sp->mapofs = mapofs;
      mapofs += dtbox + 1;
      sp++;
      sidcnt++;
      break;

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  // add internal generic
  sp->sid  = sidcnt++;
  sp->rsid = 0;
  sp->t0 = t0lo;
  sp->t1 = t1hi;
  sp->utcofs = utcofs;

  if (maxsid > 100 * 1000 * 1000) warning(0,"max service id %u",maxsid);
  rsid2sids = alloc(maxsid+1,ub4,0xff,"sid2tids",maxsid);
  for (sid = 0; sid < sidcnt; sid++) {
    sp = sids + sid;
    rsid = sp->rsid;
    if (rsid2sids[rsid] != hi32) warning(0,"service ID %u doubly defined", rsid);
    else rsid2sids[rsid] = sid;
  }
  net->rsid2sids = rsid2sids;

  info(0,"read %u sids from %s, overall period %u to %u", sidcnt, fname,t0lo,t1hi);
  net->sidcnt = sidcnt;
  net->sids = sids;
  net->sidmaps = sidmaps;
  net->maxsid = maxsid;

  ub4 gt0,gt1;

  if (tbox0 && tbox1) {
    gt0 = yymmdd2min(tbox0,utcofs);
    gt1 = yymmdd2min(tbox1,utcofs);
    if (gt0 != t0lo) { warning(0,"overall start %u != %u",gt0,t0lo); gt0 = t0lo; }
     if (gt1 != t1hi) { warning(0,"overall end %u != %u",gt1,t1hi); gt1 = t1hi; }
    if (gt1 <= gt0) { warning(0,"overall start %u beyond end %u",gt0,gt1); gt1 = gt0 + 1; }
  } else { gt0 = t0lo; gt1 = t1hi; }
  net->t0 = gt0; net->t1 = gt1;

  return 0;
}

// match with gtfstool
#define Fmt_prvsid 1
#define Fmt_diftid 2
#define Fmt_diftdep 4
#define Fmt_diftarr 8

// name id dport.id aport.id route.seq (dow.hhmm.rep.t0.t1.dur dow.hhmm.rep)+ 
static int rdexthops(netbase *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 hop,rawhopcnt,hopcnt;
  ub4 portcnt,subportcnt;
  ub4 chain,chaincnt;
  ub4 maxportid;
  struct hopbase *hops,*hp;
  struct portbase *ports,*pdep,*parr;
  struct subportbase *subports,*sbp;
  struct sidbase *sids,*sp;
  struct chainbase *chains = NULL,*cp;
  ub4 *rtid2tid = NULL,*rtidrefs = NULL;
  ub4 *id2ports, *subid2ports;
  ub4 rsid,sid,sidcnt,*rsid2sids;
  int rv;
  char *buf;
  ub4 len,linno,colno,val,namelen,valndx,id,idhi,maxid,maxrid,maxsid;
  ub4 depid,arrid,dep,arr,pid,rtype,routeid,timecnt;
  char *name;
  ub4 *vals;
  ub4 namemax = sizeof(hops->name) - 1;
  ub4 kinds[Kindcnt];
  enum txkind kind;

  ub4 *tbp,*timesbase = NULL;
  ub4 timespos = 0;
  ub4 rtid,tid,tdep,tarr,prvsid,prvtid,prvtdep,prvtarr;
  ub4 t0,t1,ht0,ht1;
  ub4 fmt,vndx,tndx;

  struct eta eta;

  aclear(kinds);
  oclear(eft);

  fmtstring(eft.mf.name,"%s/hops.txt",dir);
  fname = eft.mf.name;

  info(0,"reading hops from %s",fname);
  rv = readfile(&eft.mf,fname,1);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawhopcnt = linecnt(fname,buf,len);

  if (rawhopcnt == 0) return warning(0,"%s is empty",fname);
  hop = chain = 0;

  hops = hp = mkblock(&net->hopmem,rawhopcnt,struct hopbase,Init0,"");

  portcnt = net->portcnt;
  subportcnt = net->subportcnt;
  ports = net->ports;
  subports = net->subports;
  id2ports = net->id2ports;
  subid2ports = net->subid2ports;
  maxportid = net->maxportid;

  rsid2sids = net->rsid2sids;
  sids = net->sids;
  sidcnt = net->sidcnt;

  maxsid = net->maxsid;

  id = idhi = maxid = maxrid = 0;

  vals = eft.vals;
  name = eft.name;

  int inited = 0;
  ub4 cumhoprefs = 0;
  chaincnt = 0;

  do {

    res = nextchar(&eft);
    switch(res) {

    case Newcmd:
      namelen = eft.namelen;
      valndx = eft.valndx;
      linno = eft.linno;
      colno = eft.colno;
      docmd(hopvars,namelen,name,linno,fname,vals,valndx);
    break;

    case Newitem:
      if (inited == 0) {
        if (sumtimes) {
          timesbase = net->timesbase = mkblock(&net->timesmem,sumtimes * 4,ub4,Init0,"time timebase %u",sumtimes);
        }
        if (sid2add != hi32) {
          chaincnt = 1; // entry 0 is generic internal
          rawtripcnt++;
        } else chaincnt = 0;
        if (rawtripcnt) {
          rtid2tid = alloc(hitripid + 1,ub4,0xff,"chain tripids",hitripid);
          rtidrefs = alloc(hitripid + 1,ub4,0,"chain triprefs",hitripid);
        }
        inited = 1;
      }

      if (progress(&eta,"reading hop %u of %u, \ah%u time entries",hop,rawhopcnt,timespos)) return 1;

      namelen = eft.namelen;
      valndx = eft.valndx;

      vrb(0,"%u values",valndx);
      linno = eft.linno;
      colno = eft.colno;
      if (namelen > namemax) {
        parsewarn(FLN,fname,linno,colno,"name length %u exceeds max %u",namelen,namemax);
        namelen = namemax;
      }
      error_gt(hop+1,rawhopcnt,linno);
      if (valndx < 6) return parserr(FLN,fname,linno,colno,"missing dep,arr,type args, only %u",valndx);
      id = vals[0];
      depid = vals[1];
      arrid = vals[2];
      rtype = vals[3];
      routeid = vals[4];
      if (routeid != hi32) maxrid = max(maxrid,routeid);

      timecnt = vals[5];

      if (depid == arrid) return inerr(FLN,fname,linno,colno,"dep id %u,%x equal to arr id",depid,depid);
      if (depid > maxportid) return inerr(FLN,fname,linno,colno,"dep id %u above highest port id %u",depid,maxportid);
      else if (arrid > maxportid) return inerr(FLN,fname,linno,colno,"arr id %u above highest port id %u",arrid,maxportid);

      dep = id2ports[depid];
      if (dep == hi32) {
        dep = subid2ports[depid];
        if (dep >= subportcnt) return inerr(FLN,fname,linno,colno,"dep %u id %u above highest subport %u",dep,depid,subportcnt);
        sbp = subports + dep;
        pid = sbp->id;
        dep = id2ports[pid];
      }
      if (dep >= portcnt) return inerr(FLN,fname,linno,colno,"dep %u id %u above highest port %u",dep,depid,portcnt);

      arr = id2ports[arrid];
      if (arr == hi32) {
        arr = subid2ports[arrid];
        if (arr >= subportcnt) return inerr(FLN,fname,linno,colno,"arr %u above highest subport %u",arr,subportcnt);
        sbp = subports + arr;
        pid = sbp->id;
        arr = id2ports[pid];
      }
      if (arr >= portcnt) return inerr(FLN,fname,linno,colno,"arr %u above highest port %u",arr,portcnt);
      pdep = ports + dep;
      parr = ports + arr;

      if (timecnt && !sumtimes) return inerr(FLN,fname,linno,colno,"hop %u-%u has %u times, sumtimes var zero",depid,arrid,timecnt);
      if (timespos + timecnt > sumtimes) {
        warning(0,"%s.%u: hop %u has %u time entries, sum %u",fname,linno,hop,timecnt,sumtimes);
      }
      error_zp(timesbase,0);
      if (timesbase == NULL) return 1;
      if (timecnt * 4 > valndx - 6) return parserr(FLN,fname,linno,colno,"%u time entries, but only %u args",timecnt,valndx);

      error_zp(timesbase,timecnt);
      tbp = timesbase + timespos * 4;

      timecnt = min(timecnt,timecntlimit); // todo tmp

      tndx = 0; vndx = 6;
      rsid = rtid = tdep = tarr = 0;
      ht0 = hi32; ht1 = 0;

      if (sid2add != hi32) {
        rsid = sid2add;
        sid = rsid2sids[rsid];
        error_ge(sid,sidcnt);
        sp = sids + sid;
        sp->refcnt++;

        t0 = sp->t0;
        t1 = sp->t1;
        tdep = 100; tarr = 200; tid = 0;
        if (hop == hop2watch) info(0,"added sid %u rsid %x tid %x dep %x arr %x \ad%u-\ad%u",sid,rsid,tid,tdep,tarr,t0,t1);

        ht0 = min(ht0,t0);
        ht1 = max(ht1,t1);

        tbp[0] = sid;
        tbp[1] = tid;
        tbp[2] = tdep;
        tbp[3] = tarr;
        tbp += 4;
        tndx++;
      }

      hoplog(hop,1,"at %u %u-%u %s to %s",linno,dep,arr,pdep->name,parr->name);

      while (vndx + 4 <= valndx && tndx < timecnt) {
        prvsid = rsid; prvtid = rtid; prvtdep = tdep; prvtarr = tarr;
        fmt = vals[vndx++];
        if (fmt & Fmt_prvsid) rsid = prvsid;
        else rsid = vals[vndx++];
        rtid = vals[vndx];
        tdep = vals[vndx+1];
        tarr = vals[vndx+2];
        vndx += 3;
        if (fmt & Fmt_diftid) rtid += prvtid;

        error_gt(rtid,hitripid,hop);
        if (rawtripcnt) {
          tid = rtid2tid[rtid];
          if (tid == hi32) {
            error_ge(chaincnt,rawtripcnt);
            tid = chaincnt++;
            rtid2tid[rtid] = tid;
          }
          rtidrefs[rtid]++;
        } else tid = hi32;
        cumhoprefs++;

        if (fmt & Fmt_diftdep) tdep += prvtdep;
        if (fmt & Fmt_diftarr) tarr += prvtarr;

        if (tarr < tdep) {
          parsewarn(FLN,fname,linno,colno,"hop %u arr %u before dep %u",hop,tarr,tdep);
          tarr = tdep;
        }
        if (rsid > maxsid) return inerr(FLN,fname,linno,colno,"service id %u above highest id %u",rsid,maxsid);
        sid = rsid2sids[rsid];

        error_ge(sid,sidcnt);
        sp = sids + sid;
        sp->refcnt++;

        t0 = sp->t0;
        t1 = sp->t1;
        hoplog(hop,1,"at %u.%u rsid \ax%u tid %u,%u td \ad%u ta \ad%u",linno,tndx,rsid,tid,rtid,tdep,tarr);

        ht0 = min(ht0,t0);
        ht1 = max(ht1,t1);

        tbp[0] = sid;
        tbp[1] = tid;
        tbp[2] = tdep;
        tbp[3] = tarr;
        tbp += 4;
        tndx++;
      }
      if (tndx != timecnt) parsewarn(FLN,fname,linno,colno,"%u time entries, but only %u args",timecnt,valndx);
      error_ne(tndx,timecnt);
      if (tndx) {
        hp->t0 = ht0;
        hp->t1 = ht1 + 1440; // tdep can be above 24h
      }
      hoplog(hop,0,"t range %u-%u \ad%u \ad%u",ht0,ht1,ht0,ht1);

      if (depid == arrid) return inerr(FLN,fname,linno,colno,"dep id %u,%x equal to arr id",depid,depid);
      if (depid > maxportid) return inerr(FLN,fname,linno,colno,"dep id %u above highest port id %u",depid,maxportid);
      else if (arrid > maxportid) return inerr(FLN,fname,linno,colno,"arr id %u above highest port id %u",arrid,maxportid);

      dep = id2ports[depid];
      if (dep == hi32) {
        dep = subid2ports[depid];
        if (dep >= subportcnt) return inerr(FLN,fname,linno,colno,"dep %u id %u above highest subport %u",dep,depid,subportcnt);
        sbp = subports + dep;
        pid = sbp->id;
        dep = id2ports[pid];
      }
      if (dep >= portcnt) return inerr(FLN,fname,linno,colno,"dep %u id %u above highest port %u",dep,depid,portcnt);

      arr = id2ports[arrid];
      if (arr == hi32) {
        arr = subid2ports[arrid];
        if (arr >= subportcnt) return inerr(FLN,fname,linno,colno,"arr %u above highest subport %u",arr,subportcnt);
        sbp = subports + arr;
        pid = sbp->id;
        arr = id2ports[pid];
      }
      if (arr >= portcnt) return inerr(FLN,fname,linno,colno,"arr %u above highest port %u",arr,portcnt);
      pdep = ports + dep;

      if (dep == arr) {
        info(0,"line %u hop id %u dep %u id %u equal to arr id %u %s",linno,id,dep,depid,arrid,pdep->name);
      } else {
        parr = ports + arr;
        pdep->ndep++;
        parr->narr++;
        if (pdep->parentsta) vrb(0,"hop %u dport %u %u %s",id,dep,pdep->id,pdep->name);
        if (parr->parentsta) vrb(0,"hop %u aport %u %u %s",id,arr,parr->id,parr->name);

        hp->id  = id;
        hp->dep = dep;
        hp->arr = arr;

// todo generalise
        if (rtype_walk && rtype == rtype_walk) { hp->kind = Walk; kinds[Walk]++; routeid = hi32; }
        else if (rtype == 0 || rtype == 1 || rtype == 2) { hp->kind = Rail; kinds[Rail]++; }
        else if (rtype == 3) { hp->kind = Bus; kinds[Bus]++; }
        else if (rtype == 4) { hp->kind = Ferry; kinds[Ferry]++; }
        else { hp->kind = Unknown; kinds[Unknown]++; }

        hp->rrid = routeid;
        hp->namelen = namelen;
        memcopy(hp->name,name,namelen);
        maxid = max(maxid,id);
        hp->timespos = timespos;
        hp->timecnt = timecnt;
        timespos += timecnt;
        hp++;
        hop++;
      }
     break;  // newitem

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);

  net->timescnt = timespos;

  for (kind = 0; kind < Kindcnt; kind++) {
    val = kinds[kind];
    if (val) info(0,"%u %s hop\as",val,kindnames[kind]);
  }
  hopcnt = hop;

  info(0,"%u hops, %u time entries",hopcnt,sumtimes);

  ub4 sidrefs = 0;

  for (sid = 0; sid < sidcnt; sid++) {
    sp = sids + sid;
    if (sp->refcnt) sidrefs++;
    else warning(0,"sid %x not referenced %s",sp->sid,sp->name);
  }
  if (sidrefs < sidcnt) info(0,"%u sid\as not referenced",sidcnt - sidrefs); // todo filter ?

  ub4 *id2hops;
  ub4 varmask;

  if (maxid > 100 * 1000 * 1000) warning(0,"max hop id %u",maxid);
  id2hops = alloc(maxid+1,ub4,0xff,"id2hops",maxid);
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    id = hp->id;
    if (id2hops[id] != hi32) warning(0,"hop ID %u doubly defined", id);
    else id2hops[id] = hop;
  }
  net->id2hops = id2hops;

  showconstats(ports,portcnt);

  info(0,"read %u hops from %s", hopcnt, fname);
  net->hopcnt = hopcnt;
  net->hops = hops;
  net->hirrid = maxrid;

  // lower routeid bits are variants
  if (variants) varmask = ~(1 << variants);  // add lsb for direction
  else varmask = hi32;
  info(0,"%u possible variants, mask %x", variants,varmask);

  net->maxvariants = variants;
  net->routevarmask = varmask;

  net->hitripid = hitripid;

  if (chaincnt == 0) {
    afree(rtid2tid,"rtid2tid");
    return info0(0,"0 chains");
  }

  ub4 nilchains = 0;
  chains = alloc(chaincnt,struct chainbase,0,"chain",chaincnt);
  for (rtid = 0; rtid <= hitripid; rtid++) {
    tid = rtid2tid[rtid];
    if (tid == hi32) continue;
    cp = chains + tid;
    cp->hoprefs = rtidrefs[rtid];
    if (cp->hoprefs == 0) nilchains++;
    cp->rtid = rtid;
  }
  afree(rtid2tid,"rtid2tid");
  if (nilchains) info(0,"%u of %u chains with hops",chaincnt - nilchains,chaincnt);
  else info(0,"%u chains",chaincnt);

  net->rawchaincnt = chaincnt;
  net->chainhopcnt = cumhoprefs;
  net->chains = chains;

  return 0;
}

int readextnet(netbase *net,const char *dir)
{
  int rv;
  ub4 portcnt;

  info(0,"reading base net in tripover external format from dir %s",dir);

  rsid2logcnt = getwatchitems("rsid",rsids2log,Elemcnt(rsids2log));
  hop2logcnt = getwatchitems("hop",hops2log,Elemcnt(hops2log));

  hoplog(hi32,1,"%c",0);

  rv = rdextports(net,dir);
  if (rv) return rv;

  portcnt = net->portcnt;
  if (portcnt) net->portwrk = alloc(portcnt,ub4,0,"portwrk",portcnt);

  rv = rdexttimes(net,dir);
  if (rv) return rv;

  rv = rdexthops(net,dir);
  info(0,"done reading base net in external format, status %d",rv);
  if (rv) return rv;

  if (globs.writext) rv = net2ext(net);
  if (globs.writpdf) net2pdf(net);
  return rv;
}

static ub4 lat2ext(ub4 lat) { return lat; }
static ub4 lon2ext(ub4 lon) { return lon; }

// write port reference for name and lat/lon lookup
int wrportrefs(netbase *net)
{
  int fd;
  char buf[4096];
  ub4 pos,x,y;
  ub4 buflen = sizeof(buf);

  struct portbase *pp,*ports = net->ports;

  ub4 port,wportcnt = 0,portcnt = net->portcnt;
  const char *portsname = "portrefs.txt";

  char nowstr[64];
  const char *tz;

#ifdef NOW
  sec70toyymmdd(NOW,nowstr,sizeof(nowstr));
  tz = "utc";
#else
  strcopy(nowstr,__DATE__);
  tz = "localtime";
#endif

  info(0,"writing %u-ports reference ",portcnt);

  fd = filecreate(portsname,1);
  if (fd == -1) return 1;

  pos = fmtstring(buf,"# %s - tripover port name and lat/lon lookup table\n\n",portsname);

  pos += mysnprintf(buf,pos,buflen,"# written by tripover version %u.%u  %s %s\n\n", Version_maj,Version_min,nowstr,tz);
  pos += mysnprintf(buf,pos,buflen,"# %u ports, bounding box todo\n\n",portcnt);

  pos += mysnprintf(buf,pos,buflen,"# gid\tname\tlat\tlon\n\n");

  if (filewrite(fd,buf,pos,portsname)) return 1;

  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    if (pp->ndep == 0 && pp->narr == 0) continue;

    y = lat2ext(pp->lat);
    x = lon2ext(pp->lon);
    pos = fmtstring(buf,"%u\t%s\t%u\t%u\n", port,pp->name,y,x);
    if (filewrite(fd,buf,pos,portsname)) return 1;
    wportcnt++;
  }
  fileclose(fd,portsname);
  return info(0,"wrote %u ports to %s",wportcnt,portsname);
}

int net2ext(netbase *net)
{
  int fd;
  char buf[1024];
  ub4 pos,x,y;
  ub4 dec = globs.extdec;

  struct hopbase *hp,*hops = net->hops;
  struct portbase *pp,*ports = net->ports;

  ub4 hop,hopcnt = net->hopcnt;
  ub4 port,portcnt = net->portcnt;
  const char *portsname = "ports.txt";
  const char *hopsname = "hops.txt";

  info(0,"writing %u-ports %u-hops base net to dir '.' in external format",portcnt,hopcnt);

  fd = filecreate(portsname,1);
  if (fd == -1) return 1;

  pos = fmtstring(buf,"# hops %u ports %u\n# id name lat lon\n",hopcnt,portcnt);
  if (filewrite(fd,buf,pos,portsname)) return 1;

  // temporary: port as number
  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    y = lat2ext(pp->lat);
    x = lon2ext(pp->lon);
    if (dec) pos = fmtstring(buf,"D%u\t%s\tD%u\t%u\n", pp->id,pp->name,y,x);
    else pos = fmtstring(buf,"%x\t%s\t%x\t%x\n", pp->id,pp->name,y,x);
    if (filewrite(fd,buf,pos,portsname)) return 1;
  }
  fileclose(fd,portsname);
  info(0,"wrote %u ports to %s",portcnt,portsname);

  fd = filecreate(hopsname,1);
  if (fd == -1) return 1;

  pos = fmtstring(buf,"# hops %u ports %u\n# id dep arr\n",hopcnt,portcnt);
  if (filewrite(fd,buf,pos,hopsname)) return 1;

  for(hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    if (dec) pos = fmtstring(buf,"%s\tD%u\tD%u\tD%u\n",hp->name,hop,hp->dep,hp->arr);
    else pos = fmtstring(buf,"%s\t%x\t%x\t%x\n",hp->name,hop,hp->dep,hp->arr);
    if (filewrite(fd,buf,pos,hopsname)) return 1;
  }
  fileclose(fd,hopsname);
  info(0,"wrote %u hops to %s",hopcnt,hopsname);

  info(0,"done writing %u-ports %u-hops base net to dir '.' in external format",portcnt,hopcnt);

  return 0;
}

// write network as page content
// todo overlay planned trip in distinct color
static ub4 addnetpdf(netbase *net, char *buf, ub4 len)
{
  struct portbase *pp,*pdep,*parr,*ports = net->ports;
  struct hopbase *hp,*hops = net->hops;
  ub4 port,portcnt = net->portcnt;
  ub4 hop,hopcnt = net->hopcnt;
  ub4 x,y,x0,y0,x1,y1,arr,dep;
  ub4 lolon,lolat,hilon,hilat,dlon,dlat;
  ub4 pos,n;

  pos = mysnprintf(buf,0,len,"BT /F1 18 Tf 25 25 Td (title) Tj ET\n");

  pos += mysnprintf(buf,pos,len,"BT /F1 16 Tf\n");

  lolon = net->lonrange[0];
  hilon = net->lonrange[1];
  lolat = net->latrange[0];
  hilat = net->latrange[1];
  dlon = max(1,hilon - lolon);
  dlat = max(1,hilat - lolat);

  if (portcnt > pdfmaxports) {
    info(0,"limiting %u ports to %u for pdf",portcnt,pdfmaxports);
    portcnt = pdfmaxports;
  }
  if (hopcnt > pdfmaxhops) {
    info(0,"limiting %u hops to %u for pdf",hopcnt,pdfmaxhops);
   hopcnt = pdfmaxhops;
  }

  // temporary: port as number
  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    y = lat2pdf(pp->lat,lolat,dlat);
    x = lon2pdf(pp->lon,lolon,dlon);
    n = mysnprintf(buf,pos,len,"1 0 0 1 %u %u Tm (%u) Tj ", x,y,port);
    if (n == 0) break;
    pos += n;
  }
  pos += mysnprintf(buf,pos,len,"ET\n");

  pos += mysnprintf(buf,pos,len,"1 w\n");

  // draw direct connection as straight line
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    dep = hp->dep;
    arr = hp->arr;
    pdep = ports + dep;
    parr = ports + arr;
    y0 = lat2pdf(pdep->lat,lolat,dlat);
    x0 = lon2pdf(pdep->lon,lolon,dlon);
    y1 = lat2pdf(parr->lat,lolat,dlat);
    x1 = lon2pdf(parr->lon,lolon,dlon);
    n = mysnprintf(buf,pos,len,"%u %u m %u %u l s\n",x0,y0,x1,y1);
    if (n == 0) break;
    pos += n;
  }
  error_gt(pos,len,0);
  if (pos == len) warning(0,"pdf output buffer limit \ah%u reached: truncated", len);
  return pos;
}

/* write graphic representation of network:
  ports, hops
  currently single page only
 */
static ub4 maxcontent = 64 * 1024 * 1024;

int net2pdf(netbase *net)
{
static char pagebuf[64 * 1024];

  int fd;
  ub4 pos,xpos,cpos,xrefpos,obj;
  ub4 plen = sizeof pagebuf;
  ub4 xref[16];
  block contentblk;
  char *content;
  enum objnos { pdfnil, pdfcat, pdftree,pdfnode,pdfcontent, pdflast };
  const char *name = "net.pdf";

  info(0,"writing %u-ports %u-hops base net to %s in pdf format",net->portcnt,net->hopcnt,name);

  fd = filecreate("net.pdf",1);
  if (fd == -1) return 1;

  pos = mysnprintf(pagebuf,0,plen,"%%PDF-1.4\n");

  // patterned after simple examples in PDF reference
  xref[pdfcat] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Catalog /Pages %u 0 R >>\nendobj\n",pdfcat,pdftree);

  xref[pdftree] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Pages /Kids [%u 0 R] /Count 1 >>\nendobj\n", pdftree,pdfnode);

  xref[pdfnode] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Page /Parent %u 0 R "
    "/MediaBox [0 0 %u %u] /Contents %u 0 R /Resources"
    " << /Font << /F1 << /Type /Font /Subtype /Type1 /BaseFont /Helvetica >> >> >> >>\n"
    "endobj\n", pdfnode,pdftree,pdfscale_lon, pdfscale_lat,pdfcontent);

  xref[pdfcontent] = pos;

  xpos = pos;
  if (filewrite(fd,pagebuf,pos,name)) return 1;

  content = mkblock(&contentblk,maxcontent,char,Noinit,"pdf content buffer");

  cpos = addnetpdf(net,content,maxcontent);

  pos = mysnprintf(pagebuf,0, plen,"%u 0 obj\n << /Length %u >>\nstream\n", pdfcontent, cpos);
  if (filewrite(fd,pagebuf,pos,name)) return 1;
  xpos += pos + cpos;

  if (filewrite(fd,content,cpos,name)) return 1;

  pos = mysnprintf(pagebuf,0,plen,"endstream\nendobj\n");

  xrefpos = xpos + pos;
  pos += mysnprintf(pagebuf,pos,plen,"xref\n0 %u\n",pdflast);

  pos += mysnprintf(pagebuf,pos,plen,"%010u 65535 f \n", 0);
  for (obj = 1; obj <= pdfcontent; obj++) pos += mysnprintf(pagebuf,pos,plen,"%010u 00000 n \n", xref[obj]);

  pos += mysnprintf(pagebuf,pos,plen,"trailer\n << /Size %u /Root %u 0 R >>\n", pdflast, pdfcat);

  pos += mysnprintf(pagebuf,pos,plen,"startxref\n%u\n%s\n", xrefpos,"%%EOF");

  if (filewrite(fd,pagebuf,pos,name)) return 1;

  fileclose(fd,name);

  info(0,"done writing base net to %s in pdf format: \ah%u bytes",name,xrefpos + pos);

  return 0;
}

void ininetio(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
  mkhexmap();
}
