// gtfsprep.c - prepare gtfs feeds

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/*
  Superficial parsing and tidyup of gtfs feeds as first pass.
  input is uncompressed feed
  output is a variant of gtfs : tab-separated, unquoted and in a canonical column order
  only columns tripover uses
  this simpilies and speeds up subsequent processig by gtfstool
 */

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "base.h"
struct globs globs;

#include "os.h"
#include "mem.h"
#include "util.h"
#include "time.h"

static ub4 msgfile;
#include "msg.h"

#include "gtfsprep.h"

static int init0(char *progname)
{
  char mtimestr[64];
  char *p;

  setsigs();

  p = strrchr(progname,'/');
  globs.progname = (p ? p + 1 : progname);

  setmsglvl(Info,0,0);
  inimsg(progname,"gtfsprep.log",Msg_stamp|Msg_pos|Msg_type);
  msgfile = setmsgfile(__FILE__);
  iniassert();

#ifdef NOW
  sec70toyymmdd(NOW,mtimestr,sizeof(mtimestr));
#else
  strcopy(mtimestr,__DATE__);
#endif

  info(User,"gtfsprep %u.%u %s %s\n", Version_maj,Version_min,Version_phase,mtimestr);

  if (iniutil(0)) return 1;
  inimem();
  inios();
  globs.maxvm = 12;
  return 0;
}

extern const char *runlvlnames(enum Runlvl lvl);
const char *runlvlnames(enum Runlvl lvl) { return lvl ? "n/a" : "N/A"; }

static int streq(const char *s,const char *q) { return !strcmp(s,q); }

enum extresult { Next, Newitem, Newcmd, Eof, Parserr };

// count non-empty lines
static ub4 linecnt(const char *name,const char *buf, ub4 len)
{
  ub4 pos = 0,cnt = 0;
  const char nl = '\n';

  while (pos < len) {
    if (buf[pos] == nl) pos++;
    else {
      cnt++;
      while (pos < len && buf[pos] != nl) pos++;
    }
  }
  if (len && buf[len-1] != nl) warning(0,"%s has unterminated last line",name);
  info(0,"%s: %u data lines", name,cnt);
  return cnt;
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
  return warnfln(fln,Iter,"%s",buf);
}

/* parse comma-separated file, e.g. gtfs.
   first line has colunm names, rest are values
   values can be quoted and if so can contain commas
   quote within quoted strings are doubled
 */

#define Valcnt 32
#define Collen 256

enum extstates {Init,Val0,Val1,Val2,Val2q,Val3q,Cmd0,Cmd1,Cmd2};

struct extfmt {
  struct myfile mf;
  ub4 pos;
  enum extstates state;
  ub4 linno,colno;
  ub4 valndx,valcnt;
  ub4 ivals[Valcnt];
  ub4 vallens[Valcnt];
  char vals[Valcnt * Collen];
};

static enum extresult nextchar(struct extfmt *ef)
{
  char *fname;
  ub1 c;
  ub4 pos,len,linno,colno,valndx,valno;
  char *val,*vals;
  ub4 *vallens,vallen;
  int newitem,iscmd;

  enum extstates state;

  len = (ub4)ef->mf.len;
  pos = ef->pos;
  if (pos >= len) return Eof;

  // state
  state = ef->state;
  valndx = ef->valndx;
  linno = ef->linno + 1;
  colno = ef->colno;

  // convenience
  fname = ef->mf.name;
  vals = ef->vals;
  vallens = ef->vallens;

  c = (ub1)ef->mf.buf[pos];
  ef->pos = pos + 1;

  newitem = iscmd = 0;

//    info(0,"state %u c %c",state,c);

    switch(state) {

    case Init:
      if (c == 0xef) { // utf8 can have byte order mark
        if (len < 3 || (ub1)ef->mf.buf[pos+1] != 0xbb || (ub1)ef->mf.buf[pos+2] != 0xbf) {
          return parserr(FLN,fname,linno,colno,"headline has unexpected char 0x%x %x %x",c,ef->mf.buf[pos+1],ef->mf.buf[pos+2]);
        }
        pos += 3;
        ef->pos = pos + 1;
        c = ef->mf.buf[pos];
      }
      linno = 1; // cascade

    case Cmd0:
      switch (c) {
        case ',': return parserr(FLN,fname,linno,colno,"empty column name");
        case '\n': return parserr(FLN,fname,linno,colno,"unexpected newline");
        default:
          if (c != '_' && !(c >= 'a' && c <= 'z')) return parserr(FLN,fname,linno,colno,"headline has unexpected char '%c'",c);
          valndx = 0;
          val = vals;
          val[0] = c; vallens[0] = 1;
          state = Cmd1;
      }
      break;

    case Cmd1:
      switch (c) {
        case ',': valndx++; vallens[valndx] = 0; state = Cmd2; break;
        case '\r': break;
        case '\n': newitem = iscmd = 1; state = Val0; break;
        default:
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
      }
      break;

    case Cmd2:
      switch (c) {
        case ',': return parserr(FLN,fname,linno,colno,"empty column name");
        case '\r': break;
        case '\n': newitem = iscmd = 1; state = Val0; break;
        default:
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
          state = Cmd1;
      }
      break;

    case Val0:
      valndx = 0;
      switch (c) {
        case ',': vallens[0] = 0; valndx = 1; state = Val1; break;
        case '\r': break;
        case '\n': parsewarn(FLN,fname,linno,colno,"skipping empty line"); break;
        case '"': vallens[0] = 0; state = Val2q; break;
        case '\t': c = ' '; // cascade
        default:
          val = vals;
          val[0] = c; vallens[0] = 1;
          state = Val1;
      }
      break;

    case Val1:
      switch(c) {
        case ',': valndx++; vallens[valndx] = 0; state = Val2; break;
        case '\r': break;
        case '\n': newitem = 1; state = Val0; break;
        case '\t': c = ' '; // cascade
        default:
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
      }
      break;

    case Val2:
      switch (c) {
        case ',': valndx++; vallens[valndx] = 0; break;
        case '\r': break;
        case '\n': newitem = 1; state = Val0; break;
        case '"': state = Val2q; break;
        case '\t': c = ' '; // cascade
        default:
          val = vals + valndx * Collen;
          val[0] = c; vallens[valndx] = 1;
          state = Val1;
      }
      break;

    case Val2q:
      switch(c) {
        case '"': state = Val3q; break;
        case '\r': break;
        case '\n': return parserr(FLN,fname,linno,colno,"unexpected newline in quoted string");
        case '\t': c = ' '; // cascade
        default:
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
      }
      break;

    case Val3q:
      switch(c) {
        case '"':
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
          state = Val2q;
          break;
        case ',': valndx++; vallens[valndx] = 0; state = Val2; break;
        case '\r': break;
        case '\n': newitem = 1; state = Val0; break;
        default: return parserr(FLN,fname,linno,colno,"unexpected char '%c' after quote",c);
      }
      break;

    } // end switch state

  if (c == '\n') { linno++; colno = 1; }
  else colno++;

  ef->state = state;

  ef->valndx = valndx;
  ef->linno = linno - 1;
  ef->colno = colno;

  if (newitem) {
    for (valno = 0; valno <= valndx; valno++) {
      val = vals + valno * Collen;
      vallen = vallens[valno];
      val[vallen] = 0;
    }
    ef->valcnt = valndx + 1;
    return iscmd ? Newcmd : Newitem;
  } else return Next;
}

static ub4 addcol(char *lines,ub4 pos,char *col,ub4 collen,char c)
{
  if (collen) {
    memcpy(lines + pos,col,collen);
    pos += collen;
  }
  lines[pos++] = c;
  return pos;
}

static int rdagency(gtfsnet *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawcnt,cnt = 0;
  int rv;
  char *buf;
  ub4 len,linno,colno;
  char *val,*vals;
  ub4 vlen,*vallens;
  ub4 valcnt,valno;
  ub4 linepos = 0,linelen;
  block *mem = &net->agencymem;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/agency.txt",dir);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return warning(0,"%s is empty",fname);

  linelen = len;
  char *lines = net->agencylines = mkblock(mem,linelen,char,Noinit,"gtfs %u agency, len %u",rawcnt-1,linelen);

  const char tab = '\t';
  ub4 agency_idpos,agency_namepos,agency_tzpos,agency_urlpos;
  agency_idpos=agency_namepos=agency_tzpos=agency_urlpos = hi32;

  do {

    res = nextchar(&eft);
    vals = eft.vals;

    switch(res) {

    case Newcmd:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      if (valcnt < 4) return parserr(FLN,fname,linno,colno,"missing columns, only %u",valcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        if (streq(val,"agency_id")) agency_idpos = valno;
        else if (streq(val,"agency_name")) agency_namepos = valno;
        else if (streq(val,"agency_timezone")) agency_tzpos = valno;
        else if (streq(val,"agency_url")) agency_urlpos = valno;
        else info(0,"skipping column %s",val);
      }
      if (agency_namepos == hi32) return error(0,"%s: missing required column agency_name",fname);
      if (agency_tzpos == hi32) return error(0,"%s: missing required column agency_timezone",fname);
      if (agency_urlpos == hi32) return error(0,"%s: missing required column agency_url",fname);
      if (agency_idpos == hi32) info(0,"%s: no column agency_id",fname);
      break;

    case Newitem:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      vallens = eft.vallens;
      error_ge(cnt,rawcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        vrb0(0,"col %u val '%s'",valno,val);
      }
      if (valcnt < 4) return parserr(FLN,fname,linno,colno,"missing required columns, only %u",valcnt);

// id
      if (agency_idpos != hi32) {
        val = vals + agency_idpos * Collen;
        vlen = vallens[agency_idpos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,tab);
      }  else lines[linepos++] = tab;

// name
      val = vals + agency_namepos * Collen;
      vlen = vallens[agency_namepos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// tz
      val = vals + agency_tzpos * Collen;
      vlen = vallens[agency_tzpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// url
      val = vals + agency_urlpos * Collen;
      vlen = vallens[agency_urlpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,'\n');

      cnt++;
      break;

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"%u from %u entries",cnt,rawcnt);
  net->agencycnt = cnt;
  net->agencylinepos = linepos;

  return 0;
}

static int rdcalendar(gtfsnet *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawcnt,cnt = 0;
  int rv;
  char *buf;
  ub4 len,linno,colno;
  char *val,*vals;
  ub4 vlen,*vallens;
  ub4 valcnt,valno;
  ub4 dow;
  ub4 linepos = 0,linelen;
  block *mem = &net->calendarmem;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/calendar.txt",dir);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,0);
  if (rv) return 1;
  if (eft.mf.exist == 0) return 0;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return warning(0,"%s is empty",fname);

  linelen = len;
  char *lines = net->calendarlines = mkblock(mem,linelen,char,Noinit,"gtfs %u calendar, len %u",rawcnt-1,linelen);

  const char tab = '\t';

  ub4 service_idpos = hi32,startpos = hi32,endpos = hi32;
  ub4 dowpos[7] = {hi32,hi32,hi32,hi32,hi32,hi32,hi32};

  do {

    res = nextchar(&eft);
    vals = eft.vals;

    switch(res) {

    case Newcmd:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      if (valcnt < 3) return parserr(FLN,fname,linno,colno,"missing columns, only %u",valcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        if (streq(val,"service_id")) service_idpos = valno;
        else if (streq(val,"monday")) dowpos[0] = valno;
        else if (streq(val,"tuesday")) dowpos[1] = valno;
        else if (streq(val,"wednesday")) dowpos[2] = valno;
        else if (streq(val,"thursday")) dowpos[3] = valno;
        else if (streq(val,"friday")) dowpos[4] = valno;
        else if (streq(val,"saturday")) dowpos[5] = valno;
        else if (streq(val,"sunday")) dowpos[6] = valno;
        else if (streq(val,"start_date")) startpos = valno;
        else if (streq(val,"end_date")) endpos = valno;
        else info(0,"skipping column '%s'",val);
      }
      if (service_idpos == hi32) return error(0,"%s: missing required column service_id",fname);
      if (startpos == hi32) return error(0,"%s: missing required column start_date",fname);
      if (endpos == hi32) return error(0,"%s: missing required column end_date",fname);
      for (dow = 0; dow < 7; dow++) if (dowpos[dow] == hi32) return error(0,"%s: not all of required monday .. sunday columns present",fname);

      break;

    case Newitem:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      vallens = eft.vallens;
      error_ge(cnt,rawcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        vrb0(0,"col %u val '%s'",valno,val);
      }
      if (valcnt < 4) return parserr(FLN,fname,linno,colno,"missing required columns, only %u",valcnt);

// id
      val = vals + service_idpos * Collen;
      vlen = vallens[service_idpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// monday .. sunday
      for (dow = 0; dow < 7; dow++) {
        val = vals + dowpos[dow] * Collen;
        vlen = vallens[dowpos[dow]];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,tab);
      }

// start
      val = vals + startpos * Collen;
      vlen = vallens[startpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// end
      val = vals + endpos * Collen;
      vlen = vallens[endpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,'\n');

      cnt++;
      break;

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"%u from %u entries",cnt,rawcnt);
  net->calendarcnt = cnt;
  net->calendarlinepos = linepos;

  return 0;
}

static int rdcaldates(gtfsnet *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawcnt,cnt = 0;
  int rv;
  char *buf;
  ub4 len,linno,colno;
  char *val,*vals;
  ub4 vlen,*vallens;
  ub4 valcnt,valno;
  ub4 linepos = 0,linelen;
  block *mem = &net->caldatesmem;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/calendar_dates.txt",dir);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,0);
  if (rv) return 1;
  if (eft.mf.exist == 0) return 0;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return warning(0,"%s is empty",fname);

  linelen = len;
  char *lines = net->caldateslines = mkblock(mem,linelen,char,Noinit,"gtfs %u calendar dates, len %u",rawcnt-1,linelen);

  const char tab = '\t';
  ub4 service_idpos = hi32,extype_pos = hi32,datepos = hi32;

  do {

    res = nextchar(&eft);
    vals = eft.vals;

    switch(res) {

    case Newcmd:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      if (valcnt < 3) return parserr(FLN,fname,linno,colno,"missing columns, only %u",valcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        if (streq(val,"service_id")) service_idpos = valno;
        else if (streq(val,"date")) datepos = valno;
        else if (streq(val,"exception_type")) extype_pos = valno;
        else info(0,"skipping column %s",val);
      }
      if (service_idpos == hi32) return error(0,"%s: missing required column service_id",fname);
      if (datepos == hi32) return error(0,"%s: missing required column date",fname);
      if (extype_pos == hi32) return error(0,"%s: missing required column exception_type",fname);
      break;

    case Newitem:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      vallens = eft.vallens;
      error_ge(cnt,rawcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        vrb0(0,"col %u val '%s'",valno,val);
      }
      if (valcnt < 3) return parserr(FLN,fname,linno,colno,"missing required columns, only %u",valcnt);

// id
      val = vals + service_idpos * Collen;
      vlen = vallens[service_idpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// extype
      val = vals + extype_pos * Collen;
      vlen = vallens[extype_pos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// date
      val = vals + datepos * Collen;
      vlen = vallens[datepos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,'\n');

      cnt++;
      break;

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"%u from %u entries",cnt,rawcnt);
  net->caldatescnt = cnt;
  net->caldateslinepos = linepos;

  return 0;
}

static int rdroutes(gtfsnet *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawcnt,cnt = 0;
  int rv;
  char *buf;
  ub4 len,linno,colno;
  char *val,*vals;
  ub4 vlen,*vallens;
  ub4 valcnt,valno;
  ub4 linepos = 0,linelen;
  block *mem = &net->routemem;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/routes.txt",dir);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return warning(0,"%s is empty",fname);

  linelen = len;
  char *lines = net->routelines = mkblock(mem,linelen,char,Noinit,"gtfs %u routes, len %u",rawcnt-1,linelen);

  const char tab = '\t';
  ub4 route_idpos,agencypos,snamepos,lnamepos,descpos,rtypepos;
  route_idpos=agencypos=snamepos=lnamepos=descpos=rtypepos = hi32;

  do {

    res = nextchar(&eft);
    vals = eft.vals;

    switch(res) {

    case Newcmd:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      if (valcnt < 3) return parserr(FLN,fname,linno,colno,"missing columns, only %u",valcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        if (streq(val,"route_id")) route_idpos = valno;
        else if (streq(val,"agency_id")) agencypos = valno;
        else if (streq(val,"route_short_name")) snamepos = valno;
        else if (streq(val,"route_long_name")) lnamepos = valno;
        else if (streq(val,"route_desc")) descpos = valno;
        else if (streq(val,"route_type")) rtypepos = valno;
        else info(0,"skipping column %s",val);
      }
      if (route_idpos == hi32) return error(0,"%s: missing required column route_id",fname);
      if (rtypepos == hi32) return error(0,"%s: missing required column route_type",fname);
      break;

    case Newitem:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      vallens = eft.vallens;
      error_ge(cnt,rawcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        vrb0(0,"col %u val '%s'",valno,val);
      }
      if (valcnt < 4) return parserr(FLN,fname,linno,colno,"missing required columns, only %u",valcnt);

// id
      val = vals + route_idpos * Collen;
      vlen = vallens[route_idpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// agency
      if (agencypos != hi32) {
        val = vals + agencypos * Collen;
        vlen = vallens[agencypos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,tab);
      } else lines[linepos++] = tab;

// type
      val = vals + rtypepos * Collen;
      vlen = vallens[rtypepos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// sname
      if (snamepos != hi32) {
        val = vals + snamepos * Collen;
        vlen = vallens[snamepos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,tab);
      } else lines[linepos++] = tab;

// lname
      if (lnamepos != hi32) {
        val = vals + lnamepos * Collen;
        vlen = vallens[lnamepos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,tab);
      } else lines[linepos++] = tab;

// desc
      if (descpos != hi32) {
        val = vals + descpos * Collen;
        vlen = vallens[descpos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,'\n');
      } else lines[linepos++] = '\n';

      cnt++;
      break;

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"%u from %u entries",cnt,rawcnt);
  net->routecnt = cnt;
  net->routelinepos = linepos;

  return 0;
}

static int rdstops(gtfsnet *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawstopcnt,stopcnt = 0;
  int rv;
  char *buf;
  ub4 len,linno,colno;
  ub4 x;
  char *val,*vals;
  ub4 vlen,*vallens;
  ub4 valcnt,valno;
  ub4 linepos = 0,linelen;
  block *mem = &net->stopmem;

  struct eta eta;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/stops.txt",dir);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawstopcnt = linecnt(fname,buf, len);

  if (rawstopcnt == 0) return warning(0,"%s is empty",fname);

//  struct gtstop *sp,*stops = alloc(rawstopcnt,struct gtstop,0,"ext ports");

  linelen = len;
  char *lines = net->stoplines = mkblock(mem,linelen,char,Noinit,"gtfs %u stops, len %u",rawstopcnt-1,linelen);

  const char tab = '\t';
  ub4 stop_idpos,stop_codepos,stop_namepos,stop_descpos,stop_latpos,stop_lonpos,stop_locpos,parent_stapos;
  stop_idpos=stop_codepos=stop_namepos=stop_descpos=stop_latpos=stop_lonpos=stop_locpos=parent_stapos = hi32;

  do {

    res = nextchar(&eft);
    vals = eft.vals;

    switch(res) {
    case Newcmd:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      if (valcnt < 3) return parserr(FLN,fname,linno,colno,"missing columns, only %u",valcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        if (streq(val,"stop_id")) stop_idpos = valno;
        else if (streq(val,"stop_code")) stop_codepos = valno;
        else if (streq(val,"stop_name")) stop_namepos = valno;
        else if (streq(val,"stop_desc")) stop_descpos = valno;
        else if (streq(val,"stop_lat")) stop_latpos = valno;
        else if (streq(val,"stop_lon")) stop_lonpos = valno;
        else if (streq(val,"location_type")) stop_locpos = valno;
        else if (streq(val,"parent_station")) parent_stapos = valno;
        else info(0,"skipping column %s",val);
      }
      if (stop_idpos == hi32) return error(0,"%s: missing required column stopid",fname);
      if (stop_namepos == hi32) return error(0,"%s: missing required column stop_name",fname);
      if (stop_latpos == hi32) return error(0,"%s: missing required column stop_lat",fname);
      if (stop_lonpos == hi32) return error(0,"%s: missing required column stop_lon",fname);
      if (stop_locpos == hi32) info(0,"%s: no column location_type",fname);
      if (parent_stapos == hi32) info(0,"%s: no column parent_station",fname);
      break;

    case Newitem:

      if (progress(&eta,"reading stop %u of %u in %s",stopcnt,rawstopcnt,fname)) return 1;

      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      vallens = eft.vallens;
      error_ge(stopcnt,rawstopcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        vrb0(0,"col %u val '%s'",valno,val);
      }
      if (valcnt < 4) return parserr(FLN,fname,linno,colno,"missing required columns, only %u",valcnt);

// id
      val = vals + stop_idpos * Collen;
      vlen = vallens[stop_idpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// code
      if (stop_codepos != hi32) {
        val = vals + stop_codepos * Collen;
        vlen = vallens[stop_codepos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,tab);
      } else lines[linepos++] = tab;

// loc
      if (stop_locpos != hi32) {
        val = vals + stop_locpos * Collen;
        vlen = vallens[stop_locpos];

        bound(mem,linepos + 2,char);
        if (vlen == 0) x = '0';
        else if (vlen == 1)  x = *val;
        else { parsewarn(FLN,fname,linno,colno,"location_type %s not '0' or '1'",val); x = '0'; }
        if (x != '0' && x != '1') { parsewarn(FLN,fname,linno,colno,"location_type %c not '0' or '1'",x); x = '0'; }
        lines[linepos++] = (char)x;
      }
      lines[linepos++] = tab;

// parent
      if (parent_stapos != hi32) {
        val = vals + parent_stapos * Collen;
        vlen = vallens[parent_stapos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,tab);
      } else lines[linepos++] = tab;

//name
      val = vals + stop_namepos * Collen;
      vlen = vallens[stop_namepos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

//lat
      val = vals + stop_latpos * Collen;
      vlen = vallens[stop_latpos];
      if (vlen && *val == ' ') { val++; vlen--; }

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

//lon
      val = vals + stop_lonpos * Collen;
      vlen = vallens[stop_lonpos];
      if (vlen && *val == ' ') { val++; vlen--; }

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

//desc
      if (stop_descpos != hi32) {
        val = vals + stop_descpos * Collen;
        vlen = vallens[stop_descpos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,'\n');
      } else lines[linepos++] = '\n';

//      sp++;
      stopcnt++;
      break;  // newitem

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"%u from %u stops",stopcnt,rawstopcnt);
  net->stopcnt = stopcnt;
  net->stoplinepos = linepos;

  return 0;
}

static int rdtrips(gtfsnet *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawcnt,cnt = 0;
  int rv;
  char *buf;
  ub4 len,linno,colno;
  char *val,*vals;
  ub4 vlen,*vallens;
  ub4 valcnt,valno;
  ub4 linepos = 0,linelen;
  block *mem = &net->tripmem;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/trips.txt",dir);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return warning(0,"%s is empty",fname);

  linelen = len;
  char *lines = net->triplines = mkblock(mem,linelen,char,Noinit,"gtfs %u trips, len %u",rawcnt-1,linelen);

  const char tab = '\t';
  ub4 route_idpos,service_idpos,trip_idpos,headsignpos;
  route_idpos=service_idpos=trip_idpos=headsignpos = hi32;

  do {

    res = nextchar(&eft);
    vals = eft.vals;

    switch(res) {

    case Newcmd:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      if (valcnt < 3) return parserr(FLN,fname,linno,colno,"missing columns, only %u",valcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        if (streq(val,"route_id")) route_idpos = valno;
        else if (streq(val,"service_id")) service_idpos = valno;
        else if (streq(val,"trip_id")) trip_idpos = valno;
        else if (streq(val,"headsign")) headsignpos = valno;
        else info(0,"skipping column %s",val);
      }
      if (route_idpos == hi32) return error(0,"%s: missing required column route_id",fname);
      if (service_idpos == hi32) return error(0,"%s: missing required column service_id",fname);
      if (trip_idpos == hi32) return error(0,"%s: missing required column trip_id",fname);
      break;

    case Newitem:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      vallens = eft.vallens;
      error_ge(cnt,rawcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        vrb0(0,"col %u val '%s'",valno,val);
      }
      if (valcnt < 4) return parserr(FLN,fname,linno,colno,"missing required columns, only %u",valcnt);

// route id
      val = vals + route_idpos * Collen;
      vlen = vallens[route_idpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// service id
      val = vals + service_idpos * Collen;
      vlen = vallens[service_idpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// trip id
      val = vals + trip_idpos * Collen;
      vlen = vallens[trip_idpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// headsign
      if (headsignpos != hi32) {
        val = vals + headsignpos * Collen;
        vlen = vallens[headsignpos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,'\n');
      } else lines[linepos++] = '\n';

      cnt++;
      break;

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"%u from %u entries",cnt,rawcnt);
  net->tripcnt = cnt;
  net->triplinepos = linepos;

  return 0;
}

static int rdstoptimes(gtfsnet *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawcnt,cnt = 0;
  int rv;
  char *buf;
  ub4 len,linno,colno;
  char *val,*vals;
  ub4 vlen,*vallens;
  ub4 valcnt,valno;
  ub4 linepos = 0,linelen;
  block *mem = &net->stoptimesmem;

  struct eta eta;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/stop_times.txt",dir);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return warning(0,"%s is empty",fname);

//  struct gtstop *sp,*stops = alloc(rawstopcnt,struct gtstop,0,"ext ports");

  linelen = len;
  char *lines = net->stoptimeslines = mkblock(mem,linelen,char,Noinit,"gtfs %u stoptimess, len %u",rawcnt-1,linelen);

  const char tab = '\t';
  ub4 trip_idpos,stop_idpos,stop_seqpos,tarr_pos,tdep_pos;
  trip_idpos=stop_idpos=stop_seqpos=tarr_pos=tdep_pos = hi32;

  do {

    res = nextchar(&eft);
    vals = eft.vals;

    switch(res) {
    case Newcmd:
      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      if (valcnt < 3) return parserr(FLN,fname,linno,colno,"missing columns, only %u",valcnt);
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        if (streq(val,"trip_id")) trip_idpos = valno;
        else if (streq(val,"stop_id")) stop_idpos = valno;
        else if (streq(val,"stop_sequence")) stop_seqpos = valno;
        else if (streq(val,"arrival_time")) tarr_pos = valno;
        else if (streq(val,"departure_time")) tdep_pos = valno;
        else info(0,"skipping column %s",val);
      }
      if (trip_idpos == hi32) return error(0,"%s: missing required column trip_id",fname);
      if (stop_idpos == hi32) return error(0,"%s: missing required column stop_id",fname);
      if (stop_seqpos == hi32) return error(0,"%s: missing required column sequence_id",fname);
      if (tdep_pos == hi32) return error(0,"%s: missing required column departure_time",fname);
      if (tarr_pos == hi32) return error(0,"%s: missing required column arrival_time",fname);
      break;

    case Newitem:

      if (progress(&eta,"reading stop_time %u of %u in %s",cnt,rawcnt,fname)) return 1;

      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      vallens = eft.vallens;
      error_ge(cnt,rawcnt);
      if (valcnt < 4) return parserr(FLN,fname,linno,colno,"missing required columns, only %u",valcnt);

// tripid
      val = vals + trip_idpos * Collen;
      vlen = vallens[trip_idpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// stopid
      val = vals + stop_idpos * Collen;
      vlen = vallens[stop_idpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// seq
      val = vals + stop_seqpos * Collen;
      vlen = vallens[stop_seqpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// tarr
      val = vals + tarr_pos * Collen;
      vlen = vallens[tarr_pos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab);

// tdep
      val = vals + tdep_pos * Collen;
      vlen = vallens[tdep_pos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,'\n');

      cnt++;
      break;  // newitem

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"%u from %u lines",cnt,rawcnt);
  net->stoptimescnt = cnt;
  net->stoptimeslinepos = linepos;

  return 0;
}

static ub4 wrhdr(const char *fname,char *buf,ub4 buflen)
{
  char nowstr[64];
  const char *tz;
  ub4 pos;

#ifdef NOW
  sec70toyymmdd(NOW,nowstr,sizeof(nowstr));
  tz = "utc";
#else
  strcopy(nowstr,__DATE__);
  tz = "localtime";
#endif

  pos = mysnprintf(buf,0,buflen,"# %s - variant gtfs feed for tripover\n\n",fname);
  pos += mysnprintf(buf,pos,buflen,"# written by gtfsprep version %u.%u  %s %s\n\n", Version_maj,Version_min,nowstr,tz);
  return pos;
}

static int wragency(gtfsnet *net,const char *dir)
{
  char fname[1024];
  char buf[1024];
  ub4 buflen = sizeof(buf);
  ub4 pos;
  int fd;

  fmtstring(fname,"%s/agency.tab",dir);

  fd = filecreate(fname,1);
  if (fd == -1) return 1;

  pos = wrhdr(fname,buf,buflen);
  pos += mysnprintf(buf,pos,buflen,"# %u entries\n\n",net->stopcnt);

  if (filewrite(fd,buf,pos,fname)) return 1;

  pos = fmtstring0(buf,"agency_id\tagency_name\tagency_timezone\tagency_url\n");
  if (filewrite(fd,buf,pos,fname)) return 1;

  if (filewrite(fd,net->agencylines,net->agencylinepos,fname)) return 1;

  return fileclose(fd,fname);
}

static int wrcalendar(gtfsnet *net,const char *dir)
{
  char fname[1024];
  char buf[1024];
  ub4 buflen = sizeof(buf);
  ub4 pos;
  int fd;

  if (net->calendarcnt == 0) return 0;

  fmtstring(fname,"%s/calendar.tab",dir);

  fd = filecreate(fname,1);
  if (fd == -1) return 1;

  pos = wrhdr(fname,buf,buflen);
  pos += mysnprintf(buf,pos,buflen,"# %u entries\n\n",net->calendarcnt);

  if (filewrite(fd,buf,pos,fname)) return 1;

  pos = fmtstring0(buf,"service_id\tmonday\ttuesday\twednesday\tthursday\tfriday\tsaturday\tsunday\tstart_date\tend_date\n");
  if (filewrite(fd,buf,pos,fname)) return 1;

  if (filewrite(fd,net->calendarlines,net->calendarlinepos,fname)) return 1;

  return fileclose(fd,fname);
}

static int wrcaldates(gtfsnet *net,const char *dir)
{
  char fname[1024];
  char buf[1024];
  ub4 buflen = sizeof(buf);
  ub4 pos;
  int fd;

  if (net->caldatescnt == 0) return 0;

  fmtstring(fname,"%s/calendar_dates.tab",dir);

  fd = filecreate(fname,1);
  if (fd == -1) return 1;

  pos = wrhdr(fname,buf,buflen);
  pos += mysnprintf(buf,pos,buflen,"# %u entries\n\n",net->caldatescnt);

  if (filewrite(fd,buf,pos,fname)) return 1;

  pos = fmtstring0(buf,"service_id\texception_type\tdate\n");
  if (filewrite(fd,buf,pos,fname)) return 1;

  if (filewrite(fd,net->caldateslines,net->caldateslinepos,fname)) return 1;

  return fileclose(fd,fname);
}

static int wrroutes(gtfsnet *net,const char *dir)
{
  char fname[1024];
  char buf[1024];
  ub4 buflen = sizeof(buf);
  ub4 pos;
  int fd;

  fmtstring(fname,"%s/routes.tab",dir);

  fd = filecreate(fname,1);
  if (fd == -1) return 1;

  pos = wrhdr(fname,buf,buflen);
  pos += mysnprintf(buf,pos,buflen,"# %u entries\n\n",net->routecnt);

  if (filewrite(fd,buf,pos,fname)) return 1;

  pos = fmtstring0(buf,"route_id\tagency_id\troute_type\troute_short_name\troute_long_name\troute_desc\n");
  if (filewrite(fd,buf,pos,fname)) return 1;

  if (filewrite(fd,net->routelines,net->routelinepos,fname)) return 1;

  return fileclose(fd,fname);
}

static int wrstops(gtfsnet *net,const char *dir)
{
  char fname[1024];
  char buf[1024];
  ub4 buflen = sizeof(buf);
  ub4 pos;
  int fd;

  fmtstring(fname,"%s/stops.tab",dir);

  fd = filecreate(fname,1);
  if (fd == -1) return 1;

  pos = wrhdr(fname,buf,buflen);
  pos += mysnprintf(buf,pos,buflen,"# %u stops\n\n",net->stopcnt);

  if (filewrite(fd,buf,pos,fname)) return 1;

  pos = fmtstring0(buf,"stop_id\tstop_code\tlocation_type\tparent_station\tstop_name\tstop_lat\tstop_lon\tstop_desc\n");
  if (filewrite(fd,buf,pos,fname)) return 1;

  if (filewrite(fd,net->stoplines,net->stoplinepos,fname)) return 1;

  return fileclose(fd,fname);
}

static int wrtrips(gtfsnet *net,const char *dir)
{
  char fname[1024];
  char buf[1024];
  ub4 buflen = sizeof(buf);
  ub4 pos;
  int fd;

  fmtstring(fname,"%s/trips.tab",dir);

  fd = filecreate(fname,1);
  if (fd == -1) return 1;

  pos = wrhdr(fname,buf,buflen);
  pos += mysnprintf(buf,pos,buflen,"# %u entries\n\n",net->tripcnt);

  if (filewrite(fd,buf,pos,fname)) return 1;

  pos = fmtstring0(buf,"route_id\tservice_id\ttrip_id\ttrip_headsign\n");
  if (filewrite(fd,buf,pos,fname)) return 1;

  if (filewrite(fd,net->triplines,net->triplinepos,fname)) return 1;

  return fileclose(fd,fname);
}

static int wrstoptimes(gtfsnet *net,const char *dir)
{
  char fname[1024];
  char buf[1024];
  ub4 buflen = sizeof(buf);
  ub4 pos;
  int fd;

  fmtstring(fname,"%s/stop_times.tab",dir);

  info(0,"writing %s",fname);
  fd = filecreate(fname,1);
  if (fd == -1) return 1;

  pos = wrhdr(fname,buf,buflen);
  pos += mysnprintf(buf,pos,buflen,"# %u stops\n\n",net->stopcnt);

  if (filewrite(fd,buf,pos,fname)) return 1;

  pos = fmtstring0(buf,"trip_id\tstop_id\tstop_sequence\tarrival_time\tdeparture_time\n");
  if (filewrite(fd,buf,pos,fname)) return 1;

  if (filewrite(fd,net->stoptimeslines,net->stoptimeslinepos,fname)) return 1;

  fileclose(fd,fname);
  info(0,"wrote %s",fname);
  return 0;
}


static int readgtfs(gtfsnet *net,const char *dir)
{
  info(0,"reading gtfs files from dir %s",dir);

  if (rdagency(net,dir)) return 1;
  if (rdcalendar(net,dir)) return 1;
  if (rdcaldates(net,dir)) return 1;
  if (rdroutes(net,dir)) return 1;
  if (rdstops(net,dir)) return 1;
  if (rdtrips(net,dir)) return 1;
  if (rdstoptimes(net,dir)) return 1;

  return info0(0,"done reading gtfs files");
}

static int writegtfs(gtfsnet *net,const char *dir)
{
  if (wragency(net,dir)) return 1;
  if (wrcalendar(net,dir)) return 1;
  if (wrcaldates(net,dir)) return 1;
  if (wrroutes(net,dir)) return 1;
  if (wrstops(net,dir)) return 1;
  if (wrtrips(net,dir)) return 1;
  if (wrstoptimes(net,dir)) return 1;
  return 0;
}

static int cmd_vrb(struct cmdval *cv) {
  if (cv->valcnt) globs.msglvl = cv->uval + Error;
  else globs.msglvl++;
  setmsglvl(globs.msglvl,0,globs.limassert);
  return 0;
}

static int cmd_limassert(struct cmdval *cv) {
  globs.limassert = cv->uval;
  setmsglvl(globs.msglvl,0,globs.limassert);
  return 0;
}

static int cmd_arg(struct cmdval *cv)
{
  ub4 argc = globs.argc;
  char *dst;
  ub4 maxlen = sizeof(globs.args[0]);

  if (argc + 1 >= Elemcnt(globs.args)) return error(0,"exceeded %u arg limit",argc);

  dst = globs.args[argc];
  vrb(0,"add arg %s", cv->sval);
  strncpy(dst, cv->sval,maxlen-1);
  globs.argc = argc + 1;
  return 0;
}

static struct cmdarg cmdargs[] = {
  { "verbose|v", "[level]%u", "set or increase verbosity", cmd_vrb },
  { "assert-limit", "[limit]%u", "stop at this #assertions", cmd_limassert },
  { NULL, "dir", "gtfsprep", cmd_arg }
};

int main(int argc, char *argv[])
{
  gtfsnet net;
  char *dir;
  struct myfile mf;

  oclear(net);

  init0(argv[0]);

  if (cmdline(argc,argv,cmdargs,"gtfs prep tool")) return 1;

  if (globs.argc) dir = globs.args[0];
  else {
    if (osfileinfo(&mf,"agency.txt")) return shortusage();
    dir  = ".";    
  }

  if (readgtfs(&net,dir)) return 1;
  if (writegtfs(&net,dir)) return 1;

  return 0;
}
