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

  additinal processing:
  - filtering on transprt mode
  - merge duplicate or nearby stops by inferring common parent

  for manual entry support, a few syntax variations are understood :

- blank lines are allowed and skipped

  stop_times.txt:
  - departure time, if omitted, is equal to arrival time
  - seconds, if omitted, are 0
  - . instead of : as time separator
  - sequence_number  if first after blank line is 1, and rest upto next blank line omitted, auto-increment
    otherwise, if higher than zero, autodecrement

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

#include "math.h"

#include "gtfsprep.h"

// can be set on commandline
static double grouplimit = 10;
static bool useparentname = 0;

static char *fileext = "txt";
static bool canonin;
static bool testonly;
static ub4 dateshift = 0;

static char *prefix = "";
static ub4 prefixlen1,prefixlen = 0;

static ub4 hidate = 20000101;
static ub4 lodate = 20201231;

static bool show_omitstop = 1;

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
  initime(0);
  inimath();
  inios();
  globs.maxvm = 12;
  initime(1);

  return 0;
}

extern const char *runlvlnames(enum Runlvl lvl);
const char *runlvlnames(enum Runlvl lvl) { return lvl ? "n/a" : "N/A"; }

static int streq(const char *s,const char *q) { return !strcmp(s,q); }

// Bob Jenkin one-at-a-time, from Wikipedia
static ub4 hashcode(const char *str,ub4 slen,ub4 len)
{
  ub4 h = 0, i;

  for (i = 0; i < slen; i++) {
    h += str[i];
    h += (h << 10);
    h ^= (h >> 6);
  }
  h += (h << 3);
  h ^= (h >> 11);
  h += (h << 15);
  return h % len;
}

static hash *mkhash(ub4 len,ub4 eqlen,ub4 slen,const char *desc)
{
  hash *ht = alloc(1,hash,0,"hash",len);
  ht->len = len;
  ht->eqlen = eqlen;
  ht->desc = desc;

  info(0,"create \ah%u * %u entry hash table with \ah%u string pool",len,eqlen,slen);

  ub4 bktlen = len * eqlen;
  ub4 spoollen = ht->spoollen = slen;
  ht->bkts = mkblock(&ht->bktmem,bktlen,struct bucket,Init0,"hash %s len %u",desc,len);
  ht->strpool = mkblock(&ht->strmem,spoollen,char,Noinit,"hash %s string pool",desc);
  return ht;
}

static ub4 gethash(hash *ht,const char *str,ub4 slen,ub4 ucode)
{
  error_z(slen,0);
  ub4 len = ht->len;
  ub4 code = ucode == hi32 ? hashcode(str,slen,len) : ucode % len;
  ub4 eqlen = ht->eqlen;
  struct bucket *bkt,*bkts = ht->bkts;
  char *spool = ht->strpool;
  ub4 eq = 0;

//  info(0,"get code %u %s len %u",code,str,slen);

  bkt = bkts + code * eqlen;
  while (bkt->slen && eq++ < eqlen) {
    if (bkt->slen == slen && memcmp(spool + bkt->sofs,str,slen) == 0) {
//      info(0,"found %s in %s pos %u",str,ht->desc,eq);
      return bkt->data;
    }
    bkt++;
  }
  return hi32;
}

static ub4 addhash(hash *ht,const char *str,ub4 slen,ub4 ucode,ub4 data)
{
  ub4 len = ht->len;
  ub4 code = ucode == hi32 ? hashcode(str,slen,len) : ucode % len;
  ub4 eqlen = ht->eqlen;
  struct bucket *bkt,*bkts = ht->bkts;
  char *spool = ht->strpool;
  ub4 eq = 0;
  ub4 sofs = ht->sofs;
  ub4 cnt = ht->itemcnt;

  error_z(slen,data);
  if (sofs + slen >= ht->spoollen) {
    error(0,"hash table %s full, %u items",ht->desc,cnt);
    return hi32;
  }
  bkt = bkts + code * eqlen;
  while (bkt->slen && eq < eqlen) { bkt++; eq++; }
  if (eq == eqlen) {
    error(0,"hash %s entry %u at %u %u for %.*s exceeds %u entry limit",ht->desc,cnt,code,ucode,slen,str,eqlen);
    return hi32;
  }
//  info(0,"add code %u pos %u %.*s",code,eq,slen,str);
  memcpy(spool + sofs,str,slen);
  bkt->slen = slen;
  bkt->sofs = sofs;
  bkt->data = data;

  ht->sofs = sofs + slen;
  if (eq > ht->maxeq) {
    ht->maxeq = eq;
    infocc(eq > eqlen / 2,0,"hash %s has load %u at %u entries for %s",ht->desc,eq,cnt,str);
  }
  ht->itemcnt = cnt + 1;

  return code * eqlen + eq;
}

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
  if (len && buf[len-1] != nl) {
    return error(Ret0,"%s has unterminated last line",name);
    error_nz(canonin,cnt);
  }
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
  return warnfln(fln,0,"%s",buf);
}

/* parse comma-separated file, e.g. gtfs.
   first line has colunm names, rest are values
   values can be quoted and if so can contain commas
   quote within quoted strings are doubled
 */

#define Valcnt 32
#define Collen 256

enum extstates {Init,Val0,Val1,Val2,Val2q,Val3q,Cmd0,Cmd1,Cmd2,Cfls,Fls};

struct extfmt {
  struct myfile mf;
  ub4 pos;
  enum extstates state;
  ub4 linno,colno;
  ub4 valndx,valcnt;
  ub4 ivals[Valcnt];
  ub4 vallens[Valcnt];
  ub4 uvals[Valcnt];
  ub4 valtypes[Valcnt];
  char vals[Valcnt * Collen];
};

static enum extresult nextchar_csv(struct extfmt *ef)
{
  char *fname;
  ub1 c;
  ub4 pos,len,linno,colno,valndx,valno;
  ub4 uval;
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
  ub4 *valtypes = ef->valtypes;
  ub4 *uvals = ef->uvals;

  c = ef->mf.buf[pos];
  ef->pos = pos + 1;

  newitem = iscmd = 0;
  uval = uvals[valndx];

//    info(0,"state %u c %c",state,c);

  int neweof = 0;

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
        case ' ': break; // ignore leading ws
        case '"': break; // simple quote handling
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
        case '"': break; // simple quote handling
        case '\n': newitem = iscmd = 1; state = Val0; break;
        default:
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
          neweof = 1;
      }
      break;

    case Cmd2:
      switch (c) {
        case ',': return parserr(FLN,fname,linno,colno,"empty column name");
        case '\r': break;
        case '\n': newitem = iscmd = 1; state = Val0; break;
        case ' ': break;
        case '"': break; // simple quote handling
        default:
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
          state = Cmd1;
          neweof = 1;
      }
      break;

    case Val0:  // start of first col
      valndx = 0; uval = 0;
      switch (c) {
        case ',': vallens[0] = 0; uvals[0] = uval; valndx = 1;  vallens[1] = 0; state = Val1; break;
        case '\r': case ' ': break;
        case '\n': vrb0(0,"skipping empty line at %u",linno); break;
        case '"': vallens[0] = 0; state = Val2q; break;
        case '\t': c = ' '; // cascade
        default:
          val = vals;
          val[0] = c; vallens[0] = 1;
          state = Val1;
          if (c >= '0' && c <= '9') uval = c - '0';
          else uval = hi32;
      }
      break;

    case Val1: // within col
      switch(c) {
        case ',': uvals[valndx++] = uval; vallens[valndx] = 0; state = Val2; break;
        case '\r': break;
        case '\n': newitem = 1; state = Val0; break;
        case '\t': c = ' '; // cascade
        default:
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
          if (uval != hi32 && c >= '0' && c <= '9') uval = uval * 10 + (c - '0');
          else if (c != ' ') uval = hi32;
          neweof = 1;
      }
      break;

    case Val2: // start of subsequent cols
      switch (c) {
        case ',': uvals[valndx++] = uval; vallens[valndx] = 0; break;
        case '\r': case ' ': break;
        case '\n': newitem = 1; state = Val0; break;
        case '"': state = Val2q; uval = 0; break;
        case '\t': c = ' '; // cascade
        default:
          val = vals + valndx * Collen;
          val[0] = c; vallens[valndx] = 1;
          state = Val1;
          if (c >= '0' && c <= '9') uval = c - '0';
          else if (c != ' ') uval = hi32;
          neweof = 1;
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
          if (uval != hi32 && c >= '0' && c <= '9') uval = uval * 10 + (c - '0');
          else uval = hi32;
      }
      break;

    case Val3q:
      switch(c) {
        case '"': // escaped quote
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
          uval = hi32;
          state = Val2q;
          break;
        case ',': uvals[valndx++] = uval; vallens[valndx] = 0; state = Val2; break;
        case '\r': break;
        case '\n': newitem = 1; state = Val0; break;
        default: return parserr(FLN,fname,linno,colno,"unexpected char '%c' after quote",c);
      }
      break;

    case Cfls: case Fls: break;
    } // end switch state

  if (c == '\n') { linno++; colno = 1; }
  else colno++;

  ef->state = state;

  ef->valndx = valndx;
  ef->linno = linno - 1;
  ef->colno = colno;
  uvals[valndx] = uval;

  if (pos >= len && neweof) newitem = 1;

  if (newitem) {
    for (valno = 0; valno <= valndx; valno++) {
      val = vals + valno * Collen;
      vallen = vallens[valno];
      if (vallen == 0) uvals[valno] = hi32;
      else if (vallen && val[vallen-1] == ' ') vallens[valno] = --vallen; // strip trailing ws
      val[vallen] = 0;
      uval = uvals[valno];
      if (uval == hi32 && valtypes[valno]) {
        return parserr(FLN,fname,linno,colno,"expected numerical value, found %s",val);
      }
    }
    ef->valcnt = valndx + 1;
    return iscmd ? Newcmd : Newitem;
  } else return Next;
}

// similar to above, tab-separated canonical
static enum extresult nextchar_canon(struct extfmt *ef)
{
  char *fname;
  ub1 c;
  ub4 pos,len,linno,colno,valndx,valno;
  char *val,*vals;
  ub4 *vallens,vallen;
  ub4 uval;
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
  ub4 *uvals = ef->uvals;

  c = ef->mf.buf[pos];
  ef->pos = pos + 1;

  newitem = iscmd = 0;
  uval = uvals[valndx];

//    info(0,"state %u c %c",state,c);

    if (c == '\r') return parserr(FLN,fname,linno,colno,"unexpected char %x",c);

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
        case '#': valndx = 0; state = Cfls; break;
        case '\t': return parserr(FLN,fname,linno,colno,"empty column name");
        case '\n': valndx = 0; break;
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
        case '\t': info(0,"col %u %.*s",valndx,vallens[valndx],vals + valndx * Collen); valndx++; vallens[valndx] = 0; state = Cmd2; break;
        case '\n': newitem = iscmd = 1; state = Val0; break;
        default:
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
      }
      break;

    case Cmd2:
      switch (c) {
        case '\t': return parserr(FLN,fname,linno,colno,"empty column name");
        case '\n': newitem = iscmd = 1; state = Val0; break;
        default:
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
          state = Cmd1;
      }
      break;

    case Val0:
      valndx = 0; uval = 0;
      switch (c) {
        case '#': state = Fls; break;
        case '\t': vallens[0] = 0; uvals[0] = uval; valndx = 1; vallens[1] = 0; uvals[1] = 0; state = Val1; break;
        case '\n': break;
        default:
          val = vals;
          val[0] = c; vallens[0] = 1;
          if (c >= '0' && c <= '9') uval = c - '0';
          else uval = hi32;
          state = Val1;
      }
      break;

    case Val1:
      switch(c) {
        case '\t': uvals[valndx++] = uval; vallens[valndx] = 0; state = Val2; uvals[valndx] = 0; break;
        case '\n': newitem = 1; state = Val0; break;
        default:
          val = vals + valndx * Collen;
          vallen = vallens[valndx]; val[vallen] = c; vallens[valndx] = vallen + 1;
          if (uval != hi32 && c >= '0' && c <= '9' && uval < (hi32 / 10)) uval = uval * 10 + (c - '0');
          else uvals[valndx] = hi32;
      }
      break;

    case Val2:
      switch (c) {
        case '\t': uvals[valndx++] = uval; vallens[valndx] = 0; uvals[valndx] = 0; break;
        case '\n': newitem = 1; state = Val0; break;
        default:
          val = vals + valndx * Collen;
          val[0] = c; vallens[valndx] = 1;
          if (c >= '0' && c <= '9') uval = c - '0';
          else uval = hi32;
          state = Val1;
      }
      break;

    case Fls: if (c == '\n') state = Val0; break;
    case Cfls: if (c == '\n') state = Cmd0; break;

    case Val2q:
    case Val3q: break;

    } // end switch state

  if (c == '\n') { linno++; colno = 1; }
  else colno++;

  ef->state = state;

  ef->valndx = valndx;
  ef->linno = linno - 1;
  ef->colno = colno;

  uvals[valndx] = uval;

  if (newitem) {
    for (valno = 0; valno <= valndx; valno++) {
      val = vals + valno * Collen;
      vallen = vallens[valno];
      if (vallen == 0) uvals[valno] = hi32;
      val[vallen] = 0;
    }
    ef->valcnt = valndx + 1;
    return iscmd ? Newcmd : Newitem;
  } else return Next;
}

static enum extresult nextchar(struct extfmt *ef)
{
  if (canonin) return nextchar_canon(ef);
  else return nextchar_csv(ef);
}

static ub4 addcol(char *lines,ub4 pos,char *col,ub4 collen,char c,bool addpfx)
{
  if (addpfx && canonin == 0 && collen && prefixlen) {
    memcpy(lines + pos,prefix,prefixlen);
    pos += prefixlen;
    lines[pos++] = '/';
  }
  if (collen) {
    memcpy(lines + pos,col,collen);
    pos += collen;
  }
  lines[pos++] = c;
  return pos;
}

enum Txmode { Tram,Metro,Rail,Bus,Ferry,Cabcar,Gondola,Funicular,Plane_int,Plane_dom, Modecnt };
static const char *modenames[] = { "tram","metro","rail","bus","ferry","cable car","gondola","funicular","air-dom","air-int","unknown" };
static ub4 rmodecnts[Modecnt + 1];
static ub4 modecnts[Modecnt + 1];

// extended types from support.google.com/transitpartners/answer/3520902
static ub4 xrtype2rtype(ub4 x)
{
  error_eq(x,hi32);
  if (x >= 100 && x < 118) return Rail;
  if (x >= 200 && x < 210) return Bus;
  if (x >= 700 && x < 717) return Bus;
  if (x >= 900 && x < 907) return Rail;
  if (x >= 1000 && x < 1022) return Ferry;

  switch(x) {
  case 300: case 400: case 403: case 404: return Rail;
  case 401: case 402: case 405: case 500: case 600: return Metro;
  case 800: return Bus;
  case 1103: case 1106: case 1107: case 1112: case 1114: return Plane_int;
  case 1104: case 1105: case 1108: case 1109: case 1110: case 1111: case 1113: return Plane_dom;
  case 1200: return Ferry;

  case 0: return Tram;
  case 1: return Metro;
  case 2: return Rail;
  case 3: return Bus;
  case 4: return Ferry;
  case 5: return Cabcar;
  case 6: return Gondola;
  case 7: return Funicular;

  default: info(0,"unknown route type %u",x); return x;
  }
}

static int nobus,norail,notram,nometro,noferry,noair;

static int filter(ub4 rtype)
{
  switch(rtype) {
  case Tram: return notram;
  case Metro: return nometro;
  case Rail: return norail;
  case Bus: return nobus;
  case Ferry: return noferry;
  case Plane_int: return noair;
  case Plane_dom: return noair;
  default: return 0;
  }
}

static ub4 defagencylen;
static char defagency[128];

static int rdagency(gtfsnet *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawcnt,cnt = 0;
  int rv;
  char *buf;
  ub4 len,linno,colno;
  char *val,*vals,*idval;
  ub4 idvlen,vlen,*vallens;
  ub4 valcnt,valno;
  ub4 linepos = 0,linelen;
  block *mem = &net->agencymem;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/agency.%s",dir,fileext);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1,0);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return error(0,"%s is empty",fname);

  linelen = len + (rawcnt + 2) * prefixlen1;
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
        idval = vals + agency_idpos * Collen;
        idvlen = vallens[agency_idpos];

        bound(mem,linepos + idvlen + 1,char);

        if (idvlen && defagencylen == 0) {
          defagencylen = min(sizeof(defagency)-1,idvlen);
          memcpy(defagency,idval,idvlen);
        }
      } else { idvlen = 0; idval = NULL; }

// name
      val = vals + agency_namepos * Collen;
      vlen = vallens[agency_namepos];

      bound(mem,linepos + vlen + 1,char);

      if (vlen && defagencylen == 0) {
        defagencylen = min(sizeof(defagency)-1,vlen);
        memcpy(defagency,val,vlen);
      }

      if (idvlen) linepos = addcol(lines,linepos,idval,idvlen,tab,1);
      else if (defagencylen) linepos = addcol(lines,linepos,defagency,defagencylen,tab,1);
      else return error(0,"line %u: no agency id or name",linno);

      linepos = addcol(lines,linepos,val,vlen,tab,0);

// tz
      val = vals + agency_tzpos * Collen;
      vlen = vallens[agency_tzpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,0);

// url
      val = vals + agency_urlpos * Collen;
      vlen = vallens[agency_urlpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,'\n',0);

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
  ub4 dow,date_cd,date;
  ub4 linepos = 0,linelen;
  block *mem = &net->calendarmem;
  char datestr[64];

  oclear(eft);

  fmtstring(eft.mf.name,"%s/calendar.%s",dir,fileext);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,0,0);
  if (rv) return 1;
  if (eft.mf.exist == 0) return 0;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return warning(0,"%s is empty",fname);

  linelen = len + rawcnt * prefixlen1;
  char *lines = net->calendarlines = mkblock(mem,linelen,char,Noinit,"gtfs %u calendar, len %u",rawcnt-1,linelen);

  const char tab = '\t';

  ub4 service_idpos = hi32,startpos = hi32,endpos = hi32;
  ub4 dowpos[7] = {hi32,hi32,hi32,hi32,hi32,hi32,hi32};

  vals = eft.vals;
  ub4 *uvals = eft.uvals;

  do {

    res = nextchar(&eft);

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
      linepos = addcol(lines,linepos,val,vlen,tab,1);

// monday .. sunday
      for (dow = 0; dow < 7; dow++) {
        val = vals + dowpos[dow] * Collen;
        vlen = vallens[dowpos[dow]];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,tab,0);
      }

// start
      val = vals + startpos * Collen;
      vlen = vallens[startpos];
      date_cd = uvals[startpos];

      if (dateshift && date_cd != hi32) {
        date = cd2day(date_cd);
        date += dateshift;
        date_cd = day2cd(date);
        vlen = fmtstring(datestr,"%u",date_cd);
        val = datestr;
      }
      lodate = min(lodate,date_cd);
      hidate = max(hidate,date_cd);
      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,0);

// end
      val = vals + endpos * Collen;
      vlen = vallens[endpos];
      date_cd = uvals[endpos];

      if (dateshift && date_cd != hi32) {
        date = cd2day(date_cd);
        date += dateshift;
        date_cd = day2cd(date);
        vlen = fmtstring(datestr,"%u",date_cd);
        val = datestr;
      }
      lodate = min(lodate,date_cd);
      hidate = max(hidate,date_cd);
      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,'\n',0);

      cnt++;
      break;

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"%u from %u entries  timebox %u - %u",cnt,rawcnt,lodate,hidate);
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
  ub4 date,date_cd;
  ub4 linepos = 0,linelen;
  block *mem = &net->caldatesmem;
  char datestr[64];

  oclear(eft);

  fmtstring(eft.mf.name,"%s/calendar_dates.%s",dir,fileext);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,0,0);
  if (rv) return 1;
  if (eft.mf.exist == 0) return 0;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return warning(0,"%s is empty",fname);

  linelen = len + rawcnt * prefixlen1;
  char *lines = net->caldateslines = mkblock(mem,linelen,char,Noinit,"gtfs %u calendar dates, len %u",rawcnt-1,linelen);

  const char tab = '\t';
  ub4 service_idpos = hi32,extype_pos = hi32,datepos = hi32;

  ub4 *uvals = eft.uvals;
  vals = eft.vals;

  do {

    res = nextchar(&eft);

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
      linepos = addcol(lines,linepos,val,vlen,tab,1);

// extype
      val = vals + extype_pos * Collen;
      vlen = vallens[extype_pos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,0);

// date
      val = vals + datepos * Collen;
      vlen = vallens[datepos];
      date_cd = uvals[datepos];

      if (dateshift && date_cd != hi32) {
        date = cd2day(date_cd);
        date += dateshift;
        date_cd = day2cd(date);
        vlen = fmtstring(datestr,"%u",date_cd);
        val = datestr;
      }
      lodate = min(lodate,date_cd);
      hidate = max(hidate,date_cd);
      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,'\n',0);

      cnt++;
      break;

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"%u from %u entries  timebox %u - %u",cnt,rawcnt,lodate,hidate);
  net->caldatescnt = cnt;
  net->caldateslinepos = linepos;

  return 0;
}

static int rdroutes(gtfsnet *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawcnt,tcnt,cnt = 0;
  int rv;
  char *buf;
  ub4 len,linno,colno;
  char *val,*vals;
  ub4 vlen,*vallens;
  ub4 *uvals;
  ub4 *valtypes;
  ub4 rrid,rtype,xrtype;
  ub4 valcnt,valno;
  ub4 linepos = 0,linelen;
  block *mem = &net->routemem;
  ub4 rid;
  char *idval,*agval;
  ub4 idvlen,agvlen;
  oclear(eft);

  fmtstring(eft.mf.name,"%s/routes.%s",dir,fileext);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1,0);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return error(0,"%s is empty",fname);

  hash *routes;
  if (canonin) routes = NULL;
  else routes = net->routes = mkhash(1024 * 1024,10,rawcnt * 64,"routes");

  linelen = len + 4 * rawcnt + (2 * rawcnt) * prefixlen1 + rawcnt * defagencylen; // optional agency_id
  char *lines = net->routelines = mkblock(mem,linelen,char,Noinit,"gtfs %u routes, len %u",rawcnt-1,linelen);

  const char tab = '\t';
  ub4 route_idpos,agencypos,snamepos,lnamepos,descpos,rtypepos;
  route_idpos=agencypos=snamepos=lnamepos=descpos=rtypepos = hi32;

  vals = eft.vals;
  uvals = eft.uvals;
  valtypes = eft.valtypes;

  do {

    res = nextchar(&eft);

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
        else if (streq(val,"route_type")) { rtypepos = valno; valtypes[valno] = 1; }
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
      idval = vals + route_idpos * Collen;
      idvlen = vallens[route_idpos];
      rrid = uvals[route_idpos];

// agency
      if (agencypos != hi32) {
        agval = vals + agencypos * Collen;
        agvlen = vallens[agencypos];
      } else {
        agval = defagency;
        agvlen = defagencylen;
      }

// type
      val = vals + rtypepos * Collen;
      vlen = vallens[rtypepos];

      xrtype = uvals[rtypepos];
      infocc(xrtype == hi32,0,"line %u no rtype for '%s'",linno,val);
      rtype = xrtype2rtype(xrtype);
      rmodecnts[min(rtype,Modecnt)]++;
      vrb0(0,"route id '%s' %u type '%s' %u",idval,rrid,val,rtype);

      if (filter(rtype)) {
        vrb0(0,"filter route %s line %u on type '%s' %u",idval,linno,val,rtype);
        break;
      }
      modecnts[min(rtype,Modecnt)]++;

      bound(mem,linepos + idvlen + 1,char);
      linepos = addcol(lines,linepos,idval,idvlen,tab,1);

      bound(mem,linepos + agvlen + 1,char);
      linepos = addcol(lines,linepos,agval,agvlen,tab,1);

      bound(mem,linepos + vlen + 1,char);
      linepos += myutoa(lines + linepos,rtype);
      lines[linepos++] = tab;

      if (canonin == 0) {
        rid = gethash(routes,idval,idvlen,rrid);
        if (rid != hi32) {
          parsewarn(FLN,fname,linno,colno,"route %s already defined on line %u",idval,rid);
          break;
        }
        if (addhash(routes,idval,idvlen,rrid,linno) == hi32) return 1;
        rid = gethash(routes,idval,idvlen,rrid);
        if (rid == hi32) return error(0,"stored %s not found",idval);
      }

// sname
      if (snamepos != hi32) {
        val = vals + snamepos * Collen;
        vlen = vallens[snamepos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,tab,0);
      } else lines[linepos++] = tab;

// lname
      if (lnamepos != hi32) {
        val = vals + lnamepos * Collen;
        vlen = vallens[lnamepos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,tab,0);
      } else lines[linepos++] = tab;

// desc
      if (descpos != hi32) {
        val = vals + descpos * Collen;
        vlen = vallens[descpos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,'\n',0);
      } else lines[linepos++] = '\n';

      cnt++;
      break;

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"%u from %u entries",cnt,rawcnt);

  for (rtype = 0; rtype <= Modecnt; rtype++) {
    tcnt = rmodecnts[rtype];
    infocc(tcnt || cnt == 0,0,"%u from %u %s routes",modecnts[rtype],tcnt,modenames[rtype]);
  }

  net->routecnt = cnt;
  net->routelinepos = linepos;

  return errorcc(cnt == 0,0,"no routes from %u",rawcnt);
}

static int rdstops(gtfsnet *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawstopcnt,stopcnt,stop2,stop3,stop = 0;
  ub4 pstopcnt = 0;
  int rv;
  char *buf;
  ub4 len,linno,colno;
  ub4 x,n;
  char *val,*vals;
  ub4 vlen,*vallens;
  ub4 *uvals;
  ub4 valcnt,colcnt,valno;
  ub4 stopid,rstopid;
  ub4 linepos = 0,linelen;
  block *mem = &net->stopmem;
  block *emem = &net->estopmem;

  hash *hstops = net->stops;

  struct eta eta;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/stops.%s",dir,fileext);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1,0);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawstopcnt = linecnt(fname,buf, len);

  if (rawstopcnt == 0) return warning(0,"%s is empty",fname);

  struct gtstop *sp,*sp2,*sp3,*psp,*stops = alloc(rawstopcnt,struct gtstop,0,"ext ports",rawstopcnt);

  linelen = len + 5 * rawstopcnt + rawstopcnt * prefixlen1;
  char *lines = net->stoplines = mkblock(mem,linelen,char,Noinit,"gtfs %u stops, len %u",rawstopcnt-1,linelen);

  const char tab = '\t';

  ub4 stop_idpos,stop_codepos,stop_namepos,stop_descpos,stop_latpos,stop_lonpos,stop_locpos,parent_stapos;
  stop_idpos=stop_codepos=stop_namepos=stop_descpos=stop_latpos=stop_lonpos=stop_locpos=parent_stapos = hi32;

  sp = stops;

  vals = eft.vals;
  uvals = eft.uvals;

  colcnt = 0;

  do {

    res = nextchar(&eft);

    switch(res) {
    case Newcmd:
      valcnt = colcnt = eft.valcnt;
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
        else info(0,"skipping column '%s'",val);
      }
      if (stop_idpos == hi32) return error(0,"%s: missing required column stopid",fname);
      if (stop_namepos == hi32) return error(0,"%s: missing required column stop_name",fname);
      if (stop_latpos == hi32) return error(0,"%s: missing required column stop_lat",fname);
      if (stop_lonpos == hi32) return error(0,"%s: missing required column stop_lon",fname);
      if (stop_locpos == hi32) info(0,"%s: no column location_type",fname);
      if (parent_stapos == hi32) info(0,"%s: no column parent_station",fname);
      break;

    case Newitem:

      if (progress(&eta,"reading stop %u of %u in %s",stop,rawstopcnt,fname)) return 1;

      valcnt = eft.valcnt;
      linno = eft.linno;
      colno = eft.colno;
      vallens = eft.vallens;
      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        vrb0(0,"line %u col %u val '%s'",linno,valno,val);
      }
      if (valcnt < 4 || valcnt + 1 < colcnt) return parserr(FLN,fname,linno,colno,"missing required columns, only %u",valcnt);
      else if (valcnt != colcnt) infocc(stop == 0,0,"row has %u cols, header %u",valcnt,colcnt);

      sp->id = stop;
// id
      val = vals + stop_idpos * Collen;
      vlen = vallens[stop_idpos];

      rstopid = uvals[stop_idpos];

      if (hstops) {
        stopid = gethash(hstops,val,vlen,rstopid);
//        info(0,"line %u: %u:%s = %u hash %u",linno,vlen,val,stopid,rstopid);
        if (stopid == hi32) {
          infocc(show_omitstop,0,"line %u: omitting unreferenced stop %u:%s",linno,vlen,val);
          break;
        }
      }

      sp->gidofs = linepos;

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,1);
      sp->gidlen = linepos - sp->gidofs - 1;

// code
      if (stop_codepos != hi32) {
        val = vals + stop_codepos * Collen;
        vlen = vallens[stop_codepos];
        sp->codeofs = linepos;
        sp->codelen = vlen;
        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,tab,0);
      } else lines[linepos++] = tab;

// loc
      sp->isparent = 0;
      if (stop_locpos != hi32) {
        val = vals + stop_locpos * Collen;
        vlen = vallens[stop_locpos];

        bound(mem,linepos + 2,char);
        if (vlen == 0) x = '0';
        else if (vlen == 1)  x = *val;
        else { parsewarn(FLN,fname,linno,colno,"location_type %s not '0' or '1'",val); x = '0'; }
        if (x != '0' && x != '1') { parsewarn(FLN,fname,linno,colno,"location_type %c not '0' or '1'",x); x = '0'; }
        if (x == '1') {
          sp->isparent = 1;
          pstopcnt++;
        }
        lines[linepos++] = (char)x;
      }
      lines[linepos++] = tab;

// parent
      if (parent_stapos != hi32) {
        val = vals + parent_stapos * Collen;
        vlen = vallens[parent_stapos];

        if (vlen) {
          if (sp->isparent) parsewarn(FLN,fname,linno,colno,"parent stop with parent");
          sp->hasparent = 1;
          sp->parentofs = linepos;
          sp->parentlen = vlen;
          bound(mem,linepos + vlen + 1,char);
          linepos = addcol(lines,linepos,val,vlen,tab,1);
        }
      } else lines[linepos++] = tab;

//name
      val = vals + stop_namepos * Collen;
      vlen = vallens[stop_namepos];

      sp->nameofs = linepos;
      sp->namelen = vlen;
      memcpy(sp->name,val,min(vlen,sizeof(sp->name)-1));
      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,0);

//lat
      val = vals + stop_latpos * Collen;
      vlen = vallens[stop_latpos];
      if (vlen && *val == ' ') { val++; vlen--; }

      sp->latofs = linepos;
      if (vlen) {
        if (str2dbl(val,vlen,&sp->lat)) parsewarn(FLN,fname,linno,colno,"cannot convert coord '%.*s'",vlen,val);
        else sp->latlen = vlen;
      } else vrb0(0,"stop %s has no lat",sp->name);
      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,0);

//lon
      val = vals + stop_lonpos * Collen;
      vlen = vallens[stop_lonpos];
      if (vlen && *val == ' ') { val++; vlen--; }

      sp->lonofs = linepos;
      if (vlen) {
        if (str2dbl(val,vlen,&sp->lon)) parsewarn(FLN,fname,linno,colno,"cannot convert coord '%.*s'",vlen,val);
        else sp->lonlen = vlen;
      }
      sp->lonlen = vlen;
      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,0);

//desc
      if (stop_descpos != hi32) {
        val = vals + stop_descpos * Collen;
        vlen = vallens[stop_descpos];

        sp->descofs = linepos;
        sp->desclen = vlen;
        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,'\n',0);
      } else lines[linepos++] = '\n';

      sp++;
      stop++;
      error_gt(stop,rawstopcnt,0);
      break;  // newitem

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  stopcnt = stop;
  info(0,"%u stops from %u lines %u parents",stopcnt,rawstopcnt,pstopcnt);
  error_ge(pstopcnt,stopcnt);

  struct gtstop *pstops = allocnz(pstopcnt,struct gtstop,0,"ext pports",pstopcnt);

  psp = pstops;
  for (stop = 0; stop < stopcnt; stop++) {
    sp = stops + stop;
    if (sp->isparent == 0) continue;
    *psp++ = *sp;
  }

  // optionally let child stops inherit parent name
  ub4 pstop,plen;
  char *pofs;
  for (stop = 0; stop < stopcnt; stop++) {
    sp = stops + stop;
    if (useparentname == 0 || (sp->isparent == 0 && sp->hasparent == 0)) continue;

    if (sp->hasparent) {
      plen = sp->parentlen;
      pofs = lines + sp->parentofs;
      for (pstop = 0; pstop < pstopcnt; pstop++) {
        psp = pstops + pstop;
        if (plen != psp->gidlen || memcmp(pofs,lines + psp->gidofs,plen)) continue;
        sp->nameofs = psp->nameofs;
        sp->namelen = psp->namelen;
        break;
      }
      if (pstop == pstopcnt) warning(0,"parent %.*s not found",plen,pofs);
    }
  }

  ub4 cstopcnt = stopcnt - pstopcnt;
  info(0,"prepare grouping for %u stops",cstopcnt);

  struct gtstop *csp,*cstops = alloc(cstopcnt,struct gtstop,0,"ext cports",cstopcnt);
  ub4 id;

  ub4 cstop = 0;
  csp = cstops;
  for (stop = 0; stop < stopcnt; stop++) {
    sp = stops + stop;
    if (sp->isparent) continue;
    *csp = *sp;
    csp->id = cstop;
    csp++;
    cstop++;
  }
  error_ne(cstop,cstopcnt);

  ub4 cnt,hicnt = 0,histop = 0,sumcnt = 0;
  double d2r = M_PI / 180;
  double axislim = M_PI * 8.0e-5;
  double a1,a2,o1,o2,dist;

  for (stop = 0; stop < cstopcnt; stop++) {
    sp = cstops + stop;
    sp->group = hi32;
    sp->iparent = hi32;
    if (sp->latlen == 0 || sp->lonlen == 0) continue;
    sp->rlat = sp->lat * d2r;
    sp->rlon = sp->lon * d2r;
  }
  for (stop = 0; stop < cstopcnt; stop++) {
    if (progress(&eta,"pass 1 group stop %u of %u sum %u",stop,cstopcnt,sumcnt)) return 1;
    sp = cstops + stop;
    if (sp->latlen == 0 || sp->lonlen == 0) continue;
    cnt = sp->nearcnt;
    if (cnt >= Nearstop) continue;
    id = sp->id;
    a1 = sp->rlat; o1 = sp->rlon;
    for (stop2 = 0; stop2 < cstopcnt; stop2++) {
      if (stop2 == stop) continue;
      sp2 = cstops + stop2;
      if (sp2->latlen == 0 || sp2->lonlen == 0) continue;
      a2 = sp2->rlat; o2 = sp2->rlon;
      if (a1 - a2 > axislim || a1 - a2 < -axislim) continue;
      if (o1 - o2 > axislim || o1 - o2 < -axislim) continue;
      dist = geodist(a1,o1,a2,o2);
//      if (dist < 1e-5) error(Exit,"dist 0 for %s to %s",sp->name,sp2->name);
      if (dist > grouplimit) continue;
      sp->nears[cnt++] = stop2;
      if (cnt >= Nearstop) break;
    }
    if (cnt > hicnt) { hicnt = cnt; histop = stop; }
    sp->nearcnt = cnt;
    sumcnt += cnt;
  }

  info(0,"hicnt %u sum %u",hicnt,sumcnt);

  ub4 iparentcnt,iparent = 0;
  ub4 s2ndx,s3ndx;
  ub4 orghicnt = hicnt;

  sumcnt = 0;
  ub4 enearcnt;

  while (hicnt) {
    if (progress(&eta,"iter %u of ~%u sum %u", orghicnt - hicnt,orghicnt,sumcnt)) return 1;
//    info(0,"iter %u of ~%u sum %u", orghicnt - hicnt,orghicnt,sumcnt);

    sp = cstops + histop;
    id = sp->id;
    cnt = sp->nearcnt;
    enearcnt = 0;
    for (s2ndx = 0; s2ndx < cnt; s2ndx++) {
      stop2 = sp->nears[s2ndx];
      if (stop2 == hi32) continue;
      sp2 = cstops + stop2;
      error_ne(sp2->group,hi32);
      sp2->group = id;
      enearcnt++;

      // remove this stop from other groups
      sp->nears[s2ndx] = hi32;
      for (stop3 = 0; stop3 < cstopcnt; stop3++) {
        if (stop3 == histop) continue;
        sp3 = cstops + stop3;
        for (s3ndx = 0; s3ndx < sp3->nearcnt; s3ndx++) {
          if (sp3->nears[s3ndx] == stop2) sp3->nears[s3ndx] = hi32;
        }
      }
    }
    for (stop3 = 0; stop3 < cstopcnt; stop3++) {
      if (stop3 == histop) continue;
      sp3 = cstops + stop3;
      for (s3ndx = 0; s3ndx < sp3->nearcnt; s3ndx++) {
        if (sp3->nears[s3ndx] == histop) sp3->nears[s3ndx] = hi32;
      }
    }

    error_ne(sp->group,hi32);
    sp->group = id;
    sp->iparent = iparent;
    enearcnt++;  // include hicnt self
    sp->enearcnt = enearcnt;
    iparent++;
    sumcnt += enearcnt;

    // search next hicnt
    hicnt = 0;
    for (stop = 0; stop < cstopcnt; stop++) {
      sp = cstops + stop;
      if (sp->group != hi32) continue;
      cnt = 0;
      for (s2ndx = 0; s2ndx < sp->nearcnt; s2ndx++) {
        stop2 = sp->nears[s2ndx];
        if (stop2 != hi32) cnt++;
      }
//      info(0,"stop %u cnt %u"
      if (cnt > hicnt) { hicnt = cnt; histop = stop; }
    }
    error_gt(sumcnt,cstopcnt,iparent);
//    info(0,"hicnt %u sum %u",hicnt,sumcnt);
  }
  iparentcnt = iparent;

  ub4 plainstopcnt = cstopcnt - sumcnt;
  ub4 planstopcnt = plainstopcnt + iparentcnt;
  ub4 estopcnt = cstopcnt + iparentcnt;
  info(0,"%u plain stops %u inferred parents %u planning stops",plainstopcnt,iparentcnt,planstopcnt);

  ub4 elinelen = 0;
  ub4 iparentlen = 8 + 6 + 6;
  ub4 geopreclen = 6 + 1 + 1 + 4;
  ub4 pos,group;

  for (stop = 0; stop < cstopcnt; stop++) {
    sp = cstops + stop;
    if (sp->latlen == 0 || sp->lonlen == 0) continue;
    len = sp->gidlen + sp->codelen + sp->namelen + sp->desclen;
    len += geopreclen * 2;
    if (sp->group != hi32) len += iparentlen;
    len += 10;
    if (sp->iparent != hi32) len += (len + iparentlen);
    elinelen += len;
  }

  char *elines = net->stoplines = mkblock(emem,elinelen,char,Noinit,"gtfs %u stops, len %u",estopcnt,elinelen);

  // stops
  pos = 0;
  for (stop = 0; stop < cstopcnt; stop++) {
    sp = cstops + stop;
    if (sp->latlen == 0 || sp->lonlen == 0) continue;

    // id,code,loctype
    n = mysnprintf(elines,pos,elinelen,"%.*s\t%.*s\t0\t",sp->gidlen,lines + sp->gidofs,sp->codelen,lines + sp->codeofs);
    pos += n;
    error_ge(pos,elinelen);

    // parent
    group = sp->group;
    if (group != hi32) {
      psp = cstops + group;
      if (canonin == 0) fmtstring(psp->parentname,"%s/%u-%u-c8gaTX73",prefix,group,psp->enearcnt);
      else fmtstring(psp->parentname,"%u-%u-c8gaTX73",group,psp->enearcnt);
      n = mysnprintf(elines,pos,elinelen,"%s",psp->parentname);
      error_z(n,stop);
      pos += n;
    }
    elines[pos++] = '\t';
    error_ge(pos,elinelen);

    // name
    pos += mysnprintf(elines,pos,elinelen,"%.*s\t",sp->namelen,lines + sp->nameofs);

    // lat,lon
    if (sp->latlen) pos += mysnprintf(elines,pos,elinelen,"%f\t",sp->lat);
    else elines[pos++] = '\t';
    if (sp->lonlen) pos += mysnprintf(elines,pos,elinelen,"%f\t",sp->lon);
    else elines[pos++] = '\t';

    // desc
    pos += mysnprintf(elines,pos,elinelen,"%.*s\n",sp->desclen,lines + sp->descofs);
    bound(emem,pos,char);
    error_ge(pos + 20,elinelen);
//    info(0,"stop %u pos %u",stop,pos);
  }

  // inferred parents
  for (stop = 0; stop < cstopcnt; stop++) {
    sp = cstops + stop;
    if (sp->iparent == hi32) continue;
    pos += mysnprintf(elines,pos,elinelen,"%s\t%.*s\t1\t\t",sp->parentname,sp->codelen,lines + sp->codeofs);
    pos += mysnprintf(elines,pos,elinelen,"%.*s\t",sp->namelen,lines + sp->nameofs);
    pos += mysnprintf(elines,pos,elinelen,"%f\t%f\t",sp->lat,sp->lon);
    pos += mysnprintf(elines,pos,elinelen,"%.*s\n",sp->desclen,lines + sp->descofs);
    bound(emem,pos,char);
    error_ge(pos + 20,elinelen);
  }

  net->stopcnt = estopcnt;
  net->stoplinepos = pos;

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
  ub4 *uvals;
  ub4 rrid,rtid,tid;
  ub4 valcnt,valno;
  ub4 linepos = 0,linelen;
  block *mem = &net->tripmem;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/trips.%s",dir,fileext);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1,0);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return warning(0,"%s is empty",fname);

  hash *trips;
  if (canonin) trips = NULL;
  else trips = net->trips = mkhash(1024 * 1024,10,rawcnt * 64,"trips");

  hash *routes = net->routes;
  ub4 rid;

  linelen = len + rawcnt + 3 * rawcnt * prefixlen1;
  char *lines = net->triplines = mkblock(mem,linelen,char,Noinit,"gtfs %u trips, len %u",rawcnt-1,linelen);

  const char tab = '\t';
  ub4 route_idpos,service_idpos,trip_idpos,headsignpos;
  route_idpos=service_idpos=trip_idpos=headsignpos = hi32;

  vals = eft.vals;
  uvals = eft.uvals;

  do {

    res = nextchar(&eft);

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
      rrid = uvals[route_idpos];

      if (canonin == 0) {
        rid = gethash(routes,val,vlen,rrid); // filtered ?
        if (rid == hi32) break;
      }

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,1);

// service id
      val = vals + service_idpos * Collen;
      vlen = vallens[service_idpos];

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,1);

// trip id
      val = vals + trip_idpos * Collen;
      vlen = vallens[trip_idpos];
      rtid = uvals[trip_idpos];

      if (canonin == 0) {

        tid = gethash(trips,val,vlen,rtid);
        if (tid != hi32) {
          parsewarn(FLN,fname,linno,colno,"trip %s previously defined on line %u",val,tid);
          break;
        }
        if (addhash(trips,val,vlen,rtid,linno) == hi32) return 1;
        tid = gethash(trips,val,vlen,rtid);
        if (tid == hi32) return error(0,"stored trip %s not present",val);
      }

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,1);

// headsign
      if (headsignpos != hi32) {
        val = vals + headsignpos * Collen;
        vlen = vallens[headsignpos];

        bound(mem,linepos + vlen + 1,char);
        linepos = addcol(lines,linepos,val,vlen,'\n',0);
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

// to be refined: porpoer time parsing and cleanup
static ub4 dotime(char *p,ub4 len,char *dst)
{
  ub4 i;

  for (i = 0; i < len; i++) if (p[i] == '.') p[i] = ':';

  if (len == 8) { // hh:mm:ss
    memcpy(dst,p,8);
    return 8;
  } else if (len == 5) { // hh:mm
    memcpy(dst,p,5);
    memcpy(dst + 5,":00",3);
    return 8;
  } else return 0;
}

static int rdstoptimes(gtfsnet *net,const char *dir)
{
  enum extresult res;
  struct extfmt eft;
  const char *fname;

  ub4 rawcnt,cnt = 0;
  int rv;
  char *buf;
  ub4 len,linno = 0,colno;
  char *val,*taval,*tdval,*vals;
  ub4 rtid,*uvals;
  ub4 vlen,clen,tavlen,tdvlen,*vallens;
  ub4 valcnt,colcnt,valno;
  ub4 stopid,rstopid;
  ub4 linepos = 0,linelen;
  block *mem = &net->stoptimesmem;
  char cval[64];

  struct eta eta;

  oclear(eft);

  fmtstring(eft.mf.name,"%s/stop_times.%s",dir,fileext);
  fname = eft.mf.name;

  rv = readfile(&eft.mf,fname,1,0);
  if (rv) return 1;

  buf = eft.mf.buf;
  len = (ub4)eft.mf.len;
  rawcnt = linecnt(fname,buf, len);

  if (rawcnt == 0) return warning(0,"%s is empty",fname);

  hash *stops;
  if (canonin) stops = NULL;
  else stops = net->stops = mkhash(1024 * 1024,10,1000 * 1000 * 64,"stops");

  linelen = len + rawcnt * 13 + 2 * rawcnt * prefixlen1;
  char *lines = net->stoptimeslines = mkblock(mem,linelen,char,Noinit,"gtfs %u stoptimes, len %u",rawcnt-1,linelen);

  const char tab = '\t';
  ub4 trip_idpos,stop_idpos,stop_seqpos,tarr_pos,tdep_pos;
  trip_idpos=stop_idpos=stop_seqpos=tarr_pos=tdep_pos = hi32;

  hash *trips = net->trips;
  ub4 tid;

  ub4 seq = 0,seqline = 0;
  bool seqinc = 1;
  ub4 prvlinno = 0;
  ub4 stopcnt = 0;

  vals = eft.vals;
  uvals = eft.uvals;

  colcnt = 0;

  do {

    res = nextchar(&eft);

    switch(res) {
    case Newcmd:
      valcnt = colcnt = eft.valcnt;
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
      prvlinno = linno;
      linno = eft.linno;
      colno = eft.colno;
      vallens = eft.vallens;
      error_ge(cnt,rawcnt);

      for (valno = 0; valno < valcnt; valno++) {
        val = vals + valno * Collen;
        vrb0(0,"col %u val '%s'",valno,val);
      }

      if (valcnt < 4 || valcnt < colcnt - 1) return parserr(FLN,fname,linno,colno,"require 4 columns, has %u, header %u",valcnt,colcnt);
      else if (valcnt != colcnt) infocc(cnt == 0,0,"row has %u columns, header %u",valcnt,colcnt);

      if (linno != prvlinno + 1) lines[linepos++] = '\n';

// tripid
      val = vals + trip_idpos * Collen;
      vlen = vallens[trip_idpos];
      rtid = uvals[trip_idpos];

      if (canonin == 0) {
        tid = gethash(trips,val,vlen,rtid);
        if (tid == hi32) break;
      }

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,1);

// stopid
      val = vals + stop_idpos * Collen;
      vlen = vallens[stop_idpos];

      rstopid = uvals[stop_idpos];

      if (canonin == 0) {
        stopid = gethash(stops,val,vlen,rstopid);
        if (stopid == hi32) {
          vrb0(0,"adding new stop %u:%s hash %u",vlen,val,rstopid);
          if (addhash(stops,val,vlen,rstopid,stopcnt) == hi32) return 1;
//        if (gethash(stops,val,vlen,rstopid) != stopcnt) return 1;
          stopcnt++;
        }
      }

      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,1);

// seq
      val = vals + stop_seqpos * Collen;
      vlen = vallens[stop_seqpos];

      if (vlen) {
        seq = uvals[stop_seqpos];
        if (seq == 0) return parserr(FLN,fname,linno,colno,"sequence zero");
        else if (seq == hi32) return parserr(FLN,fname,linno,colno,"non-numerical sequence '%s'",val);
        if (linno > seqline + 1) seqinc = (seq == 1);
        seqline = linno;
      } else {
        if (seqinc) seq++; else seq--;
        if (seq == 0) return parserr(FLN,fname,linno,colno,"sequence zero, set at line %u",seqline);
        vlen = myutoa(val,seq);
      }
      bound(mem,linepos + vlen + 1,char);
      linepos = addcol(lines,linepos,val,vlen,tab,0);

// tarr,tdep
      taval = vals + tarr_pos * Collen;
      tavlen = vallens[tarr_pos];

      if (valcnt < colcnt) {
        tdval = taval;
        tdvlen = 0;
      } else {
        tdval = vals + tdep_pos * Collen;
        tdvlen = vallens[tdep_pos];
      }

      if (tdvlen && tavlen == 0) { tavlen = tdvlen; taval = tdval; }
      else if (tavlen && tdvlen == 0) { tdvlen = tavlen; tdval = taval; }

      clen = dotime(taval,tavlen,cval);
      bound(mem,linepos + clen + 1,char);
      linepos = addcol(lines,linepos,cval,clen,tab,0);

      clen = dotime(tdval,tdvlen,cval);
      bound(mem,linepos + clen + 1,char);
      linepos = addcol(lines,linepos,cval,clen,'\n',0);

      cnt++;
      break;  // newitem

    case Next: break;
    case Eof: break;
    case Parserr: return 1;
    }

  } while (res < Eof);  // each input char

  info(0,"\ah%u from \ah%u lines %u stops",cnt,rawcnt,stopcnt);
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

  osrotate(fname,0,'0');
  fd = filecreate(fname,1);
  if (fd == -1) return 1;

  pos = wrhdr(fname,buf,buflen);
  pos += mysnprintf(buf,pos,buflen,"# %u %s\n\n",net->agencycnt,net->agencycnt == 1 ? "entry" : "entries");

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

  osrotate(fname,0,'0');
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

  osrotate(fname,0,'0');
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

  osrotate(fname,0,'0');
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

  osrotate(fname,0,'0');
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

  osrotate(fname,0,'0');
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

  osrotate(fname,0,'0');
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
  if (rdtrips(net,dir)) return 1;
  if (rdstoptimes(net,dir)) return 1;
  if (rdstops(net,dir)) return 1;

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

static int cmd_mergelim(struct cmdval *cv) {
  if (cv->valcnt) grouplimit = cv->uval;
  return 0;
}

static int cmd_dateshift(struct cmdval *cv) {
  if (cv->valcnt) dateshift = cv->uval;
  info(0,"dateshift %u",dateshift);
  return 0;
}

static int cmd_parentname(struct cmdval *cv) {
  useparentname = 1;
  info(0,"%s set",cv->subarg);
  return 0;
}

static int cmd_test(struct cmdval *cv) {
  testonly = 1;
  info(0,"%s set",cv->subarg);
  return 0;
}

static int cmd_canonin(struct cmdval *cv) {
  canonin = 1;
  fileext = "tab";
  info(0,"%s set",cv->subarg);
  return 0;
}

static int cmd_notram(struct cmdval *cv) { notram = 1; return info(0,"%s set",cv->subarg); }
static int cmd_nometro(struct cmdval *cv) { nometro = 1; return info(0,"%s set",cv->subarg); }
static int cmd_norail(struct cmdval *cv) { norail = 1; return info(0,"%s set",cv->subarg); }
static int cmd_nobus(struct cmdval *cv) { nobus = 1; return info(0,"%s set",cv->subarg); }
static int cmd_noferry(struct cmdval *cv) { noferry = 1; return info(0,"%s set",cv->subarg); }
static int cmd_noair(struct cmdval *cv) { noair = 1; return info(0,"%s set",cv->subarg); }

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
  { "mergelimit|m", "[limit]%u", "merge stops witin given distance", cmd_mergelim },
  { "dateshift|m", "[limit]%u", "shift dates by given offset", cmd_dateshift },
  { "test|t", NULL, "test only, no output", cmd_test },
  { "nobus", NULL, "exclude bus routes", cmd_nobus },
  { "norail", NULL, "exclude rail routes", cmd_norail },
  { "noferry", NULL, "exclude ferry routes", cmd_noferry },
  { "notram", NULL, "exclude tram routes", cmd_notram },
  { "nometro", NULL, "exclude metro routes", cmd_nometro },
  { "noair", NULL, "exclude air routes", cmd_noair },
  { "parentname", NULL, "use parent name for stops", cmd_parentname },
  { "canonin", NULL, "use canonical input", cmd_canonin },
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

  if (globs.argc) {
    dir = globs.args[0];

    oclear(mf);
    if (osfileinfo(&mf,dir)) return oserror(0,"cannot access net directory %s",dir);
    else if (mf.isdir == 0) return error(0,"net arg %s is not a directory",dir);
    if (setmsglog(dir,"gtfsprep.log",0)) return 1;

    if (canonin == 0) {
      if (globs.argc > 1) prefix = globs.args[1];
      else prefix = dir;
      prefixlen = (ub4)strlen(dir);
      prefixlen1 = prefixlen + 1;
    }
  } else {
    if (osfileinfo(&mf,"agency.txt")) return shortusage();
    dir  = ".";    
    if (canonin == 0) return shortusage();
  }

  if (readgtfs(&net,dir)) return 1;
  if (testonly == 0 && writegtfs(&net,dir)) return 1;

  return 0;
}
