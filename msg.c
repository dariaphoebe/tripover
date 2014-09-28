// msg.c - messages

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* This file contains messaging logic used for 3 purposes:
   - console messages for the end user ( info, status, diagnostics)
   - debug logging
   - assertions

   As the program relies heavily on format flexibility, a custom print formatter
   is included here that is mostly compatible with printf, yet contains custom features

  assertions are used extensively, and are made to be enabled in production 
 */

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "base.h"

static ub4 msgfile;
#include "msg.h"

#include "os.h"
#include "time.h"

#undef hdrstop

struct filecoord {
  char name[16];
};

static struct filecoord filenames[64];
static ub4 filendx = 1;

static enum Msglvl msglvl = Vrb;
static ub4 vrblvl = 2;
static ub4 msgopts;
static int msg_fd = 2;

static const char msgnames[Msglvl_last] = "XFAEWIV";
static const char *msgnames_long[Msglvl_last] = {
  "X",
  "fatal",
  "assert",
  "error",
  "warning",
  "info",
  "verbose" };

#define MSGLEN 4096
static char msgbuf[MSGLEN];
static char ccbuf[MSGLEN];
static char ccbuf2[MSGLEN];
static ub4 cclen,ccfln;

static ub4 oserrcnt;

static ub8 progstart;

// temporary choice. unbuffered is useful for console messages,
// yet debug logging may ask for buffering
static void msgwrite(char *buf, ub4 len)
{
  int nw;

  if (len == 0) return;

  nw = (int)oswrite(msg_fd,buf,len);

  if (nw == -1) nw = (int)oswrite(2,"\nI/O error on msg write\n",24);
  if (nw == -1) oserrcnt++;
  if (msg_fd > 2) oswrite(1,buf,len);
}

// make errors appear on stderr
static void myttywrite(char *buf, ub4 len)
{
  int nw;

  if (len == 0) return;

  nw = (int)oswrite(2,buf,len);
  if (nw == -1) oserrcnt++;
}

// basic %x
static ub4 xcnv(char *dst, ub4 x)
{
  ub4 n=0,c,nib = 7;

  while (nib) {
    c = (x >> (nib * 4)) & 0xf;
    if (c) break;
    nib--;
  }
  while (nib) {
    c = (x >> (nib * 4)) & 0xf;
    c += '0';
    if (c > '9') c += ('a' - '9') - 1;
    *dst++ = (char)c;
    n++;
    nib--;
  }
  c = (x & 0xf) + '0';
  if (c > '9') c += ('a' - '9') - 1;
  *dst = (char)c;

  return n+1;
}

// basic %u
static ub4 ucnv(char *dst, ub4 x,ub4 wid,char pad)
{
  ub4 n,nn;

  if (x < 10) n = 1;
  else if (x < 100) n = 2;
  else if (x < 1000) n = 3;
  else if (x < 10000) n = 4;
  else if (x < 100000) n = 5;
  else if (x < 1000000) n = 6;
  else if (x < 10000000) n = 7;
  else if (x < 100000000) n = 8;
  else if (x < 1000000000) n = 9;
  else n = 10;

//  wid = 0;

  nn = n;
  if (wid > n) {
    while (wid > nn) { *dst++ = pad; nn++; }
  }
  dst += n;
  do *--dst = (ub1)((x % 10) + '0'); while (x /= 10);
  return nn;
}

// human-readable %u, 12.3G 
static ub4 Ucnv(char *dst, ub4 x1,ub4 x2,char c)
{
  ub4 n;

  if (x1 > 1 && x2 >= 1000) {
    n = ucnv(dst,x1+1,0,0);
    dst[n] = c;
    return n+1;
  }
  if (x1 < 10) n = 1;
  else if (x1 < 100) n = 2;
  else if (x1 < 1000) n = 3;
  else if (x1 < 10000) n = 4;
  else n = 5;

  x2 &= 0x3ff;
  if (x2 < 10) n += 1;
  else if (x2 < 100) n += 2;
  else if (x2 < 1000) n += 3;
  else n += 4;

  dst += n + 2;
  *dst-- = c;
  *dst-- = ' ';
  do *dst-- = (ub1)((x2 % 10) + '0'); while (x2 /= 10);
  *dst-- = '.';
  do *dst-- = (ub1)((x1 % 10) + '0'); while (x1 /= 10);
  return n + 3;
}

// simple %e
static ub4 ecnv(char *dst, double x)
{
  double fexp,exp;
  ub4 xscale;
  ub4 nmant,n = 0;
  char *org = dst;
  char mantissa[32];

  if (isnan(x)) { memcpy(dst,"#NaN",4); return 4; }
  else if (isinf(x)) { memcpy(dst,"#Inf",4); return 4; }

  if (x < 0) { *dst++ = '-'; x = -x; }
  if (x < 1.0e-200) { memcpy(dst,"<1e-200",7); return 7; }

  fexp = log10(x);
  if (fexp < 0) {     // |x| < 1.0
    exp = floor(fexp);
    x *= pow(10,-exp);
  } else if (fexp >= 1) { // |x| >= 10.0
    exp = floor(fexp);
    x /= pow(10,exp);
  } else {
    exp = 0;
  }
  xscale = (ub4)(x * 1.0e+6);
  memcpy(mantissa,"??????",6);
  nmant = ucnv(mantissa,xscale,0,0);
  *dst++ = mantissa[0];
  *dst++ = '.';
  if (nmant < 2) { nmant = 6; }
  memcpy(dst,mantissa + 1,nmant - 1);
  dst += nmant - 1;
  *dst++ = 'e';
  if (exp < 0) { *dst++ = '-'; exp = -exp; }
  else *dst++ = '+';
  n = ucnv(dst,(ub4)exp,0,0);

  n += (ub4)(dst - org);
  return n;
}

/* supports a basic, yet compatible subset of printf :
   %c %d %u %x %e %s %p %03u %-12.6s %ld %*s
   extensions are led in by '\a' preceding a conversion :
   \ah%u  makes the integer formatted 'human readable' like 123.8 M for 123800000
   \av%u%p interprets the pointer arg '%p' as an array of '%u' integers. thus :  
   int arr[] = { 23,65,23,54 };  printf("\av%u%p", 4, arr ); 
    shows  '[23 65 23 54]'
 */
static ub4 vsnprint(char *dst, ub4 pos, ub4 len, const char *fmt, va_list ap)
{
  const char *p = fmt;
  ub4 n = 0,x;
  ub4 wid,prec,plen;
  unsigned int uval,vlen=0,*puval;
  unsigned long luval,lx;
  int ival,alt,padleft,do_U = 0, do_vec = 0;
  long lival;
  double dval;
  char *pval;
  char c1,c2;
  char pad;

  if (!p || pos >= len || len - pos < 2) return 0;
  len -= pos; dst += pos; len--;
  while (*p && len - n > 2) {
    c1 = *p++;
    if (c1 == '\a') {
      switch(*p++) {
        case 'h': do_U = 1; break;
        case 'v': do_vec = 1; break;
        case '%': dst[n++] = c1; c1 = '%'; break;
        default: dst[n++] = c1;
      }
    }
    if (c1 == '%') {
      wid = 0; pad = ' '; padleft = 0;
      prec = len;
      c2 = *p++;
      if (c2 == '-') { padleft = 1; c2 = *p++; }
      if (c2 == '*') { wid = va_arg(ap,unsigned int); c2 = *p++; }
      if (c2 == '0') pad = c2;
      while (c2 >= '0' && c2 <= '9') {
        wid = wid * 10 + (c2 - '0');
        c2 = *p++;
      }
      wid = min(wid,len - n);
      alt = 0;
      if (c2 == '#') { alt = 1; c2 = *p++; }
      else if (c2 == '.') {
        c2 = *p++;
        if (c2 == '*') { prec = va_arg(ap,unsigned int); c2 = *p++; }
        else prec = 0;
        while (c2 >= '0' && c2 <= '9') {
          prec = prec * 10 + (c2 - '0');
          c2 = *p++;
        }
      }
      if (c2 == 'l') {
        c2 = *p++;
        switch(c2) {
        case 'u': luval = va_arg(ap,unsigned long);
                  if (len - n <= 10) break;
                  if (do_U && luval >= 1024UL) {
                    lx = luval;
                    if (lx >= 1024UL * 1024UL) n += Ucnv(dst + n,(ub4)(lx >> 20),(ub4)(lx >> 10),'M');
                    else n += Ucnv(dst + n,(ub4)(lx >> 10),(ub4)lx,'K');
                  } else n += ucnv(dst + n,(ub4)luval,wid,pad  );
                  break;
        case 'x':
        case 'p': luval = va_arg(ap,unsigned long);
                  if (len - n <= 10) break;
                  n += xcnv(dst + n,(ub4)luval);
                  n += xcnv(dst + n,(ub4)(luval >> 32));
                  break;
        case 'd': lival = va_arg(ap,long);
                  if (len - n <= 20) break;
                  if (lival < 0) { dst[n++] = '-'; n += ucnv(dst + n, (ub4)-lival,wid,pad); }
                  else n += ucnv(dst + n, (ub4)lival,wid,pad);
                  break;
        default: dst[n++] = c2;
        }
      } else {
        switch(c2) {
        case 'u': uval = va_arg(ap,unsigned int);
                  if (do_vec) { vlen = uval; break; }
                  if (len - n <= 10) break;
                  if (do_U && uval >= 1024) {
                    x = uval;
                    if (x >= 1024 * 1024) n += Ucnv(dst + n,x >> 20,x >> 10,'M');
                    else n += Ucnv(dst + n,x >> 10,x,'K');
                  } else n += ucnv(dst + n,uval,wid,pad);
                  break;
        case 'x': uval = va_arg(ap,unsigned int);
                  if (len - n <= 10) break;
                  if (alt) { dst[n++] = '0'; dst[n++] = 'x'; }
                  n += xcnv(dst + n,uval);
                  break;
        case 'd': ival = va_arg(ap,int);
                  if (len - n <= 10) break;
                  if (ival < 0) { dst[n++] = '-'; n += ucnv(dst + n, -ival,wid,pad); }
                  else n += ucnv(dst + n,ival,wid,pad);
                  break;
        case 'e': dval = va_arg(ap,double);
                  if (len - n <= 12) break;
                  n += ecnv(dst + n,dval);
                  break;
        case 'c': uval = va_arg(ap,unsigned int);
                  if (isprint(uval)) dst[n++] = (char)uval;
                  else {
                    dst[n++] = '0'; dst[n++] = 'x';
                    if (len - n <= 10) break;
                    n += xcnv(dst + n,uval);
                  }
                  break;
        case 's': pval = va_arg(ap,char *);
                  if (!pval) pval = (char *)"(nil)";
                  plen = 0; while (plen < prec && pval[plen]) plen++;
                  if (padleft) {
                    while (*pval && n < len && prec--) dst[n++] = *pval++;
                    while (wid > plen) { dst[n++] = pad; wid--; }
                  } else {
                    while (wid > plen) { dst[n++] = pad; wid--; }
                    while (*pval && n < len && prec--) dst[n++] = *pval++;
                  }
                  break;
        case 'p': puval = va_arg(ap,unsigned int *);
                  if (len - n <= 10) break;
                  if (do_vec) {
                    n += ucnv(dst + n,vlen,0,0);
                    luval = (unsigned long)puval;
                    dst[n++] = '.'; dst[n++] = '[';
                    while (vlen--) {
                      uval = *puval++;
                      if (len - n <= 10) break;
                      n += ucnv(dst + n,uval,wid,pad);
                      if (vlen) dst[n++] = '-';
                    }
                    dst[n++] = ']'; dst[n++] = ' ';
//                    n += xcnv(dst + n,(unsigned int)(luval & 0xffffffff));
//                    if (sizeof(puval) > 4) n += xcnv(dst + n,(unsigned int)(luval >> 32));
                    do_vec = 0;
                  } else {
                    if (len - n <= 10) break;
                    luval = (unsigned long)puval;
                    n += xcnv(dst + n,(unsigned int)(luval & 0xffffffff));
                    if (sizeof(puval) > 4) n += xcnv(dst + n,(unsigned int)(luval >> 32));
                  }
                  break;
        default: dst[n++] = c2;
        }
      }
      do_U = 0;
    } else {
      dst[n++] = c1;
    }
  }
  dst[n] = 0;
  if (n && len - n < 10) dst[n-1] = '!';
  return n;
}

ub4 myvsnprintf(char *dst, ub4 pos, ub4 len, const char *fmt, va_list ap)
{
  return vsnprint(dst,pos,len,fmt,ap);
}

ub4 __attribute__ ((format (printf,4,5))) mysnprintf(char *dst, ub4 pos, ub4 len, const char *fmt, ...)
{
  va_list ap;
  ub4 n;

  va_start(ap, fmt);
  n = vsnprint(dst,pos,len,fmt,ap);
  va_end(ap);
  return n;
}

// print file coords only
ub4 msgfln(char *dst,ub4 pos,ub4 len,ub4 fln,ub4 wid)
{
  ub4 line = fln & 0xffff;
  ub4 fileno = fln >> 16;

  if (fileno < Elemcnt(filenames)) return mysnprintf(dst,pos,len, "%*s%-4u ",wid,filenames[fileno].name,line);
  return mysnprintf(dst,pos,len, "*%7x*%-4u ",fileno,line);
}

// main message printer. supports decorated and undecorated style
static void __attribute__ ((nonnull(5))) msg(enum Msglvl lvl, ub4 sublvl, ub4 fline, ub4 code, const char *fmt, va_list ap)
{
  ub4 opts;
  ub4 pos = 0, maxlen = MSGLEN;
  sb4 n = 0;
  ub8 now_usec,dusec,dsec,d100usec;
  char lvlnam;

  if (code & User) {
    code &= ~User;
    opts = 0;
  } else opts = msgopts;

  if (opts & Msg_stamp) {
    now_usec = gettime_usec();
    dusec = now_usec - progstart;
    dsec = dusec / (1000 * 1000);
    d100usec = (dusec % (1000 * 1000)) / 100;
  } else dsec = d100usec = 0;
  if (lvl >= Msglvl_last) {
    pos += mysnprintf(msgbuf,pos,maxlen, "\nE unknown msglvl %u\n",lvl);
    lvl = Error;
  } else if (lvl <= Warn) { // precede errors with relevant past message
    if (cclen) {
      n = mysnprintf(ccbuf2,0,maxlen, "CC           %s ",ccbuf);
      n += msgfln(ccbuf2,n,maxlen,ccfln,0);
      n += mysnprintf(ccbuf2,n,maxlen, "\n");
      msgwrite(ccbuf2,n);
    }
    cclen = 0;
  }

  lvlnam = msgnames[lvl];

  if (*fmt == '\n') {
    msgbuf[pos++] = '\n';
  }
  if (opts & Msg_type) {
    if (lvl >= Vrb) pos += mysnprintf(msgbuf, pos, maxlen, "%c%u ",lvlnam,sublvl);
    else pos += mysnprintf(msgbuf, pos, maxlen, "%c  ",lvlnam);
  } else if (lvl <= Warn) pos += mysnprintf(msgbuf, pos, maxlen, "%s ",msgnames_long[lvl]);

  if (opts & Msg_stamp) pos += mysnprintf(msgbuf, pos, maxlen, "%03u.%04u  ",(ub4)dsec,(ub4)d100usec);
  if (opts & Msg_pos) {
    pos += msgfln(msgbuf,pos,maxlen,fline,9);
  }

  if (code & Ind) {
    n = min(code & Indent, maxlen - pos);
    if (n) memset(msgbuf + pos,' ',n);
    pos += n;
  }
  if (opts && pos < maxlen) msgbuf[pos++] = ' ';
  if (lvl == Assert) pos += mysnprintf(msgbuf, pos, maxlen, "assert\n  ");
  pos += vsnprint(msgbuf, pos, maxlen, fmt, ap);
  pos = min(pos,maxlen-1);
  msgbuf[pos++] = '\n';
  msgwrite(msgbuf, pos);
  if ( (opts & Msg_ccerr) && lvl <= Warn && msg_fd != 2) myttywrite(msgbuf,pos);
}

void vmsg(enum Msglvl lvl,ub4 fln,const char *fmt,va_list ap)
{
  msg(lvl,0,fln,0,fmt,ap);
}

int __attribute__ ((format (printf,4,5))) genmsgfln(ub4 fln,enum Msglvl lvl,ub4 code,const char *fmt,...)
{
  va_list ap;

  if (msglvl < lvl) return 0;
  va_start(ap, fmt);
  msg(lvl, 0, fln, code, fmt, ap);
  va_end(ap);
  return (lvl < Warn);
}

void __attribute__ ((format (printf,3,4))) vrbfln(ub4 fln, ub4 code, const char *fmt, ...)
{
  va_list ap;
  va_list ap1;
  ub4 lvl;

  if (code & CC) {
    code &= ~CC;
    va_start(ap1, fmt);
    cclen = vsnprint(ccbuf,0,sizeof(ccbuf),fmt,ap1);
    ccfln = fln;
    va_end(ap1);
  }
  if (msglvl < Vrb) return;
  lvl = code / V0;
  if (lvl > vrblvl) return;

  va_start(ap, fmt);
  msg(Vrb, lvl, fln, code, fmt, ap);
  va_end(ap);
}

int __attribute__ ((format (printf,3,4))) infofln(ub4 line, ub4 code, const char *fmt, ...)
{
  va_list ap;

  if (msglvl < Info) return 0;
  va_start(ap, fmt);
  msg(Info, 0, line, code, fmt, ap);
  va_end(ap);
  return 0;
}

int __attribute__ ((format (printf,3,4))) warningfln(ub4 line, ub4 code, const char *fmt, ...)
{
  va_list ap;

  if (msglvl < Warn) return 0;
  va_start(ap, fmt);
  msg(Warn, 0, line, code, fmt, ap);
  va_end(ap);
  return 0;
}

int __attribute__ ((format (printf,3,4))) errorfln(ub4 line, ub4 code, const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  msg(Error, 0, line, code, fmt, ap);
  va_end(ap);
  if (code & Exit) exit(1);
  return 1;
}

int __attribute__ ((format (printf,3,4))) assertfln(ub4 line, ub4 code, const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  msg(Assert, 0, line, code, fmt, ap);
  va_end(ap);
  if (code & Exit) exit(1);
  return 1;
}

int __attribute__ ((format (printf,3,4))) oswarningfln(ub4 line,ub4 code,const char *fmt, ...)
{
  va_list ap;
  char *errstr = getoserr();
  char buf[MSGLEN];

  va_start(ap, fmt);
  vsnprint(buf,0,sizeof(buf),fmt,ap);
  va_end(ap);
  return warningfln(line,code,"%s: %s",buf,errstr);
}

int __attribute__ ((format (printf,3,4))) oserrorfln(ub4 line,ub4 code,const char *fmt, ...)
{
  va_list ap;
  char *errstr = getoserr();
  char buf[MSGLEN];

  va_start(ap, fmt);
  vsnprint(buf,0,sizeof(buf),fmt,ap);
  va_end(ap);
  return errorfln(line,code,"%s: %s",buf,errstr);
}

int limit_gt_fln(ub4 x,ub4 lim,const char *sx,const char *slim, ub4 fln)
{
  if (x <= lim) return x;
  warningfln(fln,0,"limiting %s:%u to %s:%u",sx,x,slim,lim);
  return lim;
}

void error_ge_cc_fln(ub4 a,ub4 b,const char *sa,const char *sb,ub4 line,const char *fmt,...)
{
  va_list ap;

  if (a < b) return;

  va_start(ap,fmt);
  vmsg(Info,line,fmt,ap);
  va_end(ap);
  assertfln(line,Exit,"\n%s:%u >= %s:%u", sa,a,sb,b);
}

void error_gt_cc_fln(size_t a,size_t b,const char *sa,const char *sb,ub4 line,const char *fmt,...)
{
  va_list ap;

  if (a <= b) return;

  va_start(ap,fmt);
  vmsg(Info,line,fmt,ap);
  va_end(ap);
  assertfln(line,Exit,"\n%s:%lu > %s:%lu", sa,a,sb,b);
}

void __attribute__ ((format (printf,5,6))) progress2(struct eta *eta,ub4 fln,ub4 cur,ub4 end,const char *fmt, ...)
{
  va_list ap;
  ub8 sec = 1000 * 1000,dt,est,now = gettime_usec();
  ub4 perc;
  char buf[256];
  ub4 pos,len = sizeof(buf);

  va_start(ap,fmt);

  if (cur == 0) {
    eta->cur = eta->end = 0;
    eta->stamp = eta->start = now;
  } else if (cur >= end || now - eta->stamp < 2 * sec) return;
  eta->stamp = now;

  pos = vsnprint(buf,0,len,fmt,ap);
  va_end(ap);
  perc = (ub4)(((unsigned long)cur * 100) / end);
  perc = min(perc,100);

  dt = (now - eta->start) / sec;
  if (perc == 0) est = 0;
  else if (perc == 100) est = 0;
  else {
    dt = dt * 100 / perc;
    est = (dt * (100UL - perc)) / 100;
  }
  if (cur) pos += mysnprintf(buf,pos,len," %u%%  est %u sec",perc,(ub4)est);
  infofln(fln,0,"%s",buf);
}

// level to be set beforehand
void inimsg(char *progname, const char *logname, ub4 opts)
{
  vrb(2,"init msg for %s",logname);

  msg_fd = oscreate(logname);

  if (msg_fd == -1) msg_fd = 2;

  progstart = gettime_usec();

  msgopts = opts;
  msgfile = setmsgfile(__FILE__);
  if (msg_fd > 2 && (opts & Msg_init)) {
    infofln(0,User,"opening log %s for %s\n", logname,progname);
  }
  iniassert();
}

void setmsglvl(enum Msglvl lvl, ub4 vlvl)
{
  msglvl = lvl;
  vrblvl = vlvl;
}

// make assert and debug calls more compact
ub4 setmsgfile(const char *filename)
{
  char *ext;
  struct filecoord *fc;
  ub4 len;

  if (filendx + 1 >= Elemcnt(filenames)) return 0;
  ext = strrchr(filename,'.');
  fc = filenames + filendx;
  if (ext) len = (ub4)(ext - filename);
  else len = (ub4)strlen(filename);
  len = min(len,sizeof(fc->name) - 1);
  memcpy(fc->name,filename,len);
  return (filendx++ << 16);
}
