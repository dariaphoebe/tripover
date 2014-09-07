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
   is included here that is compatible with printf, yet contains custom features

  assertions are used extensively, and are made to be enabled in production 
 */

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "base.h"

static ub4 msgfile;
#include "msg.h"

#include "os.h"
#include "time.h"

static const char *filenames[32];
static ub4 filendx = 1;

static enum Msglvl msglvl = Vrb;
static ub4 vrblvl = 0;
static ub4 msgopts;

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

static int msg_fd;
static ub8 progstart;

// temporary choice. unbuffered is useful for console messages,
// yet debug logging may ask for buffering
static void msgwrite(char *buf, ub4 len)
{
  sb4 nw;

  nw = oswrite(msg_fd,buf,len);

  if (nw <= 0) nw = oswrite(2,"\nI/O error on msg write\n",24);
  if (nw <= 0) oserrcnt++;
}

// make errors appear on stderr
static void myttywrite(char *buf, ub4 len)
{
  sb4 nw;

  nw = oswrite(2,buf,len);
  if (nw <= 0) oserrcnt++;
}

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

static ub4 Ucnv(char *dst, ub4 x1,ub4 x2,char c)
{
  ub4 n;

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

/* supports a basic, yet compatible subset of printf :
   %c %d %u %x %s %p %03u %-12.6s %ld
   no floating point
   extensions are led in by '\a' preceding a conversion :
   \ah%u  makes the integer formatted 'human readable' like 123.8 M for 123800000
   \av%u%p interprets the pointer arg '%p' as an array of '%u' integers. thus :  
   int arr[] = { 23,65,23,54 };  printf("\av%u%p", 4, arr ); 
    shows  '[23 65 23 54]'
 */

static ub4 myvsnprintf(char *dst, ub4 pos, ub4 len, const char *fmt, va_list ap)
{
  const char *p = fmt;
  ub4 n = 0,x;
  ub4 wid,prec,plen;
  unsigned int uval,vlen=0,*puval;
  unsigned long luval,lx;
  int ival,alt,padleft,do_U = 0, do_vec = 0;
  long lival;
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
      if (c2 == '0') pad = c2;
      while (c2 >= '0' && c2 <= '9') {
        wid = wid * 10 + (c2 - '0');
        c2 = *p++;
      }
      wid = min(wid,len - n);
      alt = 0;
      if (c2 == '#') { alt = 1; c2 = *p++; }
      else if (c2 == '.') {
        prec = 0;
        c2 = *p++;
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
        case 'x': luval = va_arg(ap,unsigned long);
                  if (len - n <= 10) break;
                  n += xcnv(dst + n,(ub4)luval);
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
                  if (do_vec) {
                    while (vlen-- && n < len) {
                      uval = *puval++;
                      if (len - n <= 10) break;
                      n += ucnv(dst + n,uval,wid,pad);
                      dst[n++] = '-';
                    }
                    do_vec = 0;
                  } else {
                    if (len - n <= 10) break;
                    n += xcnv(dst + n,(unsigned int)puval);
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
  return n;
}

ub4 __attribute__ ((format (printf,4,5))) mysnprintf(char *dst, ub4 pos, ub4 len, const char *fmt, ...)
{
  va_list ap;
  ub4 n;

  va_start(ap, fmt);
  n = myvsnprintf(dst,pos,len,fmt,ap);
  va_end(ap);
  return n;
}

// main message printer. supports decorated and undecorated style
static void __attribute__ ((nonnull(5))) msg(enum Msglvl lvl, ub4 sublvl, ub4 fline, ub4 code, const char *fmt, va_list ap)
{
  ub4 line,fileno;
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
    d100usec = (dusec - dsec) / 100;
  } else dsec = d100usec = 0;
  if (lvl >= Msglvl_last) {
    pos += mysnprintf(msgbuf,pos,maxlen, "\nE unknown msglvl %u\n",lvl);
    lvl = Error;
  } else if (lvl <= Warn) { // precede errors with relevant past message
    if (cclen) {
      line = ccfln & 0xffff;
      fileno = ccfln >> 16;
      if (fileno < Elemcnt(filenames)) n = mysnprintf(ccbuf2,0,maxlen, "CC         %s %4u %s",filenames[fileno],line,ccbuf);
      else n = mysnprintf(ccbuf2,0,maxlen, "CC         *%x* %4u %s",fileno,line,ccbuf);
      msgwrite(ccbuf2,n);
    }
    cclen = 0;
  }

  lvlnam = msgnames[lvl];

  if (*fmt == '\n') {
    msgbuf[pos++] = '\n';
  }
  if (opts & Msg_type) pos += mysnprintf(msgbuf, pos, maxlen, "%c%u ",lvlnam,sublvl);
  else if (lvl <= Warn) pos += mysnprintf(msgbuf, pos, maxlen, "%s ",msgnames_long[lvl]);

  if (opts & Msg_stamp) pos += mysnprintf(msgbuf, pos, maxlen, "%03u.%04u  ",(ub4)dsec,(ub4)d100usec);
  if (opts & Msg_pos) {
    line = fline & 0xffff;
    fileno = fline >> 16;
    if (fileno < Elemcnt(filenames)) pos += mysnprintf(msgbuf, pos, maxlen, "%s %4u ",filenames[fileno],line);
    else pos += mysnprintf(msgbuf, pos, maxlen, "*%x* %4u ",fileno,line);
  }

  if (code & Ind) {
    n = min(code & 0xff, maxlen - pos);
    if (n) memset(msgbuf + pos,' ',n);
    pos += n;
  }
  if (lvl == Assert) pos += mysnprintf(msgbuf, pos, maxlen, "assert");
  pos += myvsnprintf(msgbuf, pos, maxlen, fmt, ap);
  if (pos < maxlen) msgbuf[pos++] = '\n';
  msgwrite(msgbuf, pos);
  if ( (opts & Msg_ccerr) && lvl <= Warn && msg_fd != 2) myttywrite(msgbuf,pos);
}

void __attribute__ ((format (printf,3,4))) vrbfln(ub4 fln, ub4 code, const char *fmt, ...)
{
  va_list ap;
  va_list ap1;
  ub4 lvl = (code >> 14) & 3;

  if (code & CC) {
    code &= ~CC;
    va_start(ap1, fmt);
    cclen = myvsnprintf(ccbuf,0,sizeof(ccbuf),fmt,ap1);
    ccfln = fln;
    va_end(ap1);
  }

  if (msglvl < Vrb || lvl > vrblvl) return;

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

int __attribute__ ((format (printf,3,4))) oserrorfln(ub4 line,ub4 code,const char *fmt, ...)
{
  va_list ap;
  char *errstr = getoserr();
  char buf[MSGLEN];

  va_start(ap, fmt);
  myvsnprintf(buf,0,sizeof(buf),fmt,ap);
  va_end(ap);
  errorfln(line,code,"%s: %s",buf,errstr);
  return 1;
}

void inimsg(char *progname, int fd, ub4 opts, enum Msglvl lvl, ub4 vlvl)
{
  msg_fd = fd;

  progstart = gettime_usec();

  msglvl = lvl;
  vrblvl = vlvl;

  msgopts = opts;
  filenames[0] = "(no file)";
  msgfile = setmsgfile(__FILE__);
  if (fd != 1 && fd != 2 && (opts & Msg_init)) {
    infofln(0,User,"opening log for %s", progname);
  }
  iniassert();
}

// just to make assert and debug calls more compact
ub4 setmsgfile(const char *filename)
{
  if (filendx + 1 >= Elemcnt(filenames)) return 0;
  filenames[++filendx] = filename;
  return (filendx << 16);
}
