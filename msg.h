// msg.h -messages, logging and assertions

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Defines for console, debug and assert messages
   Most entries are macros to add file and line to the functions
   Assertions are here for trivial inlining or low-overhead calling
 */

enum Msglvl { Msglvl_nil,Fatal,Assert,Error,Warn,Info,Vrb,Msglvl_last };

enum Msgcode {
  Indent = 0xff,
  Exit = 0x100,   // exit program
  CC   = 0x200,   // keep to precede next error
  User = 0x400, // user-style, undecorated
  Ind  = 0x800,  // indent

  GT   = 0x1000,
  GE   = 0x2000,

  Iter = 0x4000,

  V0   = 0x10000, V1 = 0x20000, V3 = 0x30000
};

enum Msgopts { Msg_init = 1, Msg_stamp = 2, Msg_pos = 4, Msg_type = 8, Msg_ccerr = 16 };

struct eta {
  ub8 now,stamp,start;
  ub4 cur,end;
};

// arrange file coords
#define FLN (__LINE__|msgfile)
#define caller __LINE__|msgfile

#define genmsg(lvl,code,fmt,...) genmsgfln(FLN,(lvl),(code),(fmt),__VA_ARGS__)
#define vrb(code,fmt,...) vrbfln(FLN,(code),(fmt),__VA_ARGS__)
#define info(code,fmt,...) infofln(FLN,(code),(fmt),__VA_ARGS__)
#define warning(code,fmt,...) warningfln(FLN,(code),(fmt),__VA_ARGS__)
#define error(code,fmt,...) errorfln(FLN,(code),(fmt),__VA_ARGS__)
#define oserror(code,fmt,...) oserrorfln(FLN,(code),(fmt),__VA_ARGS__)
#define oswarning(code,fmt,...) oswarningfln(FLN,(code),(fmt),__VA_ARGS__)

#define info0(code,s) info0fln(FLN,(code),(s))

// no misprint: access first two format args
#define progress(eta,fmt,cur,end,...) progress2((eta),FLN,(cur),(end),(fmt),(cur),(end),__VA_ARGS__)

extern void progress2(struct eta *eta,ub4 fln,ub4 cur,ub4 end,const char *fmt, ...) __attribute__ ((format (printf,5,6)));

extern ub4 mysnprintf(char *dst, ub4 pos, ub4 len, const char *fmt, ...) __attribute__ ((format (printf,4,5)));

#ifdef va_arg
  extern ub4 myvsnprintf(char *dst, ub4 pos, ub4 len, const char *fmt, va_list ap);
  extern void vmsg(enum Msglvl lvl,ub4 fln,const char *fmt,va_list ap);
#endif

#define fmtstring(dst,fmt,...) mysnprintf((dst),0,sizeof (dst),(fmt),__VA_ARGS__)

extern ub4 setmsgfile(const char *filename);
extern ub4 msgfln(char *dst,ub4 pos,ub4 len,ub4 fln,ub4 wid);
extern void msg_write(const char *buf,ub4 len);

extern void inimsg(char *progname, const char *logname, ub4 opts);
extern void eximsg(void);

extern void setmsglvl(enum Msglvl lvl, ub4 vlvl);

// assertions: error_eq(a,b) to be read as 'error if a equals b'
// when failing, both names and values are shown
#define error_eq(a,b) error_eq_fln((a),(b),#a,#b,FLN)
#define error_ne(a,b) error_ne_fln((a),(b),#a,#b,FLN)
#define error_z(a,b) error_z_fln((a),(b),#a,#b,FLN)
#define error_nz(a,b) error_nz_fln((a),(b),#a,#b,FLN)
#define error_zz(a,b) error_zz_fln((a),(b),#a,#b,FLN)
#define error_zp(a,b) error_zp_fln((void*)(a),(b),#a,#b,FLN)
#define error_gt(a,b) error_gt_fln((a),(b),#a,#b,FLN)
#define error_ge(a,b) error_ge_fln((a),(b),#a,#b,FLN)
#define error_le(a,b) error_le_fln((a),(b),#a,#b,FLN)
#define error_lt(a,b) error_lt_fln((a),(b),#a,#b,FLN)

#define error_gt2(a1,a2,b) error_gt2_fln((a1),(a2),(b),#a1,#a2,#b,FLN)

#define error_eq_cc(a,b,fmt,...) if ((a) == (b)) error_cc_fln((a),(b),#a,#b,"==",FLN,fmt,__VA_ARGS__)
#define error_z_cc(a,fmt,...) if ((a) == 0) error_cc_fln((a),0,#a,"0","==",FLN,fmt,__VA_ARGS__)

#define error_ge_cc(a,b,fmt,...) error_ge_cc_fln((a),(b),#a,#b,FLN,fmt,__VA_ARGS__)
#define error_gt_cc(a,b,fmt,...) error_gt_cc_fln((a),(b),#a,#b,FLN,fmt,__VA_ARGS__)

#define error_ovf(a,b) error_ovf_fln((a),sizeof(b),#a,#b,FLN)

#define limit_gt(x,lim,arg) (x) = limit_gt_fln((x),(lim),(arg),#x,#lim,#arg,FLN)

extern int genmsgfln(ub4 fln,enum Msglvl lvl,ub4 code,const char *fmt,...) __attribute__ ((format (printf,4,5)));
extern void vrbfln(ub4 fln, ub4 code, const char *fmt, ...) __attribute__ ((format (printf,3,4)));
extern int infofln(ub4 fln, ub4 code, const char *fmt, ...) __attribute__ ((format (printf,3,4)));
extern int warningfln(ub4 fln, ub4 code, const char *fmt, ...) __attribute__ ((format (printf,3,4)));
extern int errorfln(ub4 fln, ub4 code, const char *fmt, ...) __attribute__ ((format (printf,3,4)));
extern int oserrorfln(ub4 fln,ub4 code,const char *fmt, ...) __attribute__ ((format (printf,3,4)));
extern int oswarningfln(ub4 fln,ub4 code,const char *fmt, ...) __attribute__ ((format (printf,3,4)));
extern int assertfln(ub4 fln, ub4 code, const char *fmt, ...) __attribute__ ((format (printf,3,4)));

extern int info0fln(ub4 fln, ub4 code, const char *s);

extern ub4 limit_gt_fln(ub4 x,ub4 lim,ub4 arg,const char *sx,const char *slim,const char *sarg,ub4 fln);

extern void error_cc_fln(ub4 a,ub4 b,const char *sa,const char *sb,const char *cc,ub4 line,const char *fmt,...);

extern void error_ge_cc_fln(ub4 a,ub4 b,const char *sa,const char *sb,ub4 line,const char *fmt,...);
extern void error_gt_cc_fln(size_t a,size_t b,const char *sa,const char *sb,ub4 line,const char *fmt,...);

extern void enter(ub4 fln);
extern void leave(ub4 fln);

static void error_eq_fln(ub4 a,ub4 b,const char *sa,const char *sb,ub4 line)
{
  if (a != b) return;

  assertfln(line,Exit,"%s:\ah%u == %s:\ah%u", sa,a,sb,b);
}

static void error_ne_fln(size_t a,size_t b,const char *sa,const char *sb,ub4 line)
{
  if (a == b) return;

  assertfln(line,Exit,"%s:\ah%lu != %s:\ah%lu", sa,a,sb,b);
}

static void error_gt_fln(size_t a,size_t b,const char *sa,const char *sb,ub4 line)
{
  if (a <= b) return;

  assertfln(line,Exit,"%s:\ah%lu > %s:\ah%lu", sa,a,sb,b);
}

static void error_ge_fln(size_t a,size_t b,const char *sa,const char *sb,ub4 line)
{
  if (a < b) return;

  assertfln(line,Exit,"%s:\ah%lu >= %s:\ah%lu", sa,a,sb,b);
}

static void error_lt_fln(ub4 a,ub4 b,const char *sa,const char *sb,ub4 line)
{
  if (a >= b) return;

  assertfln(line,Exit,"%s:%u < %s:%u", sa,a,sb,b);
}

static void error_le_fln(ub4 a,ub4 b,const char *sa,const char *sb,ub4 line)
{
  if (a > b) return;

  assertfln(line,Exit,"%s:%u <= %s:%u", sa,a,sb,b);
}

static void error_z_fln(size_t a,ub4 b,const char *sa,const char *sb,ub4 line)
{
  if (a != 0) return;

  assertfln(line,Exit,"%s = 0 at %s:%u", sa,sb,b);
}

static void error_nz_fln(ub4 a,ub4 b,const char *sa,const char *sb,ub4 line)
{
  if (a == 0) return;

  assertfln(line,Exit,"%s:%u != 0 at %s:%u", sa,a,sb,b);
}

static void error_zz_fln(size_t a,size_t b,const char *sa,const char *sb,ub4 line)
{
  if (a == 0) assertfln(line,Exit,"%s = 0", sa);
  else if (b == 0) assertfln(line,Exit,"%s = 0", sb);
}

static void error_zp_fln(void *a,ub4 b,const char *sa,const char *sb,ub4 line)
{
  if (a) return;

  assertfln(line,Exit,"%s == nil at %s:%u", sa,sb,b);
}

static void error_gt2_fln(size_t a1,size_t a2,ub8 b,const char *sa1,const char *sa2,const char *sb,ub4 line)
{
  if (a1 > b) assertfln(line,Exit,"%s:%lu > %s:%lu", sa1,a1,sb,b);
  if (a2 > b) assertfln(line,Exit,"%s:%lu > %s:%lu", sa2,a2,sb,b);
  if ((ub8)a1 + (ub8)a2 > b) assertfln(line,Exit,"%s:%lu+%s:%lu > %s:%lu", sa1,a1,sa2,a2,sb,b);
}

static ub4 ovfsizes[] = { 0, 0xff, 0xffff, 0, 0xffffffff, 0, 0, 0, 0xffffffff };

static void error_ovf_fln(ub4 a,ub4 b,const char *sa,const char *sb,ub4 line)
{
  ub4 bb;

  if (b > 7) assertfln(line,Exit,"%s:%u overflows sizeof %s:%u", sa,a,sb,b);

  bb = ovfsizes[b];
  if (a < bb) return;

  assertfln(line,Exit,"%s:%u overflows sizeof %s:%u", sa,a,sb,b);
}

// dummy to prevent 'unused' warnings, as above are static
static void iniassert(void)
{
  error_lt(1,1);
  error_le(2,1);
  error_gt(1,1);
  error_ge(1,2);
  error_eq(1,2);
  error_ne(1,1);
  error_z(1,0);
  error_nz(0,2);
  error_zz(1,1);
  error_zp("",0);

  error_gt2(1,0,1);

  error_ovf(1024,ub2);
}
