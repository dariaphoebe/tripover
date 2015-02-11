// os.c - operating system specifics

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define _POSIX_C_SOURCE 200112L

#undef USE_GLIBC_EXT

#define _BSD_SOURCE
#include <sys/mman.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/resource.h>

#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include <errno.h>
#include <signal.h>

#ifdef USE_GLIBC_EXT
 #include <execinfo.h>
#endif

#include <string.h>
#include <stdio.h>
#include <time.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"

static ub4 msgfile;
#include "msg.h"

#include "os.h"
#include "util.h"

// copy at tentative messages
static char msginfo[1024];
static ub4 msginfolen;

static pid_t mypid;
static char pidstr[64];
static ub4 pidstrlen;

int oscreate(const char *name)
{
  int fd = open(name,O_CREAT|O_RDWR|O_TRUNC,0644);
  return fd;
}

int osopen(const char *name)
{
  int fd = open(name,O_RDONLY);
  return fd;
}

int osfdinfo(struct myfile *mf,int fd)
{
  struct stat ino;

  if (fstat(fd,&ino)) return 1;
  mf->mtime = ino.st_mtime;
  mf->len = ino.st_size;
  return 0;
}

int osfileinfo(struct myfile *mf,const char *name)
{
  struct stat ino;

  if (stat(name,&ino)) return 1;
  mf->mtime = ino.st_mtime;
  mf->len = ino.st_size;
  mf->isdir = S_ISDIR(ino.st_mode);
  mf->isfile = S_ISREG(ino.st_mode);
  return 0;
}

int osrewind(int fd)
{
  off_t rv;

  rv = lseek(fd,0,SEEK_SET);
  return (rv == -1 ? 1 : 0);
}

long oswrite(int fd, const void *buf,ub4 len)
{
  return write(fd,buf,len);
}

long osread(int fd,void *buf,size_t len)
{
  return read(fd,buf,len);
}

int osclose(int fd)
{
  return close(fd);
}

int osremove(const char *name)
{
  return unlink(name);
}

int osdup2(int oldfd,int newfd)
{
  return dup2(oldfd,newfd);
}

char *getoserr(void)
{
  return strerror(errno);
}

int osrotate(const char *name,const char old,const char new)
{
  char oldname[1024];
  char newname[1024];

  if (old) fmtstring(oldname,"%s.%c",name,old);
  else fmtstring(oldname,"%s",name);
  fmtstring(newname,"%s.%c",name,new);
  return rename(oldname,newname);
}

// write to stderr and eventual logfile
static void wrstderrlog(const char *buf,ub4 len)
{
  int fd = globs.msg_fd;

  oswrite(2,buf,len);
  oswrite(1,buf,len);
  if (fd > 0 && fd != 2) oswrite(fd,buf,len);
}

#ifdef MAP_ANONYMOUS
void *osmmap(size_t len)
{
  void *p = mmap(NULL,len,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0);
  return p;
}
int osmunmap(void *p,size_t len)
{
  int rv = munmap(p,len);
  return rv;
}
#else
#include <stdlib.h>
void *osmmap(size_t len)
{
  void *p = malloc(len);
  return p;
}
int osmunmap(void *p,size_t len)
{
  free(p);
}
#endif

static void mysigint(int __attribute__ ((unused)) sig,siginfo_t *si,void * __attribute__ ((unused)) pp)
{
  int n = globs.sigint++;

  if (globs.sig) _exit(1);

  if (msginfolen) {
    wrstderrlog(msginfo,msginfolen);
  }
  if (n == 0) info0(0,"interrupting: waiting to end current task");
  else if (n == 1) info0(0,"interrupting: waiting to end current subtask");
  else {
    info(0,"interrupted: code %d",si->si_code);
   _exit(1);
  }
}

static void __attribute__ ((noreturn)) mysigact(int sig,siginfo_t *si,void * __attribute__ ((unused)) pp)
{
  char buf[245];
  ub4 pos;
  size_t adr,nearby;
  int code;
  const char *codestr = "";

  globs.sig++;

  if (msginfolen) {
    wrstderrlog(msginfo,msginfolen);
  }

#ifdef USE_GLIBC_EXT
  int msgfd = globs.msg_fd;
  void *btbuf[32];
  int btcnt = backtrace(btbuf,Elemcnt(btbuf));

  backtrace_symbols_fd(btbuf,btcnt,2);
  if (msgfd > 0 && msgfd != 2) backtrace_symbols_fd(btbuf,btcnt,msgfd);
#endif

  switch(sig) {
  case SIGSEGV:
    adr = (size_t)si->si_addr;
    nearby = nearblock(adr);
    if (adr == nearby) pos = fmtstring(buf,"\nsigsegv at %lx\n", (unsigned long)adr);
    else pos = mysnprintf(buf,0,sizeof buf,"\nsigsegv at %lx near %lx\n", (unsigned long)adr,(unsigned long)nearby);
    wrstderrlog(buf,pos);
    wrstderrlog(pidstr,pidstrlen);
    pause();

  case SIGBUS:
    adr = (size_t)si->si_addr;
    nearby = nearblock(adr);
    if (adr == nearby) pos = fmtstring(buf,"\nsigbus at %lx\n", (unsigned long)adr);
    else pos = mysnprintf(buf,0,sizeof buf,"\nsigbus at %lx near %lx\n", (unsigned long)adr,(unsigned long)nearby);
    wrstderrlog(buf,pos);
    wrstderrlog(pidstr,pidstrlen);
    pause();

  case SIGFPE:
    code = si->si_code;
    switch(code) {
    case FPE_INTDIV: codestr = "int div\n"; break;
    case FPE_INTOVF: codestr = "int ovf\n"; break;
    }
    pos = fmtstring(buf,"\nsigfpe errno %d code %d ",si->si_errno,code);
    wrstderrlog(buf,pos);
    wrstderrlog(codestr,8);
    wrstderrlog(pidstr,pidstrlen);
    pause();

  default:
    pos = fmtstring(buf,"\nsignal %u\n", sig);
    wrstderrlog(buf,pos);
    wrstderrlog(pidstr,pidstrlen);
    pause();
  }
  _exit(1);
}

int setsigs(void)
{
  struct sigaction sa;

  memset(&sa,0,sizeof(sa));

  mypid = getpid();
  globs.pid = (int)mypid;
  pidstrlen = fmtstring(pidstr,"pid %u\n",(ub4)mypid);

  sa.sa_sigaction = mysigact;
  sa.sa_flags = SA_SIGINFO;

  sigaction(SIGSEGV, &sa,NULL);
  sigaction(SIGFPE, &sa,NULL);
  sigaction(SIGBUS, &sa,NULL);

  sa.sa_sigaction = mysigint;
  sigaction(SIGINT, &sa,NULL);

  return 0;
}

void osmillisleep(ub4 msec)
{
  struct timespec ts;
  ub8 msec2;

  ts.tv_sec = msec / 1000;
  msec2 = (ub8)msec % 1000;
  ts.tv_nsec = msec2 * 1000UL * 1000UL;

  nanosleep(&ts,NULL);
}

static const char namepattern[] = "p_glob_542346b6_5dfa.rcv";

// arrange reply to previous query
int setqentry(struct myfile *mfreq,struct myfile *mfrep,const char *ext)
{
  char repname[512];
  char *p,*q,*r,*extpos,*qlim,*rlim;
  int fd;

  qlim = mfrep->name + sizeof(mfrep->name) - 4;
  rlim = repname + sizeof(repname) - 4;

  extpos = strstr(mfreq->name,".rcv");
  if (!extpos) return warning(0,"no previous received query for %s",mfreq->name);
  p = mfreq->name;
  q = mfrep->name;
  r = repname;
  while (p < extpos && q < qlim && r < rlim) {
    *q++ = *p;
    *r++ = *p++;
  }
  strcpy(q,".ren");
  strcpy(r,ext);

  vrb0(0,"write %s len %u",mfrep->name,(ub4)(mfrep->len));
  fd = oscreate(mfrep->name);
  if (fd == -1) return oserror(0,"cannot create %s",mfrep->name);
  oswrite(fd,mfrep->buf,(ub4)mfrep->len);
  osclose(fd);

// rename to hand over to client
  if (rename(mfrep->name,repname)) oswarning(0,"cannot rename %s to %s",mfrep->name,repname);
  return 0;
}

/* get newest entry in dir with given ext


  filename format: cmd_region_time_client_server.ext
  sub = submitted
  rcv = received
  rep = reply

  local client deletes used entries
 */
int getqentry(const char *qdir,struct myfile *mf,const char *region,const char *ext)
{
  DIR *dir;
  int fd,rv;
  size_t len,namelen;
  char *buf;
  ssize_t nr;
  struct dirent *de;
  char timestr[64];
  char regionstr[64];
  char name[256];
  char loname[256];
  char hiname[256];
  char oldname[512];
  char newname[512];
  char *pname,*dname,*p,*q,*extpos;
  ub4 stamp,histamp,lostamp;
  struct stat ino;
  ub8 usec = gettime_usec();
  ub4 now = (ub4)(usec / (1000 * 1000));
  ub4 iter = 0;

  clear(mf);

  if (!qdir || !*qdir) return error(0,"nil queue directory for %u",globs.serverid);

  histamp = 0; lostamp = hi32;

  dir = opendir(qdir);

  if (!dir) {
    switch(errno) {
    case ENOENT: return oswarning(0,"directory %s does not exist",qdir);
    case EACCES: return oswarning(0,"cannot access directory %s",qdir);
    default: return oserror(0,"cannot access directory %s",qdir);
    }
  }

  mf->direxist = 1;
  stamp = 0;

  do {
    de = readdir(dir);
    if (!de) break;
    dname = de->d_name;
    if (dname[0] == '.') continue;

    iter++;
    namelen = strlen(dname);
    if (namelen + 1 < sizeof(namepattern)) { info(0,"expected pattern %s, found %s",namepattern,dname); continue; }
    extpos = strstr(dname,ext);
    if (!extpos) { vrb(0,"skip %s on extension",dname); continue; }

    // basename
    q = name; p = dname;
    while (q + 1 < name + sizeof(name) && *p != '.') *q++ = *p++;
    *q = 0;

    // region
    q = regionstr; p = dname + 2;
    while (q + 1 < regionstr + sizeof(regionstr) && *p != '_') *q++ = *p++;
    *q = 0;
    if (*p != '_' || strcmp(regionstr,region)) {
      vrb(0,"skip %s on region not %s",dname,region);
      continue;
    }

    // timestamp
    p++; q = timestr;
    while (q + 1 < timestr + sizeof(timestr) && *p != '_') *q++ = *p++;
    *q = 0;
    if (*p != '_' || q == timestr) { vrb(0,"skip %s on no timestamp",dname); continue; }

    if (hex2ub4(timestr,&stamp)) { vrb(0,"skip %s on no timestamp",dname); continue; }
    vrb(0,"%s stamp %u",name,stamp);
    if (stamp > now) warning(0,"%s has time %u %u secs in the future",dname,stamp,stamp - now);
    else if (now - stamp > Queryage) {
      infocc(iter == 1,0,"skip %s on age %u secs",dname,now - stamp);
      continue;
    }
    if (stamp > histamp) {
      histamp = stamp;
      strcopy(hiname,name);
    }
    if (stamp < lostamp) {
      lostamp = stamp;
      strcopy(loname,name);
    }
  } while (1);

  closedir(dir);

  if (histamp && hiname[0] != 'p') pname = hiname;
  else if (lostamp != hi32) pname = loname;
  else return genmsg(Vrb,0,"no requests found, stamp %u",stamp);

  mf->basename = (ub4)strlen(qdir) + 1;

  fmtstring(oldname,"%s/%s%s",qdir,pname,ext);
  fmtstring(newname,"%s/%s_%u%s",qdir,pname,globs.serverid,".rcv");
  vrb0(0,"rename %s to %s",oldname,newname);
  rv = rename(oldname,newname);
  if (rv) {
    switch(errno) {
    case ENOENT: return info(0,"did not rename %s to %s: not existing",oldname,newname);
    case EACCES: case EBUSY: return oswarning(0,"cannot rename %s to %s",oldname,newname);
    default: return oserror(0,"cannot rename %s to %s",oldname,newname);
    }
  }
  fd = osopen(newname);
  if (fd == -1) return oswarning(0,"cannot open %s",newname);
  rv = fstat(fd,&ino);
  if (rv == -1) { oswarning(0,"cannot access %s",newname); return osclose(fd); }
  strcopy(mf->name,newname);
  mf->exist = 1;
  len = ino.st_size;
  if (len == 0) {
    osclose(fd);
    return info(0,"%s is empty",newname);
  } else if (len >= Maxquerysize) {
    osclose(fd);
    return warning(0,"%s exceeds %u",newname,Maxquerysize);
  } else if (len >= sizeof(mf->localbuf)) {
    buf = alloc((ub4)len,char,0,"query",stamp);
    mf->alloced = 1;
  } else {
    buf = mf->localbuf;
  }
  mf->buf = buf;
  mf->len = len;
  nr = osread(fd,buf,len);
  if (nr == -1) { oswarning(0,"cannot read %s",newname); return osclose(fd); }
  osclose(fd);
  if (nr != (ssize_t)len) return warning(0,"partial read %u of %u bytes of %s",(ub4)nr,(ub4)len,newname);
  else vrb0(0,"read %u bytes of %s",(ub4)len,newname);
  return 0;
}

ub8 gettime_usec(void)
{
  struct timeval tv;
  ub8 usec;

  gettimeofday(&tv,0);
  usec = (ub8)tv.tv_sec * 1000UL * 1000UL;
  usec += tv.tv_usec;
  return usec;
}

static int rlimit(int res,rlim_t lim,const char *desc,int show)
{
  struct rlimit rlim;

  if (getrlimit(res,&rlim)) return oserror(0,"cannot get resource limit for %s",desc);

  rlim.rlim_cur = min(lim,rlim.rlim_max);
  if (setrlimit(res,&rlim)) return oserror(0,"cannot set resource limit for %s",desc);
  return infovrb(show,0,"resource limit for %s set to \ah%lu",desc,(unsigned long)lim);
}

// physical mem in mb
ub4 osmeminfo(void)
{
#if (defined _SC_PHYS_PAGES) && (defined _SC_PAGESIZE)

  ub8 pagesize,pagecnt,mb;
  long lval;

  lval = sysconf(_SC_PAGESIZE);
  if (lval < 1024) return 0;
  pagesize = (ub8)lval;
  lval = sysconf(_SC_PHYS_PAGES);
  if (lval < 1024) return 0;
  pagecnt = (ub8)lval;
  mb = (pagesize * pagecnt) >> 20;
  return (ub4)mb;
#else
  return 0;
#endif
}

int oslimits(void)
{
  int rv = 0;
  rlim_t maxvm;

  if (globs.maxvm < hi24) {
    maxvm = (ub8)(globs.maxvm + 4) * 1024 * 1024 * 1024;
    rv |= rlimit(RLIMIT_AS,maxvm,"virtual memory +4 GB margin",1);
  }

  rv |= rlimit(RLIMIT_CORE,0,"core size",0);
  return rv;
}

void setmsginfo(char *buf,ub4 len)
{
  memcpy(msginfo,buf,min(len,sizeof(msginfo)));
  msginfolen = len;
}

void inios(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
  globs.serverid = getpid();
}
