// os.c - operating system specifics

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define _POSIX_C_SOURCE 199309L

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/resource.h>

#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include <errno.h>
#include <signal.h>

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

int oscreate(const char *name)
{
  int fd = creat(name,0644);
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

char *getoserr(void)
{
  return strerror(errno);
}

static void __attribute__ ((noreturn)) mysigact(int sig,siginfo_t *si,void * __attribute__ ((unused)) p)
{
  char buf[1024];
  ub4 pos;
  size_t adr,nearby;

  switch(sig) {
  case SIGSEGV:
    adr = (size_t)(si->si_addr);
    nearby = nearblock(adr);
    pos = mysnprintf(buf,0,sizeof buf,"\nsigsegv at %lx near %lx\n", (unsigned long)adr,(unsigned long)nearby);
    break;
  default: pos = mysnprintf(buf,0,sizeof buf,"\nsignal %u\n", sig);
  }
  oswrite(2,buf,pos);
  _exit(1);
}

int setsigs(void)
{
  struct sigaction sa;

  memset(&sa,0,sizeof(sa));
  sa.sa_sigaction = mysigact;
  sa.sa_flags = SA_SIGINFO;

  sigaction(SIGSEGV, &sa,NULL);
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

static const char namepattern[] = "p_glob_542346b6_5dfa_24057.rcv";

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

  info(0,"write %s len %u",mfrep->name,(ub4)(mfrep->len));
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

  local cliont deletes used entries
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
  char *pname,*p,*q,*extpos;
  ub4 stamp,histamp,lostamp;
  ino_t inolo,inohi;
  struct stat ino;

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
    if (de->d_name[0] == '.') continue;
    namelen = strlen(de->d_name);
    if (namelen + 1 < sizeof(namepattern)) { info(0,"expected pattern %s",namepattern); continue; }
    extpos = strstr(de->d_name,ext);
    if (!extpos) { vrb(0,"skip %s on extension",de->d_name); continue; }

    // basename
    q = name; p = de->d_name;
    while (q + 1 < name + sizeof(name) && *p != '.') *q++ = *p++;
    *q = 0;

    // region
    q = regionstr; p = de->d_name + 2;
    while (q + 1 < regionstr + sizeof(regionstr) && *p != '_') *q++ = *p++;
    *q = 0;
    if (*p != '_' || strcmp(regionstr,region)) {
      vrb(0,"skip %s on region not %s",de->d_name,region);
      continue;
    }

    // timestamp
    p++; q = timestr;
    while (q + 1 < timestr + sizeof(timestr) && *p != '_') *q++ = *p++;
    *q = 0;
    if (*p != '_' || q == timestr) { vrb(0,"skip %s on no timestamp",de->d_name); continue; }

    if (hex2ub4(timestr,&stamp)) { vrb(0,"skip %s on no timestamp",de->d_name); continue; }
    vrb(0,"%s stamp %u",name,stamp);
    if (stamp > histamp) {
      histamp = stamp;
      inohi = de->d_ino;
      strcopy(hiname,name);
    }
    if (stamp < lostamp) {
      lostamp = stamp;
      inolo = de->d_ino;
      strcopy(loname,name);
    }
  } while (1);

  closedir(dir);

  if (histamp && hiname[0] != 'p') pname = hiname;
  else if (lostamp != hi32) pname = loname;
  else return info(0,"no requests found, stamp %u",stamp);

  mf->basename = (ub4)strlen(qdir) + 1;

  fmtstring(oldname,"%s/%s%s",qdir,pname,ext);
  fmtstring(newname,"%s/%s_%u%s",qdir,pname,globs.serverid,".rcv");
  info(0,"rename %s to %s",oldname,newname);
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
  else info(0,"read %u bytes of %s",(ub4)len,newname);
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

static int rlimit(int res,rlim_t lim,const char *desc)
{
  struct rlimit rlim;

  if (getrlimit(res,&rlim)) return oserror(0,"cannot get resource limit for %s",desc);

  rlim.rlim_cur = lim;
  if (setrlimit(res,&rlim)) return oserror(0,"cannot set resource limit for %s",desc);
  return info(0,"resource limit for %s set to \ah%lu",desc,lim);
}

int oslimits(void)
{
  int rv;

  rv = rlimit(RLIMIT_AS,Maxmem,"virtual memory");
  rv |= rlimit(RLIMIT_CORE,1024 * 1024,"core size");
  return rv;
}

void inios(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
  globs.serverid = getpid();
}
