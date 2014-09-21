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

#include <fcntl.h>
#include <unistd.h>

#include <errno.h>
#include <signal.h>

#include <string.h>

#include "base.h"
#include "mem.h"

static ub4 msgfile;
#include "msg.h"

#include "os.h"

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

  if (fstat(fd,&ino)) { osclose(fd); return 1; }
  mf->mtime = ino.st_mtime;
  mf->len = ino.st_size;
  return 0;
}

int oswrite(int fd, const void *buf,ub4 len)
{
  return (int)write(fd,buf,len);
}

int osread(int fd,void *buf,size_t len)
{
  ssize_t nread;

  nread = read(fd,buf,len);
  if (nread != (ssize_t)len) { osclose(fd); return 1; }
  return 0;
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
}
